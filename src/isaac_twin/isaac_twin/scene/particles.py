"""PhysX PBD granular powder for the Isaac twin (``powder.model: particles``).

Same interface ``build_cell.py`` uses on ``PowderCell``: ``bed_g``,
``payload_g``, ``rs3_g``, ``table_g``, ``fill_depth_m``, ``reset()`` and
``step(dt, base_to_tcp, vibration)``. Grains are simulated on the GPU and
read back from USD for the accounting in ``isaac_twin.particle_powder``.
"""

from __future__ import annotations

import numpy as np
from omni.physx.scripts import particleUtils, physicsUtils
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

from isaac_twin.particle_powder import SCOOP, ParticleAccounting, ParticleTotals

ROOT = "/World/Powder"


def configure_physics_scenes(stage, gravity: bool, gpu: bool) -> list[str]:
    """Gravity and GPU dynamics on every physics scene (the URDF importer adds one)."""
    paths = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Scene):
            continue
        UsdPhysics.Scene(prim).CreateGravityMagnitudeAttr(9.81 if gravity else 0.0)
        if gpu:
            api = PhysxSchema.PhysxSceneAPI.Apply(prim)
            api.CreateEnableGPUDynamicsAttr(True)
            api.CreateBroadphaseTypeAttr("GPU")
            api.CreateEnableCCDAttr(False)
        paths.append(str(prim.GetPath()))
    return paths


def disable_link_gravity(stage, root: str) -> int:
    """The arm is position driven; gravity on its links only adds tracking error."""
    n = 0
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(root) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
            n += 1
    return n


def add_static_collider(prim, approximation: str = "convexDecomposition", sdf_resolution: int = 256) -> None:
    """Static collider on a layout prim. Meshes: ``convexDecomposition`` (thick
    convex pieces), ``sdf`` or ``none`` (triangle mesh; grains slip through
    concave seams such as RS6's floor/ramp crease)."""
    UsdPhysics.CollisionAPI.Apply(prim)
    if not prim.IsA(UsdGeom.Mesh):
        return
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
    if approximation == "sdf":
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim).CreateSdfResolutionAttr(int(sdf_resolution))
    elif approximation == "convexDecomposition":
        api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
        api.CreateMaxConvexHullsAttr(64)
        api.CreateHullVertexLimitAttr(32)
        api.CreateVoxelResolutionAttr(1_000_000)
        api.CreateShrinkWrapAttr(True)
        api.CreateErrorPercentageAttr(0.5)


def filter_robot_from(stage, robot_root: str, cell_root: str) -> int:
    """Robot links never collide with the vessels/table (MTC lets the scoop touch
    them and the drives are stiff); particles still collide with both.

    Filtered pairs per collider: putting the robot in a ``CollisionGroup``
    made the position-driven arm drift off its targets."""
    links = [
        prim.GetPath()
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(robot_root) and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    n = 0
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(cell_root) and prim.HasAPI(UsdPhysics.CollisionAPI):
            rel = UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
            for link in links:
                rel.AddTarget(link)
            n += 1
    return n


def shell_voxels(tris: np.ndarray, voxel_m: float) -> np.ndarray:
    """Centres of the ``voxel_m`` cells the mesh surface passes through."""
    from scoop_vision.mesh import sample_surface

    pts = sample_surface(np.asarray(tris, dtype=float), voxel_m / 3.0)
    return np.unique(np.floor(pts / voxel_m).astype(np.int64), axis=0) * voxel_m + voxel_m / 2


def scoop_collider(
    stage, robot_root: str, tool_link: str, tris_tool: np.ndarray, kind: str = "voxels", resolution: int = 256,
    voxel_m: float = 0.003,
) -> str:
    """Swap the importer's box collider on the scoop for one that leaves the bowl open.

    ``voxels``: a compound of cubes on the scoop surface (robust to the STL's
    non-manifold edges); ``sdf`` / ``convexDecomposition``: mesh approximations."""
    link = None
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(robot_root) and prim.GetName() == tool_link and prim.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            link = prim
            break
    if link is None:
        raise RuntimeError(f"no rigid body {tool_link} under {robot_root}")
    # The importer's colliders sit under instanceable prims, which plain
    # child iteration does not enter; de-instance them so they can be edited.
    while True:
        proxies = [p for p in _colliders(link) if p.IsInstanceProxy()]
        if not proxies:
            break
        anc = proxies[0].GetParent()
        while anc and not anc.IsInstance():
            anc = anc.GetParent()
        if not anc:
            raise RuntimeError(f"cannot de-instance {proxies[0].GetPath()}")
        anc.SetInstanceable(False)
    old = _colliders(link)
    for prim in old:
        UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    path = f"{link.GetPath()}/scoop_collider"
    if kind == "voxels":
        UsdGeom.Xform.Define(stage, path)
        centres = shell_voxels(tris_tool, voxel_m)
        for k, c in enumerate(centres):
            cube = UsdGeom.Cube.Define(stage, f"{path}/v{k}")
            cube.CreateSizeAttr(float(voxel_m) * 1.05)
            cube.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in c]))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return f"{path} ({len(centres)} cubes of {voxel_m * 1000:.1f} mm, {len(old)} imported colliders off)"
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = np.asarray(tris_tool, dtype=np.float32).reshape(-1, 3)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(pts) // 3, 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(pts), dtype=np.int32)))
    mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(kind)
    if kind == "sdf":
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim).CreateSdfResolutionAttr(int(resolution))
    return f"{path} ({kind}, {len(old)} imported colliders off)"


def _colliders(prim) -> list:
    return [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI)]


class ParticlePowder:
    def __init__(
        self,
        stage,
        accounting: ParticleAccounting,
        seed_base: np.ndarray,
        pcfg: dict,
        fill_depth_m: float,
        reseed=None,
    ) -> None:
        """``seed_base``: particle centres (base frame). ``reseed(fill_depth_m)``
        returns new centres for ``reset(fill_depth_m)``."""
        self.accounting = accounting
        self.cfg = pcfg
        self.fill_depth_m = fill_depth_m
        self.reseed = reseed
        self.spacing = float(pcfg["spacing_m"])
        self.readback_period = 1.0 / float(pcfg.get("readback_hz", 10.0))
        self.kick = float(pcfg.get("vibration_kick_m_s", 0.15))
        self.rng = np.random.default_rng(0)
        self.totals = ParticleTotals()
        self.labels = np.zeros(0, dtype=np.int8)
        self._since_read = np.inf

        # Seeded on an ``s`` lattice; a rest radius under s/2 leaves clearance,
        # so the bed does not start compressed and blow grains through the floor.
        s = self.spacing
        rest = self.radius = float(pcfg.get("rest_ratio", 0.45)) * s
        system_path = f"{ROOT}/system"
        particleUtils.add_physx_particle_system(
            stage,
            Sdf.Path(system_path),
            contact_offset=rest + 0.1 * s,
            rest_offset=rest,
            particle_contact_offset=rest + 0.1 * s,
            solid_rest_offset=rest,
            fluid_rest_offset=0.6 * rest,
            enable_ccd=bool(pcfg.get("ccd", True)),
            solver_position_iterations=int(pcfg.get("solver_iterations", 16)),
            max_velocity=float(pcfg.get("max_velocity_m_s", 1.0)),
            max_depenetration_velocity=float(pcfg.get("max_depenetration_m_s", 0.2)),
            max_neighborhood=96,
        )
        mat_path = f"{ROOT}/physics_material"
        UsdShade.Material.Define(stage, mat_path)
        particleUtils.add_pbd_particle_material(
            stage,
            Sdf.Path(mat_path),
            friction=float(pcfg.get("friction", 0.6)),
            particle_friction_scale=float(pcfg.get("particle_friction_scale", 1.0)),
            damping=float(pcfg.get("damping", 0.1)),
            cohesion=float(pcfg.get("cohesion", 0.02)),
            adhesion=float(pcfg.get("adhesion", 0.0)),
        )
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(system_path), Sdf.Path(mat_path))

        n = len(seed_base)
        self.points = particleUtils.add_physx_particleset_points(
            stage,
            Sdf.Path(f"{ROOT}/grains"),
            Vt.Vec3fArray.FromNumpy(np.asarray(seed_base, dtype=np.float32)),
            Vt.Vec3fArray.FromNumpy(np.zeros((n, 3), dtype=np.float32)),
            Vt.FloatArray.FromNumpy(np.full(n, s, dtype=np.float32)),
            Sdf.Path(system_path),
            self_collision=True,
            fluid=False,
            particle_group=0,
            particle_mass=float(pcfg.get("particle_mass_kg", s ** 3 * 550.0)),
            density=0.0,
        )
        look = UsdShade.Material.Define(stage, f"{ROOT}/look")
        shader = UsdShade.Shader.Define(stage, f"{ROOT}/look/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*[float(c) for c in pcfg.get("color_rgb", (0.8, 0.08, 0.06))])
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(self.points.GetPrim()).Bind(look)
        self.count = n

    @property
    def bed_g(self) -> float:
        return self.totals.bed_g

    @property
    def payload_g(self) -> float:
        return self.totals.payload_g

    @property
    def rs3_g(self) -> float:
        return self.totals.rs3_g

    @property
    def table_g(self) -> float:
        return self.totals.table_g

    def positions(self) -> np.ndarray:
        return np.asarray(self.points.GetPointsAttr().Get(), dtype=np.float64).reshape(-1, 3)

    def reset(self, fill_depth_m: float | None = None) -> None:
        if fill_depth_m is not None:
            self.fill_depth_m = fill_depth_m
        if self.reseed is None:
            return
        pts = np.asarray(self.reseed(self.fill_depth_m), dtype=np.float32)
        n = len(pts)
        self.points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts))
        self.points.GetVelocitiesAttr().Set(Vt.Vec3fArray.FromNumpy(np.zeros((n, 3), dtype=np.float32)))
        self.points.GetWidthsAttr().Set(Vt.FloatArray.FromNumpy(np.full(n, self.spacing, dtype=np.float32)))
        self.count = n
        self._since_read = np.inf

    def step(self, dt: float, base_to_tcp: np.ndarray, vibration: float = 0.0) -> None:
        self._since_read += dt
        vibrating = vibration > 0 and self.kick > 0
        if not vibrating and self._since_read < self.readback_period:
            return
        self._since_read = 0.0
        pos = self.positions()
        bad = ~np.isfinite(pos).all(axis=1)
        if bad.any():
            print(f"[isaac_twin] {int(bad.sum())} particles have non-finite positions", flush=True)
            pos = np.where(bad[:, None], -10.0, pos)
        self.labels = self.accounting.classify(pos, base_to_tcp)
        self.totals = self.accounting.totals(self.labels)
        if vibrating:
            self._shake(vibration)

    def _shake(self, vibration: float) -> None:
        """Vibration shakes the grains in the scoop loose (random velocity kick)."""
        held = self.labels == SCOOP
        if not held.any():
            return
        vel = np.array(self.points.GetVelocitiesAttr().Get(), dtype=np.float32).reshape(-1, 3)
        if len(vel) != len(held):
            return
        vel[held] += self.rng.normal(0.0, self.kick * vibration, size=(int(held.sum()), 3)).astype(np.float32)
        self.points.GetVelocitiesAttr().Set(Vt.Vec3fArray.FromNumpy(vel))
