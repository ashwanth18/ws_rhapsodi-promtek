#!/usr/bin/env python3
"""Isaac Sim twin of the Niryo scooping cell. Run with Isaac's ``python.sh``.

Use ``src/isaac_twin/scripts/run_isaac_twin.sh`` (it expands the URDF and sets
up the ROS domain). Builds, from the same files the ROS stack reads:

* Niryo Ned3 Pro + scoop from the URDF, position-driven over
  ``/isaac_joint_states`` / ``/isaac_joint_commands``, and ``/clock``;
* table, RS6 and RS3 from ``config/layouts/<layout_id>.yaml``;
* the D455 at the hand-eye calib pose (depth + colour);
* a powder bed in RS6 that the scoop carves, and the mass that lands in RS3.

Powder ROS interface (Isaac's bundled rclpy):
  /isaac_twin/rs3_mass_g, /isaac_twin/payload_g, /isaac_twin/bed_mass_g (Float64)
  /isaac_twin/reset_powder (std_srvs/Trigger)
  /vibration/intensity (Float64, subscribed)
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[3]
for _pkg in ("isaac_twin", "scoop_vision"):
    sys.path.insert(0, str(_SRC / _pkg))


def _args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", required=True, help="Expanded Niryo URDF (absolute mesh paths)")
    ap.add_argument("--config", default=str(_SRC / "isaac_twin" / "config" / "twin.yaml"))
    ap.add_argument("--layout-id", default="")
    ap.add_argument("--layouts-dir", default=str(_SRC.parent / "config" / "layouts"))
    ap.add_argument("--calibration", default="", help="Calib name (default from twin.yaml)")
    ap.add_argument("--fill-depth", type=float, default=None, help="Powder depth above the RS6 floor (m)")
    ap.add_argument(
        "--powder", choices=("particles", "heightfield"), default="", help="Powder model (default: twin.yaml powder.model)"
    )
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--max-seconds", type=float, default=0.0, help="Quit after this much sim time (0 = run)")
    ap.add_argument("--dump-particles", default="", help="Save particle positions and the TCP pose (base frame, .npz) on exit")
    ap.add_argument(
        "--replay-scoop",
        type=float,
        default=0.0,
        help="Tuning without ROS: after this many seconds, run the authored scoop + shake-off directly",
    )
    return ap.parse_args()


ARGS = _args()

from isaacsim import SimulationApp  # noqa: E402

APP = SimulationApp({"headless": ARGS.headless, "width": 1280, "height": 720})

import carb  # noqa: E402

# No DLSS: it renders the 848x480 depth product at half resolution.
carb.settings.get_settings().set("/rtx/post/aa/op", 0)

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
APP.update()

import rclpy  # noqa: E402  (Isaac's Jazzy rclpy, available once the bridge is on)
from std_msgs.msg import Float64  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from isaac_twin import cell  # noqa: E402
from isaac_twin.kinematics import Chain  # noqa: E402
from isaac_twin.particle_powder import ParticleAccounting, bed_depth_m, particle_grams, seed_bed  # noqa: E402
from isaac_twin.powder import PowderCell, ScoopParams  # noqa: E402
from isaac_twin.scene.d455 import add_d455  # noqa: E402
from isaac_twin.scene.particles import (  # noqa: E402
    ParticlePowder,
    add_static_collider,
    configure_physics_scenes,
    disable_link_gravity,
    filter_robot_from,
    scoop_collider,
)
from isaac_twin.scoop_path import (  # noqa: E402
    cartesian_path,
    chained_ik,
    joint_path,
    load_scoop_poses,
    pose_matrix,
    sample_knots,
    timed_scoop,
)
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from scoop_vision.transforms import apply_mtc_shape  # noqa: E402
from isaac_twin.scene.joint_bridge import create_joint_bridge  # noqa: E402
from scoop_vision.container import ContainerModel  # noqa: E402
from scoop_vision.mesh import load_stl  # noqa: E402
from scoop_vision.scoop_tool import ScoopTool  # noqa: E402

CELL_ROOT = "/World/Cell"
# Room floor under the table, so depth beyond the table is not empty.
FLOOR_BELOW_BASE_M = 0.75


def _log(msg: str) -> None:
    print(f"[isaac_twin] {msg}", flush=True)


def _transform(points: np.ndarray, t: np.ndarray) -> np.ndarray:
    return points @ t[:3, :3].T + t[:3, 3]


def _set_matrix(prim, t: np.ndarray) -> None:
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(np.asarray(t, dtype=float).T.tolist()))


def _material(stage, path: str, rgb, roughness: float = 0.6):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(c) for c in rgb]))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _bind(prim, mat) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def _tri_mesh(stage, path: str, tris: np.ndarray):
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = tris.reshape(-1, 3)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(tris), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(pts), dtype=np.int32)))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    return mesh


def build_layout(stage, layout_doc: dict, vessel_collider: str = ""):
    """Table/RS6/RS3 prims. Visual only unless ``vessel_collider`` (particle powder);
    the robot is filtered from them either way, since it is position driven
    and MTC lets the scoop touch the task vessel."""
    UsdGeom.Xform.Define(stage, CELL_ROOT)
    objects = {}
    for obj in cell.layout_objects(layout_doc):
        xf = UsdGeom.Xform.Define(stage, f"{CELL_ROOT}/{obj.id}")
        _set_matrix(xf.GetPrim(), obj.pose)
        mat = _material(stage, f"{CELL_ROOT}/Looks/{obj.id}", obj.color)
        if obj.geometry_type == "mesh":
            tris = load_stl(cell.resolve_uri(obj.mesh_resource)) * obj.scale
            prim = _tri_mesh(stage, f"{CELL_ROOT}/{obj.id}/mesh", tris).GetPrim()
            objects[obj.id] = (obj, tris)
        else:
            cube = UsdGeom.Cube.Define(stage, f"{CELL_ROOT}/{obj.id}/box")
            cube.CreateSizeAttr(1.0)
            UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in obj.dimensions]))
            prim = cube.GetPrim()
            objects[obj.id] = (obj, None)
        _bind(prim, mat)
        if vessel_collider:
            add_static_collider(prim, vessel_collider)
        _log(f"layout object {obj.id} ({obj.geometry_type}) at {np.round(obj.pose[:3, 3], 4).tolist()}")
    floor = UsdGeom.Cube.Define(stage, f"{CELL_ROOT}/floor")
    floor.CreateSizeAttr(1.0)
    floor_xf = UsdGeom.Xformable(floor)
    floor_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -FLOOR_BELOW_BASE_M - 0.005))
    floor_xf.AddScaleOp().Set(Gf.Vec3f(6.0, 6.0, 0.01))
    _bind(floor.GetPrim(), _material(stage, f"{CELL_ROOT}/Looks/floor", (0.35, 0.35, 0.37)))
    if vessel_collider:
        add_static_collider(floor.GetPrim())
    return objects


def import_robot(urdf_path: str, robot_cfg: dict) -> str:
    _, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = False
    cfg.fix_base = True
    cfg.make_default_prim = False
    cfg.self_collision = False
    cfg.create_physics_scene = True
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    ok, root = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=urdf_path, import_config=cfg, get_articulation_root=True
    )
    if not ok or not root:
        raise RuntimeError(f"URDF import failed for {urdf_path}")
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(float(robot_cfg["stiffness"]))
            drive.CreateDampingAttr(float(robot_cfg["damping"]))
            drive.CreateMaxForceAttr(float(robot_cfg["max_force"]))
            n += 1
    _log(f"robot articulation root {root}; tuned {n} joint drives")
    return root


def set_gravity(stage, enabled: bool) -> None:
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            UsdPhysics.Scene(prim).CreateGravityMagnitudeAttr(9.81 if enabled else 0.0)


def add_lights(stage) -> None:
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(800.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 20.0, 0.0))


class PowderMesh:
    """USD mesh of the bed; topology fixed, z updated when the bed changes."""

    def __init__(self, stage, path: str, bed, color) -> None:
        self.bed = bed
        self.cells, counts, idx = bed.mesh()
        self.mesh = UsdGeom.Mesh.Define(stage, path)
        self.mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(counts))
        self.mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(idx))
        self.mesh.CreateSubdivisionSchemeAttr("none")
        self.mesh.CreateDoubleSidedAttr(True)
        self._points = self.mesh.CreatePointsAttr()
        _bind(self.mesh.GetPrim(), _material(stage, f"{path}_look", color, roughness=0.95))
        self.version = -1
        self.update()

    def update(self) -> None:
        if self.bed.version == self.version:
            return
        self.version = self.bed.version
        self._points.Set(Vt.Vec3fArray.FromNumpy(self.bed.mesh_points(self.cells).astype(np.float32)))


class CapacityWorker(threading.Thread):
    """Scoop capacity (~0.3 s each) off the render thread; PowderCell waits for it."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.powder = None
        self._gravity = None
        self._backlog: list[np.ndarray] = []
        self._event = threading.Event()

    def request(self, gravity_tcp: np.ndarray) -> None:
        self._gravity = np.array(gravity_tcp, dtype=float)
        self._event.set()

    def prewarm(self, gravities: list[np.ndarray]) -> None:
        """Compute these when idle, so a fast pass through them does not hold the payload."""
        self._backlog.extend(np.array(g, dtype=float) for g in gravities)
        self._event.set()

    def run(self) -> None:
        while True:
            if not self._backlog:
                self._event.wait()
            self._event.clear()
            gravity, self._gravity = self._gravity, None
            if gravity is None and self._backlog:
                gravity = self._backlog.pop(0)
            if gravity is not None and self.powder is not None:
                self.powder.bowl.compute(gravity)


def _scoop_path_poses(poses_yaml: Path, base_to_container: np.ndarray, chain: Chain) -> list[np.ndarray]:
    """base_link -> tcp along the authored scoop (straight-line and joint-space).
    scoop_vision shifts only translate it, so these cover every planned scoop's tilts."""
    if not poses_yaml.is_file():
        return []
    poses = load_scoop_poses(poses_yaml)
    out = [base_to_container @ tcp for tcp, _seg in cartesian_path(poses)]
    joints = chained_ik(chain, [base_to_container @ pose_matrix(p) for p in poses])
    if joints is not None:
        out += [tcp for tcp, _seg in joint_path(chain, joints)]
    return out


class PowderRos:
    def __init__(self, powder: PowderCell) -> None:
        self.powder = powder
        self.vibration = 0.0
        self.node = rclpy.create_node("isaac_twin_sim")
        self.pubs = {
            name: self.node.create_publisher(Float64, f"/isaac_twin/{name}", 10)
            for name in ("rs3_mass_g", "payload_g", "bed_mass_g")
        }
        self.node.create_subscription(Float64, "/vibration/intensity", self._on_vibration, 10)
        self.node.create_service(Trigger, "/isaac_twin/reset_powder", self._on_reset)
        self._reset_requested = False

    def _on_vibration(self, msg) -> None:
        self.vibration = float(np.clip(msg.data, 0.0, 1.0))

    def _on_reset(self, _req, resp):
        self._reset_requested = True
        resp.success = True
        resp.message = f"powder reset to {self.powder.fill_depth_m * 1000:.0f} mm"
        return resp

    def take_reset(self) -> bool:
        requested, self._reset_requested = self._reset_requested, False
        return requested

    def publish(self) -> None:
        p = self.powder
        for name, value in (("rs3_mass_g", p.rs3_g), ("payload_g", p.payload_g), ("bed_mass_g", p.bed_g)):
            self.pubs[name].publish(Float64(data=float(value)))


def main() -> None:
    with open(ARGS.config, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    layout_id = ARGS.layout_id or cfg["layout_id"]
    layout_path = Path(ARGS.layouts_dir) / f"{layout_id}.yaml"
    layout_doc = cell.load_layout(layout_path)
    calib_path = cell.find_calibration(ARGS.calibration or cfg["calibration"])
    render_hz = float(cfg["render_hz"])
    rendering_dt = 1.0 / render_hz
    pcfg = cfg["powder"]
    model = ARGS.powder or pcfg.get("model", "heightfield")
    particles = model == "particles"
    physics_hz = float(pcfg["particles"].get("physics_hz", cfg["physics_hz"]) if particles else cfg["physics_hz"])
    _log(f"layout {layout_path}, calibration {calib_path}, powder model {model}, physics {physics_hz:.0f} Hz")

    sim = SimulationContext(stage_units_in_meters=1.0, physics_dt=1.0 / physics_hz, rendering_dt=rendering_dt)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    add_lights(stage)

    vessel_collider = str(pcfg["particles"].get("vessel_collider", "convexDecomposition")) if particles else ""
    objects = build_layout(stage, layout_doc, vessel_collider=vessel_collider)
    root = import_robot(ARGS.urdf, cfg["robot"])
    tool_cfg = cell.robot_tool_config("niryo")
    tool_tris = load_stl(cell.resolve_uri(tool_cfg["mesh_resource"])) * 0.001
    tool = ScoopTool(tool_tris, tool_cfg["tcp_visual_offset_xyz"])
    if particles:
        scenes = configure_physics_scenes(stage, gravity=True, gpu=True)
        robot_top = "/" + root.strip("/").split("/")[0]
        n_links = disable_link_gravity(stage, robot_top)
        n_filtered = filter_robot_from(stage, robot_top, CELL_ROOT)
        part = pcfg["particles"]
        scoop = scoop_collider(
            stage,
            robot_top,
            "tool_link",
            tool_tris,
            kind=str(part.get("scoop_collider", "voxels")),
            resolution=int(part.get("scoop_sdf_resolution", 256)),
            voxel_m=float(part.get("scoop_voxel_m", 0.003)),
        )
        _log(
            f"particles: gravity + GPU dynamics on {scenes}, gravity off on {n_links} robot links, "
            f"robot filtered from {n_filtered} cell colliders, scoop {scoop}"
        )
    else:
        set_gravity(stage, bool(cfg["robot"].get("gravity", False)))

    base_to_link = cell.camera_link_in_base(cell.load_calibration(calib_path))
    add_d455(stage, base_to_link, cfg["camera"], render_hz)
    _log(f"D455 camera_link at {np.round(base_to_link[:3, 3], 4).tolist()}")
    create_joint_bridge(root)

    # Powder in the task container, poured into the first other mesh container.
    task_id = str(layout_doc.get("task_container_id") or "rs6")
    task_obj, task_tris = objects[task_id]
    rs6_model = ContainerModel(task_tris)
    pour_ids = [k for k, (_, tris) in objects.items() if tris is not None and k != task_id]
    rs3_model = rs3_pose = None
    if pour_ids:
        rs3_obj, rs3_tris = objects[pour_ids[0]]
        rs3_model, rs3_pose = ContainerModel(rs3_tris), rs3_obj.pose
        _log(f"pour target {pour_ids[0]}")

    with open(ARGS.urdf, encoding="utf-8") as fh:
        urdf_xml = fh.read()
    chain = Chain(urdf_xml, "base_link", "tcp_link")

    fill_depth = float(ARGS.fill_depth if ARGS.fill_depth is not None else pcfg["fill_depth_m"])
    bed_mesh = None
    if particles:
        part = pcfg["particles"]
        spacing = float(part["spacing_m"])
        settle = float(part.get("settle_ratio", 1.0))
        accounting = ParticleAccounting(
            rs6_model,
            task_obj.pose,
            rs3_model,
            rs3_pose,
            tool.tris_tcp,
            particle_grams(spacing, float(pcfg["density_g_per_ml"]), settle),
        )

        def reseed(depth: float) -> np.ndarray:
            pts = seed_bed(rs6_model, depth / settle, spacing, jitter=float(part.get("jitter", 0.04)))
            return _transform(pts, task_obj.pose)

        powder = ParticlePowder(stage, accounting, reseed(fill_depth), part, fill_depth, reseed=reseed)
        _log(
            f"particle bed {powder.count} grains x {accounting.particle_g * 1000:.1f} mg = "
            f"{powder.count * accounting.particle_g:.0f} g ({fill_depth * 1000:.0f} mm settled, "
            f"seeded {fill_depth / settle * 1000:.0f} mm, {spacing * 1000:.1f} mm grains)"
        )
    else:
        worker = CapacityWorker()
        powder = PowderCell(
            rs6_model,
            task_obj.pose,
            rs3_model,
            rs3_pose,
            tool,
            fill_depth_m=fill_depth,
            params=ScoopParams.from_twin_config(cfg),
            on_capacity_miss=worker.request,
            capacity_cache_dir=Path.home() / ".cache" / "isaac_twin",
        )
        worker.powder = powder
        worker.start()
        warm = _scoop_path_poses(Path(ARGS.layouts_dir) / layout_id / "poses.yaml", task_obj.pose, chain)
        gravities = [g for pose in warm for g in powder.capacity_gravities(pose)]
        worker.prewarm(list({powder.bowl.key(g): g for g in gravities if powder.bowl.cached(g) is None}.values()))
        bed_mesh = PowderMesh(stage, f"{CELL_ROOT}/{task_id}/powder", powder.bed, pcfg["color_rgb"])
        _log(
            f"powder bed {powder.bed_g:.0f} g ({powder.fill_depth_m * 1000:.0f} mm), "
            f"interior cells {powder.bed.mask.sum()}"
        )

    rclpy.init()
    ros = PowderRos(powder)

    APP.update()
    sim.initialize_physics()
    sim.play()
    robot = SingleArticulation(root)
    robot.initialize()
    dof_names = list(robot.dof_names)
    _log(f"articulation dofs {dof_names}; publishing on ROS_DOMAIN_ID from env")

    replay = None
    if ARGS.replay_scoop:
        poses = apply_mtc_shape(load_scoop_poses(Path(ARGS.layouts_dir) / layout_id / "poses.yaml"))
        ik = chained_ik(chain, [task_obj.pose @ pose_matrix(p) for p in poses])
        if ik is None:
            raise RuntimeError("authored scoop is not reachable by IK")
        q_now = dict(zip(dof_names, robot.get_joint_positions().tolist()))
        replay = timed_scoop(chain, ik, np.array([q_now[n] for n in chain.joint_names]))
        _log(f"replaying the authored scoop at t={ARGS.replay_scoop:.0f}s for {replay[-1][0]:.1f}s")

    mesh_period = 1.0 / float(pcfg["mesh_update_hz"])
    last_mesh = last_pub = last_status = last_replay_log = 0.0
    wall_start = last_status_wall = time.monotonic()
    sim_t = 0.0
    while APP.is_running():
        sim.step(render=True)
        if not sim.is_playing():
            continue
        sim_t += rendering_dt
        if ros.take_reset():
            powder.reset()
            _log("powder reset")
        q = dict(zip(dof_names, robot.get_joint_positions().tolist()))
        vibration = ros.vibration
        if replay is not None and sim_t >= ARGS.replay_scoop:
            target, vibration = sample_knots(replay, sim_t - ARGS.replay_scoop)
            by_name = dict(zip(chain.joint_names, target))
            robot.apply_action(ArticulationAction(joint_positions=np.array([by_name.get(n, q[n]) for n in dof_names])))
            if sim_t - last_replay_log >= 1.0:
                err = max(abs(by_name[n] - q[n]) for n in chain.joint_names if n in q)
                tcp_z = chain.tip_pose(q)[2, 3]
                _log(
                    f"replay t={sim_t - ARGS.replay_scoop:.1f}s vib={vibration:.2f} track_err={err:.3f}rad tcp_z={tcp_z:.3f} "
                    f"bed={powder.bed_g:.0f}g scoop={powder.payload_g:.1f}g rs3={powder.rs3_g:.1f}g table={powder.table_g:.1f}g"
                )
                last_replay_log = sim_t
        powder.step(rendering_dt, chain.tip_pose(q), vibration)
        if bed_mesh is not None and sim_t - last_mesh >= mesh_period:
            bed_mesh.update()
            last_mesh = sim_t
        if sim_t - last_pub >= 0.1:
            ros.publish()
            last_pub = sim_t
        if sim_t - last_status >= 10.0:
            wall = time.monotonic()
            depth = ""
            if particles:
                surface = bed_depth_m(rs6_model, task_obj.pose, powder.positions()) + powder.radius
                depth = f" depth={surface * 1000:.0f}mm"
            _log(
                f"t={sim_t:.0f}s rtf={10.0 / max(wall - last_status_wall, 1e-6):.2f} bed={powder.bed_g:.0f}g "
                f"scoop={powder.payload_g:.1f}g rs3={powder.rs3_g:.1f}g table={powder.table_g:.1f}g{depth}"
            )
            last_status, last_status_wall = sim_t, wall
        rclpy.spin_once(ros.node, timeout_sec=0.0)
        # Hold sim time to wall time so ros2_control and the BT see real rates;
        # after a slow frame (first render, shader compile) do not sprint.
        ahead = sim_t - (time.monotonic() - wall_start)
        if ahead > 0:
            time.sleep(ahead)
        elif ahead < -0.5:
            wall_start = time.monotonic() - sim_t
        if ARGS.max_seconds and sim_t >= ARGS.max_seconds:
            break

    if ARGS.dump_particles and particles:
        q = dict(zip(dof_names, robot.get_joint_positions().tolist()))
        np.savez(ARGS.dump_particles, positions=powder.positions(), tcp=chain.tip_pose(q))
        _log(f"particles saved to {ARGS.dump_particles}")
    ros.node.destroy_node()
    rclpy.shutdown()
    sim.stop()
    APP.close()


if __name__ == "__main__":
    main()
