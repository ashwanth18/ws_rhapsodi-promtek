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
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--max-seconds", type=float, default=0.0, help="Quit after this much sim time (0 = run)")
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
from isaac_twin.powder import PowderCell, ScoopParams  # noqa: E402
from isaac_twin.scene.d455 import add_d455  # noqa: E402
from isaac_twin.scoop_path import cartesian_path, chained_ik, joint_path, load_scoop_poses, pose_matrix  # noqa: E402
from isaac_twin.scene.joint_bridge import create_joint_bridge  # noqa: E402
from scoop_vision.container import ContainerModel  # noqa: E402
from scoop_vision.mesh import load_stl  # noqa: E402
from scoop_vision.scoop_tool import ScoopTool  # noqa: E402

CELL_ROOT = "/World/Cell"
# Room floor under the table, so depth beyond the table is not empty.
FLOOR_BELOW_BASE_M = 0.75


def _log(msg: str) -> None:
    print(f"[isaac_twin] {msg}", flush=True)


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


def build_layout(stage, layout_doc: dict):
    """Table/RS6/RS3 as visual-only prims (no colliders: the arm is position
    driven and MTC lets the scoop touch the task vessel)."""
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
        _log(f"layout object {obj.id} ({obj.geometry_type}) at {np.round(obj.pose[:3, 3], 4).tolist()}")
    floor = UsdGeom.Cube.Define(stage, f"{CELL_ROOT}/floor")
    floor.CreateSizeAttr(1.0)
    floor_xf = UsdGeom.Xformable(floor)
    floor_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -FLOOR_BELOW_BASE_M - 0.005))
    floor_xf.AddScaleOp().Set(Gf.Vec3f(6.0, 6.0, 0.01))
    _bind(floor.GetPrim(), _material(stage, f"{CELL_ROOT}/Looks/floor", (0.35, 0.35, 0.37)))
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
    """Scoop capacity (~0.2 s each) off the render thread; PowderCell waits for it."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.powder = None
        self._pose = None
        self._backlog: list[np.ndarray] = []
        self._event = threading.Event()

    def request(self, base_to_tcp: np.ndarray) -> None:
        self._pose = base_to_tcp.copy()
        self._event.set()

    def prewarm(self, poses: list[np.ndarray]) -> None:
        """Compute these when idle, so a fast pass through them does not hold the payload."""
        self._backlog.extend(p.copy() for p in poses)
        self._event.set()

    def run(self) -> None:
        while True:
            if not self._backlog:
                self._event.wait()
            self._event.clear()
            pose, self._pose = self._pose, None
            if pose is None and self._backlog:
                pose = self._backlog.pop(0)
            if pose is not None and self.powder is not None:
                self.powder.capacity(pose)


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
    _log(f"layout {layout_path}, calibration {calib_path}")

    sim = SimulationContext(
        stage_units_in_meters=1.0, physics_dt=1.0 / float(cfg["physics_hz"]), rendering_dt=rendering_dt
    )
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    add_lights(stage)

    objects = build_layout(stage, layout_doc)
    root = import_robot(ARGS.urdf, cfg["robot"])
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
    tool_cfg = cell.robot_tool_config("niryo")
    tool = ScoopTool(load_stl(cell.resolve_uri(tool_cfg["mesh_resource"])) * 0.001, tool_cfg["tcp_visual_offset_xyz"])

    pcfg = cfg["powder"]
    worker = CapacityWorker()
    powder = PowderCell(
        rs6_model,
        task_obj.pose,
        rs3_model,
        rs3_pose,
        tool,
        fill_depth_m=float(ARGS.fill_depth if ARGS.fill_depth is not None else pcfg["fill_depth_m"]),
        params=ScoopParams.from_twin_config(cfg),
        on_capacity_miss=worker.request,
    )
    worker.powder = powder
    worker.start()
    warm = _scoop_path_poses(Path(ARGS.layouts_dir) / layout_id / "poses.yaml", task_obj.pose, chain)
    worker.prewarm(list({PowderCell._capacity_key(p): p for p in warm}.values()))
    bed_mesh = PowderMesh(stage, f"{CELL_ROOT}/{task_id}/powder", powder.bed, pcfg["color_rgb"])
    _log(f"powder bed {powder.bed_g:.0f} g ({powder.fill_depth_m * 1000:.0f} mm), interior cells {powder.bed.mask.sum()}")

    rclpy.init()
    ros = PowderRos(powder)

    APP.update()
    sim.initialize_physics()
    sim.play()
    robot = SingleArticulation(root)
    robot.initialize()
    dof_names = list(robot.dof_names)
    _log(f"articulation dofs {dof_names}; publishing on ROS_DOMAIN_ID from env")

    mesh_period = 1.0 / float(pcfg["mesh_update_hz"])
    last_mesh = last_pub = last_status = 0.0
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
        powder.step(rendering_dt, chain.tip_pose(q), ros.vibration)
        if sim_t - last_mesh >= mesh_period:
            bed_mesh.update()
            last_mesh = sim_t
        if sim_t - last_pub >= 0.1:
            ros.publish()
            last_pub = sim_t
        if sim_t - last_status >= 10.0:
            wall = time.monotonic()
            _log(
                f"t={sim_t:.0f}s rtf={10.0 / max(wall - last_status_wall, 1e-6):.2f} bed={powder.bed_g:.0f}g "
                f"scoop={powder.payload_g:.1f}g rs3={powder.rs3_g:.1f}g"
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

    ros.node.destroy_node()
    rclpy.shutdown()
    sim.stop()
    APP.close()


if __name__ == "__main__":
    main()
