#!/usr/bin/env python3
"""Powder height map + next-scoop planner for the table-mounted D455.

Services (node name ``scoop_vision``):

* ``~/capture`` (Trigger): fuse depth frames into a powder height map.
  Refuses while the arm is between the camera and the container. Call it
  while the arm is at the weighing container.
* ``~/plan`` (robot_common_msgs/PlanScoop): best shift of the authored
  scoop for the current height map. The map is marked stale as soon as the
  TCP enters the container, so every scoop needs a fresh capture.
* ``~/check_container_alignment`` (Trigger): compare the container rim seen
  by the camera with the active layout.
* ``~/fit_container_pose`` (Trigger): camera calibration of the task
  container. Fits X/Y/Z/yaw from the rim and writes a layout *proposal*
  (plus re-anchored poses) under ``proposal_dir``; nothing is applied.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import threading
import time
from dataclasses import fields

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.msg import ParameterType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robot_common_msgs.msg import CellLayoutActive
from robot_common_msgs.srv import ApplyCellLayout, PlanScoop
from scoop_vision.alignment import check_alignment, fit_container_offset
from scoop_vision.container import ContainerModel
from scoop_vision.heightmap import SurfaceMap, build_surface, unproject_depth
from scoop_vision.layout import resolve_scene_path, task_container_mesh, write_layout_proposal
from scoop_vision.mesh import load_stl, resolve_resource
from scoop_vision.planner import PlannerParams, ScoopPlanner
from scoop_vision.scoop_tool import ScoopTool
from scoop_vision.transforms import Pose, apply_mtc_shape, interpolate_path, quat_to_matrix

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def _tf_matrix(tf) -> tuple[np.ndarray, np.ndarray]:
    r = tf.transform.rotation
    t = tf.transform.translation
    return quat_to_matrix((r.x, r.y, r.z, r.w)), np.array([t.x, t.y, t.z])


def _depth_to_meters(msg: Image, scale: float) -> np.ndarray:
    if msg.encoding in ("16UC1", "mono16"):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
        return raw[:, : msg.width].astype(np.float32) * scale
    if msg.encoding == "32FC1":
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
        return raw[:, : msg.width].copy()
    raise ValueError(f"Unsupported depth encoding {msg.encoding}")


class ScoopVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("scoop_vision")

        self.declare_parameter("container_frame", "scooping_container_frame")
        self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("stride", 2)
        self.declare_parameter("capture_frames", 8)
        self.declare_parameter("capture_timeout_s", 6.0)
        self.declare_parameter("min_measured_fraction", 0.5)
        self.declare_parameter("max_heightmap_age_s", 900.0)
        self.declare_parameter("robots_yaml", "")
        self.declare_parameter("robot_key", "niryo")
        self.declare_parameter("tool_mesh", "")
        self.declare_parameter("tool_mesh_scale", 0.001)
        self.declare_parameter("tcp_offset_xyz", [0.0, 0.0, 0.0])
        # Used only until /cell_layout/active arrives (bench / no layout manager).
        self.declare_parameter("container_mesh", "")
        self.declare_parameter("container_mesh_scale", 0.001)
        # Local copy of config/layouts when /cell_layout/active points at a
        # path that only exists inside the ros-prod container (/ws/config/...).
        self.declare_parameter("layouts_dir", "")
        self.declare_parameter("mtc_node", "/scooping_mtc_node")
        self.declare_parameter("eef_frame", "tcp_link")
        # Kinematic chain, base → tip, for the camera-occlusion test.
        self.declare_parameter(
            "occlusion_frames",
            ["elbow_link", "forearm_link", "wrist_link", "hand_link", "tool_link", "tcp_link"],
        )
        self.declare_parameter("occlusion_radius_m", 0.07)
        # Every capture also checks the bin rim against the layout; a height
        # map from a misplaced bin would put walls inside the "powder".
        self.declare_parameter("require_alignment", True)
        self.declare_parameter("alignment_tolerance_xy_m", 0.01)
        self.declare_parameter("alignment_tolerance_z_m", 0.01)
        self.declare_parameter("proposal_dir", "~/.ros/scoop_vision/proposals")
        # Walls are grown sideways by this much before clearance checks, on top
        # of planner.wall_clearance_m: covers bin-pose / hand-eye error.
        self.declare_parameter("wall_margin_xy_m", 0.012)
        # Only return shifts whose 5 poses all have an IK solution (MoveIt
        # /compute_ik); a high bed can lift the approach out of reach.
        self.declare_parameter("check_reachability", True)
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("ik_group", "arm")
        self.declare_parameter("ik_timeout_s", 0.1)
        for f in fields(PlannerParams):
            self.declare_parameter(f"planner.{f.name}", f.default)

        self._lock = threading.Lock()
        self._frames_lock = threading.Lock()
        self._collect: list[tuple[Image, CameraInfo]] | None = None
        self._collect_n = 0
        self._info: CameraInfo | None = None
        self._poses: PoseArray | None = None
        self._layout: CellLayoutActive | None = None
        self._container: ContainerModel | None = None
        self._container_key: tuple | None = None
        self._tool: ScoopTool | None = None
        self._planner: ScoopPlanner | None = None
        self._planner_key: tuple | None = None
        self._surface: SurfaceMap | None = None
        self._surface_stamp = 0.0
        self._stale_reason = "no capture yet"
        self._layout_hash = ""
        self._last_proposal: dict | None = None
        self._anim: list[tuple[np.ndarray, np.ndarray]] = []
        self._anim_i = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        sub_group = MutuallyExclusiveCallbackGroup()
        srv_group = ReentrantCallbackGroup()
        self._client_group = MutuallyExclusiveCallbackGroup()
        self._client_group_apply = MutuallyExclusiveCallbackGroup()
        self._client_group_ik = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value), self._on_depth,
            qos_profile_sensor_data, callback_group=sub_group,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("depth_info_topic").value), self._on_info,
            qos_profile_sensor_data, callback_group=sub_group,
        )
        self.create_subscription(
            PoseArray, "/scoop_poses", self._on_poses,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=sub_group,
        )
        self.create_subscription(
            CellLayoutActive, "/cell_layout/active", self._on_layout, _LATCHED,
            callback_group=sub_group,
        )

        self._surface_pub = self.create_publisher(PointCloud2, "~/surface", _LATCHED)
        self._marker_pub = self.create_publisher(MarkerArray, "~/plan_markers", _LATCHED)
        self._plan_pub = self.create_publisher(String, "~/last_plan", _LATCHED)
        self._stats_pub = self.create_publisher(String, "~/surface_stats", _LATCHED)
        self._fit_pub = self.create_publisher(Marker, "~/fitted_container", _LATCHED)
        # Animated scoop: volatile, so it never replaces the latched plan markers.
        self._motion_pub = self.create_publisher(Marker, "~/scoop_motion", 10)

        self.create_service(Trigger, "~/capture", self._srv_capture, callback_group=srv_group)
        self.create_service(PlanScoop, "~/plan", self._srv_plan, callback_group=srv_group)
        self.create_service(
            Trigger, "~/check_container_alignment", self._srv_alignment, callback_group=srv_group,
        )
        self.create_service(
            Trigger, "~/fit_container_pose", self._srv_fit_container, callback_group=srv_group,
        )
        self.create_service(
            Trigger, "~/apply_container_fit", self._srv_apply_fit, callback_group=srv_group,
        )
        self._apply_layout = self.create_client(
            ApplyCellLayout, "/cell_layout/apply", callback_group=self._client_group_apply
        )
        self.create_timer(1.0 / 15.0, self._animate, callback_group=sub_group)
        self._ik = self.create_client(
            GetPositionIK, str(self.get_parameter("ik_service").value),
            callback_group=self._client_group_ik,
        )
        self.create_timer(0.2, self._watch_eef, callback_group=sub_group)

        self._mtc_params = AsyncParameterClient(
            self, str(self.get_parameter("mtc_node").value), callback_group=self._client_group,
        )

        self._load_tool()
        fallback = str(self.get_parameter("container_mesh").value).strip()
        if fallback:
            self._set_container(fallback, float(self.get_parameter("container_mesh_scale").value), "param")
        self.get_logger().info("scoop_vision ready (capture with the arm out of the camera view, e.g. CameraClear)")

    # ------------------------------------------------------------------ setup

    def _load_tool(self) -> None:
        mesh = str(self.get_parameter("tool_mesh").value).strip()
        offset = list(self.get_parameter("tcp_offset_xyz").value)
        if not mesh:
            path = str(self.get_parameter("robots_yaml").value).strip()
            if not path:
                path = resolve_resource("package://scooping_controller/config/robots.yaml")
            robots = yaml.safe_load(open(path, encoding="utf-8"))["robots"]
            tool = robots[str(self.get_parameter("robot_key").value)]["tool"]
            mesh = tool["mesh_resource"]
            offset = tool["tcp_visual_offset_xyz"]
        scale = float(self.get_parameter("tool_mesh_scale").value)
        tris = load_stl(resolve_resource(mesh)) * scale
        self._tool = ScoopTool(tris, offset)
        self._tool_mesh_uri = mesh if "://" in mesh else f"file://{os.path.abspath(mesh)}"
        self._tool_scale = scale
        self._tcp_offset = np.asarray(offset, dtype=np.float64)
        self.get_logger().info(f"Scoop tool {mesh}, tcp offset {offset}")

    def _set_container(self, mesh: str, scale: float, source: str) -> None:
        key = (mesh, scale)
        if key == self._container_key:
            return
        tris = load_stl(resolve_resource(mesh)) * scale
        model = ContainerModel(
            tris, wall_margin_xy=float(self.get_parameter("wall_margin_xy_m").value)
        )
        with self._lock:
            self._container = model
            self._container_key = key
            self._planner = None
            self._surface = None
            self._stale_reason = "no capture for this container yet"
        self.get_logger().info(
            f"Container from {source}: {os.path.basename(mesh)} floor {model.floor_z:.3f} m, "
            f"rim {model.rim_z:.3f} m, interior "
            f"{(model.interior_max - model.interior_min).round(3).tolist()} m"
        )

    def _on_layout(self, msg: CellLayoutActive) -> None:
        self._layout = msg
        if msg.layout_hash != self._layout_hash:
            # Same bin mesh but a new pose still moves every wall and the
            # task frame: drop the old height map and plan.
            with self._lock:
                self._layout_hash = msg.layout_hash
                self._surface = None
                self._planner = None
                self._stale_reason = (
                    f"layout {msg.layout_id} ({msg.layout_hash}) applied since the last capture"
                    if self._surface_stamp
                    else "no capture yet"
                )
            self._clear_plan_markers()
        try:
            path = resolve_scene_path(
                msg.scene_yaml_path, str(self.get_parameter("layouts_dir").value).strip()
            )
            mesh, scale = task_container_mesh(path, msg.task_container_id)
            self._set_container(mesh, scale, f"layout {msg.layout_id} ({msg.task_container_id})")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Could not load task container from layout: {exc}")

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _on_poses(self, msg: PoseArray) -> None:
        self._poses = msg

    def _on_depth(self, msg: Image) -> None:
        with self._frames_lock:
            if self._collect is not None and len(self._collect) < self._collect_n and self._info:
                self._collect.append((msg, self._info))

    # --------------------------------------------------------------- geometry

    def _lookup(self, target: str, source: str, timeout: float = 0.5):
        return self._tf_buffer.lookup_transform(
            target, source, Time(), timeout=Duration(seconds=timeout)
        )

    def _occlusion(self, info: CameraInfo, container: ContainerModel) -> str | None:
        """Name of an arm frame blocking the camera's view, else None."""
        cam = info.header.frame_id
        cframe = str(self.get_parameter("container_frame").value)
        rot, t = _tf_matrix(self._lookup(cam, cframe))
        corners = container.interior_corners() @ rot.T + t
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        u = fx * corners[:, 0] / corners[:, 2] + cx
        v = fy * corners[:, 1] / corners[:, 2] + cy
        box = (u.min(), u.max(), v.min(), v.max())
        far = float(corners[:, 2].max())
        radius = float(self.get_parameter("occlusion_radius_m").value)

        chain = []
        for frame in self.get_parameter("occlusion_frames").value:
            tf = self._lookup(cam, frame)
            p = tf.transform.translation
            chain.append((frame, np.array([p.x, p.y, p.z])))
        samples = []
        for (fa, a), (_, b) in zip(chain, chain[1:]):
            for s in np.linspace(0.0, 1.0, 6):
                samples.append((fa, a + s * (b - a)))
        samples.append(chain[-1])
        for frame, p in samples:
            if p[2] <= 0.05 or p[2] >= far:
                continue
            pu = fx * p[0] / p[2] + cx
            pv = fy * p[1] / p[2] + cy
            pr = fx * radius / p[2]
            if box[0] - pr <= pu <= box[1] + pr and box[2] - pr <= pv <= box[3] + pr:
                return frame
        return None

    def _watch_eef(self) -> None:
        container = self._container
        if container is None or self._surface is None or self._stale_reason:
            return
        try:
            tf = self._lookup(
                str(self.get_parameter("container_frame").value),
                str(self.get_parameter("eef_frame").value),
                timeout=0.0,
            )
        except TransformException:
            return
        p = tf.transform.translation
        m = 0.02
        inside = (
            container.interior_min[0] - m <= p.x <= container.interior_max[0] + m
            and container.interior_min[1] - m <= p.y <= container.interior_max[1] + m
            and p.z < container.rim_z + 0.03
        )
        if inside:
            self._stale_reason = "scoop entered the container after the last capture"
            self._clear_plan_markers()
            self.get_logger().info("Height map marked stale: TCP entered the container")

    def _capture_points(self) -> tuple[list[np.ndarray], CameraInfo]:
        """Collect depth frames as container-frame point sets (arm must be clear)."""
        container = self._container
        if container is None:
            raise RuntimeError("No task container yet (waiting for /cell_layout/active)")
        info = self._info
        if info is None:
            raise RuntimeError("No depth camera_info received; is the D455 running?")
        blocker = self._occlusion(info, container)
        if blocker:
            raise RuntimeError(f"Arm ({blocker}) blocks the camera view of the container")

        n = int(self.get_parameter("capture_frames").value)
        with self._frames_lock:
            self._collect = []
            self._collect_n = n
        deadline = time.monotonic() + float(self.get_parameter("capture_timeout_s").value)
        while time.monotonic() < deadline:
            with self._frames_lock:
                if len(self._collect) >= n:
                    break
            time.sleep(0.02)
        with self._frames_lock:
            got = self._collect or []
            self._collect = None
        if not got:
            raise RuntimeError("No depth frames received within the capture timeout")

        info = got[-1][1]
        blocker = self._occlusion(info, container)
        if blocker:
            raise RuntimeError(f"Arm ({blocker}) moved into view during capture")

        cframe = str(self.get_parameter("container_frame").value)
        rot, t = _tf_matrix(self._lookup(cframe, info.header.frame_id))
        scale = float(self.get_parameter("depth_scale").value)
        stride = int(self.get_parameter("stride").value)
        frames = []
        for img, inf in got:
            depth = _depth_to_meters(img, scale)
            pts = unproject_depth(depth, inf.k[0], inf.k[4], inf.k[2], inf.k[5], stride=stride)
            frames.append(pts @ rot.T + t)
        return frames, info

    # --------------------------------------------------------------- services

    def _check_alignment(self, frames: list[np.ndarray]):
        return check_alignment(
            np.vstack(frames),
            self._container,
            tolerance_xy_m=float(self.get_parameter("alignment_tolerance_xy_m").value),
            tolerance_z_m=float(self.get_parameter("alignment_tolerance_z_m").value),
        )

    def _srv_capture(self, _req, resp: Trigger.Response) -> Trigger.Response:
        try:
            frames, _ = self._capture_points()
            container = self._container
            if bool(self.get_parameter("require_alignment").value):
                alignment = self._check_alignment(frames)
                if not alignment.ok:
                    raise RuntimeError(
                        f"{alignment.message}. Fix the task container pose in the layout "
                        "(or set require_alignment:=false at your own risk)"
                    )
            surface = build_surface(frames, container)
            need = float(self.get_parameter("min_measured_fraction").value)
            if surface.measured_fraction < need:
                raise RuntimeError(
                    f"Only {surface.measured_fraction:.0%} of the container has depth "
                    f"(need {need:.0%}); check lighting / exposure"
                )
            with self._lock:
                self._surface = surface
                self._surface_stamp = time.time()
                self._stale_reason = ""
            stats = surface.stats(container)
            self._publish_surface(surface, container)
            self._stats_pub.publish(String(data=json.dumps(dict(stats, stamp=self._surface_stamp))))
            resp.success = True
            resp.message = (
                f"Captured {surface.frames} frames: {surface.measured_fraction:.0%} measured, "
                f"surface {stats['surface_min_m']:.3f}–{stats['surface_max_m']:.3f} m, "
                f"~{stats['powder_volume_m3'] * 1e6:.0f} ml powder"
            )
        except (RuntimeError, ValueError, TransformException) as exc:
            with self._lock:
                self._surface = None
                self._stale_reason = f"last capture failed: {exc}"
            resp.success = False
            resp.message = f"capture failed: {exc}"
        self.get_logger().info(resp.message)
        return resp

    def _planner_params(self) -> PlannerParams:
        return PlannerParams(**{
            f.name: type(f.default)(self.get_parameter(f"planner.{f.name}").value)
            for f in fields(PlannerParams)
        })

    def _mtc_shape(self) -> tuple[float, float, float]:
        names = ["manual_sweep_scale", "manual_pitch_offset_rad", "manual_lift_offset_z"]
        if not self._mtc_params.wait_for_services(timeout_sec=2.0):
            raise RuntimeError(f"{self.get_parameter('mtc_node').value} parameters unavailable")
        future = self._mtc_params.get_parameters(names)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("Timed out reading scoop shape parameters from the MTC node")
        # Undeclared parameters come back NOT_SET; keep the MTC defaults then.
        defaults = (1.0, 0.0, 0.0)
        return tuple(  # type: ignore[return-value]
            float(v.double_value) if v.type == ParameterType.PARAMETER_DOUBLE else d
            for v, d in zip(future.result().values, defaults)
        )

    def _get_planner(self) -> ScoopPlanner:
        poses_msg = self._poses
        if poses_msg is None or len(poses_msg.poses) != 5:
            raise RuntimeError("Need the 5 authored poses on /scoop_poses")
        cframe = str(self.get_parameter("container_frame").value)
        if poses_msg.header.frame_id and poses_msg.header.frame_id != cframe:
            raise RuntimeError(f"/scoop_poses is in {poses_msg.header.frame_id}, expected {cframe}")
        sweep, pitch, lift = self._mtc_shape()
        params = self._planner_params()
        poses = [
            Pose(
                (p.position.x, p.position.y, p.position.z),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            )
            for p in poses_msg.poses
        ]
        key = (tuple(poses), sweep, pitch, lift, self._container_key)
        if self._planner is None or self._planner_key != key:
            shaped = apply_mtc_shape(poses, sweep_scale=sweep, pitch_offset_rad=pitch, lift_offset_z=lift)
            self._planner = ScoopPlanner(self._container, self._tool, shaped, params)
            self._planner_key = key
            self.get_logger().info(
                f"Planner rebuilt: bowl {self._planner.capacity_m3 * 1e6:.0f} ml, authored path "
                f"clearance walls {self._planner.authored_wall_clearance_m * 1000:.0f} mm "
                f"(+{self._container.wall_margin_xy * 1000:.0f} mm sideways margin), floor "
                f"{self._planner.authored_floor_clearance_m * 1000:.0f} mm"
            )
        self._planner.params = params
        return self._planner

    def _srv_plan(self, req: PlanScoop.Request, resp: PlanScoop.Response) -> PlanScoop.Response:
        try:
            if self._container is None:
                raise RuntimeError("No task container yet (waiting for /cell_layout/active)")
            surface = self._surface
            if surface is None or self._stale_reason:
                raise RuntimeError(f"No fresh height map ({self._stale_reason}); call capture first")
            age = time.time() - self._surface_stamp
            max_age = req.max_heightmap_age_s or float(self.get_parameter("max_heightmap_age_s").value)
            resp.heightmap_age_s = age
            if age > max_age:
                raise RuntimeError(f"Height map is {age:.0f} s old (max {max_age:.0f} s)")
            planner = self._get_planner()
            reachable = None
            if bool(self.get_parameter("check_reachability").value):
                if not self._ik.wait_for_service(timeout_sec=2.0):
                    raise RuntimeError(
                        f"{self._ik.srv_name} unavailable (needed for check_reachability)"
                    )
                reachable = self._reachable
            plan = planner.plan(surface, reachable=reachable)
        except (RuntimeError, ValueError) as exc:
            resp.success = False
            resp.message = f"plan failed: {exc}"
            self.get_logger().warning(resp.message)
            return resp

        resp.success = plan.success
        resp.message = plan.message
        resp.pattern_offset_x = plan.offset_x
        resp.pattern_offset_y = plan.offset_y
        resp.pattern_offset_z = plan.offset_z
        resp.predicted_fill_ratio = plan.predicted_fill_ratio
        resp.predicted_volume_m3 = plan.predicted_volume_m3
        resp.capacity_m3 = plan.capacity_m3
        resp.max_penetration_m = plan.max_penetration_m
        resp.surface_height_m = plan.surface_height_m
        resp.min_clearance_m = plan.min_clearance_m
        resp.container_empty = plan.container_empty

        record = dict(
            plan.to_dict(),
            stamp=time.time(),
            heightmap_stamp=self._surface_stamp,
            heightmap_age_s=resp.heightmap_age_s,
            layout_id=self._layout.layout_id if self._layout else "",
            params=planner.params.__dict__,
            surface_stats=surface.stats(self._container),
        )
        self._plan_pub.publish(String(data=json.dumps(record)))
        if plan.success:
            self._publish_plan_markers(plan, planner)
        else:
            self._clear_plan_markers()
        self.get_logger().info(resp.message)
        return resp

    _POSE_NAMES = ("approach", "contact", "scoop", "lift", "transport_ready")

    def _reachable(self, poses: list[Pose]) -> tuple[bool, str]:
        """IK (no collision) for every shifted pose, seeded from the current state."""
        frame = str(self.get_parameter("container_frame").value)
        timeout = float(self.get_parameter("ik_timeout_s").value)
        for name, pose in zip(self._POSE_NAMES, poses):
            req = GetPositionIK.Request()
            req.ik_request.group_name = str(self.get_parameter("ik_group").value)
            req.ik_request.ik_link_name = str(self.get_parameter("eef_frame").value)
            req.ik_request.robot_state.is_diff = True
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = int(timeout)
            req.ik_request.timeout.nanosec = int((timeout % 1.0) * 1e9)
            target = PoseStamped()
            target.header.frame_id = frame
            target.pose.position = Point(x=pose.position[0], y=pose.position[1], z=pose.position[2])
            (target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w) = (float(v) for v in pose.orientation)
            req.ik_request.pose_stamped = target
            future = self._ik.call_async(req)
            deadline = time.monotonic() + timeout + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.005)
            result = future.result() if future.done() else None
            if result is None:
                return False, f"{name} IK timed out"
            if result.error_code.val != 1:  # MoveItErrorCodes.SUCCESS
                return False, f"{name} out of reach"
        return True, ""

    def _srv_alignment(self, _req, resp: Trigger.Response) -> Trigger.Response:
        try:
            frames, _ = self._capture_points()
            result = self._check_alignment(frames)
            resp.success = result.ok
            resp.message = result.message
        except (RuntimeError, ValueError, TransformException) as exc:
            resp.success = False
            resp.message = f"alignment check failed: {exc}"
        self.get_logger().info(resp.message)
        return resp

    def _srv_fit_container(self, _req, resp: Trigger.Response) -> Trigger.Response:
        try:
            layout = self._layout
            if layout is None or self._container is None:
                raise RuntimeError("No /cell_layout/active yet")
            frames, _ = self._capture_points()
            fit = fit_container_offset(
                np.vstack(frames),
                self._container,
                tolerance_xy_m=float(self.get_parameter("alignment_tolerance_xy_m").value),
                tolerance_z_m=float(self.get_parameter("alignment_tolerance_z_m").value),
            )
            if fit.rim_cells_seen == 0 or fit.at_search_limit:
                raise RuntimeError(fit.message)
            self._publish_fitted_container(fit)
            if fit.ok:
                resp.success = True
                resp.message = f"{fit.message}. No layout change needed."
                return resp
            path = resolve_scene_path(
                layout.scene_yaml_path, str(self.get_parameter("layouts_dir").value).strip()
            )
            poses = None
            if self._poses is not None and len(self._poses.poses) == 5:
                poses = [
                    (
                        (p.position.x, p.position.y, p.position.z),
                        (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
                    )
                    for p in self._poses.poses
                ]
            out_dir = os.path.join(
                os.path.expanduser(str(self.get_parameter("proposal_dir").value)),
                f"{time.strftime('%Y%m%dT%H%M%S')}_{layout.layout_id}",
            )
            out = write_layout_proposal(
                path, layout.task_container_id, fit, out_dir, current_poses=poses,
                poses_meta={"layout_id": layout.layout_id, "tool_id": layout.tool_id},
            )
            self._last_proposal = dict(out, target=path, layout_id=layout.layout_id)
            pos = ", ".join(f"{v:.4f}" for v in out["position_xyz"])
            resp.success = True
            resp.message = (
                f"{fit.message}. Proposed {layout.task_container_id} position_xyz [{pos}], "
                f"yaw {out['yaw_deg']:.2f} deg → {out['layout']}"
                + (" (+ poses_keep_world_path.yaml)" if poses else "")
                + ". NOT applied: check the orange bin in RViz, then apply_container_fit."
            )
        except Exception as exc:  # noqa: BLE001 - always answer the caller
            resp.success = False
            resp.message = f"container fit failed: {exc}"
        self.get_logger().info(resp.message)
        return resp

    # ---------------------------------------------------------------- visuals

    def _publish_fitted_container(self, fit) -> None:
        """Task-container mesh where the camera found it (orange), for RViz."""
        if self._container_key is None:
            return
        mesh, scale = self._container_key
        m = Marker(ns="scoop_vision_fit", id=0, type=Marker.MESH_RESOURCE, action=Marker.ADD)
        m.header.frame_id = str(self.get_parameter("container_frame").value)
        m.header.stamp = self.get_clock().now().to_msg()
        m.mesh_resource = mesh
        m.pose.position = Point(x=fit.shift_x_m, y=fit.shift_y_m, z=fit.z_offset_m)
        half = np.radians(fit.yaw_offset_deg) / 2.0
        m.pose.orientation.z = float(np.sin(half))
        m.pose.orientation.w = float(np.cos(half))
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.0, 0.5
        self._fit_pub.publish(m)


    def _publish_surface(self, surface: SurfaceMap, container: ContainerModel) -> None:
        xs, ys = container.grid.centers()
        m = container.interior
        pts = np.column_stack([xs[m], ys[m], surface.height[m]]).astype(np.float32)
        header = Header(frame_id=str(self.get_parameter("container_frame").value))
        header.stamp = self.get_clock().now().to_msg()
        self._surface_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))

    def _clear_plan_markers(self) -> None:
        self._anim = []
        frame = str(self.get_parameter("container_frame").value)
        clear_all = Marker(action=Marker.DELETEALL)
        clear_all.header.frame_id = frame
        self._marker_pub.publish(MarkerArray(markers=[clear_all]))
        motion = Marker(ns="scoop_motion", id=0, action=Marker.DELETE)
        motion.header.frame_id = frame
        self._motion_pub.publish(motion)

    def _scoop_marker(self, ns: int | str, mid: int, rot: np.ndarray, t: np.ndarray, rgba) -> Marker:
        """Scoop mesh with ``tcp_link`` at ``(rot, t)`` (container frame)."""
        m = Marker(ns=str(ns), id=mid, type=Marker.MESH_RESOURCE, action=Marker.ADD)
        m.header.frame_id = str(self.get_parameter("container_frame").value)
        m.header.stamp = self.get_clock().now().to_msg()
        m.mesh_resource = self._tool_mesh_uri
        tool = t - rot @ self._tcp_offset  # tool_link origin (identity visual origin)
        m.pose.position = Point(x=float(tool[0]), y=float(tool[1]), z=float(tool[2]))
        qx, qy, qz, qw = _matrix_to_quat(rot)
        m.pose.orientation.x, m.pose.orientation.y = qx, qy
        m.pose.orientation.z, m.pose.orientation.w = qz, qw
        m.scale.x = m.scale.y = m.scale.z = self._tool_scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _publish_plan_markers(self, plan, planner: ScoopPlanner) -> None:
        """Next scoop in RViz: TCP path, ghost scoop at each waypoint, and an
        animated scoop running the motion (ns ``scoop_motion``)."""
        frame = str(self.get_parameter("container_frame").value)
        stamp = self.get_clock().now().to_msg()
        d = np.array([plan.offset_x, plan.offset_y, plan.offset_z])
        poses = [
            Pose(tuple(np.asarray(p.position) + d), p.orientation) for p in planner.poses
        ]
        path = interpolate_path(poses, max_step_m=0.004, max_step_rad=0.03)
        clear_all = Marker(action=Marker.DELETEALL)
        clear_all.header.frame_id = frame
        markers = [clear_all]

        line = Marker(ns="scoop_path", id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
        line.header.frame_id = frame
        line.header.stamp = stamp
        line.pose.orientation.w = 1.0
        line.scale.x = 0.003
        line.color.g, line.color.b, line.color.a = 0.9, 0.4, 1.0
        line.points = [Point(x=float(t[0]), y=float(t[1]), z=float(t[2])) for _, t, _ in path]
        markers.append(line)

        names = ("approach", "contact", "scoop", "lift", "transport")
        for i, pose in enumerate(poses):
            rot = quat_to_matrix(pose.orientation)
            markers.append(
                self._scoop_marker("scoop_waypoints", i, rot, np.asarray(pose.position), (0.8, 0.9, 1.0, 0.25))
            )
            label = Marker(ns="scoop_waypoint_labels", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.header.frame_id = frame
            label.header.stamp = stamp
            label.pose.position = Point(
                x=float(pose.position[0]), y=float(pose.position[1]), z=float(pose.position[2]) + 0.03
            )
            label.pose.orientation.w = 1.0
            label.scale.z = 0.012
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = names[i]
            markers.append(label)

        pts = plan.swept_points[:: max(1, len(plan.swept_points) // 3000)]
        cloud = Marker(ns="scoop_swept_volume", id=0, type=Marker.POINTS, action=Marker.ADD)
        cloud.header.frame_id = frame
        cloud.header.stamp = stamp
        cloud.scale.x = cloud.scale.y = 0.003
        cloud.color.g, cloud.color.b, cloud.color.a = 0.8, 0.3, 0.25
        cloud.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in pts]
        cloud.pose.orientation.w = 1.0
        markers.append(cloud)

        text = Marker(ns="scoop_summary", id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.header.frame_id = frame
        text.header.stamp = stamp
        top = pts[np.argmax(pts[:, 2])]
        text.pose.position = Point(x=float(top[0]), y=float(top[1]), z=float(top[2]) + 0.06)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.02
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = (
            f"next scoop: fill {plan.predicted_fill_ratio:.0%}  depth {plan.max_penetration_m * 1000:.0f} mm\n"
            f"shift ({plan.offset_x * 1000:+.0f}, {plan.offset_y * 1000:+.0f}, {plan.offset_z * 1000:+.0f}) mm  "
            f"walls {plan.min_wall_clearance_m * 1000:.0f}+{planner.container.wall_margin_xy * 1000:.0f} mm"
        )
        markers.append(text)
        self._marker_pub.publish(MarkerArray(markers=markers))
        self._anim = [(rot, t) for rot, t, _ in path]
        self._anim_i = 0

    def _animate(self) -> None:
        anim = self._anim
        if not anim:
            return
        # Hold one second at the end, then loop.
        i = self._anim_i % (len(anim) + 15)
        self._anim_i += 1
        rot, t = anim[min(i, len(anim) - 1)]
        self._motion_pub.publish(self._scoop_marker("scoop_motion", 0, rot, t, (1.0, 0.55, 0.1, 0.9)))

    def _srv_apply_fit(self, _req, resp: Trigger.Response) -> Trigger.Response:
        """Copy the last fit_container_pose proposal over the layout and apply it."""
        try:
            prop = self._last_proposal
            if prop is None:
                raise RuntimeError("No container fit in this session; run fit_container_pose first")
            if self._layout is None or self._layout.layout_id != prop["layout_id"]:
                raise RuntimeError("Active layout changed since the fit; fit again")
            backup = os.path.join(
                os.path.dirname(prop["layout"]), os.path.basename(prop["target"]) + ".before"
            )
            shutil.copy2(prop["target"], backup)
            shutil.copy2(prop["layout"], prop["target"])
            if not self._apply_layout.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(
                    f"Wrote {prop['target']} but /cell_layout/apply is unavailable; relaunch the stack"
                )
            future = self._apply_layout.call_async(ApplyCellLayout.Request(layout_id=prop["layout_id"]))
            deadline = time.monotonic() + 60.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            result = future.result() if future.done() else None
            if result is None:
                raise RuntimeError(f"Wrote {prop['target']}; /cell_layout/apply timed out")
            if not result.success:
                raise RuntimeError(f"Wrote {prop['target']}; apply refused: {result.message}")
            self._last_proposal = None
            resp.success = True
            resp.message = (
                f"Applied: {prop['target']} (backup {backup}). {result.message}. "
                "Scoop poses follow the bin; capture again before planning."
            )
        except Exception as exc:  # noqa: BLE001 - always answer the caller
            resp.success = False
            resp.message = f"apply container fit failed: {exc}"
        self.get_logger().info(resp.message)
        return resp


def _matrix_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return float(x), float(y), float(z), float(w)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScoopVisionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
