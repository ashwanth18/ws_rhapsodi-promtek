#!/usr/bin/env python3
"""Drive Niryo via /move_to and sample easy_handeye2 for eye-on-base calibration."""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from camera_robot_calibration.pose_sampling import SamplePose, generate_sample_poses, poses_to_stamped

try:
    from robot_common_msgs.action import MoveTo
except ImportError:  # pragma: no cover - workspace must provide msgs
    MoveTo = None  # type: ignore

try:
    from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, TakeSample
except ImportError:  # pragma: no cover
    TakeSample = None  # type: ignore
    ComputeCalibration = None  # type: ignore
    SaveCalibration = None  # type: ignore


class HandeyeMoveItSampler(Node):
    def __init__(self) -> None:
        super().__init__("handeye_moveit_sampler")

        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("eef_link", "tcp_link")
        self.declare_parameter("tracking_marker_frame", "charuco_board")
        self.declare_parameter("tracking_base_frame", "camera_color_optical_frame")
        self.declare_parameter("num_samples", 12)
        self.declare_parameter("rotation_deg", 25.0)
        self.declare_parameter("translation_m", 0.02)
        self.declare_parameter("settle_s", 1.5)
        self.declare_parameter("detect_timeout_s", 5.0)
        # Driver rosbridge often needs 15–30s; do not abort on the first miss.
        self.declare_parameter("tf_ready_timeout_s", 60.0)
        self.declare_parameter("velocity_scaling", 0.15)
        self.declare_parameter("acceleration_scaling", 0.15)
        self.declare_parameter("move_to_action", "/move_to")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("take_sample_service", "/easy_handeye2/calibration/take_sample")
        self.declare_parameter(
            "compute_calibration_service",
            "/easy_handeye2/calibration/compute_calibration",
        )
        self.declare_parameter(
            "save_calibration_service",
            "/easy_handeye2/calibration/save_calibration",
        )

        if MoveTo is None:
            raise RuntimeError("robot_common_msgs.action.MoveTo is not available")
        if TakeSample is None or ComputeCalibration is None or SaveCalibration is None:
            raise RuntimeError("easy_handeye2_msgs services are not available")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        action_name = str(self.get_parameter("move_to_action").value)
        self._move_client = ActionClient(self, MoveTo, action_name)
        self._take_sample = self.create_client(
            TakeSample, str(self.get_parameter("take_sample_service").value)
        )
        self._compute = self.create_client(
            ComputeCalibration,
            str(self.get_parameter("compute_calibration_service").value),
        )
        self._save = self.create_client(
            SaveCalibration, str(self.get_parameter("save_calibration_service").value)
        )

        self._running = False
        self._worker: Optional[threading.Thread] = None
        if bool(self.get_parameter("auto_start").value):
            self._start_timer = self.create_timer(2.0, self._kickoff)

    def _wait_future(self, future, timeout_s: float) -> bool:
        """Block a worker thread without spinning the node's executor."""
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.05)
        return bool(future.done())

    def _kickoff(self) -> None:
        if self._running:
            return
        self._running = True
        self._start_timer.cancel()
        self.get_logger().info("Starting hand-eye sampling sequence")
        # Timer callbacks already run inside rclpy.spin. Do the long work on a
        # thread so TF callbacks keep flowing (no nested spin_once).
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

    def _run_worker(self) -> None:
        try:
            ok = self.run_calibration()
            if ok:
                self.get_logger().info("Hand-eye calibration finished successfully")
            else:
                self.get_logger().error("Hand-eye calibration finished with errors")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Hand-eye calibration crashed: {exc}")

    def _lookup_eef_pose(self) -> Optional[PoseStamped]:
        base = str(self.get_parameter("robot_base_frame").value)
        eef = str(self.get_parameter("eef_link").value)
        timeout_s = float(self.get_parameter("tf_ready_timeout_s").value)
        deadline = time.monotonic() + timeout_s
        last_exc: Optional[str] = None
        self.get_logger().info(
            f"Waiting up to {timeout_s:.0f}s for TF {base} -> {eef}"
        )
        while time.monotonic() < deadline and rclpy.ok():
            try:
                tf = self._tf_buffer.lookup_transform(
                    base, eef, Time(), timeout=Duration(seconds=0.5)
                )
                pose = PoseStamped()
                pose.header.frame_id = base
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = tf.transform.translation.x
                pose.pose.position.y = tf.transform.translation.y
                pose.pose.position.z = tf.transform.translation.z
                pose.pose.orientation = tf.transform.rotation
                return pose
            except TransformException as exc:
                last_exc = str(exc)
                time.sleep(0.2)
        self.get_logger().error(
            f"TF {base} -> {eef} unavailable after {timeout_s:.0f}s: {last_exc}"
        )
        return None

    def _wait_for_marker_tf(self, timeout_s: float) -> bool:
        camera = str(self.get_parameter("tracking_base_frame").value)
        marker = str(self.get_parameter("tracking_marker_frame").value)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            try:
                self._tf_buffer.lookup_transform(
                    camera, marker, Time(), timeout=Duration(seconds=0.2)
                )
                return True
            except TransformException:
                time.sleep(0.1)
        return False

    def _move_to(self, target: PoseStamped) -> bool:
        dry_run = bool(self.get_parameter("dry_run").value)
        if dry_run:
            p = target.pose.position
            o = target.pose.orientation
            self.get_logger().info(
                f"[dry_run] would MoveTo "
                f"xyz=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
                f"quat=({o.x:.3f},{o.y:.3f},{o.z:.3f},{o.w:.3f})"
            )
            return True

        if not self._move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveTo action server not available")
            return False

        goal = MoveTo.Goal()
        goal.target_pose = target
        goal.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        goal.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        goal.use_cartesian = False

        send_future = self._move_client.send_goal_async(goal)
        if not self._wait_future(send_future, 30.0):
            self.get_logger().warning("MoveTo goal send timed out")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("MoveTo goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, 120.0):
            self.get_logger().warning("MoveTo result timed out")
            return False
        result = result_future.result()
        if result is None:
            self.get_logger().warning("MoveTo result missing")
            return False
        if not result.result.success:
            self.get_logger().warning(f"MoveTo failed: {result.result.message}")
            return False
        return True

    def _call_take_sample(self) -> bool:
        dry_run = bool(self.get_parameter("dry_run").value)
        if dry_run:
            self.get_logger().info("[dry_run] would call take_sample")
            return True
        if not self._take_sample.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("take_sample service unavailable")
            return False
        future = self._take_sample.call_async(TakeSample.Request())
        if not self._wait_future(future, 10.0):
            self.get_logger().error("take_sample timed out")
            return False
        resp = future.result()
        if resp is None:
            self.get_logger().error("take_sample returned no response")
            return False
        n = len(resp.samples.samples) if resp.samples is not None else 0
        self.get_logger().info(f"Sample accepted ({n} total)")
        return True

    def _compute_and_save(self) -> bool:
        dry_run = bool(self.get_parameter("dry_run").value)
        if dry_run:
            self.get_logger().info("[dry_run] would compute and save calibration")
            return True

        if not self._compute.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("compute_calibration service unavailable")
            return False
        future = self._compute.call_async(ComputeCalibration.Request())
        if not self._wait_future(future, 30.0):
            self.get_logger().error("compute_calibration timed out")
            return False
        resp = future.result()
        if resp is None or not resp.valid:
            self.get_logger().error("compute_calibration failed or invalid")
            return False
        self.get_logger().info("Calibration computed successfully")

        if not self._save.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("save_calibration service unavailable")
            return False
        save_future = self._save.call_async(SaveCalibration.Request())
        if not self._wait_future(save_future, 10.0):
            self.get_logger().error("save_calibration timed out")
            return False
        save_resp = save_future.result()
        if save_resp is None or not save_resp.success:
            self.get_logger().error("save_calibration failed")
            return False
        path = save_resp.filepath.data if save_resp.filepath else "?"
        self.get_logger().info(f"Calibration saved to {path}")
        return True

    def run_calibration(self) -> bool:
        seed = self._lookup_eef_pose()
        if seed is None:
            return False

        poses = generate_sample_poses(
            SamplePose(
                seed.pose.position.x,
                seed.pose.position.y,
                seed.pose.position.z,
                seed.pose.orientation.x,
                seed.pose.orientation.y,
                seed.pose.orientation.z,
                seed.pose.orientation.w,
            ),
            num_samples=int(self.get_parameter("num_samples").value),
            rotation_deg=float(self.get_parameter("rotation_deg").value),
            translation_m=float(self.get_parameter("translation_m").value),
            include_seed=True,
        )
        stamped = poses_to_stamped(
            poses, frame_id=str(self.get_parameter("robot_base_frame").value)
        )
        self.get_logger().info(f"Prepared {len(stamped)} sample poses")

        accepted = 0
        settle_s = float(self.get_parameter("settle_s").value)
        detect_timeout_s = float(self.get_parameter("detect_timeout_s").value)

        for idx, target in enumerate(stamped):
            self.get_logger().info(f"Sample pose {idx + 1}/{len(stamped)}")
            if not self._move_to(target):
                self.get_logger().warning(f"Skipping pose {idx + 1}: motion failed")
                continue

            # Settle so TF and vision stabilize
            end = time.monotonic() + settle_s
            while time.monotonic() < end and rclpy.ok():
                time.sleep(0.1)

            if not bool(self.get_parameter("dry_run").value):
                if not self._wait_for_marker_tf(detect_timeout_s):
                    self.get_logger().warning(
                        f"Skipping pose {idx + 1}: ChArUco TF not visible"
                    )
                    continue

            if not self._call_take_sample():
                self.get_logger().warning(f"Skipping pose {idx + 1}: take_sample failed")
                continue
            accepted += 1

        self.get_logger().info(f"Accepted {accepted}/{len(stamped)} samples")
        if accepted < 3 and not bool(self.get_parameter("dry_run").value):
            self.get_logger().error("Need at least 3 samples to compute calibration")
            return False
        return self._compute_and_save()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandeyeMoveItSampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
