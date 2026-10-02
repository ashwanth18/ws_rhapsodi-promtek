#!/usr/bin/env python3
"""RViz Publish Point → /move_to (plan first, then execute if reachable)."""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

try:
    from robot_common_msgs.action import MoveTo
except ImportError:  # pragma: no cover
    MoveTo = None  # type: ignore


class ClickMoveNode(Node):
    def __init__(self) -> None:
        super().__init__("click_move")

        if MoveTo is None:
            raise RuntimeError("robot_common_msgs.action.MoveTo is not available")

        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eef_link", "tcp_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("move_to_action", "/move_to")
        self.declare_parameter("velocity_scaling", 0.10)
        self.declare_parameter("acceleration_scaling", 0.10)
        self.declare_parameter("min_range_m", 0.05)
        self.declare_parameter("max_range_m", 0.85)
        self.declare_parameter("execute", True)
        self.declare_parameter("marker_topic", "/click_move/marker")

        self._base = str(self.get_parameter("base_frame").value)
        self._eef = str(self.get_parameter("eef_link").value)
        self._camera = str(self.get_parameter("camera_frame").value)
        self._execute = bool(self.get_parameter("execute").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._move = ActionClient(
            self, MoveTo, str(self.get_parameter("move_to_action").value)
        )
        self._marker_pub = self.create_publisher(
            Marker, str(self.get_parameter("marker_topic").value), 10
        )
        self._busy = threading.Lock()
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("clicked_point_topic").value),
            self._on_click,
            10,
        )
        self.get_logger().info(
            "Click-move ready: RViz Publish Point → plan → "
            f"{'execute' if self._execute else 'plan only'}"
        )

    def _wait_future(self, future, timeout_s: float) -> bool:
        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        while not future.done() and rclpy.ok():
            if self.get_clock().now() > deadline:
                return False
            threading.Event().wait(0.05)
        return bool(future.done())

    def _on_click(self, msg: PointStamped) -> None:
        if not self._busy.acquire(blocking=False):
            self.get_logger().warning("Ignoring click: previous move still running")
            return
        threading.Thread(target=self._handle_click, args=(msg,), daemon=True).start()

    def _publish_marker(self, frame: str, x: float, y: float, z: float, ok: bool) -> None:
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "click_move"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.03
        m.color.a = 0.9
        if ok:
            m.color.g = 0.85
        else:
            m.color.r = 0.9
        self._marker_pub.publish(m)

    def _transform_point(self, msg: PointStamped, dest: str) -> Optional[PointStamped]:
        src = msg.header.frame_id or dest
        try:
            tf = self._tf_buffer.lookup_transform(
                dest, src, Time(), timeout=Duration(seconds=1.0)
            )
        except TransformException as exc:
            self.get_logger().error(f"TF {dest} <- {src}: {exc}")
            return None
        out = do_transform_point(msg, tf)
        out.header.frame_id = dest
        return out

    def _lookup_eef(self) -> Optional[PoseStamped]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base, self._eef, Time(), timeout=Duration(seconds=1.0)
            )
        except TransformException as exc:
            self.get_logger().error(f"TF {self._base} -> {self._eef}: {exc}")
            return None
        pose = PoseStamped()
        pose.header.frame_id = self._base
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _send_move(self, target: PoseStamped, *, plan_only: bool) -> tuple[bool, str]:
        if not self._move.wait_for_server(timeout_sec=8.0):
            return False, "MoveTo server not available"
        goal = MoveTo.Goal()
        goal.target_pose = target
        goal.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        goal.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        goal.use_cartesian = False
        goal.plan_only = plan_only
        send = self._move.send_goal_async(goal)
        if not self._wait_future(send, 20.0):
            return False, "goal send timed out"
        handle = send.result()
        if handle is None or not handle.accepted:
            return False, "goal rejected"
        result_fut = handle.get_result_async()
        if not self._wait_future(result_fut, 90.0):
            return False, "result timed out"
        packed = result_fut.result()
        if packed is None:
            return False, "empty result"
        return bool(packed.result.success), str(packed.result.message)

    def _handle_click(self, msg: PointStamped) -> None:
        try:
            in_base = self._transform_point(msg, self._base)
            if in_base is None:
                return
            x, y, z = in_base.point.x, in_base.point.y, in_base.point.z
            if any(not math.isfinite(v) for v in (x, y, z)):
                self.get_logger().warning("Click has NaN/inf; ignored")
                return

            in_cam = self._transform_point(in_base, self._camera)
            if in_cam is not None:
                cx, cy, cz = in_cam.point.x, in_cam.point.y, in_cam.point.z
                cam_range = math.sqrt(cx * cx + cy * cy + cz * cz)
                min_r = float(self.get_parameter("min_range_m").value)
                max_r = float(self.get_parameter("max_range_m").value)
                if cam_range < min_r or cam_range > max_r:
                    self.get_logger().warning(
                        f"Click range {cam_range:.3f}m from camera outside "
                        f"[{min_r}, {max_r}]; ignored"
                    )
                    self._publish_marker(self._base, x, y, z, False)
                    return

            eef = self._lookup_eef()
            if eef is None:
                self._publish_marker(self._base, x, y, z, False)
                return

            target = PoseStamped()
            target.header.frame_id = self._base
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x = x
            target.pose.position.y = y
            target.pose.position.z = z
            target.pose.orientation = eef.pose.orientation

            self.get_logger().info(
                f"Click base xyz=({x:.3f},{y:.3f},{z:.3f}); planning…"
            )
            ok, msg_txt = self._send_move(target, plan_only=True)
            if not ok:
                self.get_logger().warning(f"Unreachable: {msg_txt}")
                self._publish_marker(self._base, x, y, z, False)
                return
            self.get_logger().info(f"Plan ok: {msg_txt}")
            self._publish_marker(self._base, x, y, z, True)
            if not self._execute:
                return
            ok, msg_txt = self._send_move(target, plan_only=False)
            if ok:
                self.get_logger().info(f"Moved: {msg_txt}")
            else:
                self.get_logger().error(f"Execute failed: {msg_txt}")
                self._publish_marker(self._base, x, y, z, False)
        finally:
            self._busy.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClickMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
