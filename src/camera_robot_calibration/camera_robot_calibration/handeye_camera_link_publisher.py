#!/usr/bin/env python3
"""Publish eye-on-base calib as ``base → camera_link`` (keeps RealSense TF intact)."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from camera_robot_calibration.tf_compose import base_to_camera_link
from easy_handeye2.handeye_calibration import load_calibration


class HandeyeCameraLinkPublisher(Node):
    def __init__(self) -> None:
        super().__init__("handeye_camera_link_publisher")
        self.declare_parameter("name", "")
        self.declare_parameter("camera_link_frame", "camera_link")

        name = str(self.get_parameter("name").value).strip()
        if not name:
            raise RuntimeError("parameter 'name' is required")

        self._camera_link = str(self.get_parameter("camera_link_frame").value).strip()
        self._calibration = load_calibration(name)
        params = self._calibration.parameters
        if params.calibration_type != "eye_on_base":
            raise RuntimeError(
                f"Only eye_on_base is supported (got {params.calibration_type})"
            )
        self._parent = params.robot_base_frame
        self._optical = params.tracking_base_frame

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._broadcaster = StaticTransformBroadcaster(self)
        self._published = False
        self._timer = self.create_timer(0.5, self._try_publish)
        self.get_logger().info(
            f"Waiting for {self._camera_link} → {self._optical} so calib "
            f"{self._parent} → {self._optical} can be published as "
            f"{self._parent} → {self._camera_link}"
        )

    def _try_publish(self) -> None:
        if self._published:
            return
        try:
            tf_lo = self._tf_buffer.lookup_transform(
                self._camera_link,
                self._optical,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Still waiting for {self._camera_link} → {self._optical}: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._parent
        out.child_frame_id = self._camera_link
        out.transform = base_to_camera_link(
            self._calibration.transform, tf_lo.transform
        )
        self._broadcaster.sendTransform(out)
        self._published = True
        t = out.transform.translation
        self.get_logger().info(
            f"Published static {self._parent} → {self._camera_link} "
            f"[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}] m "
            f"(composed with {self._camera_link} → {self._optical})"
        )
        self._timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandeyeCameraLinkPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
