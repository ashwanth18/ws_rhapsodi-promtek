#!/usr/bin/env python3
"""Static D455 frames under ``camera_link``, as realsense2_camera publishes them.

With these, ``handeye_camera_link_publisher`` composes the calib into
``base_link -> camera_link`` exactly as on the real cell.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from isaac_twin.cell import D455_COLOR_OFFSET_M, OPTICAL_FROM_BODY_XYZW


def _tf(parent: str, child: str, xyz, xyzw) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in xyz)
    r = t.transform.rotation
    r.x, r.y, r.z, r.w = (float(v) for v in xyzw)
    return t


class SimCameraTf(Node):
    def __init__(self) -> None:
        super().__init__("sim_camera_tf")
        self.declare_parameter("camera_name", "camera")
        name = str(self.get_parameter("camera_name").value)
        link = f"{name}_link"
        ident = (0.0, 0.0, 0.0, 1.0)
        transforms = [
            _tf(link, f"{name}_depth_frame", (0, 0, 0), ident),
            _tf(f"{name}_depth_frame", f"{name}_depth_optical_frame", (0, 0, 0), OPTICAL_FROM_BODY_XYZW),
            _tf(link, f"{name}_color_frame", D455_COLOR_OFFSET_M, ident),
            _tf(f"{name}_color_frame", f"{name}_color_optical_frame", (0, 0, 0), OPTICAL_FROM_BODY_XYZW),
        ]
        stamp = self.get_clock().now().to_msg()
        for t in transforms:
            t.header.stamp = stamp
        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Published D455 frames under {link}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimCameraTf()
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
