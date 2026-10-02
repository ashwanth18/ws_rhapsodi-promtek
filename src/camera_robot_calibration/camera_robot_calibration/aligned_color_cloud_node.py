#!/usr/bin/env python3
"""Point cloud from aligned depth + color intrinsics (hand-eye camera frame)."""

from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

from camera_robot_calibration.aligned_cloud import unproject_aligned, xyzrgb_to_cloud


class AlignedColorCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("aligned_color_cloud")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("cloud_topic", "/camera/color/points")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("stride", 2)
        self.declare_parameter("z_min", 0.12)
        self.declare_parameter("z_max", 1.6)

        self._bridge = CvBridge()
        self._k: np.ndarray | None = None
        self._color_bgr: np.ndarray | None = None
        self._color_rgb = False
        self._frame = str(self.get_parameter("camera_frame").value)

        # SensorDataQoS so MoveIt PointCloudOctomapUpdater (best-effort) matches.
        self._pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            self._on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Aligned color cloud: "
            f"{self.get_parameter('depth_topic').value} → "
            f"{self.get_parameter('cloud_topic').value} ({self._frame})"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self._k = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_color(self, msg: Image) -> None:
        enc = (msg.encoding or "").lower()
        try:
            if enc in ("rgb8", "rgb8; compressed"):
                self._color_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                self._color_rgb = True
            else:
                self._color_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self._color_rgb = False
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"color convert failed: {exc}", throttle_duration_sec=5.0)

    def _on_depth(self, msg: Image) -> None:
        if self._k is None or self._color_bgr is None:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"depth convert failed: {exc}", throttle_duration_sec=5.0)
            return
        if depth.ndim != 2:
            return
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) * 0.001
        else:
            depth_m = depth.astype(np.float32)

        color = self._color_bgr
        if color.shape[0] != depth_m.shape[0] or color.shape[1] != depth_m.shape[1]:
            self.get_logger().warning(
                f"color {color.shape[:2]} != depth {depth_m.shape} (need aligned depth)",
                throttle_duration_sec=5.0,
            )
            return

        stride = int(self.get_parameter("stride").value)
        xyz, valid = unproject_aligned(
            depth_m,
            float(self._k[0, 0]),
            float(self._k[1, 1]),
            float(self._k[0, 2]),
            float(self._k[1, 2]),
            stride=stride,
            z_min=float(self.get_parameter("z_min").value),
            z_max=float(self.get_parameter("z_max").value),
        )
        if xyz.shape[0] == 0:
            return
        pix = color[::stride, ::stride][valid]
        if self._color_rgb:
            rgb = pix
        else:
            rgb = pix[:, ::-1]
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self._frame
        self._pub.publish(xyzrgb_to_cloud(header, xyz, rgb))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignedColorCloudNode()
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
