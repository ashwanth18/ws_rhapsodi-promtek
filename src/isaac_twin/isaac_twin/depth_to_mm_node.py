#!/usr/bin/env python3
"""Isaac depth (32FC1, metres) -> D455-style depth (16UC1, mm) + noise.

scoop_vision reads ``/camera/depth/image_rect_raw`` with ``depth_scale``
0.001, as on the real cell, so it stays untouched. Noise and holes are added
so its multi-frame median and ``min_measured_fraction`` get exercised.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthToMm(Node):
    def __init__(self) -> None:
        super().__init__("depth_to_mm")
        p = self.declare_parameter
        p("input_depth_topic", "/isaac/camera/depth/image_raw")
        p("input_info_topic", "/isaac/camera/depth/camera_info")
        p("output_depth_topic", "/camera/depth/image_rect_raw")
        p("output_info_topic", "/camera/depth/camera_info")
        p("cloud_topic", "/camera/depth/color/points")
        p("min_range_m", 0.4)
        p("max_range_m", 6.0)
        p("noise_base_m", 0.0008)
        p("noise_quadratic", 0.002)
        p("hole_fraction", 0.01)
        p("seed", 0)
        p("publish_cloud", True)
        p("cloud_stride", 4)
        p("cloud_rate_hz", 2.0)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        seed = int(g("seed"))
        self._rng = np.random.default_rng(seed if seed else None)
        self._min = float(g("min_range_m"))
        self._max = float(g("max_range_m"))
        self._sigma0 = float(g("noise_base_m"))
        self._sigma2 = float(g("noise_quadratic"))
        self._holes = float(g("hole_fraction"))
        self._cloud = bool(g("publish_cloud"))
        self._stride = max(1, int(g("cloud_stride")))
        self._cloud_period = 1.0 / max(0.1, float(g("cloud_rate_hz")))
        self._last_cloud = None
        self._info: CameraInfo | None = None

        self._pub_depth = self.create_publisher(Image, str(g("output_depth_topic")), 5)
        self._pub_info = self.create_publisher(CameraInfo, str(g("output_info_topic")), 5)
        self._pub_cloud = self.create_publisher(PointCloud2, str(g("cloud_topic")), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(g("input_info_topic")), self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, str(g("input_depth_topic")), self._on_depth, qos_profile_sensor_data)

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding != "32FC1":
            self.get_logger().error(f"Expected 32FC1 depth, got {msg.encoding}", throttle_duration_sec=10.0)
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        z = rows[:, : msg.width * 4].copy().view(np.float32).reshape(msg.height, msg.width).astype(np.float64)

        valid = np.isfinite(z) & (z >= self._min) & (z <= self._max)
        zv = z[valid]
        z[valid] = zv + self._rng.normal(0.0, 1.0, zv.shape) * (self._sigma0 + self._sigma2 * zv * zv)
        if self._holes > 0:
            valid &= self._rng.random(z.shape) >= self._holes
        mm = np.where(valid, np.clip(np.rint(z * 1000.0), 0, 65535), 0).astype(np.uint16)

        out = Image()
        out.header = msg.header
        out.height, out.width = mm.shape
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.step = out.width * 2
        out.data = mm.tobytes()
        self._pub_depth.publish(out)
        if self._info is not None:
            info = self._info
            info.header = msg.header
            self._pub_info.publish(info)
            self._maybe_cloud(msg, mm, info)

    def _maybe_cloud(self, msg: Image, mm: np.ndarray, info: CameraInfo) -> None:
        if not self._cloud:
            return
        now = self.get_clock().now()
        if self._last_cloud is not None and (now - self._last_cloud).nanoseconds * 1e-9 < self._cloud_period:
            return
        self._last_cloud = now
        s = self._stride
        sub = mm[::s, ::s].astype(np.float32) * 0.001
        v, u = np.nonzero(sub > 0)
        z = sub[v, u]
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        x = (u * s - cx) * z / fx
        y = (v * s - cy) * z / fy
        pts = np.stack([x, y, z], axis=1).astype(np.float32)

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        self._pub_cloud.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthToMm()
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
