import json
import os
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

try:
    from cv_bridge import CvBridge  # type: ignore
except Exception:  # pragma: no cover
    CvBridge = None  # type: ignore

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore

try:
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover
    np = None  # type: ignore

try:
    from sensor_msgs_py import point_cloud2  # type: ignore
except Exception:  # pragma: no cover
    point_cloud2 = None  # type: ignore


@dataclass
class SyncedFrame:
    image: Image
    cloud: Optional[PointCloud2]
    depth: Optional[Image]
    depth_info: Optional[CameraInfo]
    seq: int


class RealSenseSnapshotAnalyzer(Node):
    """
    Subscribes to an Image + PointCloud2 pair (ideally aligned/organized) and
    provides a Trigger service:
      - waits for the next synced pair
      - computes depth stats from pointcloud z (meters)
      - optionally saves the image to disk
    """

    def __init__(self) -> None:
        super().__init__("realsense_snapshot_analyzer")

        self._sub_cb_group = ReentrantCallbackGroup()
        self._srv_cb_group = ReentrantCallbackGroup()

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "pointcloud_topic", "/camera/depth/color/points"
        )
        self.declare_parameter(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("depth_camera_info_topic", "")
        self.declare_parameter("use_sensor_qos", True)
        # Max allowed stamp difference when pairing image<->cloud (seconds).
        # Kept as sync_slop_sec for backwards compatibility with older params.
        self.declare_parameter("sync_slop_sec", 0.2)
        self.declare_parameter("sync_queue_size", 30)
        self.declare_parameter("snapshot_timeout_sec", 2.0)

        # ROI in pixel coordinates (on organized cloud/image).
        # If roi_w/roi_h <= 0 => full frame.
        self.declare_parameter("roi_x", 0)
        self.declare_parameter("roi_y", 0)
        self.declare_parameter("roi_w", 0)
        self.declare_parameter("roi_h", 0)

        self.declare_parameter("save_image", False)
        self.declare_parameter("save_depth", False)
        self.declare_parameter("save_pointcloud_ply", False)
        self.declare_parameter("ply_stride", 2)
        self.declare_parameter("depth_clip_min_m", 0.05)
        self.declare_parameter("depth_clip_max_m", 2.0)
        self.declare_parameter("depth_colormap", "turbo")
        self.declare_parameter("depth_colormap_invert", False)
        self.declare_parameter("save_dir", "/tmp/realsense_snapshots")

        if np is None:
            raise RuntimeError(
                "numpy is required for realsense_snapshot_analyzer"
            )
        if point_cloud2 is None:
            raise RuntimeError(
                "sensor_msgs_py is required for realsense_snapshot_analyzer"
            )

        self._bridge = CvBridge() if CvBridge is not None else None

        image_topic = str(self.get_parameter("image_topic").value)
        cloud_topic = str(self.get_parameter("pointcloud_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        depth_info_topic = str(
            self.get_parameter("depth_camera_info_topic").value
        )
        queue_size = int(self.get_parameter("sync_queue_size").value)
        use_sensor_qos = bool(self.get_parameter("use_sensor_qos").value)
        qos = qos_profile_sensor_data if use_sensor_qos else 10

        self._seq = 0
        self._cv = threading.Condition()

        self._has_depth = bool(depth_topic)
        self._has_depth_info = bool(depth_info_topic)

        self._image_buf: Deque[Tuple[int, Image]] = deque(maxlen=queue_size)
        self._cloud_buf: Deque[Tuple[int, PointCloud2]] = deque(
            maxlen=queue_size
        )
        self._depth_buf: Deque[Tuple[int, Image]] = deque(maxlen=queue_size)

        self._image_count = 0
        self._cloud_count = 0
        self._depth_count = 0
        self._depth_info_count = 0

        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos,
            callback_group=self._sub_cb_group,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self._on_cloud,
            qos,
            callback_group=self._sub_cb_group,
        )
        self._depth_sub = None
        if self._has_depth:
            self._depth_sub = self.create_subscription(
                Image,
                depth_topic,
                self._on_depth,
                qos,
                callback_group=self._sub_cb_group,
            )

        self._depth_info_sub = None
        self._depth_info: Optional[CameraInfo] = None
        if self._has_depth_info:
            self._depth_info_sub = self.create_subscription(
                CameraInfo,
                depth_info_topic,
                self._on_depth_info,
                qos,
                callback_group=self._sub_cb_group,
            )

        self._srv = self.create_service(
            Trigger,
            "~/capture",
            self._on_capture,
            callback_group=self._srv_cb_group,
        )

        self.get_logger().info(
            f"Listening to image='{image_topic}' cloud='{cloud_topic}'. "
            "Service: ~capture (std_srvs/Trigger)."
        )
        if self._has_depth:
            self.get_logger().info(f"Also listening to depth='{depth_topic}'.")
        if self._has_depth_info:
            self.get_logger().info(
                f"Also listening to depth camera_info='{depth_info_topic}'."
            )

    @staticmethod
    def _stamp_ns(msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )

    def _on_image(self, msg: Image) -> None:
        with self._cv:
            self._image_count += 1
            self._image_buf.append((self._stamp_ns(msg), msg))
            self._cv.notify_all()

    def _on_cloud(self, msg: PointCloud2) -> None:
        with self._cv:
            self._cloud_count += 1
            self._cloud_buf.append((self._stamp_ns(msg), msg))
            self._cv.notify_all()

    def _on_depth(self, msg: Image) -> None:
        with self._cv:
            self._depth_count += 1
            self._depth_buf.append((self._stamp_ns(msg), msg))
            self._cv.notify_all()

    def _on_depth_info(self, msg: CameraInfo) -> None:
        with self._cv:
            self._depth_info_count += 1
            self._depth_info = msg
            self._cv.notify_all()

    def _pick_closest(
        self, target_ns: int, buf: Deque[Tuple[int, object]]
    ) -> Optional[Tuple[int, object]]:
        if not buf:
            return None
        return min(buf, key=lambda t: abs(t[0] - target_ns))

    def _wait_for_pair(self, timeout: Duration) -> Optional[SyncedFrame]:
        deadline = self.get_clock().now() + timeout
        with self._cv:
            while True:
                if self._image_buf:
                    img_ns, img = self._image_buf[-1]
                    slop_ns = int(
                        float(self.get_parameter("sync_slop_sec").value) * 1e9
                    )

                    cloud = None
                    if self._cloud_buf:
                        cloud_pick = self._pick_closest(
                            img_ns, self._cloud_buf
                        )
                        if cloud_pick is not None:
                            cloud_ns, cloud_obj = cloud_pick
                            if abs(cloud_ns - img_ns) <= slop_ns:
                                cloud = cloud_obj  # type: ignore

                    depth = None
                    if self._has_depth and self._depth_buf:
                        depth_pick = self._pick_closest(
                            img_ns, self._depth_buf
                        )
                        if depth_pick is not None:
                            depth_ns, depth_obj = depth_pick
                            if abs(depth_ns - img_ns) <= slop_ns:
                                depth = depth_obj  # type: ignore

                    # Prefer cloud if available, but allow depth-only capture.
                    if cloud is not None or depth is not None:
                        self._seq += 1
                        return SyncedFrame(
                            image=img,
                            cloud=cloud,
                            depth=depth,
                            depth_info=self._depth_info,
                            seq=self._seq,
                        )
                remaining = deadline - self.get_clock().now()
                if remaining.nanoseconds <= 0:
                    return None
                self._cv.wait(timeout=remaining.nanoseconds / 1e9)

    def _roi(self, width: int, height: int) -> Tuple[int, int, int, int]:
        roi_x = int(self.get_parameter("roi_x").value)
        roi_y = int(self.get_parameter("roi_y").value)
        roi_w = int(self.get_parameter("roi_w").value)
        roi_h = int(self.get_parameter("roi_h").value)

        if roi_w <= 0 or roi_h <= 0:
            return 0, 0, width, height

        x0 = max(0, min(width - 1, roi_x))
        y0 = max(0, min(height - 1, roi_y))
        x1 = max(x0 + 1, min(width, x0 + roi_w))
        y1 = max(y0 + 1, min(height, y0 + roi_h))
        return x0, y0, x1 - x0, y1 - y0

    def _cloud_to_xyz(self, cloud: PointCloud2) -> "np.ndarray":
        # Prefer numpy fast path if available.
        read_np = getattr(point_cloud2, "read_points_numpy", None)
        if callable(read_np):
            xyz = read_np(cloud, field_names=("x", "y", "z"))
            # shape: (N, 3)
            return xyz

        # Fallback: convert iterator to numpy.
        pts = point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=False
        )
        return np.asarray(list(pts), dtype=np.float32)

    def _save_bgr_image(self, img_msg: Image) -> Optional[str]:
        if not bool(self.get_parameter("save_image").value):
            return None
        if self._bridge is None or cv2 is None:
            self.get_logger().warn(
                "save_image=true but cv_bridge/cv2 is not available"
            )
            return None

        save_dir = str(self.get_parameter("save_dir").value)
        os.makedirs(save_dir, exist_ok=True)
        stamp = img_msg.header.stamp
        fname = f"snapshot_{stamp.sec}_{stamp.nanosec}.png"
        path = os.path.join(save_dir, fname)

        cv_img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        cv2.imwrite(path, cv_img)
        return path

    def _depth_to_meters(self, depth_msg: Image) -> "np.ndarray":
        if self._bridge is None:
            raise RuntimeError("cv_bridge is required for depth conversion")

        depth = self._bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough"
        )
        if depth_msg.encoding in ("16UC1", "mono16"):
            # RealSense aligned depth is commonly uint16 in millimeters.
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            # Assume meters (e.g., 32FC1).
            depth_m = depth.astype(np.float32)
        return depth_m

    def _save_depth_images(
        self, depth_msg: Optional[Image]
    ) -> Tuple[Optional[str], Optional[str]]:
        if depth_msg is None or not bool(
            self.get_parameter("save_depth").value
        ):
            return None, None
        if self._bridge is None or cv2 is None:
            self.get_logger().warn(
                "save_depth=true but cv_bridge/cv2 is not available"
            )
            return None, None

        save_dir = str(self.get_parameter("save_dir").value)
        os.makedirs(save_dir, exist_ok=True)
        stamp = depth_msg.header.stamp

        depth_m = self._depth_to_meters(depth_msg)
        clip_min = float(self.get_parameter("depth_clip_min_m").value)
        clip_max = float(self.get_parameter("depth_clip_max_m").value)
        if clip_max <= clip_min:
            clip_max = clip_min + 1e-3

        # Save raw depth as uint16 millimeters for easy viewing.
        depth_mm = np.clip(depth_m * 1000.0, 0.0, 65535.0).astype(np.uint16)
        raw_path = os.path.join(
            save_dir, f"depth_raw_{stamp.sec}_{stamp.nanosec}.png"
        )
        cv2.imwrite(raw_path, depth_mm)

        # Save a colormap for visualization (invalid/zero shown as black).
        d = depth_m.copy()
        valid = np.isfinite(d) & (d > 0.0)
        d = np.clip(d, clip_min, clip_max)
        norm = np.zeros_like(d, dtype=np.uint8)
        norm[valid] = (
            (d[valid] - clip_min)
            / (clip_max - clip_min)
            * 255.0
        ).astype(np.uint8)
        if bool(self.get_parameter("depth_colormap_invert").value):
            norm = 255 - norm

        cmap_name = str(self.get_parameter("depth_colormap").value).lower()
        cmap_map = {
            "turbo": cv2.COLORMAP_TURBO,
            "jet": cv2.COLORMAP_JET,
            "viridis": cv2.COLORMAP_VIRIDIS,
            "magma": cv2.COLORMAP_MAGMA,
            "inferno": cv2.COLORMAP_INFERNO,
            "plasma": cv2.COLORMAP_PLASMA,
            "ocean": cv2.COLORMAP_OCEAN,
            "bone": cv2.COLORMAP_BONE,
        }
        if cmap_name in ("gray", "grey", "grayscale"):
            color = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
        else:
            cmap = cmap_map.get(cmap_name, cv2.COLORMAP_TURBO)
            color = cv2.applyColorMap(norm, cmap)
        color[~valid] = (0, 0, 0)
        color_path = os.path.join(
            save_dir, f"depth_colormap_{stamp.sec}_{stamp.nanosec}.png"
        )
        cv2.imwrite(color_path, color)

        return raw_path, color_path

    def _save_cloud_ply(
        self,
        img_msg: Image,
        cloud_msg: Optional[PointCloud2],
        depth_msg: Optional[Image],
        depth_info: Optional[CameraInfo],
    ) -> Optional[str]:
        if not bool(self.get_parameter("save_pointcloud_ply").value):
            return None

        save_dir = str(self.get_parameter("save_dir").value)
        os.makedirs(save_dir, exist_ok=True)
        stamp = (
            cloud_msg.header.stamp
            if cloud_msg is not None
            else depth_msg.header.stamp
        )
        path = os.path.join(save_dir, f"cloud_{stamp.sec}_{stamp.nanosec}.ply")

        stride = max(1, int(self.get_parameter("ply_stride").value))

        xyz = None
        organized = False
        if cloud_msg is not None:
            xyz = self._cloud_to_xyz(cloud_msg)
            # Only colorize if we have organized cloud and matching image size.
            organized = (
                cloud_msg.height > 1
                and cloud_msg.width > 1
                and (cloud_msg.height * cloud_msg.width == xyz.shape[0])
                and img_msg.height == cloud_msg.height
                and img_msg.width == cloud_msg.width
            )

        rgb = None
        if organized and self._bridge is not None:
            try:
                bgr = self._bridge.imgmsg_to_cv2(
                    img_msg, desired_encoding="bgr8"
                )
                rgb = bgr[..., ::-1]  # BGR -> RGB
            except Exception:
                rgb = None

        # Build point list (ROI + stride if organized;
        # otherwise stride on flat).
        points = []
        if cloud_msg is not None and xyz is not None and organized:
            h = int(cloud_msg.height)
            w = int(cloud_msg.width)
            xyz_img = xyz.reshape((h, w, 3))
            x0, y0, rw, rh = self._roi(w, h)
            for y in range(y0, y0 + rh, stride):
                for x in range(x0, x0 + rw, stride):
                    X, Y, Z = xyz_img[y, x]
                    if not (np.isfinite(Z) and Z > 0.0):
                        continue
                    if rgb is not None:
                        r, g, b = (int(c) for c in rgb[y, x])
                        points.append((float(X), float(Y), float(Z), r, g, b))
                    else:
                        points.append((float(X), float(Y), float(Z)))
        elif cloud_msg is not None and xyz is not None:
            for i in range(0, xyz.shape[0], stride):
                X, Y, Z = xyz[i]
                if not (np.isfinite(Z) and Z > 0.0):
                    continue
                points.append((float(X), float(Y), float(Z)))
        else:
            # Fallback: compute xyz from depth image + intrinsics (CameraInfo).
            if depth_msg is None or depth_info is None:
                self.get_logger().warn(
                    "Cannot save PLY: missing PointCloud2 and missing "
                    "depth image or depth CameraInfo."
                )
                return None
            depth_m = self._depth_to_meters(depth_msg)
            h, w = depth_m.shape[:2]
            fx = float(depth_info.k[0])
            fy = float(depth_info.k[4])
            cx = float(depth_info.k[2])
            cy = float(depth_info.k[5])

            x0, y0, rw, rh = self._roi(w, h)
            for v in range(y0, y0 + rh, stride):
                for u in range(x0, x0 + rw, stride):
                    Z = float(depth_m[v, u])
                    if not (np.isfinite(Z) and Z > 0.0):
                        continue
                    X = (float(u) - cx) * Z / fx
                    Y = (float(v) - cy) * Z / fy
                    points.append((X, Y, Z))

        if not points:
            self.get_logger().warn("No valid points to save as PLY")
            return None

        has_color = rgb is not None and organized
        with open(path, "w", encoding="utf-8") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            if has_color:
                f.write(
                    "property uchar red\n"
                    "property uchar green\n"
                    "property uchar blue\n"
                )
            f.write("end_header\n")
            if has_color:
                for X, Y, Z, r, g, b in points:
                    f.write(f"{X} {Y} {Z} {r} {g} {b}\n")
            else:
                for X, Y, Z in points:
                    f.write(f"{X} {Y} {Z}\n")

        return path

    def _analyze(self, img_msg: Image, cloud_msg: PointCloud2) -> dict:
        xyz = self._cloud_to_xyz(cloud_msg)

        # If organized, we can map points to pixels (height x width).
        is_organized = (
            cloud_msg.height > 1
            and cloud_msg.width > 1
            and (cloud_msg.height * cloud_msg.width == xyz.shape[0])
        )
        if is_organized:
            h = int(cloud_msg.height)
            w = int(cloud_msg.width)
            xyz_img = xyz.reshape((h, w, 3))
            x0, y0, rw, rh = self._roi(w, h)
            roi_xyz = xyz_img[y0:y0 + rh, x0:x0 + rw, :]
            roi_xyz = roi_xyz.reshape((-1, 3))
            roi_desc = {"x": x0, "y": y0, "w": rw, "h": rh}
        else:
            roi_xyz = xyz
            roi_desc = {
                "x": 0,
                "y": 0,
                "w": 0,
                "h": 0,
                "note": "unorganized_pointcloud",
            }

        z = roi_xyz[:, 2]
        valid = np.isfinite(z) & (z > 0.0)

        if int(valid.sum()) == 0:
            return {
                "roi": roi_desc,
                "valid_points": 0,
                "depth_m": None,
                "closest_range_m": None,
            }

        z_valid = z[valid]
        xyz_valid = roi_xyz[valid]

        ranges = np.linalg.norm(xyz_valid, axis=1)
        closest = float(np.min(ranges))

        depth_stats = {
            "min": float(np.min(z_valid)),
            "max": float(np.max(z_valid)),
            "mean": float(np.mean(z_valid)),
            "median": float(np.median(z_valid)),
            "p10": float(np.percentile(z_valid, 10)),
            "p90": float(np.percentile(z_valid, 90)),
        }

        return {
            "roi": roi_desc,
            "valid_points": int(valid.sum()),
            "depth_m": depth_stats,
            "closest_range_m": closest,
            "cloud_frame": cloud_msg.header.frame_id,
            "image_frame": img_msg.header.frame_id,
        }

    def _on_capture(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        _ = request

        timeout = Duration(
            seconds=float(self.get_parameter("snapshot_timeout_sec").value)
        )
        frame = self._wait_for_pair(timeout=timeout)
        if frame is None:
            response.success = False
            with self._cv:
                img_c = self._image_count
                cloud_c = self._cloud_count
                depth_c = self._depth_count
                depth_info_c = self._depth_info_count
                last_img = self._image_buf[-1][0] if self._image_buf else None
                last_cloud = (
                    self._cloud_buf[-1][0] if self._cloud_buf else None
                )
            response.message = (
                "Timed out waiting for image+cloud pair within "
                f"{timeout.nanoseconds/1e9:.2f}s. "
                f"rx_counts(image={img_c},cloud={cloud_c},depth={depth_c},"
                f"depth_info={depth_info_c}) "
                f"last_stamps_ns(image={last_img},cloud={last_cloud})."
            )
            return response

        save_path = self._save_bgr_image(frame.image)
        depth_raw_path, depth_color_path = self._save_depth_images(frame.depth)
        ply_path = self._save_cloud_ply(
            frame.image, frame.cloud, frame.depth, frame.depth_info
        )

        if frame.cloud is not None:
            analysis = self._analyze(frame.image, frame.cloud)
            analysis["analysis_source"] = "pointcloud2"
        elif frame.depth is not None:
            depth_m = self._depth_to_meters(frame.depth)
            valid = np.isfinite(depth_m) & (depth_m > 0.0)
            if int(valid.sum()) == 0:
                analysis = {"valid_depth_pixels": 0, "depth_m": None}
            else:
                d = depth_m[valid]
                analysis = {
                    "valid_depth_pixels": int(valid.sum()),
                    "depth_m": {
                        "min": float(np.min(d)),
                        "max": float(np.max(d)),
                        "mean": float(np.mean(d)),
                        "median": float(np.median(d)),
                    },
                }
            analysis["analysis_source"] = "depth_image"
        else:
            analysis = {"analysis_source": "none"}
        if save_path is not None:
            analysis["saved_image"] = save_path
        if depth_raw_path is not None:
            analysis["saved_depth_raw"] = depth_raw_path
        if depth_color_path is not None:
            analysis["saved_depth_colormap"] = depth_color_path
        if ply_path is not None:
            analysis["saved_cloud_ply"] = ply_path

        response.success = True
        response.message = json.dumps(
            analysis, separators=(",", ":"), sort_keys=True
        )
        return response


def main() -> None:
    rclpy.init()
    node = RealSenseSnapshotAnalyzer()
    try:
        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
