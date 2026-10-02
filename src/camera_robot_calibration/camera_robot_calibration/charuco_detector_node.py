#!/usr/bin/env python3
"""Detect a ChArUco board and publish its TF relative to the camera optical frame."""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from camera_robot_calibration.charuco_board import load_board_config, make_charuco_board


class CharucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_detector")

        self.declare_parameter("board_config", "")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("overlay_topic", "/charuco_detector/overlay")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("marker_frame", "charuco_board")
        self.declare_parameter("min_charuco_corners", 6)
        self.declare_parameter("publish_tf", True)

        board_config = self.get_parameter("board_config").get_parameter_value().string_value
        self._cfg = load_board_config(board_config or None)
        self._board, self._dictionary = make_charuco_board(self._cfg)
        self._detector = cv2.aruco.CharucoDetector(self._board)

        self._bridge = CvBridge()
        self._tf_broadcaster = TransformBroadcaster(self)
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._have_detection = False

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        overlay_topic = self.get_parameter("overlay_topic").value

        self._overlay_pub = self.create_publisher(Image, overlay_topic, 10)
        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(
            f"ChArUco detector ready: {self._cfg['squares_x']}x{self._cfg['squares_y']} "
            f"{self._cfg['dictionary']} on {image_topic}"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    def _on_image(self, msg: Image) -> None:
        if self._camera_matrix is None or self._dist_coeffs is None:
            return

        try:
            self._on_image_impl(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"ChArUco callback failed: {exc}")

    def _on_image_impl(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"cv_bridge failed: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        charuco_corners, charuco_ids, marker_corners, marker_ids = self._detector.detectBoard(
            gray
        )

        overlay = frame.copy()
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)
        if charuco_ids is not None and len(charuco_ids) > 0:
            cv2.aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids)

        min_corners = int(self.get_parameter("min_charuco_corners").value)
        detected = (
            charuco_ids is not None
            and charuco_corners is not None
            and len(charuco_ids) >= min_corners
        )
        self._have_detection = bool(detected)

        if detected:
            pose = self._estimate_pose(charuco_corners, charuco_ids)
            if pose is not None:
                rvec, tvec = pose
                cv2.drawFrameAxes(
                    overlay,
                    self._camera_matrix,
                    self._dist_coeffs,
                    rvec,
                    tvec,
                    float(self._cfg["square_length_m"]) * 2.0,
                )
                if bool(self.get_parameter("publish_tf").value):
                    self._publish_tf(msg.header.stamp, rvec, tvec)

        try:
            self._overlay_pub.publish(self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"overlay publish failed: {exc}")

    def _estimate_pose(self, charuco_corners, charuco_ids):
        """OpenCV 4.7+ removed estimatePoseCharucoBoard; use matchImagePoints + solvePnP."""
        if hasattr(cv2.aruco, "estimatePoseCharucoBoard"):
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners,
                charuco_ids,
                self._board,
                self._camera_matrix,
                self._dist_coeffs,
                None,
                None,
            )
            return (rvec, tvec) if ok else None

        obj_pts, img_pts = self._board.matchImagePoints(charuco_corners, charuco_ids)
        if obj_pts is None or img_pts is None or len(obj_pts) < 4:
            return None
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts,
            img_pts,
            self._camera_matrix,
            self._dist_coeffs,
        )
        return (rvec, tvec) if ok else None

    def _publish_tf(self, stamp, rvec, tvec) -> None:
        rot, _ = cv2.Rodrigues(rvec)
        # Convert rotation matrix to quaternion
        qw = float(np.sqrt(max(0.0, 1.0 + rot[0, 0] + rot[1, 1] + rot[2, 2])) / 2.0)
        qx = float((rot[2, 1] - rot[1, 2]) / (4.0 * qw)) if qw > 1e-8 else 0.0
        qy = float((rot[0, 2] - rot[2, 0]) / (4.0 * qw)) if qw > 1e-8 else 0.0
        qz = float((rot[1, 0] - rot[0, 1]) / (4.0 * qw)) if qw > 1e-8 else 0.0

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = str(self.get_parameter("camera_frame").value)
        tf.child_frame_id = str(self.get_parameter("marker_frame").value)
        t = np.asarray(tvec, dtype=np.float64).reshape(3)
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

        # OpenCV origin is a sheet corner. Center sits on the scoop; the
        # corner is ~10 cm off the mesh and looks like a "depth" error.
        center = TransformStamped()
        center.header.stamp = stamp
        center.header.frame_id = tf.child_frame_id
        center.child_frame_id = f"{tf.child_frame_id}_center"
        center.transform.translation.x = 0.5 * float(self._cfg["squares_x"]) * float(
            self._cfg["square_length_m"]
        )
        center.transform.translation.y = 0.5 * float(self._cfg["squares_y"]) * float(
            self._cfg["square_length_m"]
        )
        center.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(center)

    @property
    def have_detection(self) -> bool:
        return self._have_detection


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
