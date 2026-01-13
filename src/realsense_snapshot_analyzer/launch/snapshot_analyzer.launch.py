from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="realsense_snapshot_analyzer",
                executable="snapshot_analyzer",
                name="realsense_snapshot_analyzer",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/camera/color/image_raw",
                        "pointcloud_topic": (
                            "/camera/camera/depth/color/points"
                        ),
                        "depth_topic": "/camera/camera/depth/image_rect_raw",
                        "depth_camera_info_topic": (
                            "/camera/camera/depth/camera_info"
                        ),
                        "use_sensor_qos": True,
                        "sync_slop_sec": 0.2,
                        "sync_queue_size": 10,
                        "snapshot_timeout_sec": 5.0,
                        "roi_x": 0,
                        "roi_y": 0,
                        "roi_w": 0,
                        "roi_h": 0,
                        "save_image": True,
                        "save_depth": True,
                        "save_pointcloud_ply": True,
                        "ply_stride": 2,
                        "depth_clip_min_m": 0.05,
                        "depth_clip_max_m": 1.0,
                        "depth_colormap": "turbo",
                        "depth_colormap_invert": False,
                        "save_dir": PathJoinSubstitution(
                            [
                                FindPackageShare(
                                    "realsense_snapshot_analyzer"
                                ),
                                "snapshots",
                            ]
                        ),
                    }
                ],
            )
        ]
    )
