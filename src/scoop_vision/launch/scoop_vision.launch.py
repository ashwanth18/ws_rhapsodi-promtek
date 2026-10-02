#!/usr/bin/env python3
"""D455 depth + hand-eye TF + scoop_vision, alongside a running scooping stack.

Needs the stack's ``/cell_layout/active``, ``/scoop_poses``,
``scooping_mtc_node`` and ``base_link → scooping_container_frame`` TF.
Unlike the calibration launches this does not force ROS_LOCALHOST_ONLY.
"""

from __future__ import annotations

import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _ensure_calib_home(context, *args, **kwargs):
    """easy_handeye2 only reads ~/.ros2/easy_handeye2; seed it from the repo copy."""
    name = LaunchConfiguration("calibration_name").perform(context).strip()
    dest = Path.home() / ".ros2" / "easy_handeye2" / "calibrations" / f"{name}.calib"
    if dest.is_file():
        return []
    src = Path(get_package_share_directory("camera_robot_calibration")) / "calibrations" / f"{name}.calib"
    if src.is_file():
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)
    return []


def _realsense(context, *args, **kwargs):
    rs_args = {
        "camera_namespace": "",
        "camera_name": "camera",
        "enable_depth": "true",
        "enable_color": LaunchConfiguration("enable_color").perform(context),
        "align_depth.enable": "false",
        # Raw depth cloud only for RViz checks; the node reads the depth image.
        "pointcloud.enable": LaunchConfiguration("use_rviz").perform(context),
        "depth_module.depth_profile": "848,480,15",
        # Temporal + spatial smoothing help on white, low-texture flour; the
        # node also takes a median over several frames.
        "spatial_filter.enable": "true",
        "temporal_filter.enable": "true",
    }
    serial = LaunchConfiguration("serial_no").perform(context).strip()
    if serial:
        rs_args["serial_no"] = f"_{serial}"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])]
            ),
            launch_arguments=rs_args.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution([FindPackageShare("scoop_vision"), "config", "scoop_vision.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_realsense", default_value="true"),
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("publish_calibration", default_value="true"),
            DeclareLaunchArgument("calibration_name", default_value="niryo_d455_eob"),
            DeclareLaunchArgument("params_file", default_value=params),
            # Native (non-Docker) runs: e.g. layouts_dir:=$PWD/config/layouts
            DeclareLaunchArgument("layouts_dir", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            OpaqueFunction(function=_ensure_calib_home),
            OpaqueFunction(
                function=_realsense,
                condition=IfCondition(LaunchConfiguration("start_realsense")),
            ),
            Node(
                package="camera_robot_calibration",
                executable="handeye_camera_link_publisher",
                name="handeye_camera_link_publisher",
                output="screen",
                parameters=[{"name": LaunchConfiguration("calibration_name")}],
                condition=IfCondition(LaunchConfiguration("publish_calibration")),
            ),
            Node(
                package="scoop_vision",
                executable="scoop_vision_node",
                name="scoop_vision",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"layouts_dir": LaunchConfiguration("layouts_dir")},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                # No name=: __node remaps every node in the RViz process
                # (panels, MoveIt displays) to one name.
                arguments=[
                    "-d",
                    PathJoinSubstitution([FindPackageShare("scoop_vision"), "rviz", "scoop_vision.rviz"]),
                ],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
