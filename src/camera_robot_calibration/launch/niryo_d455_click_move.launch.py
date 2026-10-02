#!/usr/bin/env python3
"""Cold-start Niryo + D455 RGB-D cloud + hand-eye TF + RViz click-to-move."""

from __future__ import annotations

import os
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _ensure_calib_home(context, *args, **kwargs):
    name = LaunchConfiguration("calibration_name").perform(context).strip() or "niryo_d455_eob"
    dest_dir = Path.home() / ".ros2" / "easy_handeye2" / "calibrations"
    dest = dest_dir / f"{name}.calib"
    if dest.is_file():
        return []
    share = Path(get_package_share_directory("camera_robot_calibration"))
    src = share / "calibrations" / f"{name}.calib"
    if not src.is_file():
        # Source tree fallback when not yet installed.
        ws_src = Path(__file__).resolve().parents[1] / "calibrations" / f"{name}.calib"
        src = ws_src if ws_src.is_file() else src
    dest_dir.mkdir(parents=True, exist_ok=True)
    if src.is_file():
        shutil.copy2(src, dest)
    return []


def _setup(context, *args, **kwargs):
    share = get_package_share_directory("camera_robot_calibration")
    serial_no = LaunchConfiguration("serial_no").perform(context).strip()
    rviz_config = os.path.join(share, "rviz", "click_move.rviz")
    if not os.path.isfile(rviz_config):
        rviz_config = str(
            Path(__file__).resolve().parents[1] / "rviz" / "click_move.rviz"
        )

    rs_args = {
        "camera_namespace": "",
        "camera_name": "camera",
        "enable_color": "true",
        "enable_depth": "true",
        "align_depth.enable": "true",
        "enable_sync": "true",
        "pointcloud.enable": "false",
    }
    if serial_no:
        rs_args["serial_no"] = f"_{serial_no}"

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("camera_robot_calibration"),
                        "launch",
                        "niryo_robot_bringup.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "eef_link": "tcp_link",
            "enable_octomap": LaunchConfiguration("enable_octomap"),
        }.items(),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=rs_args.items(),
    )

    publish = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("camera_robot_calibration"),
                        "launch",
                        "niryo_d455_publish.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "name": LaunchConfiguration("calibration_name"),
        }.items(),
    )

    board_config = os.path.join(share, "config", "charuco_board.yaml")
    if not os.path.isfile(board_config):
        board_config = str(
            Path(__file__).resolve().parents[1] / "config" / "charuco_board.yaml"
        )
    charuco = Node(
        package="camera_robot_calibration",
        executable="charuco_detector",
        name="charuco_detector",
        output="screen",
        parameters=[
            {
                "board_config": board_config,
                "image_topic": "/camera/color/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "overlay_topic": "/charuco_detector/overlay",
                "camera_frame": "camera_color_optical_frame",
                "marker_frame": "charuco_board",
            }
        ],
    )

    aligned_cloud = Node(
        package="camera_robot_calibration",
        executable="aligned_color_cloud",
        name="aligned_color_cloud",
        output="screen",
        parameters=[
            {
                "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                "color_topic": "/camera/color/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "cloud_topic": "/camera/color/points",
                "camera_frame": "camera_color_optical_frame",
                "stride": 2,
            }
        ],
    )

    enable_containers = _truthy(
        LaunchConfiguration("enable_containers").perform(context)
    )
    container_scene_yaml = LaunchConfiguration("container_scene_yaml").perform(context)
    if not container_scene_yaml:
        container_scene_yaml = os.path.join(
            get_package_share_directory("scooping_controller"),
            "config",
            "container_scene",
            "niryo_real.yaml",
        )

    collisions = Node(
        package="scooping_controller",
        executable="planning_scene_collision_publisher",
        name="planning_scene_collision_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {"frame_id": "base_link", "use_sim_time": False},
        ],
    )
    container_markers = Node(
        package="scooping_controller",
        executable="container_marker_publisher",
        name="container_marker_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {"frame_id": "base_link", "use_sim_time": False},
        ],
    )

    click = Node(
        package="camera_robot_calibration",
        executable="click_move",
        name="click_move",
        output="screen",
        parameters=[
            {
                "base_frame": "base_link",
                "eef_link": "tcp_link",
                "camera_frame": "camera_color_optical_frame",
                "execute": True,
                "velocity_scaling": 0.10,
                "acceleration_scaling": 0.10,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    actions = [
        bringup,
        realsense,
        TimerAction(period=5.0, actions=[aligned_cloud, charuco]),
        TimerAction(period=6.0, actions=[publish]),
    ]
    if enable_containers:
        actions.append(TimerAction(period=7.0, actions=[collisions, container_markers]))
    actions.extend(
        [
            TimerAction(period=8.0, actions=[click]),
            TimerAction(period=9.0, actions=[rviz]),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            DeclareLaunchArgument(
                "serial_no",
                default_value="351322303477",
                description="D455 serial",
            ),
            DeclareLaunchArgument(
                "calibration_name",
                default_value="niryo_d455_eob",
                description="easy_handeye2 calibration name",
            ),
            DeclareLaunchArgument(
                "enable_octomap",
                default_value="true",
                description="Point-cloud octomap voxels (needs moveit_ros_perception)",
            ),
            DeclareLaunchArgument(
                "enable_containers",
                default_value="true",
                description="Authored RS3/RS6/table collisions and RViz markers",
            ),
            DeclareLaunchArgument(
                "container_scene_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("scooping_controller"),
                        "config",
                        "container_scene",
                        "niryo_real.yaml",
                    ]
                ),
                description="Authored RS3/RS6/table collision scene",
            ),
            OpaqueFunction(function=_ensure_calib_home),
            OpaqueFunction(function=_setup),
        ]
    )
