#!/usr/bin/env python3
"""Niryo + D455 eye-on-base calibration (ChArUco + easy_handeye2 + MoveIt sampler)."""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_robot_cfg(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in {path}")
    return data


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("camera_robot_calibration")
    robot_cfg_path = LaunchConfiguration("robot_config").perform(context)
    if not robot_cfg_path:
        robot_cfg_path = os.path.join(share, "config", "robots", "niryo_d455.yaml")
    cfg = _load_robot_cfg(robot_cfg_path)

    board_config = LaunchConfiguration("board_config").perform(context)
    if not board_config:
        board_config = os.path.join(share, "config", "charuco_board.yaml")

    serial_no = LaunchConfiguration("serial_no").perform(context).strip()
    if not serial_no:
        serial_no = str(cfg.get("realsense", {}).get("serial_no", "") or "")

    frames = cfg.get("frames", {})
    topics = cfg.get("topics", {})
    sampling = cfg.get("sampling", {})
    rs = cfg.get("realsense", {})

    calibration_name = str(cfg.get("calibration_name", "niryo_d455_eob"))
    calibration_type = str(cfg.get("calibration_type", "eye_on_base"))

    dry_run = LaunchConfiguration("dry_run").perform(context)
    if dry_run == "":
        dry_run = "true" if sampling.get("dry_run") else "false"

    bringup_robot = LaunchConfiguration("bringup_robot")
    start_sampler = LaunchConfiguration("start_sampler")
    start_handeye = LaunchConfiguration("start_handeye").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    use_rqt = LaunchConfiguration("use_rqt")
    start_realsense = LaunchConfiguration("start_realsense")

    actions = []

    # Optional isolated robot bringup (driver + move_group + move_to).
    actions.append(
        IncludeLaunchDescription(
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
                "eef_link": str(frames.get("eef_link", "tcp_link")),
            }.items(),
            condition=IfCondition(bringup_robot),
        )
    )

    # RealSense with namespace remapped to /camera/... (Niryo recording contract).
    rs_args = {
        "camera_namespace": str(rs.get("camera_namespace", "")),
        "camera_name": str(rs.get("camera_name", "camera")),
        "enable_color": "true",
        "enable_depth": "false",
        "pointcloud.enable": "false",
        "align_depth.enable": "false",
    }
    if serial_no:
        rs_args["serial_no"] = f"_{serial_no}"  # realsense launch expects leading _

    actions.append(
        IncludeLaunchDescription(
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
            condition=IfCondition(start_realsense),
        )
    )

    charuco = Node(
        package="camera_robot_calibration",
        executable="charuco_detector",
        name="charuco_detector",
        output="screen",
        parameters=[
            {
                "board_config": board_config,
                "image_topic": str(topics.get("image", "/camera/color/image_raw")),
                "camera_info_topic": str(
                    topics.get("camera_info", "/camera/color/camera_info")
                ),
                "overlay_topic": str(
                    topics.get("overlay", "/charuco_detector/overlay")
                ),
                "camera_frame": str(
                    frames.get("tracking_base_frame", "camera_color_optical_frame")
                ),
                "marker_frame": str(
                    frames.get("tracking_marker_frame", "charuco_board")
                ),
            }
        ],
    )
    actions.append(charuco)

    # Do not construct these Nodes unless requested — launch looks up
    # package 'easy_handeye2' at parse time even with IfCondition.
    if start_handeye:
        actions.append(
            Node(
                package="easy_handeye2",
                executable="handeye_server",
                name="handeye_server",
                output="screen",
                parameters=[
                    {
                        "name": calibration_name,
                        "calibration_type": calibration_type,
                        "robot_base_frame": str(
                            frames.get("robot_base_frame", "base_link")
                        ),
                        "robot_effector_frame": str(
                            frames.get("robot_effector_frame", "hand_link")
                        ),
                        "tracking_base_frame": str(
                            frames.get(
                                "tracking_base_frame", "camera_color_optical_frame"
                            )
                        ),
                        "tracking_marker_frame": str(
                            frames.get("tracking_marker_frame", "charuco_board")
                        ),
                        "freehand_robot_movement": True,
                    }
                ],
            )
        )
        actions.append(
            Node(
                package="easy_handeye2",
                executable="rqt_calibrator.py",
                name="handeye_rqt_calibrator",
                output="screen",
                parameters=[
                    {
                        "name": calibration_name,
                        "calibration_type": calibration_type,
                        "robot_base_frame": str(
                            frames.get("robot_base_frame", "base_link")
                        ),
                        "robot_effector_frame": str(
                            frames.get("robot_effector_frame", "hand_link")
                        ),
                        "tracking_base_frame": str(
                            frames.get(
                                "tracking_base_frame", "camera_color_optical_frame"
                            )
                        ),
                        "tracking_marker_frame": str(
                            frames.get("tracking_marker_frame", "charuco_board")
                        ),
                    }
                ],
                condition=IfCondition(use_rqt),
            )
        )

    sampler = Node(
        package="camera_robot_calibration",
        executable="handeye_moveit_sampler",
        name="handeye_moveit_sampler",
        output="screen",
        parameters=[
            {
                "robot_base_frame": str(frames.get("robot_base_frame", "base_link")),
                "eef_link": str(frames.get("eef_link", "tcp_link")),
                "tracking_base_frame": str(
                    frames.get("tracking_base_frame", "camera_color_optical_frame")
                ),
                "tracking_marker_frame": str(
                    frames.get("tracking_marker_frame", "charuco_board")
                ),
                "num_samples": int(sampling.get("num_samples", 12)),
                "rotation_deg": float(sampling.get("rotation_deg", 25.0)),
                "translation_m": float(sampling.get("translation_m", 0.02)),
                "settle_s": float(sampling.get("settle_s", 1.5)),
                "detect_timeout_s": float(sampling.get("detect_timeout_s", 5.0)),
                "velocity_scaling": float(sampling.get("velocity_scaling", 0.15)),
                "acceleration_scaling": float(
                    sampling.get("acceleration_scaling", 0.15)
                ),
                "move_to_action": str(sampling.get("move_to_action", "/move_to")),
                "dry_run": dry_run.lower() in ("1", "true", "yes"),
                "auto_start": True,
            }
        ],
        condition=IfCondition(start_sampler),
    )
    # Delay sampler so camera / handeye_server / move_to come up first.
    actions.append(TimerAction(period=8.0, actions=[sampler]))

    return actions


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("camera_robot_calibration")
    return LaunchDescription(
        [
            # Keep Jaka / Pi scooping_stack off this graph (DDS).
            # Niryo rosbridge is TCP to ROBOT_IP, so LOCALHOST is safe here.
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            DeclareLaunchArgument(
                "robot_config",
                default_value=os.path.join(
                    share, "config", "robots", "niryo_d455.yaml"
                ),
                description="Robot/camera calibration YAML",
            ),
            DeclareLaunchArgument(
                "board_config",
                default_value=os.path.join(share, "config", "charuco_board.yaml"),
                description="ChArUco board YAML",
            ),
            DeclareLaunchArgument(
                "serial_no",
                default_value="",
                description="RealSense serial (empty = first device)",
            ),
            DeclareLaunchArgument(
                "bringup_robot",
                default_value="true",
                description="Start Niryo driver + move_group + move_to",
            ),
            DeclareLaunchArgument(
                "start_realsense",
                default_value="true",
                description="Start realsense2_camera",
            ),
            DeclareLaunchArgument(
                "start_sampler",
                default_value="true",
                description="Start MoveIt hand-eye sampler",
            ),
            DeclareLaunchArgument(
                "start_handeye",
                default_value="true",
                description="Start easy_handeye2 server (not needed for camera/board check)",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="Print poses / skip motion and easy_handeye2 writes",
            ),
            DeclareLaunchArgument(
                "use_rqt",
                default_value="false",
                description="Also start easy_handeye2 rqt calibrator",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
