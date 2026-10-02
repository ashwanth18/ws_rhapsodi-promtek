#!/usr/bin/env python3
"""Minimal Niryo Ned3 Pro bringup for hand-eye calibration (no scooping scene)."""

from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _truthy(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _move_group(context, *args, **kwargs):
    enable_octomap = _truthy(LaunchConfiguration("enable_octomap").perform(context))
    sensors_3d_yaml = os.path.join(
        get_package_share_directory("camera_robot_calibration"),
        "config",
        "sensors_3d.yaml",
    )
    if not os.path.isfile(sensors_3d_yaml):
        sensors_3d_yaml = os.path.join(
            os.path.dirname(__file__), "..", "config", "sensors_3d.yaml"
        )

    urdf_path = os.path.join(
        get_package_share_directory("niryo_robot_description"),
        "urdf",
        "ned3pro",
        "niryo_ned3pro.urdf.xacro",
    )
    move_group_controller_params = os.path.join(
        get_package_share_directory("camera_robot_calibration"),
        "config",
        "move_group_controller_params.yaml",
    )

    builder = (
        MoveItConfigsBuilder(
            "niryo_ned3pro", package_name="niryo_ned3pro_moveit_config"
        )
        .robot_description(file_path=urdf_path)
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="stomp",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"],
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
    )
    if enable_octomap:
        try:
            get_package_share_directory("moveit_ros_perception")
            builder = builder.sensors_3d(file_path=sensors_3d_yaml)
        except PackageNotFoundError:
            pass
    moveit_config = builder.to_moveit_configs()

    parameters = [
        moveit_config.to_dict(),
        move_group_controller_params,
        {"trajectory_execution": {"allowed_start_tolerance": 0.05}},
        {"moveit_manage_controllers": False},
        {"use_sim_time": False},
    ]
    if enable_octomap:
        parameters.extend(
            [
                {"octomap_frame": "base_link"},
                {"octomap_resolution": 0.02},
                {"max_range": 1.5},
            ]
        )

    return [
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    output="screen",
                    parameters=parameters,
                    name="move_group",
                )
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    targets_yaml_default = os.path.join(
        get_package_share_directory("robot_moveit"), "config", "targets.yaml"
    )
    urdf_path = os.path.join(
        get_package_share_directory("niryo_robot_description"),
        "urdf",
        "ned3pro",
        "niryo_ned3pro.urdf.xacro",
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf_path]),
            value_type=str,
        )
    }

    # Laptop RSP publishes the scoop URDF. Bridging Niryo /tf gives a second
    # tcp_link (~10 cm flange vs ~33 cm scoop) so RobotModel flickers.
    driver_whitelist = os.path.join(
        get_package_share_directory("camera_robot_calibration"),
        "config",
        "niryo_driver_no_tf.yaml",
    )
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("niryo_ned_ros2_driver"),
                        "launch",
                        "driver.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"whitelist_params_file": driver_whitelist}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        name="robot_state_publisher",
    )

    move_to_server = Node(
        package="robot_moveit",
        executable="move_to_server_node",
        output="screen",
        parameters=[
            {
                "planning_group": "arm",
                "eef_link": LaunchConfiguration("eef_link"),
                "targets_yaml": LaunchConfiguration("targets_yaml"),
                "velocity_scaling": 0.15,
                "acceleration_scaling": 0.15,
                "use_sim_time": False,
            }
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            DeclareLaunchArgument("eef_link", default_value="tcp_link"),
            DeclareLaunchArgument("targets_yaml", default_value=targets_yaml_default),
            DeclareLaunchArgument(
                "enable_octomap",
                default_value="false",
                description="Fill MoveIt octomap from /camera/color/points",
            ),
            TimerAction(period=3.0, actions=[driver]),
            robot_state_publisher,
            OpaqueFunction(function=_move_group),
            TimerAction(period=4.0, actions=[move_to_server]),
        ]
    )
