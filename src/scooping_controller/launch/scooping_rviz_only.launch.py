#!/usr/bin/env python3
"""Launch RViz against an already-running scooping stack (robot-aware).

Used for laptop Lexium/Jaka sessions where compose stays headless
(use_rviz:=false) and the operator runs RViz natively on the host ROS graph
(network_mode: host). Supplies robot_description / SRDF / kinematics so
MotionPlanning and PlanningScene displays work.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from robot_profiles import (
    resolve_robot,
    resolve_rviz_config,
    robot_profile,
    xacro_command_args,
)


def _rviz_setup(context, *args, **kwargs):
    explicit = LaunchConfiguration("robot").perform(context).strip()
    robot_key = resolve_robot(explicit or None)
    profile = robot_profile(robot_key)
    # Prefer real profile (URDF/MoveIt package match the live cell); fall back
    # to sim if real is absent.
    mode_profile = profile.get("real") or profile.get("sim") or {}
    moveit_package = mode_profile["moveit_package"]

    rviz_config_explicit = LaunchConfiguration("rviz_config").perform(context).strip()
    rviz_config = resolve_rviz_config(profile, rviz_config_explicit or None)

    xacro_executable = FindExecutable(name="xacro").perform(context)
    robot_description_content = Command(
        xacro_command_args(xacro_executable, mode_profile["urdf"])
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Absolute URDF path for MoveItConfigsBuilder (expects a file under the
    # moveit package or an absolute path).
    urdf_file = os.path.join(
        get_package_share_directory(mode_profile["urdf"]["package"]),
        mode_profile["urdf"]["path"],
    )

    pipelines = list(
        mode_profile.get("planning_pipelines")
        or [profile.get("planning_pipeline") or "ompl"]
    )
    default_pipeline = profile.get("planning_pipeline") or pipelines[0]

    builder = (
        MoveItConfigsBuilder(
            profile["moveit_robot_name"], package_name=moveit_package
        )
        .robot_description(file_path=urdf_file)
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(
            file_path=f"config/{profile['moveit_robot_name']}.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline=default_pipeline,
            pipelines=pipelines,
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
    )
    moveit_config = builder.to_moveit_configs()

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser(
            "~/.ros/scooping_controller/warehouse_data.sqlite"
        ),
    }

    scooping_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": False},
        ],
    )
    return [scooping_rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="",
                description=(
                    "Robot key (niryo|jaka). Empty resolves from device.yaml "
                    "robot_type / ROBOT_TYPE."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description=(
                    "RViz config path. Empty selects the robot profile's rviz "
                    "entry, else scooping_controller/config/scooping.rviz."
                ),
            ),
            OpaqueFunction(function=_rviz_setup),
        ]
    )
