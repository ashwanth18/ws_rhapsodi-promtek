#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("scooping_controller"),
                "config",
                "scooping.rviz",
            ]
        ),
        description="RViz config for the scooping workflow",
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser(
            "~/.ros/scooping_controller/warehouse_data.sqlite"
        ),
    }

    urdf_file = os.path.join(
        get_package_share_directory("niryo_robot_description"),
        "urdf",
        "ned3pro",
        "niryo_ned3pro.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "niryo_ned3pro", package_name="niryo_ned3pro_moveit_config"
        )
        .robot_description(file_path=urdf_file)
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="stomp",
            pipelines=[
                "ompl",
                "chomp",
                "pilz_industrial_motion_planner",
                "stomp",
            ]
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    scooping_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(
        [
            declare_rviz_config,
            scooping_rviz,
        ]
    )
