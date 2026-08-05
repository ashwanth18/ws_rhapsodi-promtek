#!/usr/bin/env python3
"""Laptop-only scooping authoring loop with mock_components hardware.

No Gazebo, no Niryo driver, no Pi. Pair with scripts/dev_bench.sh which
exports ROS_DOMAIN_ID=42 so this graph cannot collide with a live cell.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    poses_yaml = LaunchConfiguration("poses_yaml")
    seed_poses_yaml = LaunchConfiguration("seed_poses_yaml")
    targets_yaml = LaunchConfiguration("targets_yaml")
    container_scene_yaml = LaunchConfiguration("container_scene_yaml")
    scoop_frame_id = LaunchConfiguration("scoop_frame_id")
    layout_id = LaunchConfiguration("layout_id")

    default_layouts_dir = os.environ.get(
        "CELL_LAYOUTS_DIR",
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "..", "..", "..", "config", "layouts"
            )
        ),
    )

    moveit_share = get_package_share_directory("niryo_ned3pro_moveit_config")
    initial_positions = os.path.join(moveit_share, "config", "initial_positions.yaml")
    urdf_xacro = os.path.join(moveit_share, "config", "niryo_ned3pro.urdf.xacro")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_xacro,
            " ",
            f"initial_positions_file:={initial_positions}",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("scooping_controller"),
            "config",
            "ros2_controllers_mock.yaml",
        ]
    )
    move_group_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("scooping_controller"),
            "config",
            "move_group_controller_params.yaml",
        ]
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "niryo_ned3pro", package_name="niryo_ned3pro_moveit_config"
        )
        .robot_description(
            file_path="config/niryo_ned3pro.urdf.xacro",
            mappings={"initial_positions_file": initial_positions},
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="stomp",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"],
        )
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml, {"use_sim_time": False}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "niryo_robot_follow_joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_controller_params,
            {"trajectory_execution": {"allowed_start_tolerance": 0.05}},
            {"moveit_manage_controllers": False},
            {"use_sim_time": False},
        ],
    )

    scooping_task_frame = Node(
        package="scooping_controller",
        executable="scooping_task_frame_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {
                "parent_frame_id": "base_link",
                "child_frame_id": scoop_frame_id,
                "task_container_id": "scooping_container",
                "use_sim_time": False,
            },
        ],
    )

    marker_server = Node(
        package="scooping_controller",
        executable="scooping_marker_server",
        output="screen",
        parameters=[
            container_scene_yaml,
            {
                "scoop_frame_id": scoop_frame_id,
                "goal_frame_id": "base_link",
                "poses_yaml": poses_yaml,
                "seed_poses_yaml": seed_poses_yaml,
                "layouts_dir": default_layouts_dir,
                "poses_env": "bench",
                "layout_id": layout_id,
                "authored_in": "bench",
                "use_sim_time": False,
            },
        ],
    )

    container_marker = Node(
        package="scooping_controller",
        executable="container_marker_publisher",
        output="screen",
        parameters=[container_scene_yaml, {"frame_id": "base_link", "use_sim_time": False}],
    )

    planning_scene_collisions = Node(
        package="scooping_controller",
        executable="planning_scene_collision_publisher",
        output="screen",
        parameters=[container_scene_yaml, {"frame_id": "base_link", "use_sim_time": False}],
    )

    move_to_server = Node(
        package="robot_moveit",
        executable="move_to_server_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "planning_group": "arm",
                "eef_link": "tcp_link",
                "targets_yaml": targets_yaml,
                "trajectory_action_server": (
                    "/niryo_robot_follow_joint_trajectory_controller/"
                    "follow_joint_trajectory"
                ),
                "use_sim_time": False,
            }
        ],
    )

    target_recorder = Node(
        package="robot_moveit",
        executable="target_recorder_node",
        output="screen",
        parameters=[
            {
                "planning_group": "arm",
                "eef_link": "tcp_link",
                "targets_yaml": targets_yaml,
                "pose_source": "auto",
                "use_sim_time": False,
            }
        ],
    )

    scooping_mtc = Node(
        package="scooping_controller",
        executable="scooping_mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            container_scene_yaml,
            {
                "use_sim_time": False,
                "group": "arm",
                "ik_frame": "tcp_link",
                "frame_id": scoop_frame_id,
                "trajectory_controller": "niryo_robot_follow_joint_trajectory_controller",
                "trajectory_action_server": (
                    "/niryo_robot_follow_joint_trajectory_controller/"
                    "follow_joint_trajectory"
                ),
            },
        ],
    )

    cell_layout_manager = Node(
        package="scooping_controller",
        executable="cell_layout_manager",
        output="screen",
        parameters=[
            {
                "layouts_dir": default_layouts_dir,
                "initial_layout_id": layout_id,
                "use_sim_time": False,
            }
        ],
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
            {"use_sim_time": False},
        ],
        condition=IfCondition(use_rviz),
    )

    after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[trajectory_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("scooping_controller"),
                        "config",
                        "authoring.rviz",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "poses_yaml",
                default_value="",
                description=(
                    "Optional override; empty derives "
                    "poses_bench_<layout_id>.yaml under ~/.ros/scooping_controller"
                ),
            ),
            DeclareLaunchArgument(
                "seed_poses_yaml",
                default_value="",
                description=(
                    "Optional override; empty uses "
                    "<layouts_dir>/<layout_id>/poses.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "targets_yaml",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_moveit"), "targets.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "container_scene_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("scooping_controller"),
                        "config",
                        "container_scene_real.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "scoop_frame_id", default_value="scooping_container_frame"
            ),
            DeclareLaunchArgument("layout_id", default_value="dual-container"),
            robot_state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            after_jsb,
            TimerAction(period=1.0, actions=[move_group_node]),
            TimerAction(period=1.5, actions=[scooping_task_frame, marker_server]),
            TimerAction(
                period=1.7, actions=[container_marker, planning_scene_collisions]
            ),
            TimerAction(period=2.0, actions=[move_to_server, target_recorder, scooping_mtc]),
            TimerAction(period=2.2, actions=[cell_layout_manager]),
            TimerAction(period=2.5, actions=[scooping_rviz]),
        ]
    )
