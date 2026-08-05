#!/usr/bin/env python3
"""Laptop-only scooping authoring loop with mock_components hardware.

No Gazebo, no real driver, no Pi. Pair with scripts/dev_bench.sh which
exports ROS_DOMAIN_ID=42 so this graph cannot collide with a live cell.
Robot selected via robot:= / ROBOT_TYPE / device.yaml (robots.yaml mock:).
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from robot_profiles import (
    default_targets_path,
    package_path,
    resolve_robot,
    robot_profile,
    xacro_command_args,
)


def _default_layouts_dir() -> str:
    return os.environ.get(
        "CELL_LAYOUTS_DIR",
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "..", "..", "..", "config", "layouts"
            )
        ),
    )


def _robot_bench_setup(context, *args, **kwargs):
    explicit = LaunchConfiguration("robot").perform(context).strip()
    robot_key = resolve_robot(explicit or None)
    profile = robot_profile(robot_key)
    mock_profile = profile.get("mock")
    if not mock_profile:
        raise RuntimeError(
            f"Robot '{robot_key}' has no mock: profile in robots.yaml; "
            "cannot start the bench loop."
        )
    timing = mock_profile.get("timing") or {}

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    poses_yaml = LaunchConfiguration("poses_yaml")
    seed_poses_yaml = LaunchConfiguration("seed_poses_yaml")
    targets_yaml = LaunchConfiguration("targets_yaml")
    container_scene_yaml = LaunchConfiguration("container_scene_yaml")
    scoop_frame_id = LaunchConfiguration("scoop_frame_id")
    layout_id = LaunchConfiguration("layout_id")
    layouts_dir = LaunchConfiguration("layouts_dir")

    legacy_targets = PathJoinSubstitution(
        [FindPackageShare("robot_moveit"), "targets.yaml"]
    ).perform(context)
    if targets_yaml.perform(context) == legacy_targets:
        targets_yaml = default_targets_path(profile)

    legacy_scene = PathJoinSubstitution(
        [
            FindPackageShare("scooping_controller"),
            "config",
            "container_scene_real.yaml",
        ]
    ).perform(context)
    if container_scene_yaml.perform(context) == legacy_scene:
        container_scene_yaml = package_path(mock_profile["scene"])

    base_frame = profile["base_frame"]
    planning_group = profile["planning_group"]
    eef_link = profile["eef_link"]
    controller = profile["follow_joint_trajectory_controller"]
    traj_action = f"/{controller}/follow_joint_trajectory"
    moveit_package = mock_profile["moveit_package"]
    moveit_robot_name = profile["moveit_robot_name"]

    moveit_share = get_package_share_directory(moveit_package)
    initial_positions = os.path.join(moveit_share, "config", "initial_positions.yaml")

    xacro_executable = PathJoinSubstitution([FindExecutable(name="xacro")])
    # Append initial_positions_file for mock URDFs that support it.
    urdf_spec = dict(mock_profile["urdf"])
    xacro_args = list(urdf_spec.get("xacro_args") or [])
    if not any(arg.startswith("initial_positions_file:=") for arg in xacro_args):
        xacro_args.append(f"initial_positions_file:={initial_positions}")
    urdf_spec["xacro_args"] = xacro_args

    robot_description_content = Command(
        xacro_command_args(xacro_executable, urdf_spec)
    )
    robot_description = {"robot_description": robot_description_content}
    controllers_yaml = package_path(mock_profile["controllers"])
    mg_params_spec = mock_profile.get("move_group_controller_params") or {
        "package": "scooping_controller",
        "path": "config/move_group_controller_params.yaml",
    }
    move_group_controller_params = package_path(mg_params_spec)

    xacro_mappings = {"initial_positions_file": initial_positions}
    for xacro_arg in urdf_spec.get("xacro_args", []):
        if ":=" in xacro_arg:
            key, value = xacro_arg.split(":=", 1)
            xacro_mappings[key] = value

    builder = MoveItConfigsBuilder(
        moveit_robot_name, package_name=moveit_package
    )
    if robot_key == "niryo":
        moveit_config = (
            builder.robot_description(
                file_path="config/niryo_ned3pro.urdf.xacro",
                mappings={"initial_positions_file": initial_positions},
            )
            .joint_limits(file_path="config/joint_limits.yaml")
            .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(
                default_planning_pipeline=profile["planning_pipeline"],
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"],
            )
            .to_moveit_configs()
        )
    else:
        moveit_config = (
            builder.robot_description(mappings=xacro_mappings)
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
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "/controller_manager"],
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
                "parent_frame_id": base_frame,
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
                "goal_frame_id": base_frame,
                "poses_yaml": poses_yaml,
                "seed_poses_yaml": seed_poses_yaml,
                "layouts_dir": layouts_dir,
                "poses_env": "bench",
                "layout_id": layout_id,
                "authored_in": "bench",
                "robot_key": robot_key,
                "tool_mesh_resource": profile["tool"]["mesh_resource"],
                "tcp_visual_offset_xyz": profile["tool"]["tcp_visual_offset_xyz"],
                "use_sim_time": False,
            },
        ],
    )

    container_marker = Node(
        package="scooping_controller",
        executable="container_marker_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {"frame_id": base_frame, "use_sim_time": False},
        ],
    )

    planning_scene_collisions = Node(
        package="scooping_controller",
        executable="planning_scene_collision_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {"frame_id": base_frame, "use_sim_time": False},
        ],
    )

    move_to_server = Node(
        package="robot_moveit",
        executable="move_to_server_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "planning_group": planning_group,
                "eef_link": eef_link,
                "targets_yaml": targets_yaml,
                "trajectory_action_server": traj_action,
                "planning_pipeline": profile["planning_pipeline"],
                "position_only_goal": profile["position_only_goal"],
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
                "planning_group": planning_group,
                "eef_link": eef_link,
                "targets_yaml": targets_yaml,
                "pose_source": "auto",
                "record_frame": base_frame,
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
                "group": planning_group,
                "ik_frame": eef_link,
                "frame_id": scoop_frame_id,
                "planning_scene_frame_id": base_frame,
                "trajectory_controller": controller,
                "trajectory_action_server": traj_action,
            },
        ],
    )

    cell_layout_manager = Node(
        package="scooping_controller",
        executable="cell_layout_manager",
        output="screen",
        parameters=[
            {
                "layouts_dir": layouts_dir,
                "initial_layout_id": layout_id,
                "robot_key": robot_key,
                "base_frame": base_frame,
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

    return [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        after_jsb,
        TimerAction(
            period=float(timing.get("move_group_delay", 1.0)),
            actions=[move_group_node],
        ),
        TimerAction(
            period=float(timing.get("task_frame_delay", 1.5)),
            actions=[scooping_task_frame, marker_server],
        ),
        TimerAction(
            period=float(timing.get("container_delay", 1.7)),
            actions=[container_marker, planning_scene_collisions],
        ),
        TimerAction(
            period=float(timing.get("move_to_delay", 2.0)),
            actions=[move_to_server, target_recorder, scooping_mtc],
        ),
        TimerAction(
            period=float(timing.get("target_recorder_delay", 2.2)),
            actions=[cell_layout_manager],
        ),
        TimerAction(
            period=float(timing.get("rviz_delay", 2.5)),
            actions=[scooping_rviz],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="",
                description=(
                    "Robot key (niryo|jaka). Empty resolves from ROBOT_TYPE / "
                    "device.yaml (defaults to niryo)."
                ),
            ),
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
                "layouts_dir",
                default_value=_default_layouts_dir(),
                description="Directory containing versioned cell-layout YAML files",
            ),
            DeclareLaunchArgument(
                "scoop_frame_id", default_value="scooping_container_frame"
            ),
            DeclareLaunchArgument("layout_id", default_value="dual-container"),
            OpaqueFunction(function=_robot_bench_setup),
        ]
    )
