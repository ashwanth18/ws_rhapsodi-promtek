#!/usr/bin/env python3
"""Production scooping stack — robot selected from robots.yaml / device.yaml."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    package_share_path,
    resolve_robot,
    robot_profile,
    xacro_command_args,
)


def _robot_real_setup(context, *args, **kwargs):
    explicit = LaunchConfiguration("robot").perform(context).strip()
    robot_key = resolve_robot(explicit or None)
    profile = robot_profile(robot_key)
    real_profile = profile["real"]
    timing = real_profile.get("timing") or {}

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    poses_yaml = LaunchConfiguration("poses_yaml")
    seed_poses_yaml = LaunchConfiguration("seed_poses_yaml")
    targets_yaml = LaunchConfiguration("targets_yaml")
    scoop_frame_id = LaunchConfiguration("scoop_frame_id")
    pattern_offset_y = LaunchConfiguration("pattern_offset_y")
    post_lift_vibration_enabled = LaunchConfiguration("post_lift_vibration_enabled")
    post_lift_vibration_duration_s = LaunchConfiguration(
        "post_lift_vibration_duration_s"
    )
    post_lift_vibration_intensity = LaunchConfiguration(
        "post_lift_vibration_intensity"
    )
    post_lift_vibration_publish_rate_hz = LaunchConfiguration(
        "post_lift_vibration_publish_rate_hz"
    )
    container_scene_yaml = LaunchConfiguration("container_scene_yaml")
    layouts_dir = LaunchConfiguration("layouts_dir")
    poses_env = LaunchConfiguration("poses_env")
    drivers_list_file = LaunchConfiguration("drivers_list_file")
    whitelist_params_file = LaunchConfiguration("whitelist_params_file")
    driver_log_level = LaunchConfiguration("driver_log_level")

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
        container_scene_yaml = package_path(real_profile["scene"])

    base_frame = profile["base_frame"]
    planning_group = profile["planning_group"]
    eef_link = profile["eef_link"]
    controller = profile["follow_joint_trajectory_controller"]
    traj_action = f"/{controller}/follow_joint_trajectory"
    moveit_package = real_profile["moveit_package"]
    moveit_robot_name = profile["moveit_robot_name"]

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser(
            "~/.ros/scooping_controller/warehouse_data.sqlite"
        ),
    }

    mg_params_spec = real_profile.get("move_group_controller_params") or {
        "package": "scooping_controller",
        "path": "config/move_group_controller_params.yaml",
    }
    move_group_controller_params = package_path(mg_params_spec)

    xacro_executable = PathJoinSubstitution([FindExecutable(name="xacro")])
    urdf_share = package_share_path(real_profile["urdf"])
    robot_description_content = Command(
        xacro_command_args(xacro_executable, real_profile["urdf"])
    )
    robot_description = {"robot_description": robot_description_content}

    xacro_mappings = {}
    for xacro_arg in real_profile["urdf"].get("xacro_args", []):
        if ":=" in xacro_arg:
            key, value = xacro_arg.split(":=", 1)
            xacro_mappings[key] = value

    builder = MoveItConfigsBuilder(
        moveit_robot_name, package_name=moveit_package
    )
    # Prefer explicit file paths when present (Niryo); Jaka uses package defaults.
    if robot_key == "niryo":
        moveit_config = (
            builder.robot_description(file_path=urdf_share)
            .joint_limits(file_path="config/joint_limits.yaml")
            .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(
                default_planning_pipeline=profile["planning_pipeline"],
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"],
            )
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .to_moveit_configs()
        )
    else:
        moveit_config = (
            builder.robot_description(mappings=xacro_mappings)
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .to_moveit_configs()
        )

    actions = []

    driver_spec = real_profile.get("driver")
    if driver_spec:
        driver_launch_args = []
        if robot_key == "niryo":
            driver_launch_args = [
                ("drivers_list_file", drivers_list_file),
                ("whitelist_params_file", whitelist_params_file),
                ("log_level", driver_log_level),
            ]
        driver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare(driver_spec["package"]),
                            driver_spec["path"],
                        ]
                    )
                ]
            ),
            launch_arguments=driver_launch_args,
        )
        actions.append(TimerAction(period=3.0, actions=[driver_launch]))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        name="robot_state_publisher",
    )
    actions.append(robot_state_publisher)

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
            warehouse_ros_config,
        ],
        name="move_group",
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
                "poses_env": poses_env,
                "authored_in": "real",
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

    cell_layout_manager = Node(
        package="scooping_controller",
        executable="cell_layout_manager",
        output="screen",
        parameters=[
            {
                "layouts_dir": layouts_dir,
                "robot_key": robot_key,
                "base_frame": base_frame,
                "use_sim_time": False,
            }
        ],
    )

    move_to_server = Node(
        package="robot_moveit",
        executable="move_to_server_node",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {
                "planning_group": planning_group,
                "eef_link": eef_link,
                "constrain_upright": False,
                "upright_roll_tolerance_rad": 0.0872665,
                "upright_pitch_tolerance_rad": 0.0872665,
                "upright_yaw_tolerance_rad": 3.14159265,
                "constrain_transport_pitch": False,
                "transport_pitch_from_horizontal_rad": -0.34906585,
                "transport_roll_tolerance_rad": 3.14159265,
                "transport_pitch_tolerance_rad": 0.0872665,
                "transport_yaw_tolerance_rad": 3.14159265,
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
        respawn=True,
        respawn_delay=3.0,
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
                "pattern_offset_y": pattern_offset_y,
                "post_lift_vibration_enabled": post_lift_vibration_enabled,
                "post_lift_vibration_duration_s": post_lift_vibration_duration_s,
                "post_lift_vibration_intensity": post_lift_vibration_intensity,
                "post_lift_vibration_publish_rate_hz": post_lift_vibration_publish_rate_hz,
            },
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
            warehouse_ros_config,
            {"use_sim_time": False},
        ],
        condition=IfCondition(use_rviz),
    )

    actions.extend(
        [
            TimerAction(
                period=float(timing.get("move_group_delay", 2.0)),
                actions=[move_group_node],
            ),
            TimerAction(
                period=float(timing.get("move_to_delay", 12.0)),
                actions=[move_to_server],
            ),
            TimerAction(
                period=float(timing.get("target_recorder_delay", 12.2)),
                actions=[target_recorder],
            ),
            TimerAction(
                period=float(timing.get("task_frame_delay", 5.0)),
                actions=[scooping_task_frame],
            ),
            TimerAction(
                period=float(timing.get("marker_delay", 5.0)),
                actions=[marker_server],
            ),
            TimerAction(
                period=float(timing.get("container_delay", 5.5)),
                actions=[container_marker],
            ),
            TimerAction(
                period=float(timing.get("collisions_delay", 5.7)),
                actions=[planning_scene_collisions],
            ),
            TimerAction(
                period=float(timing.get("collisions_delay", 5.7)) + 0.2,
                actions=[cell_layout_manager],
            ),
            TimerAction(
                period=float(timing.get("mtc_delay", 6.0)),
                actions=[scooping_mtc],
            ),
            TimerAction(
                period=float(timing.get("rviz_delay", 7.0)),
                actions=[scooping_rviz],
            ),
        ]
    )
    return actions


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
                "use_rviz",
                default_value="true",
                description="Launch RViz for real-hardware authoring and monitoring",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("scooping_controller"),
                        "config",
                        "scooping.rviz",
                    ]
                ),
                description="RViz config for the scooping workflow",
            ),
            DeclareLaunchArgument(
                "poses_yaml",
                default_value="",
                description=(
                    "Optional override for the device-local pose cache. Empty lets "
                    "scooping_marker_server derive "
                    "~/.ros/scooping_controller/poses_real_<layout_id>.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "seed_poses_yaml",
                default_value="",
                description=(
                    "Optional override for versioned seed poses. Empty lets "
                    "scooping_marker_server use <layouts_dir>/<layout_id>/poses.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "poses_env",
                default_value="real",
                description="Pose cache environment tag (real|bench|sim)",
            ),
            DeclareLaunchArgument(
                "targets_yaml",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_moveit"), "targets.yaml"]
                ),
                description="YAML file used by MoveTo and RecordTarget named poses",
            ),
            DeclareLaunchArgument(
                "scoop_frame_id",
                default_value="scooping_container_frame",
                description="Task frame used for scoop pose authoring and planning",
            ),
            DeclareLaunchArgument(
                "pattern_offset_y",
                default_value="0.0",
                description="Translate all scoop poses along scoop task-frame Y before planning",
            ),
            DeclareLaunchArgument(
                "post_lift_vibration_enabled",
                default_value="true",
                description="Enable post-lift shake-off by default",
            ),
            DeclareLaunchArgument(
                "post_lift_vibration_duration_s",
                default_value="5.0",
                description="Default post-lift shake-off duration in seconds",
            ),
            DeclareLaunchArgument(
                "post_lift_vibration_intensity",
                default_value="0.75",
                description="Default post-lift shake-off intensity in normalized 0..1 units",
            ),
            DeclareLaunchArgument(
                "post_lift_vibration_publish_rate_hz",
                default_value="10.0",
                description="Keepalive publish rate for post-lift shake-off",
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
                description="Container scene calibration for RViz and MoveIt",
            ),
            DeclareLaunchArgument(
                "layouts_dir",
                default_value="/ws/config/layouts",
                description="Directory containing versioned cell-layout YAML files",
            ),
            DeclareLaunchArgument(
                "drivers_list_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("niryo_ned_ros2_driver"),
                        "config",
                        "drivers_list.yaml",
                    ]
                ),
                description="Driver list for the Niryo hardware launch (ignored for other robots)",
            ),
            DeclareLaunchArgument(
                "whitelist_params_file",
                default_value="",
                description="Optional Niryo driver whitelist parameter file",
            ),
            DeclareLaunchArgument(
                "driver_log_level",
                default_value="INFO",
                description="Niryo driver log level",
            ),
            OpaqueFunction(function=_robot_real_setup),
        ]
    )
