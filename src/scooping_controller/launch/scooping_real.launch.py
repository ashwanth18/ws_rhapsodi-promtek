#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    poses_yaml = LaunchConfiguration("poses_yaml")
    seed_poses_yaml = LaunchConfiguration("seed_poses_yaml")
    targets_yaml = LaunchConfiguration("targets_yaml")
    scoop_frame_id = LaunchConfiguration("scoop_frame_id")
    pattern_offset_y = LaunchConfiguration("pattern_offset_y")
    post_lift_vibration_enabled = LaunchConfiguration("post_lift_vibration_enabled")
    post_lift_vibration_duration_s = LaunchConfiguration("post_lift_vibration_duration_s")
    post_lift_vibration_intensity = LaunchConfiguration("post_lift_vibration_intensity")
    post_lift_vibration_publish_rate_hz = LaunchConfiguration("post_lift_vibration_publish_rate_hz")
    container_scene_yaml = LaunchConfiguration("container_scene_yaml")
    drivers_list_file = LaunchConfiguration("drivers_list_file")
    whitelist_params_file = LaunchConfiguration("whitelist_params_file")
    driver_log_level = LaunchConfiguration("driver_log_level")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for real-hardware authoring and monitoring",
    )
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
    declare_poses_yaml = DeclareLaunchArgument(
        "poses_yaml",
        default_value=os.path.expanduser("~/.ros/scooping_controller/poses_real.yaml"),
        description="YAML file used by RViz Save/Load scoop pose buttons",
    )
    declare_seed_poses_yaml = DeclareLaunchArgument(
        "seed_poses_yaml",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("scooping_controller"),
                "config",
                "poses_real_seed.yaml",
            ]
        ),
        description=(
            "Checked-in seed YAML copied on first startup when poses_yaml is "
            "missing"
        ),
    )
    declare_targets_yaml = DeclareLaunchArgument(
        "targets_yaml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robot_moveit"), "targets.yaml"]
        ),
        description="YAML file used by MoveTo and RecordTarget named poses",
    )
    declare_scoop_frame_id = DeclareLaunchArgument(
        "scoop_frame_id",
        default_value="scooping_container_frame",
        description="Task frame used for scoop pose authoring and planning",
    )
    declare_pattern_offset_y = DeclareLaunchArgument(
        "pattern_offset_y",
        default_value="0.0",
        description="Translate all scoop poses along scoop task-frame Y before planning",
    )
    declare_post_lift_vibration_enabled = DeclareLaunchArgument(
        "post_lift_vibration_enabled",
        default_value="true",
        description="Enable post-lift shake-off by default",
    )
    declare_post_lift_vibration_duration_s = DeclareLaunchArgument(
        "post_lift_vibration_duration_s",
        default_value="5.0",
        description="Default post-lift shake-off duration in seconds",
    )
    declare_post_lift_vibration_intensity = DeclareLaunchArgument(
        "post_lift_vibration_intensity",
        default_value="0.75",
        description="Default post-lift shake-off intensity in normalized 0..1 units",
    )
    declare_post_lift_vibration_publish_rate_hz = DeclareLaunchArgument(
        "post_lift_vibration_publish_rate_hz",
        default_value="10.0",
        description="Keepalive publish rate for post-lift shake-off",
    )
    declare_container_scene_yaml = DeclareLaunchArgument(
        "container_scene_yaml",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("scooping_controller"),
                "config",
                "container_scene_real.yaml",
            ]
        ),
        description="Container scene calibration for RViz and MoveIt",
    )
    declare_drivers_list_file = DeclareLaunchArgument(
        "drivers_list_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("niryo_ned_ros2_driver"),
                "config",
                "drivers_list.yaml",
            ]
        ),
        description="Driver list for the Niryo hardware launch",
    )
    declare_whitelist_params_file = DeclareLaunchArgument(
        "whitelist_params_file",
        default_value="",
        description="Optional Niryo driver whitelist parameter file",
    )
    declare_driver_log_level = DeclareLaunchArgument(
        "driver_log_level",
        default_value="INFO",
        description="Niryo driver log level",
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
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", urdf_file]
    )
    robot_description = {"robot_description": robot_description_content}

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
        .robot_description(file_path=urdf_file)
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="stomp",
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    driver_launch = IncludeLaunchDescription(
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
        launch_arguments=[
            ("drivers_list_file", drivers_list_file),
            ("whitelist_params_file", whitelist_params_file),
            ("log_level", driver_log_level),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
        name="robot_state_publisher",
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
                "parent_frame_id": "base_link",
                "child_frame_id": scoop_frame_id,
                "task_container_id": "scooping_container",
                "use_sim_time": False,
            }
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
                "use_sim_time": False,
            }
        ],
    )

    container_marker = Node(
        package="scooping_controller",
        executable="container_marker_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {
                "frame_id": "base_link",
                "use_sim_time": False,
            },
        ],
    )

    planning_scene_collisions = Node(
        package="scooping_controller",
        executable="planning_scene_collision_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {
                "frame_id": "base_link",
                "use_sim_time": False,
            },
        ],
    )

    move_to_server = Node(
        package="robot_moveit",
        executable="move_to_server_node",
        output="screen",
        parameters=[
            {
                "planning_group": "arm",
                "eef_link": "tcp_link",
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
                "trajectory_action_server": "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory",
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
                "trajectory_action_server": "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory",
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

    return LaunchDescription(
        [
            declare_use_rviz,
            declare_rviz_config,
            declare_poses_yaml,
            declare_seed_poses_yaml,
            declare_targets_yaml,
            declare_scoop_frame_id,
            declare_pattern_offset_y,
            declare_post_lift_vibration_enabled,
            declare_post_lift_vibration_duration_s,
            declare_post_lift_vibration_intensity,
            declare_post_lift_vibration_publish_rate_hz,
            declare_container_scene_yaml,
            declare_drivers_list_file,
            declare_whitelist_params_file,
            declare_driver_log_level,
            # Start the Niryo rosbridge driver slightly later so a cold robot boot
            # is less likely to miss the first connection attempt. Respawn in
            # niryo_ned_ros2_driver/launch/driver.launch.py covers remaining races.
            TimerAction(period=3.0, actions=[driver_launch]),
            robot_state_publisher,
            TimerAction(period=2.0, actions=[move_group_node]),
            TimerAction(period=4.0, actions=[move_to_server]),
            TimerAction(period=4.2, actions=[target_recorder]),
            TimerAction(period=4.8, actions=[scooping_task_frame]),
            TimerAction(period=5.0, actions=[marker_server]),
            TimerAction(period=5.5, actions=[container_marker]),
            TimerAction(period=5.7, actions=[planning_scene_collisions]),
            TimerAction(period=6.0, actions=[scooping_mtc]),
            TimerAction(period=7.0, actions=[scooping_rviz]),
        ]
    )
