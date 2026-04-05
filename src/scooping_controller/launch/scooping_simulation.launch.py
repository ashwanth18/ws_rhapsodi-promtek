#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    headless = LaunchConfiguration("headless")
    rviz_config = LaunchConfiguration("rviz_config")
    poses_yaml = LaunchConfiguration("poses_yaml")
    targets_yaml = LaunchConfiguration("targets_yaml")
    pattern_offset_y = LaunchConfiguration("pattern_offset_y")
    post_lift_vibration_enabled = LaunchConfiguration("post_lift_vibration_enabled")
    post_lift_vibration_duration_s = LaunchConfiguration("post_lift_vibration_duration_s")
    post_lift_vibration_intensity = LaunchConfiguration("post_lift_vibration_intensity")
    post_lift_vibration_publish_rate_hz = LaunchConfiguration("post_lift_vibration_publish_rate_hz")
    container_scene_yaml = LaunchConfiguration("container_scene_yaml")
    world = LaunchConfiguration("world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    declare_use_gazebo_gui = DeclareLaunchArgument(
        "use_gazebo_gui",
        default_value="true",
        description="Enable Gazebo GUI",
    )
    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Disable Gazebo GUI when true",
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
        default_value=os.path.expanduser(
            "~/.ros/scooping_controller/poses.yaml"
        ),
        description="YAML file used by RViz Save/Load scoop pose buttons",
    )
    declare_targets_yaml = DeclareLaunchArgument(
        "targets_yaml",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("robot_moveit"),
                "targets.yaml",
            ]
        ),
        description="YAML file used by MoveTo and RecordTarget named poses",
    )
    declare_pattern_offset_y = DeclareLaunchArgument(
        "pattern_offset_y",
        default_value="0.0",
        description=(
            "Translate all scoop poses along base_link Y before planning"
        ),
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
        default_value="0.5",
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
                "container_scene_sim.yaml",
            ]
        ),
        description="Container scene calibration for RViz and MoveIt",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("scooping_controller"),
                "worlds",
                "scooping_container_world.sdf",
            ]
        ),
        description="Gazebo world file for the scooping workflow",
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser(
            "~/.ros/scooping_controller/warehouse_data.sqlite"
        ),
    }

    sim_urdf = PathJoinSubstitution(
        [
            FindPackageShare("niryo_robot_description"),
            "urdf",
            "ned3pro",
            "niryo_ned3pro_gazebo.urdf.xacro",
        ]
    )
    robot_description_content_sim = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", sim_urdf]
    )
    robot_description_sim = {
        "robot_description": robot_description_content_sim
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("niryo_robot_moveit_interface"),
            "config",
            "ros2_controllers.yaml",
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
            "niryo_ned3pro", package_name="niryo_robot_moveit_interface"
        )
        .robot_description(mappings={})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        [
            PathJoinSubstitution(
                [FindPackageShare("scooping_controller"), "models"]
            ),
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            (
                "gz_args",
                [
                    PythonExpression(
                        [
                            "' -r -v 4 -s ' if '",
                            headless,
                            "' == 'true' else ' -r -v 4 '",
                        ]
                    ),
                    world,
                ],
            ),
            (
                "gui",
                PythonExpression(
                    [
                        "'false' if '",
                        headless,
                        "' == 'true' else '",
                        use_gazebo_gui,
                        "'",
                    ]
                ),
            ),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        name="gazebo_bridge",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_sim, {"use_sim_time": use_sim_time}],
        name="robot_state_publisher",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "niryo_ned3pro",
            "-allow_renaming",
            "true",
        ],
        name="spawn_robot",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            robot_controllers,
        ],
        name="joint_state_broadcaster_spawner",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "niryo_robot_follow_joint_trajectory_controller",
            "--param-file",
            robot_controllers,
        ],
        name="arm_controller_spawner",
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
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
        name="move_group",
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    marker_server = Node(
        package="scooping_controller",
        executable="scooping_marker_server",
        output="screen",
        parameters=[
            {
                "frame_id": "base_link",
                "poses_yaml": poses_yaml,
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
                "use_sim_time": use_sim_time,
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
                "use_sim_time": use_sim_time,
            }
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
                "use_sim_time": use_sim_time,
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
                "use_sim_time": use_sim_time,
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
                "use_sim_time": use_sim_time,
                "group": "arm",
                "ik_frame": "tcp_link",
                "frame_id": "base_link",
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
            {"use_sim_time": use_sim_time},
        ],
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    start_scooping_stack = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[
                TimerAction(period=2.0, actions=[move_group_node]),
                TimerAction(period=4.0, actions=[move_to_server]),
                TimerAction(period=4.2, actions=[target_recorder]),
                TimerAction(period=5.0, actions=[marker_server]),
                TimerAction(period=5.5, actions=[container_marker]),
                TimerAction(period=5.7, actions=[planning_scene_collisions]),
                TimerAction(period=6.0, actions=[scooping_mtc]),
                TimerAction(period=7.0, actions=[scooping_rviz]),
            ],
        )
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_use_gazebo_gui,
            declare_headless,
            declare_rviz_config,
            declare_poses_yaml,
            declare_targets_yaml,
            declare_pattern_offset_y,
            declare_post_lift_vibration_enabled,
            declare_post_lift_vibration_duration_s,
            declare_post_lift_vibration_intensity,
            declare_post_lift_vibration_publish_rate_hz,
            declare_container_scene_yaml,
            declare_world,
            gazebo_resource_path,
            gazebo_launch,
            bridge,
            robot_state_publisher,
            static_tf,
            gz_spawn_entity,
            start_controllers,
            start_arm_controller,
            start_scooping_stack,
        ]
    )
