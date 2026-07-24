#!/usr/bin/env python3

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from robot_profiles import (
    default_targets_path,
    package_path,
    robot_profile,
    xacro_command_args,
)


def _gz_clock_bridge(context):
    """Bridge GZ world clock → ROS /clock.

    ros_gz_bridge README: if another publisher exists on gz /clock, Gazebo publishes only
    /world/<world>/clock. Use an explicit one-way CLOCK-QoS bridge (not CLI + remap only).
    """
    world_name = LaunchConfiguration("gz_world_name").perform(context).strip()
    gz_clock_topic = f"/world/{world_name}/clock"
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            name="gazebo_bridge",
            parameters=[
                {"bridge_names": ["gz_clock_bridge"]},
                {"bridges.gz_clock_bridge.ros_topic_name": "/clock"},
                {"bridges.gz_clock_bridge.gz_topic_name": gz_clock_topic},
                {"bridges.gz_clock_bridge.ros_type_name": "rosgraph_msgs/msg/Clock"},
                {"bridges.gz_clock_bridge.gz_type_name": "gz.msgs.Clock"},
                {"bridges.gz_clock_bridge.direction": "GZ_TO_ROS"},
                {"bridges.gz_clock_bridge.lazy": False},
                {"bridges.gz_clock_bridge.qos_profile": "CLOCK"},
            ],
        )
    ]


def _robot_sim_setup(context):
    """Build niryo Ned3 vs JAKA ZU5 gz-sim bringup (exclusive robot stacks)."""
    robot_lc = LaunchConfiguration("robot")
    robot = robot_lc.perform(context).strip().lower()
    profile = robot_profile(robot)
    sim_profile = profile["sim"]

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    poses_yaml = LaunchConfiguration("poses_yaml")
    seed_poses_yaml = LaunchConfiguration("seed_poses_yaml")
    targets_yaml = LaunchConfiguration("targets_yaml")
    scoop_frame_id = LaunchConfiguration("scoop_frame_id")
    pattern_offset_y = LaunchConfiguration("pattern_offset_y")
    post_lift_vibration_enabled = LaunchConfiguration("post_lift_vibration_enabled")
    post_lift_vibration_duration_s = LaunchConfiguration("post_lift_vibration_duration_s")
    post_lift_vibration_intensity = LaunchConfiguration("post_lift_vibration_intensity")
    post_lift_vibration_publish_rate_hz = LaunchConfiguration(
        "post_lift_vibration_publish_rate_hz"
    )
    container_scene_yaml = LaunchConfiguration("container_scene_yaml")

    # Pick a robot-specific saved-targets file: the legacy targets.yaml is in
    # Niryo "base_link" frame; Jaka uses link0. Users can still override via
    # targets_yaml:=<path>; we only substitute when the launch default is in use.
    legacy_targets_yaml_default = PathJoinSubstitution(
        [FindPackageShare("robot_moveit"), "targets.yaml"]
    ).perform(context)
    targets_yaml_value = targets_yaml.perform(context)
    if targets_yaml_value == legacy_targets_yaml_default:
        targets_yaml = default_targets_path(profile)

    legacy_scene_yaml_default = PathJoinSubstitution(
        [
            FindPackageShare("scooping_controller"),
            "config",
            "container_scene_sim.yaml",
        ]
    ).perform(context)
    if container_scene_yaml.perform(context) == legacy_scene_yaml_default:
        container_scene_yaml = package_path(sim_profile["scene"])

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser(
            "~/.ros/scooping_controller/warehouse_data.sqlite"
        ),
    }

    move_group_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("scooping_controller"),
            "config",
            "move_group_controller_params.yaml",
        ]
    )

    cfg = {
        "moveit_robot_name": profile["moveit_robot_name"],
        "moveit_package": sim_profile["moveit_package"],
        "planning_group": profile["planning_group"],
        "eef_link": profile["eef_link"],
        "base_link_frame": profile["base_frame"],
        "spawn_model_name": sim_profile["spawn_model_name"],
        "follow_joint_trajectory_controller": profile[
            "follow_joint_trajectory_controller"
        ],
        "planning_pipeline": profile["planning_pipeline"],
        "position_only_goal": profile["position_only_goal"],
        "tool_mesh_resource": profile["tool"]["mesh_resource"],
        "tcp_visual_offset_xyz": profile["tool"]["tcp_visual_offset_xyz"],
    }

    xacro_executable = PathJoinSubstitution([FindExecutable(name="xacro")])
    robot_description_content_sim = Command(
        xacro_command_args(xacro_executable, sim_profile["urdf"])
    )
    robot_description_sim = {
        "robot_description": ParameterValue(
            robot_description_content_sim,
            value_type=str,
        )
    }
    robot_controllers = package_path(sim_profile["controllers"])

    xacro_mappings = {}
    for xacro_arg in sim_profile["urdf"].get("xacro_args", []):
        if ":=" in xacro_arg:
            key, value = xacro_arg.split(":=", 1)
            xacro_mappings[key] = value

    if profile["key"] == "niryo":

        moveit_config = (
            MoveItConfigsBuilder(
                cfg["moveit_robot_name"], package_name=cfg["moveit_package"]
            )
            .robot_description(mappings={})
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(
                default_planning_pipeline=cfg["planning_pipeline"],
                pipelines=sim_profile["planning_pipelines"],
            )
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .to_moveit_configs()
        )

        traj_spawner_args = [
            cfg["follow_joint_trajectory_controller"],
            "--param-file",
            robot_controllers,
        ]
        mg_extra_params = [
            move_group_controller_params,
        ]
    else:
        moveit_config = (
            MoveItConfigsBuilder(
                cfg["moveit_robot_name"], package_name=cfg["moveit_package"]
            )
            .robot_description(mappings=xacro_mappings)
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .to_moveit_configs()
        )

        traj_spawner_args = [
            cfg["follow_joint_trajectory_controller"],
            "--param-file",
            robot_controllers,
        ]
        mg_extra_params = []

    if sim_profile.get("publish_world_to_base_tf", False):
        static_tf_block = [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="log",
                arguments=[
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    cfg["base_link_frame"],
                ],
            )
        ]
    else:
        static_tf_block = []

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            cfg["spawn_model_name"],
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
        arguments=traj_spawner_args,
        name="arm_controller_spawner",
    )

    move_group_params = [
        moveit_config.to_dict(),
        *mg_extra_params,
        {"trajectory_execution": {"allowed_start_tolerance": 0.05}},
        {"moveit_manage_controllers": False},
        {"use_sim_time": use_sim_time},
        warehouse_ros_config,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        name="move_group",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_sim, {"use_sim_time": use_sim_time}],
        name="robot_state_publisher",
    )

    base_frame = cfg["base_link_frame"]

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
                "use_sim_time": use_sim_time,
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
                "tool_mesh_resource": cfg["tool_mesh_resource"],
                "tcp_visual_offset_xyz": cfg["tcp_visual_offset_xyz"],
                "use_sim_time": use_sim_time,
            },
        ],
    )

    container_marker = Node(
        package="scooping_controller",
        executable="container_marker_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {
                "frame_id": base_frame,
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
                "frame_id": base_frame,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    traj_action_server = "/" + cfg["follow_joint_trajectory_controller"] + "/follow_joint_trajectory"

    move_to_server = Node(
        package="robot_moveit",
        executable="move_to_server_node",
        output="screen",
        parameters=[
            {
                "planning_group": cfg["planning_group"],
                "eef_link": cfg["eef_link"],
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
                "trajectory_action_server": traj_action_server,
                "planning_pipeline": cfg["planning_pipeline"],
                "position_only_goal": cfg["position_only_goal"],
                "use_sim_time": use_sim_time,
            },
        ],
    )

    target_recorder = Node(
        package="robot_moveit",
        executable="target_recorder_node",
        output="screen",
        parameters=[
            {
                "planning_group": cfg["planning_group"],
                "eef_link": cfg["eef_link"],
                "targets_yaml": targets_yaml,
                "record_frame": base_frame,
                "use_sim_time": use_sim_time,
            },
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
                "group": cfg["planning_group"],
                "ik_frame": cfg["eef_link"],
                "frame_id": scoop_frame_id,
                # MTC adds container/table collisions in this frame; default is
                # base_link (Niryo). For Jaka the robot root is link0.
                "planning_scene_frame_id": base_frame,
                "pattern_offset_y": pattern_offset_y,
                "post_lift_vibration_enabled": post_lift_vibration_enabled,
                "post_lift_vibration_duration_s": post_lift_vibration_duration_s,
                "post_lift_vibration_intensity": post_lift_vibration_intensity,
                "post_lift_vibration_publish_rate_hz": post_lift_vibration_publish_rate_hz,
                "trajectory_controller": cfg["follow_joint_trajectory_controller"],
                "trajectory_action_server": traj_action_server,
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

    # Callable form avoids Launch Jazzy + RegisterEventHandler + multiple TimerActions
    # hitting normalize_to_list_of_entities bugs ("bool is not iterable" on teardown).
    def _on_trajectory_controller_spawner_exit(_event, _context):
        return [
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

    start_scooping_stack = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=_on_trajectory_controller_spawner_exit,
        )
    )

    return [
        robot_state_publisher,
        *static_tf_block,
        gz_spawn_entity,
        start_controllers,
        start_arm_controller,
        start_scooping_stack,
    ]


def generate_launch_description():
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")

    declare_robot = DeclareLaunchArgument(
        "robot",
        default_value="niryo",
        description="Robot arm for sim: 'niryo' (NED3 Pro) or 'jaka' (ZU5 + MoveIt config in this workspace).",
    )

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
            [
                FindPackageShare("robot_moveit"),
                "targets.yaml",
            ]
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
        description=(
            "Translate all scoop poses along scoop task-frame Y before planning"
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

    declare_gz_world_name = DeclareLaunchArgument(
        "gz_world_name",
        default_value="scooping_container_world",
        description=(
            "Must match the <world name=\"...\"> in the loaded SDF; used for "
            "/clock ros_gz_bridge (GZ Sim publishes world-scoped clock topics)."
        ),
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

    return LaunchDescription(
        [
            declare_robot,
            declare_use_sim_time,
            declare_use_gazebo_gui,
            declare_headless,
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
            declare_world,
            declare_gz_world_name,
            gazebo_resource_path,
            gazebo_launch,
            OpaqueFunction(function=_gz_clock_bridge),
            OpaqueFunction(function=_robot_sim_setup),
        ]
    )
