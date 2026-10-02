#!/usr/bin/env python3
"""Scooping stack + scoop_vision against the Isaac Sim twin (no hardware).

Same nodes as ``scooping_real.launch.py`` + ``scoop_vision.launch.py``, with
the Niryo driver and RealSense replaced by Isaac:

* ros2_control ``TopicBasedSystem`` on ``/isaac_joint_states`` / ``/isaac_joint_commands``
  (controller ``niryo_robot_follow_joint_trajectory_controller``, as on the cell);
* D455 frames (``sim_camera_tf``), Isaac depth -> 16UC1 (``depth_to_mm``);
* ``/weight`` from the twin scale.

Every node runs on Isaac's ``/clock``. Start Isaac first
(``scripts/run_isaac_twin.sh``), then, from the workspace root:

    source src/isaac_twin/scripts/twin_env.sh
    ros2 launch isaac_twin scooping_isaac.launch.py
"""

from __future__ import annotations

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

sys.path.insert(0, os.path.join(get_package_share_directory("scooping_controller"), "launch"))
from robot_profiles import package_path, package_share_path, robot_profile, xacro_command_args  # noqa: E402

TWIN_DOMAIN_ID = "77"


def _arg(context, name: str) -> str:
    return context.launch_configurations.get(name, "").strip()


def _workspace_root() -> str:
    env = os.environ.get("ISAAC_TWIN_WS", "").strip()
    if env:
        return env
    from isaac_twin.cell import workspace_root

    return str(workspace_root())


def _scoop_vision_params(nodes_yaml: str) -> str:
    """Write the twin's merged scoop_vision params file (see ``cell.twin_scoop_vision_params``)."""
    from isaac_twin.cell import twin_scoop_vision_params

    out_dir = os.path.join(os.path.expanduser("~"), ".cache", "isaac_twin")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "scoop_vision_twin.yaml")
    with open(out, "w", encoding="utf-8") as fh:
        yaml.safe_dump(twin_scoop_vision_params(nodes_yaml), fh, sort_keys=False)
    return out


def _isaac_joint_positions(topic: str, timeout_s: float) -> dict[str, float]:
    """One message from Isaac's joint state publisher (own rclpy context)."""
    import time

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    ctx = rclpy.Context()
    rclpy.init(args=[], context=ctx)
    try:
        node = rclpy.create_node("isaac_twin_launch_probe", context=ctx)
        got: list[JointState] = []
        node.create_subscription(JointState, topic, got.append, qos_profile_sensor_data)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        deadline = time.monotonic() + timeout_s
        while not got and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        node.destroy_node()
    finally:
        rclpy.shutdown(context=ctx)
    if not got:
        raise RuntimeError(
            f"No {topic} within {timeout_s:.0f} s. Start Isaac first: src/isaac_twin/scripts/run_isaac_twin.sh"
        )
    return dict(zip(got[0].name, got[0].position))


def _check_isolation() -> None:
    """Refuse to share a graph with a real cell (Pi / Niryo on the LAN)."""
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    discovery = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "")
    if domain != TWIN_DOMAIN_ID or discovery != "LOCALHOST":
        raise RuntimeError(
            f"Twin needs ROS_DOMAIN_ID={TWIN_DOMAIN_ID} and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST "
            f"(got {domain!r}, {discovery!r}). Run: source src/isaac_twin/scripts/twin_env.sh"
        )


def _setup(context, *args, **kwargs):
    if _arg(context, "allow_any_domain") != "true":
        _check_isolation()

    profile = robot_profile("niryo")
    twin = profile["isaac"]
    timing = twin.get("timing") or {}
    base_frame = profile["base_frame"]
    planning_group = profile["planning_group"]
    eef_link = profile["eef_link"]
    controller = profile["follow_joint_trajectory_controller"]
    traj_action = f"/{controller}/follow_joint_trajectory"
    scoop_frame_id = "scooping_container_frame"

    layouts_dir = _arg(context, "layouts_dir") or os.path.join(_workspace_root(), "config", "layouts")
    layouts_dir = os.path.abspath(os.path.expanduser(layouts_dir))
    layout_id = _arg(context, "layout_id")
    layout_yaml = os.path.join(layouts_dir, f"{layout_id}.yaml")
    if not os.path.isfile(layout_yaml):
        raise RuntimeError(f"{layout_yaml} not found; pass layouts_dir:=<ws>/config/layouts")
    with open(layout_yaml, encoding="utf-8") as fh:
        layout_doc = yaml.safe_load(fh) or {}
    task_container_id = str(layout_doc.get("task_container_id") or "rs6")
    tool_id = str(layout_doc.get("tool_id") or "")
    seed_poses_yaml = os.path.join(layouts_dir, layout_id, "poses.yaml")

    targets_yaml = _arg(context, "targets_yaml")
    if not targets_yaml:
        rel = (layout_doc.get("targets_by_robot") or {}).get("niryo") or layout_doc.get("targets_yaml")
        targets_yaml = os.path.join(layouts_dir, rel) if rel else package_share_path(profile["targets"])

    container_scene_yaml = package_path(twin["scene"])
    move_group_controller_params = package_path(twin["move_group_controller_params"])
    controllers_yaml = package_path(twin["controllers"])

    xacro_exe = PathJoinSubstitution([FindExecutable(name="xacro")])
    xacro_cmd = xacro_command_args(xacro_exe, twin["urdf"])
    for joint, position in _isaac_joint_positions("/isaac_joint_states", 30.0).items():
        xacro_cmd.extend([" ", f"initial_{joint}:={position:.6f}"])
    robot_description = {"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)}
    moveit_config = (
        MoveItConfigsBuilder(profile["moveit_robot_name"], package_name=twin["moveit_package"])
        .robot_description(file_path=package_share_path(twin["urdf"]))
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline=profile["planning_pipeline"],
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"],
        )
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml],
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "/controller_manager"],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_controller_params,
            {"trajectory_execution": {"allowed_start_tolerance": 0.05}},
            {"moveit_manage_controllers": False},
        ],
    )
    task_frame = Node(
        package="scooping_controller",
        executable="scooping_task_frame_publisher",
        output="screen",
        parameters=[
            container_scene_yaml,
            {"parent_frame_id": base_frame, "child_frame_id": scoop_frame_id, "task_container_id": task_container_id},
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
                "poses_yaml": "",
                "seed_poses_yaml": seed_poses_yaml,
                "layouts_dir": layouts_dir,
                # Own pose cache (~/.ros/scooping_controller/poses_isaac_<layout>.yaml):
                # twin edits never touch the real, bench or Gazebo caches.
                "poses_env": "isaac",
                "layout_id": layout_id,
                "task_container_id": task_container_id,
                "tool_id": tool_id,
                "authored_in": "isaac",
                "robot_key": "niryo",
                "tool_mesh_resource": profile["tool"]["mesh_resource"],
                "tcp_visual_offset_xyz": profile["tool"]["tcp_visual_offset_xyz"],
            },
        ],
    )
    container_marker = Node(
        package="scooping_controller",
        executable="container_marker_publisher",
        output="screen",
        parameters=[container_scene_yaml, {"frame_id": base_frame}],
    )
    collisions = Node(
        package="scooping_controller",
        executable="planning_scene_collision_publisher",
        output="screen",
        parameters=[container_scene_yaml, {"frame_id": base_frame}],
    )
    layout_manager = Node(
        package="scooping_controller",
        executable="cell_layout_manager",
        output="screen",
        parameters=[
            {"layouts_dir": layouts_dir, "initial_layout_id": layout_id, "robot_key": "niryo", "base_frame": base_frame}
        ],
    )
    move_to = Node(
        package="robot_moveit",
        executable="move_to_server_node",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {
                "planning_group": planning_group,
                "eef_link": eef_link,
                "targets_yaml": targets_yaml,
                "trajectory_action_server": traj_action,
                "planning_pipeline": profile["planning_pipeline"],
                "position_only_goal": profile["position_only_goal"],
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
            }
        ],
    )
    mtc = Node(
        package="scooping_controller",
        executable="scooping_mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            container_scene_yaml,
            {
                "group": planning_group,
                "ik_frame": eef_link,
                "frame_id": scoop_frame_id,
                "planning_scene_frame_id": base_frame,
                "trajectory_controller": controller,
                "trajectory_action_server": traj_action,
                "post_lift_vibration_enabled": True,
                "post_lift_vibration_duration_s": 5.0,
                "post_lift_vibration_intensity": 0.75,
                "post_lift_vibration_publish_rate_hz": 10.0,
                "post_lift_vibration_settle_s": 1.5,
            },
        ],
    )

    nodes_yaml = os.path.join(get_package_share_directory("isaac_twin"), "config", "twin_nodes.yaml")
    camera_tf = Node(package="isaac_twin", executable="sim_camera_tf", output="screen", parameters=[nodes_yaml])
    depth = Node(
        package="isaac_twin", executable="depth_to_mm_node", name="depth_to_mm", output="screen", parameters=[nodes_yaml]
    )
    scale = Node(
        package="isaac_twin",
        executable="twin_scale_node",
        name="twin_scale",
        output="screen",
        parameters=[nodes_yaml],
        condition=IfCondition(context.launch_configurations["with_scale"]),
    )
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("scoop_vision"), "launch", "scoop_vision.launch.py")
        ),
        launch_arguments={
            "start_realsense": "false",
            "publish_calibration": "true",
            "calibration_name": _arg(context, "calibration_name"),
            "params_file": _scoop_vision_params(nodes_yaml),
            "layouts_dir": layouts_dir,
            "use_rviz": "false",
        }.items(),
    )
    with_flow = IfCondition(context.launch_configurations["with_flow"])
    incline = Node(
        package="pouring_controller",
        executable="incline_control_node",
        output="screen",
        parameters=[nodes_yaml],
        condition=with_flow,
    )
    pour = Node(
        package="pouring_controller",
        executable="pour_server_node",
        output="screen",
        parameters=[nodes_yaml],
        condition=with_flow,
    )
    orchestrator = Node(
        package="robot_orchestrator",
        executable="orchestrator_node",
        output="screen",
        parameters=[
            {
                "tree_file": os.path.join(
                    get_package_share_directory("robot_orchestrator"), "bt_trees", "webhook_weightment.xml"
                ),
                "webhook_layout_id": layout_id,
                "batch_layout_id": layout_id,
                "lightsout_layout_id": layout_id,
            }
        ],
        condition=with_flow,
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", _arg(context, "rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(context.launch_configurations["use_rviz"]),
    )

    return [
        rsp,
        control_node,
        jsb_spawner,
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[jtc_spawner])),
        camera_tf,
        depth,
        scale,
        TimerAction(period=float(timing.get("move_group_delay", 2.0)), actions=[move_group]),
        TimerAction(period=float(timing.get("task_frame_delay", 2.5)), actions=[task_frame, marker_server]),
        TimerAction(period=float(timing.get("container_delay", 2.7)), actions=[container_marker, collisions]),
        TimerAction(period=float(timing.get("collisions_delay", 2.8)) + 0.2, actions=[layout_manager]),
        TimerAction(period=float(timing.get("move_to_delay", 5.0)), actions=[move_to, target_recorder]),
        TimerAction(period=float(timing.get("mtc_delay", 5.5)), actions=[mtc]),
        TimerAction(period=float(timing.get("vision_delay", 6.0)), actions=[vision]),
        TimerAction(period=float(timing.get("mtc_delay", 5.5)) + 1.0, actions=[incline, pour, orchestrator]),
        TimerAction(period=float(timing.get("rviz_delay", 4.0)), actions=[rviz]),
    ]


def generate_launch_description() -> LaunchDescription:
    rviz_default = os.path.join(get_package_share_directory("scoop_vision"), "rviz", "scoop_vision.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("layout_id", default_value="dual-container"),
            DeclareLaunchArgument("layouts_dir", default_value="", description="Default: <ws>/config/layouts"),
            DeclareLaunchArgument(
                "targets_yaml", default_value="", description="Default: the layout's targets_by_robot.niryo"
            ),
            DeclareLaunchArgument("calibration_name", default_value="niryo_d455_eob"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default),
            DeclareLaunchArgument("with_scale", default_value="true", description="Twin scale on /weight"),
            DeclareLaunchArgument(
                "with_flow",
                default_value="true",
                description="Pouring controller + orchestrator (bt_start_webhook_weightment)",
            ),
            DeclareLaunchArgument(
                "allow_any_domain",
                default_value="false",
                description="Skip the ROS_DOMAIN_ID=77 / LOCALHOST check (CI only)",
            ),
            SetParameter(name="use_sim_time", value=True),
            OpaqueFunction(function=_setup),
        ]
    )
