"""Bring up the Lexium driver together with MoveIt 2 (move_group + RViz).

Reuses the existing jaka_zu5 MoveIt config and the shared demo launch helper
with use_rviz_sim:=false, so move_group talks to the real robot through the
Lexium driver's FollowJointTrajectory action server.

RViz is started with config/moveit_lexium.rviz, which includes the Lexium
Safety panel (power on/off, enable/disable, clear error, stop).

Example:
    ros2 launch lexium_driver lexium_moveit.launch.py ip:=192.168.88.82
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from jaka_moveit_launches.launches import (
    generate_move_group_launch,
    generate_rsp_launch,
)


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("lexium_driver"), "config", "lexium_driver.yaml"
    )
    lexium_rviz_launch = os.path.join(
        get_package_share_directory("lexium_driver"),
        "launch",
        "lexium_moveit_rviz.launch.py",
    )

    moveit_config = (
        MoveItConfigsBuilder("jaka_zu5", package_name="jaka_zu5_moveit_config")
        .to_moveit_configs()
    )
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "ip", default_value="192.168.88.82", description="Lexium controller IP"
        )
    )
    ld.add_action(SetLaunchConfiguration("use_rviz_sim", "false"))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    ld.add_action(
        Node(
            package="lexium_driver",
            executable="lexium_driver",
            name="lexium_driver",
            output="screen",
            parameters=[config, {"ip": LaunchConfiguration("ip")}],
        )
    )

    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    ld.add_action(generate_rsp_launch(moveit_config))
    ld.add_action(generate_move_group_launch(moveit_config))
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lexium_rviz_launch),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    return ld
