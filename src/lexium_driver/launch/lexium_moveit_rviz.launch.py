"""RViz launch for Lexium + MoveIt with the safety panel pre-loaded."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
    add_debuggable_node,
)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("jaka_zu5", package_name="jaka_zu5_moveit_config")
        .to_moveit_configs()
    )
    lexium_rviz = os.path.join(
        get_package_share_directory("lexium_driver"),
        "config",
        "moveit_lexium.rviz",
    )

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareLaunchArgument("rviz_config", default_value=lexium_rviz))

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time", default="false"),
                value_type=bool,
            )
        },
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld
