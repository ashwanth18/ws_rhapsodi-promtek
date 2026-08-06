"""Launch only the Lexium driver node (no MoveIt).

Example:
    ros2 launch lexium_driver lexium_driver.launch.py ip:=192.168.88.82
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("lexium_driver"), "config", "lexium_driver.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ip", default_value="192.168.88.82", description="Lexium controller IP"
            ),
            Node(
                package="lexium_driver",
                executable="lexium_driver",
                name="lexium_driver",
                output="screen",
                parameters=[config, {"ip": LaunchConfiguration("ip")}],
            ),
        ]
    )
