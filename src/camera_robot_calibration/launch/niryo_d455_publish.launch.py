#!/usr/bin/env python3
"""Publish a saved Niryo D455 eye-on-base calibration as TF.

Publishes ``robot_base_frame → camera_link`` (not optical). RealSense already
parents ``camera_color_optical_frame`` under ``camera_link``; a second parent
on the optical frame splits TF so the depth cloud never reaches ``base_link``.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            DeclareLaunchArgument(
                "name",
                default_value="niryo_d455_eob",
                description="easy_handeye2 calibration name (must match calibrate)",
            ),
            DeclareLaunchArgument(
                "camera_link_frame",
                default_value="camera_link",
                description="RealSense camera_link (parent of optical frames)",
            ),
            Node(
                package="camera_robot_calibration",
                executable="handeye_camera_link_publisher",
                name="handeye_camera_link_publisher",
                output="screen",
                parameters=[
                    {
                        "name": LaunchConfiguration("name"),
                        "camera_link_frame": LaunchConfiguration("camera_link_frame"),
                    }
                ],
            ),
        ]
    )
