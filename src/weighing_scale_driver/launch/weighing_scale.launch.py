#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    baud = LaunchConfiguration('baud', default='115200')
    topic = LaunchConfiguration('topic', default='weight')

    declare_port = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    declare_baud = DeclareLaunchArgument('baud', default_value='115200')
    declare_topic = DeclareLaunchArgument('topic', default_value='weight')

    node = Node(
        package='weighing_scale_driver',
        executable='weighing_scale_node',
        name='weighing_scale_node',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud,
            'topic': topic,
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_baud,
        declare_topic,
        node,
    ])

#!/usr/bin/env python3
"""
Launch the weighing scale serial reader node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')
    baud = LaunchConfiguration('baud', default='115200')
    topic = LaunchConfiguration('topic', default='weight')

    declare_port = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    declare_baud = DeclareLaunchArgument('baud', default_value='115200')
    declare_topic = DeclareLaunchArgument('topic', default_value='weight')

    node = Node(
        package='weighing_scale_driver',
        executable='weighing_scale_node',
        name='weighing_scale_node',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud,
            'topic': topic,
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_baud,
        declare_topic,
        node,
    ])


