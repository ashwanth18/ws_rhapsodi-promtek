#!/usr/bin/env python3
"""
Launch file for Niryo MoveIt Control Node

This launch file starts the MoveIt control node that can execute
various robot movements and demonstrations.

Author: Learning ROS 2 Robotics
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Generate the launch description for the MoveIt control node.
    """
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_python_node = LaunchConfiguration('use_python_node', default='true')
    use_cpp_node = LaunchConfiguration('use_cpp_node', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_python_node = DeclareLaunchArgument(
        'use_python_node',
        default_value='false',
        description='Use Python MoveIt control node'
    )
    
    declare_use_cpp_node = DeclareLaunchArgument(
        'use_cpp_node',
        default_value='true',
        description='Use C++ MoveIt control node'
    )
    
    # Python MoveIt control node
    python_moveit_control_node = Node(
        package='niryo_robot_control',
        executable='moveit_control_node',
        name='python_moveit_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_python_node)
    )
    
    # C++ MoveIt control node
    cpp_moveit_control_node = Node(
        package='niryo_robot_control',
        executable='moveit_control_node_cpp',
        name='cpp_moveit_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_cpp_node)
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_use_python_node,
        declare_use_cpp_node,
        
        # MoveIt control nodes
        python_moveit_control_node,
        cpp_moveit_control_node,
    ]) 
