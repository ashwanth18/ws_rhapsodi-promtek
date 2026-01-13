import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Get URDF via xacro - Use Gazebo-specific URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('niryo_robot_description'),
                 'urdf', 'ned3pro', 'niryo_ned3pro_gazebo.urdf.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('niryo_robot_moveit_interface'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',  # Updated for ROS 2 Jazzy
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'niryo_ned3pro', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'niryo_robot_follow_joint_trajectory_controller',
            '--param-file',
            robot_controllers,
        ]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',  # Updated for ROS 2 Jazzy
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],  # Updated for Gazebo
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),  # Updated for ROS 2 Jazzy
                                       'launch',
                                       'gz_sim.launch.py'])]),  # Use gz_sim.launch.py
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]  # Use default empty world
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),

        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
