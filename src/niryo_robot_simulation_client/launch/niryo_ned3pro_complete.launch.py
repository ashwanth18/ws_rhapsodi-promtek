#!/usr/bin/env python3
"""
Complete launch file for Niryo NED3 Pro robot simulation with MoveIt integration.

This launch file starts:
1. Gazebo simulation environment
2. Robot spawn and controllers
3. MoveIt planning and visualization
4. All necessary bridges and state publishers

Author: Learning ROS 2 Robotics
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Generate the launch description for the complete Niryo NED3 Pro setup.
    """
    
    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================
    
    # Launch Arguments
    mode = LaunchConfiguration('mode', default='sim')  # 'sim' or 'real'
    use_micro_ros = LaunchConfiguration('use_micro_ros', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui', default='true')
    
    # Declare launch arguments
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description="Operation mode: 'sim' for Gazebo, 'real' for hardware"
    )

    declare_use_micro_ros = DeclareLaunchArgument(
        'use_micro_ros',
        default_value='false',
        description='Use Micro-ROS for communication'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    declare_use_gazebo_gui = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
      # ros2 warehouse sqlite
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": "/home/ashwanth/rhapsodi-promtek/warehouse_data.sqlite",
    }
    # ============================================================================
    # ROBOT DESCRIPTION
    # ============================================================================
    
    # Get URDF via xacro - choose Gazebo or real robot URDF based on mode
    sim_urdf = PathJoinSubstitution(
        [FindPackageShare('niryo_robot_description'), 'urdf', 'ned3pro', 'niryo_ned3pro_gazebo.urdf.xacro']
    )
    robot_description_content_sim = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', sim_urdf
    ])

    robot_description_sim = {'robot_description': robot_description_content_sim}
    
    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('niryo_robot_moveit_interface'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

        
    # ============================================================================
    # REAL HARDWARE COMPONENTS
    # ============================================================================
    
    # Hardware driver
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('niryo_ned_ros2_driver'), 'launch', 'driver.launch.py'])]
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
    )

   
    # Micro-ROS agent
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyAMC0'],
        condition=IfCondition(use_micro_ros)
    )
    
    # ============================================================================
    # SIMULATION COMPONENTS
    # ============================================================================
    
    # Robot State Publisher
    node_robot_state_publisher_sim = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_sim, {'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"])}],
        name='robot_state_publisher',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    
    # Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        name='gazebo_bridge',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    
    # Spawn robot in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'niryo_ned3pro', '-allow_renaming', 'true'],
        name='spawn_robot',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    
    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
        ],
        name='joint_state_broadcaster_spawner',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'niryo_robot_follow_joint_trajectory_controller',
            '--param-file',
            robot_controllers,
        ],
        name='arm_controller_spawner',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    
    # ============================================================================
    # MOVEIT COMPONENTS
    # ============================================================================
    
    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "niryo_ned3pro", package_name="niryo_robot_moveit_interface"
        )
        .robot_description(mappings={})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    
    # MoveIt move_group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution": {"allowed_start_tolerance": 0.05}},
            {"use_sim_time": PythonExpression(["'", mode, "' == 'sim'"]) },
            warehouse_ros_config
        ],
        name='move_group'
    )
    
    # RViz configuration
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("niryo_robot_moveit_interface"), "config", rviz_base]
    )
    
    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": PythonExpression(["'", mode, "' == 'sim'"])}
        ],
        condition=IfCondition(use_rviz)
    )
    
    # Static TF publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    moveTo =  Node(
            package="robot_moveit",
            executable="move_to_server_node",
            output="screen",
            parameters=[moveit_config.to_dict(), {
                "planning_group": "arm",
                "targets_yaml": "/home/ashwanth/rhapsodi-promtek/src/robot_moveit/config/targets.yaml",
            }],
        )
    # incline_executor removed; using pouring controller to drive joints
  
    # ============================================================================
    # EVENT HANDLERS (DEPENDENCY MANAGEMENT)
    # ============================================================================
    

    # Start controllers after robot spawn
    start_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    
    # Start arm controller after joint state broadcaster
    start_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    
    # Start MoveIt after controllers are ready (with delay)
    start_moveit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[
                TimerAction(
                    period=2.0,  # Wait 2 seconds for controllers to fully initialize
                    actions=[run_move_group_node]
                )
            ],
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )

   
    
    # In real mode, start move_group immediately (no Gazebo/controllers)
    run_move_group_real = TimerAction(
        period=0.1,
        actions=[run_move_group_node],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
    )
    # Start move_to_server after move_group is ready in sim or real
    run_move_to_server = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=run_move_group_node,
            on_start=[TimerAction(period=1.0, actions=[moveTo])],
        )
    )
    # Start incline executor normally; it uses MoveIt configs directly
    # ============================================================================
    # GAZEBO WORLD LAUNCH
    # ============================================================================
    
    # Launch Gazebo environment
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_sim.launch.py'])]
        ),
        launch_arguments=[
            ('gz_args', [' -r -v 4 empty.sdf']),  # Use default empty world
            ('gui', use_gazebo_gui)
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )

    
    # ============================================================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_mode,
        declare_use_micro_ros,
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_gazebo_gui,
        rviz_config_arg,
        
        # Gazebo environment (sim only)
        gazebo_launch,
        
        # Real hardware components
        driver_launch,
        # micro_ros_agent,

        # Simulation components
        bridge,
        node_robot_state_publisher_sim,
        gz_spawn_entity,
        
        # Event handlers for dependency management
        start_controllers,
        start_arm_controller,
        start_moveit,
        run_move_group_real,
        run_move_to_server,
        
        # MoveIt components (started by event handlers)
        static_tf,
        rviz_node,
    ]) 