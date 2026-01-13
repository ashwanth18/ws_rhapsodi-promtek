import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    # ... all our other code goes here

    launch_arguments = {
    }
    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "niryo_ned3pro", package_name="niryo_robot_moveit_interface"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )



    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}, {"octomap_resolution": 0.01}, {"replan_delay": 0.1}],
    )



    # Get the path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("niryo_robot_moveit_interface"), "config", rviz_base]
    )

    # Launch RViz
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
            {"use_sim_time": True}
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("niryo_robot_moveit_interface"),
        "config",
        "ros2_controllers.yaml",
    )
    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
            static_tf,
            run_move_group_node,

        ]
    )

