from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for rosbridge websocket server'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='',
        description='Address to bind to (empty for all interfaces)'
    )
    
    delay_arg = DeclareLaunchArgument(
        'delay_between_messages',
        default_value='0.0',  # Changed from 0 to 0.0 (DOUBLE instead of INTEGER)
        description='Delay between messages in seconds'
    )

    # Rosbridge websocket node
    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'delay_between_messages': LaunchConfiguration('delay_between_messages'),
        }],
        output='screen'
    )

    # Rosapi node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        delay_arg,
        rosbridge_websocket_node,
        rosapi_node,
    ])








