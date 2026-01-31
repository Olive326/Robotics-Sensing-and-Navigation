from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for GPS device'
        ),
        
        Node(
            package='gps_driver',
            executable='gps_driver',
            name='gps_driver_node',
            parameters=[{'port': LaunchConfiguration('port')}],
            output='screen'
        )
    ])
