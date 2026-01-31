from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'file_path',
            default_value='',
            description='Path to RTK GNGGA data file'
        ),
        
        Node(
            package='rtk_driver',
            executable='rtk_driver',
            name='rtk_driver_node',
            parameters=[{'file_path': LaunchConfiguration('file_path')}],
            output='screen'
        )
    ])
