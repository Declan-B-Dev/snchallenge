from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frequency', default_value='1.0'),
        
        Node(
            package='snchallenge',
            executable='waypoint',
            name='waypoint',
            output='screen',
            parameters=[
                {'frequency': LaunchConfiguration('frequency')},
            ]
        ),
    ])
