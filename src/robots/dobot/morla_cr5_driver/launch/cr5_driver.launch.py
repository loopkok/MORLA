from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('resolved_path', description='Compiled single-device snapshot'),
        Node(package='morla_cr5_driver', executable='cr5_driver', name='cr5_driver',
             output='screen', parameters=[{'resolved_path': LaunchConfiguration('resolved_path')}]),
    ])
