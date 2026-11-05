from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('resolved_path', description='Compiled FR3 assembly snapshot'),
        Node(package='morla_fr3_driver', executable='fr3_driver', name='fr3_driver',
             output='screen', parameters=[{'resolved_path': LaunchConfiguration('resolved_path')}]),
    ])
