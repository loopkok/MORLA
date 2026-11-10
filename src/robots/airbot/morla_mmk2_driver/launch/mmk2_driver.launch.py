from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('resolved_path',description='Compiled MMK2 SDK assembly snapshot'),
        Node(package='morla_mmk2_driver',executable='mmk2_driver',name='mmk2_driver',output='screen',parameters=[{'resolved_path':LaunchConfiguration('resolved_path')}])])
