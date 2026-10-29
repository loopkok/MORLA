from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('resolved_path',description='Compiled EPG-only MORLA snapshot'),
        Node(package='morla_epg_driver',executable='epg_driver',name='epg_driver',
             output='screen',parameters=[{'resolved_path':LaunchConfiguration('resolved_path')}]),
    ])
