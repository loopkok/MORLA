"""Generic compiled-system entry point. Starts disarmed; never calls enable/home."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('resolved_path', description='Immutable resolved_system.json path'),
        DeclareLaunchArgument('connect_inputs', default_value='true'),
        Node(package='morla_runtime', executable='runtime_node', name='morla_runtime',
             output='screen', parameters=[{'resolved_path': LaunchConfiguration('resolved_path'),
                 'connect_inputs': ParameterValue(LaunchConfiguration('connect_inputs'), value_type=bool)}]),
    ])
