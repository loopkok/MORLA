from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device',default_value=''),DeclareLaunchArgument('fake',default_value='false'),
        Node(package='morla_vader5_input',executable='vader5_input',output='screen',parameters=[
            {'device':LaunchConfiguration('device'),'fake':ParameterValue(LaunchConfiguration('fake'),value_type=bool)}]),
    ])
