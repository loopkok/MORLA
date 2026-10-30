"""Quest receiver + disarmed CR5 VR session. The host loads the selected adapter."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('assembly',default_value=PathJoinSubstitution([
            FindPackageShare('morla_cr5_driver'),'config','quest_right.fake.yaml'])),
        DeclareLaunchArgument('deployment',default_value=''),
        DeclareLaunchArgument('port',default_value='8080'),
        DeclareLaunchArgument('store',default_value='~/.local/share/morla-cr5-vr'),
        ExecuteProcess(cmd=[FindExecutable(name='ros2'),'run','morla_cr5_driver','cr5_vr_session',
            '--assembly',LaunchConfiguration('assembly'),'--deployment',LaunchConfiguration('deployment'),
            '--port',LaunchConfiguration('port'),'--store',LaunchConfiguration('store')],output='screen'),
    ])
