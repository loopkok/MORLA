"""Optional common Web host; its CLI is not an rclpy node."""
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def start_session(context):
    executable = Path(get_package_prefix('morla_web_manager')) / 'lib/morla_web_manager/morla-composable-session'
    command = [str(executable)]
    for name in ('assembly', 'port', 'store'):
        command.extend(['--' + name, LaunchConfiguration(name).perform(context)])
    deployment = LaunchConfiguration('deployment').perform(context)
    if deployment:
        command.extend(['--deployment', deployment])
    return [ExecuteProcess(cmd=command, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('assembly', default_value=PathJoinSubstitution([
            FindPackageShare('morla_fr3_driver'), 'config', 'quest.fake.yaml'])),
        DeclareLaunchArgument('deployment', default_value=''),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('store', default_value='~/.local/share/morla-fr3'),
        OpaqueFunction(function=start_session),
    ])
