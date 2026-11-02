"""CR5 MuJoCo bench driven by the real Vader 5 Pro (pad input only)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('morla_vader5_input')
    return LaunchDescription([
        DeclareLaunchArgument('assembly', default_value=PathJoinSubstitution([
            share, 'config', 'cr5.sim.yaml'])),
        # Real pad needs a deployment carrying device/hidraw path and asset identity
        # (config/input.real.yaml on the bench host).
        DeclareLaunchArgument('deployment', default_value=''),
        DeclareLaunchArgument('input_backend', default_value='fake'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('store', default_value='~/.local/share/morla-cr5-sim'),
        ExecuteProcess(cmd=[FindExecutable(name='ros2'), 'run', 'morla_web_manager', 'morla-composable-session',
            '--assembly', LaunchConfiguration('assembly'),
            '--deployment', LaunchConfiguration('deployment'),
            '--input-backend', LaunchConfiguration('input_backend'),
            '--port', LaunchConfiguration('port'),
            '--store', LaunchConfiguration('store')], output='screen'),
    ])
