"""CR5 MuJoCo bench with the shared web composition UI."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('assembly',default_value=PathJoinSubstitution([
            FindPackageShare('morla_cr5_driver'),'config','quest.sim.yaml'])),
        DeclareLaunchArgument('deployment',default_value=''),
        DeclareLaunchArgument('input_backend',default_value='fake'),
        DeclareLaunchArgument('port',default_value='8080'),
        DeclareLaunchArgument('store',default_value='~/.local/share/morla-cr5-sim'),
        ExecuteProcess(cmd=[FindExecutable(name='ros2'),'run','morla_web_manager','morla-composable-session',
            '--input-backend',LaunchConfiguration('input_backend'),
            '--assembly',LaunchConfiguration('assembly'),'--deployment',LaunchConfiguration('deployment'),
            '--port',LaunchConfiguration('port'),'--store',LaunchConfiguration('store')],output='screen'),
    ])
