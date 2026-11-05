"""Optional common Web host; install morla_web_manager alongside the driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('assembly', default_value=PathJoinSubstitution([
            FindPackageShare('morla_fr3_driver'), 'config', 'quest.fake.yaml'])),
        DeclareLaunchArgument('deployment', default_value=''),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('store', default_value='~/.local/share/morla-fr3'),
        Node(package='morla_web_manager', executable='morla-composable-session',
             output='screen', arguments=[
                 '--assembly', LaunchConfiguration('assembly'),
                 '--deployment', LaunchConfiguration('deployment'),
                 '--port', LaunchConfiguration('port'),
                 '--store', LaunchConfiguration('store')]),
    ])
