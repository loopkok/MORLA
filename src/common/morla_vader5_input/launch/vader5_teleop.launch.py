from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution,FindExecutable,PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share=FindPackageShare('morla_vader5_input')
    return LaunchDescription([
        DeclareLaunchArgument('device',default_value=''),DeclareLaunchArgument('fake_input',default_value='true'),
        DeclareLaunchArgument('assembly',default_value=PathJoinSubstitution([share,'config','cr5.fake.yaml'])),
        DeclareLaunchArgument('deployment',default_value=''),DeclareLaunchArgument('port',default_value='8080'),
        DeclareLaunchArgument('store',default_value='~/.local/share/morla-vader5'),
        ExecuteProcess(cmd=[FindExecutable(name='ros2'),'run','morla_web_manager','morla-composable-session',
            '--assembly',LaunchConfiguration('assembly'),'--deployment',LaunchConfiguration('deployment'),
            '--input-device',LaunchConfiguration('device'),
            '--input-backend',PythonExpression(["'fake' if '",LaunchConfiguration('fake_input'),"'.lower() == 'true' else 'real'"]),
            '--port',LaunchConfiguration('port'),'--store',LaunchConfiguration('store')],output='screen'),
    ])
