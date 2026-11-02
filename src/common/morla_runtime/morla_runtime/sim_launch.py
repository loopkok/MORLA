"""Shared launch description used by robot-owned MuJoCo packages."""

def description(package, config='composable.sim.yaml'):
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, ExecuteProcess
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
    from launch_ros.substitutions import FindPackageShare
    defaults = {'assembly': PathJoinSubstitution([FindPackageShare(package), 'config', config]),
                'deployment': '', 'input_backend': 'fake', 'sim_mode': 'kinematic',
                'sim_display': 'web', 'log_dir': '', 'port': '8080', 'store': '~/.local/share/'+package}
    declarations = [DeclareLaunchArgument(key, default_value=value,
                    **({'choices':['physics','kinematic']} if key=='sim_mode' else
                       {'choices':['web','local','both']} if key=='sim_display' else {}))
                    for key,value in defaults.items()]
    command = [FindExecutable(name='ros2'), 'run', 'morla_web_manager', 'morla-composable-session']
    for argument, key in [('assembly','assembly'),('deployment','deployment'),('input-backend','input_backend'),
                          ('sim-mode','sim_mode'),('sim-display','sim_display'),('log-dir','log_dir'),('port','port'),('store','store')]:
        command.extend(['--'+argument, LaunchConfiguration(key)])
    return LaunchDescription([*declarations, ExecuteProcess(cmd=command, output='screen')])
