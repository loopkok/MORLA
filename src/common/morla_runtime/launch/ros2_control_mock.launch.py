"""Optional Humble controller-manager prototype using GenericSystem only.

This launch contains no vendor or physical hardware plugin. It is an integration
fixture for the ROS JointTrajectory projection, not a physical simulator.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('morla_runtime'))
    namespace = '/morla_ros2_control_prototype'
    return LaunchDescription([
        Node(package='controller_manager', executable='ros2_control_node', namespace=namespace,
             parameters=[{'robot_description': (share / 'config/ros2_control_mock.urdf').read_text()},
                         str(share / 'config/ros2_control_mock.yaml')]),
        Node(package='controller_manager', executable='spawner', namespace=namespace,
             arguments=['joint_state_broadcaster', '--controller-manager', namespace + '/controller_manager',
                        '--controller-manager-timeout', '20']),
        Node(package='controller_manager', executable='spawner', namespace=namespace,
             arguments=['morla_mock_controller', '--controller-manager', namespace + '/controller_manager',
                        '--controller-manager-timeout', '20']),
    ])
