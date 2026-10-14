import os
from pathlib import Path

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_path('astral_arm_clean_description')
    urdf_path = str(pkg_path / 'urdf' / 'astral_robot_clean.urdf')
    default_rviz_path = str(pkg_path / 'rviz' / 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='true', choices=['true', 'false'],
        description='Enable joint_state_publisher_gui')
    pub_rate_arg = DeclareLaunchArgument(
        name='pub_rate', default_value='200',
        description='Publishing rate for joint states')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'rate': LaunchConfiguration('pub_rate')}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_path],
    )

    return LaunchDescription([
        gui_arg,
        pub_rate_arg,
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz,
    ])
