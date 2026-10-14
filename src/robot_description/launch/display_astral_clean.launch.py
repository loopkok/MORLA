from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("robot_description"))
    urdf = share / "urdf/astral/arm_clean/astral_robot_clean.urdf"
    rviz = share / "rviz/astral_clean.rviz"
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": urdf.read_text(encoding="utf-8")}],
        ),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(rviz)], output="screen"),
    ])
