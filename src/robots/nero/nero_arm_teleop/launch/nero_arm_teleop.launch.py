"""Launch one Nero IK teleop publisher with its side profile parameters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    side = LaunchConfiguration("arm_side").perform(context).strip().lower()
    if side not in ("left", "right"):
        raise RuntimeError("arm_side must be left or right")
    package = get_package_share_directory("nero_arm_teleop")
    config = LaunchConfiguration("params_file").perform(context).strip()
    if not config:
        config = os.path.join(package, "config", f"nero_teleop_{side}.yaml")
    return [
        Node(
            package="nero_arm_teleop",
            executable="nero_arm_teleop_node",
            name="nero_arm_teleop",
            namespace=f"/{side}_arm",
            output="screen",
            parameters=[config],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_side", default_value="left"),
            DeclareLaunchArgument("params_file", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
