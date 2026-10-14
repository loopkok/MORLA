"""Launch one Nero driver; hardware SDK remains an explicit backend choice."""

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
    package = get_package_share_directory("nero_robot_control")
    config = LaunchConfiguration("params_file").perform(context).strip()
    if not config:
        config = os.path.join(package, "config", f"nero_driver_{side}.yaml")
    return [
        Node(
            package="nero_robot_control",
            executable="nero_robot_driver",
            name="nero_robot_driver",
            namespace=f"/{side}_arm",
            output="screen",
            parameters=[
                config,
                {"sdk_backend": LaunchConfiguration("sdk_backend")},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_side", default_value="left"),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument(
                "sdk_backend",
                default_value="fake",
                description="fake (safe software backend) | hardware (pyAgxArm/CAN)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
