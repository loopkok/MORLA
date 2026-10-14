"""Launch Astral RobotMain ROS2 driver (astral_robot_sdk).

Usage:
  # Dry-run (no hardware / no SDK connect)
  ros2 launch astral_robot_control astral_drivers.launch.py dry_run:=true

  # Real board
  ros2 launch astral_robot_control astral_drivers.launch.py \\
    control_board_ip:=192.168.10.2 local_port:=8081
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("astral_robot_control")
    default_cfg = os.path.join(pkg, "config", "astral_robot.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_cfg,
                description="YAML params file",
            ),
            DeclareLaunchArgument(
                "control_board_ip",
                default_value=os.environ.get("ASTRAL_BOARD_IP", "192.168.10.2"),
            ),
            DeclareLaunchArgument(
                "local_port",
                default_value=os.environ.get("ASTRAL_LOCAL_PORT", "8081"),
            ),
            DeclareLaunchArgument(
                "local_ip",
                default_value=os.environ.get("ASTRAL_LOCAL_IP", "0.0.0.0"),
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="true = no SDK connect; log commands only",
            ),
            DeclareLaunchArgument(
                "auto_ready",
                default_value="true",
                description="call one_click_ready on startup",
            ),
            DeclareLaunchArgument(
                "driver_log_file",
                default_value="",
                description="empty → off. Non-empty JSONL path for the driver "
                "diagnostics log (cmd/send/state/srv/spike records) — 抓电机抽",
            ),
            Node(
                package="astral_robot_control",
                executable="astral_robot_driver",
                name="astral_robot_driver",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "control_board_ip": LaunchConfiguration("control_board_ip"),
                        "local_port": ParameterValue(
                            LaunchConfiguration("local_port"), value_type=int
                        ),
                        "local_ip": LaunchConfiguration("local_ip"),
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "auto_ready": ParameterValue(
                            LaunchConfiguration("auto_ready"), value_type=bool
                        ),
                        "driver_log_file": LaunchConfiguration("driver_log_file"),
                    },
                ],
            ),
        ]
    )
