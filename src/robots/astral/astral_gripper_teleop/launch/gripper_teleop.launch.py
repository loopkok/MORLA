"""Quest3 pinch → /left_gripper/command (+ joint_commands).

Usage:
  ros2 launch astral_gripper_teleop gripper_teleop.launch.py
  ros2 launch astral_gripper_teleop gripper_teleop.launch.py hand_side:=left
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
    pkg = get_package_share_directory("astral_gripper_teleop")
    default_cfg = os.path.join(pkg, "config", "gripper_teleop.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_cfg),
            DeclareLaunchArgument(
                "hand_side",
                default_value="left",
                description="left | right",
            ),
            DeclareLaunchArgument(
                "controller_joy_topic",
                default_value="",
                description=(
                    "Touch controller Joy topic for trigger→gripper. Empty = "
                    "off (pinch only). Caller resolves the side, e.g. "
                    "quest3/left_controller_joy."
                ),
            ),
            DeclareLaunchArgument(
                "log_interval_s",
                default_value="0.0",
                description="状态日志间隔秒；0 = 关闭（避免 2s 一条刷屏）",
            ),
            Node(
                package="astral_gripper_teleop",
                executable="pinch_gripper_node",
                name="pinch_gripper_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "hand_side": LaunchConfiguration("hand_side"),
                        "controller_joy_topic": LaunchConfiguration(
                            "controller_joy_topic"
                        ),
                        "log_interval_s": ParameterValue(
                            LaunchConfiguration("log_interval_s"), value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
