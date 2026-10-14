"""Launch wujihand_driver node(s) for real Wuji Hand hardware.

Topic namespaces match wujihand_retargeting:
  /{left,right}_hand/joint_commands
  /{left,right}_hand/joint_states

Usage:
  ros2 launch wujihand_control wujihand_drivers.launch.py hand_side:=right
  ros2 launch wujihand_control wujihand_drivers.launch.py hand_side:=both \\
    right_serial:=YOUR_SN
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wujihand_control.hand_defaults import (
    DRIVER_DIAGNOSTICS_RATE,
    DRIVER_FILTER_CUTOFF_FREQ,
    DRIVER_PUBLISH_RATE,
    LEFT_HAND_NAME,
    LEFT_HAND_SERIAL,
    RIGHT_HAND_NAME,
    RIGHT_HAND_SERIAL,
)


def _setup(context, *args, **kwargs):
    side = LaunchConfiguration("hand_side").perform(context).strip().lower()
    if side not in ("left", "right", "both"):
        raise RuntimeError("hand_side must be left|right|both")

    publish_rate = float(
        LaunchConfiguration("publish_rate").perform(context) or DRIVER_PUBLISH_RATE
    )
    filter_hz = float(
        LaunchConfiguration("filter_cutoff_freq").perform(context)
        or DRIVER_FILTER_CUTOFF_FREQ
    )
    diag_rate = float(
        LaunchConfiguration("diagnostics_rate").perform(context)
        or DRIVER_DIAGNOSTICS_RATE
    )

    left_serial = LaunchConfiguration("left_serial").perform(context)
    right_serial = LaunchConfiguration("right_serial").perform(context)
    left_name = LaunchConfiguration("left_hand_name").perform(context) or LEFT_HAND_NAME
    right_name = (
        LaunchConfiguration("right_hand_name").perform(context) or RIGHT_HAND_NAME
    )

    nodes = []
    sides = ("left", "right") if side == "both" else (side,)
    for s in sides:
        serial = left_serial if s == "left" else right_serial
        name = left_name if s == "left" else right_name
        # Prefer serial; if empty, let driver connect by handedness.
        params = {
            "serial_number": str(serial or ""),
            "hand_side": "" if serial else s,
            "publish_rate": publish_rate,
            "filter_cutoff_freq": filter_hz,
            "diagnostics_rate": diag_rate,
        }
        nodes.append(
            Node(
                package="wujihand_driver",
                executable="wujihand_driver_node",
                name="wujihand_driver",
                namespace=name,
                parameters=[params],
                output="screen",
                emulate_tty=True,
            )
        )
    return nodes


def generate_launch_description():
    default_side = os.environ.get("GLOVE_HAND_SIDE", "right")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand_side",
                default_value=default_side,
                description="left|right|both",
            ),
            DeclareLaunchArgument(
                "left_serial",
                default_value=LEFT_HAND_SERIAL,
                description="Left hand USB serial (empty → connect by hand_side)",
            ),
            DeclareLaunchArgument(
                "right_serial",
                default_value=RIGHT_HAND_SERIAL,
                description="Right hand USB serial (empty → connect by hand_side)",
            ),
            DeclareLaunchArgument(
                "left_hand_name",
                default_value=LEFT_HAND_NAME,
                description="ROS namespace (must match retarget hand_name)",
            ),
            DeclareLaunchArgument(
                "right_hand_name",
                default_value=RIGHT_HAND_NAME,
                description="ROS namespace (must match retarget hand_name)",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value=str(DRIVER_PUBLISH_RATE),
            ),
            DeclareLaunchArgument(
                "filter_cutoff_freq",
                default_value=str(DRIVER_FILTER_CUTOFF_FREQ),
            ),
            DeclareLaunchArgument(
                "diagnostics_rate",
                default_value=str(DRIVER_DIAGNOSTICS_RATE),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
