"""Profile-selected Nero arm stack (driver + analytic IK teleop).

This P5 launch deliberately wires only the arm components. XHand profile
components join through the standard adapter in the later XHand integration.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from morla_core.profile import ProfileRegistry


def _setup(context, *args, **kwargs):
    profile_id = LaunchConfiguration("profile_id").perform(context).strip()
    profile = ProfileRegistry.from_index().load(profile_id)
    if profile.robot_id != "nero":
        raise RuntimeError(f"profile_id {profile_id!r} is not a Nero profile")
    arms = [
        component
        for component in profile.values["components"]
        if component.get("type") == "nero_arm" and component.get("enabled")
    ]
    if not arms:
        raise RuntimeError(f"profile_id {profile_id!r} has no enabled Nero arm")
    sides = sorted({component["side"] for component in arms})

    input_nodes = []
    if LaunchConfiguration("with_input").perform(context).lower() in ("true", "1", "yes"):
        allow_legacy = LaunchConfiguration("allow_legacy_packets").perform(context).lower() in (
            "true", "1", "yes"
        )
        input_config = os.path.join(
            get_package_share_directory("morla_input"), "config", "quest3_mocap.yaml"
        )
        input_nodes.append(
            Node(
                package="morla_input",
                executable="quest3_udp_mocap",
                name="quest3_udp_mocap",
                output="screen",
                parameters=[
                    input_config,
                    {
                        "arm_side": "both" if len(sides) == 2 else sides[0],
                        "landmark_preprocess": "raw",
                        "allow_legacy_packets": allow_legacy,
                    },
                ],
            )
        )

    start_gate = Node(
        package="morla_bringup",
        executable="controller_start_gate",
        name="controller_start_gate",
        output="screen",
        parameters=[{"joy_topic": "quest3/left_controller_joy", "button_index": 5}],
    )
    nero_driver_share = get_package_share_directory("nero_robot_control")
    teleop_share = get_package_share_directory("nero_arm_teleop")
    drivers = []
    teleops = []
    for component in arms:
        side = component["side"]
        driver_config = os.path.join(
            get_package_share_directory(component["driver_package"]),
            component["driver_params_file"],
        )
        teleop_config = os.path.join(
            get_package_share_directory(component["teleop_package"]),
            component["teleop_params_file"],
        )
        drivers.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nero_driver_share, "launch", "nero_robot_control.launch.py")
                ),
                launch_arguments={
                    "arm_side": side,
                    "sdk_backend": LaunchConfiguration("sdk_backend"),
                    "params_file": driver_config,
                }.items(),
            )
        )
        teleops.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(teleop_share, "launch", "nero_arm_teleop.launch.py")
                ),
                launch_arguments={
                    "arm_side": side,
                    "params_file": teleop_config,
                }.items(),
            )
        )
    xhand_count = sum(
        bool(component.get("enabled")) and component.get("type") == "xhand"
        for component in profile.values["components"]
    )
    if xhand_count:
        print(
            f"[nero_arm_stack] profile={profile_id}: {xhand_count} XHand component(s) "
            "declared; this arm-only launch does not start hand writers"
        )
    return [*input_nodes, start_gate, *drivers, *teleops]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile_id", default_value="nero_left_xhand"),
            DeclareLaunchArgument("sdk_backend", default_value="fake", description="fake | hardware"),
            DeclareLaunchArgument("with_input", default_value="true"),
            DeclareLaunchArgument("allow_legacy_packets", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
