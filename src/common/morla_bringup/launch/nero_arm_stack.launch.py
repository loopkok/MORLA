"""Profile-selected Nero arm stack (driver + analytic IK teleop).

This P5 launch deliberately wires only the arm components. XHand profile
components join through the standard adapter in the later XHand integration.
"""

from __future__ import annotations

import os
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from morla_core.profile import ProfileRegistry
from morla_core.coordinate_contract import build_coordinate_contract
from morla_core.resource_locator import resolve_robot_asset
from morla_bringup.profile_state import profile_description_actions


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
    coordinate_contract = build_coordinate_contract(profile.values)

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
                        "coordinate_basis_id": coordinate_contract.input_basis_id,
                        "coordinate_contract_version": coordinate_contract.version,
                        "kinematics_hash": coordinate_contract.kinematics_hash,
                    },
                ],
            )
        )

    start_gate = Node(
        package="morla_bringup",
        executable="controller_start_gate",
        name="controller_start_gate",
        output="screen",
        parameters=[{
            "joy_topic": "quest3/left_controller_joy",
            "button_index": 5,
            "use_control_coordinator": True,
        }],
    )
    workpos_gate = Node(
        package="morla_bringup",
        executable="controller_workpos_gate",
        name="controller_workpos_gate",
        output="screen",
        parameters=[{
            "joy_topic": "quest3/left_controller_joy",
            "button_index": 0,
            "use_control_coordinator": True,
        }],
    )
    nero_driver_share = get_package_share_directory("nero_robot_control")
    teleop_share = get_package_share_directory("nero_arm_teleop")
    drivers = []
    teleops = []
    for component in arms:
        side = component["side"]
        model_path = resolve_robot_asset(str(component.get("description_model_id", "")))
        alignment = coordinate_contract.resolve_arm_input_rotation(side, str(model_path))
        tool_transform = coordinate_contract.resolve_arm_tool_transform(side, str(model_path))
        arm_coordinates = coordinate_contract.arm(side)
        tool = arm_coordinates["tool"]
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
                    "flange_frame_id": arm_coordinates["physical_flange_frame"],
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
                    "input_alignment": json.dumps(alignment.reshape(-1).tolist()),
                    "kinematics_hash": coordinate_contract.kinematics_hash,
                    "coordinate_contract_version": str(coordinate_contract.version),
                    "solver_tip_to_tcp_transform": json.dumps(
                        tool_transform.reshape(-1).tolist()
                    ),
                    "solver_base_frame": arm_coordinates["solver_base_frame"],
                    "solver_tip_frame": arm_coordinates["solver_tip_frame"],
                    "solver_adapter_id": arm_coordinates["solver_adapter_id"],
                    "tool_frame": tool["frame"],
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
    coordinator = Node(
        package="morla_bringup",
        executable="control_coordinator_node",
        name="morla_control_coordinator",
        output="screen",
        parameters=[{
            "profile_id": profile_id,
            "operating_mode": "real"
            if LaunchConfiguration("sdk_backend").perform(context).strip().lower() == "hardware"
            else "sim",
        }],
    )
    arm_only_profile = dict(profile.values)
    arm_only_profile["components"] = [component for component in profile.values["components"]
                                     if component.get("enabled")
                                     and component.get("type") == "nero_arm"]
    description_actions = profile_description_actions(arm_only_profile)
    return [*description_actions, *input_nodes, start_gate, workpos_gate,
            *drivers, *teleops, coordinator]


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
