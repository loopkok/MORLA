"""Launch one Nero IK teleop publisher with its side profile parameters."""

import os
import json

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
    alignment_text = LaunchConfiguration("input_alignment").perform(context).strip()
    extra = {}
    if alignment_text:
        try:
            alignment = json.loads(alignment_text)
        except json.JSONDecodeError as exc:
            raise RuntimeError("input_alignment must be a JSON array of 9 numbers") from exc
        if not isinstance(alignment, list) or len(alignment) != 9:
            raise RuntimeError("input_alignment must be a JSON array of 9 numbers")
        extra["vr_to_arm_rot_after_unity_to_robot"] = alignment
    contract_text = LaunchConfiguration("kinematics_hash").perform(context).strip()
    if contract_text:
        extra["kinematics_hash"] = contract_text
    contract_version = LaunchConfiguration("coordinate_contract_version").perform(context).strip()
    if contract_version:
        extra["coordinate_contract_version"] = int(contract_version)
    tool_text = LaunchConfiguration("tcp_offset").perform(context).strip()
    if tool_text:
        extra["tcp_offset"] = json.loads(tool_text)
    transform_text = LaunchConfiguration("solver_tip_to_tcp_transform").perform(context).strip()
    if transform_text:
        transform = json.loads(transform_text)
        if not isinstance(transform, list) or len(transform) != 16:
            raise RuntimeError("solver_tip_to_tcp_transform must be a JSON array of 16 numbers")
        extra["solver_tip_to_tcp_transform"] = transform
        extra["use_solver_tip_to_tcp_transform"] = True
    for name in ("solver_base_frame", "solver_tip_frame", "solver_adapter_id", "tool_frame"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            extra[name] = value
    return [
        Node(
            package="nero_arm_teleop",
            executable="nero_arm_teleop_node",
            name="nero_arm_teleop",
            namespace=f"/{side}_arm",
            output="screen",
            parameters=[config, extra],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_side", default_value="left"),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("input_alignment", default_value=""),
            DeclareLaunchArgument("kinematics_hash", default_value=""),
            DeclareLaunchArgument("coordinate_contract_version", default_value=""),
            DeclareLaunchArgument("tcp_offset", default_value=""),
            DeclareLaunchArgument("solver_tip_to_tcp_transform", default_value=""),
            DeclareLaunchArgument("solver_base_frame", default_value=""),
            DeclareLaunchArgument("solver_tip_frame", default_value=""),
            DeclareLaunchArgument("solver_adapter_id", default_value=""),
            DeclareLaunchArgument("tool_frame", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
