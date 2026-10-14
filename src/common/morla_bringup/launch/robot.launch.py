"""Unified MORLA robot entrypoint; dispatches one profile to its native stack."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from morla_core.profile import ProfileRegistry
from morla_bringup.robot_profile_launch import select_robot_stack


def _setup(context, *args, **kwargs):
    profile_id = LaunchConfiguration("profile_id").perform(context).strip()
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    sdk_backend = LaunchConfiguration("sdk_backend").perform(context).strip().lower()
    profile = ProfileRegistry.from_index().load(profile_id)
    stack = select_robot_stack(profile, mode, sdk_backend)
    share = get_package_share_directory(stack.package)
    launch_path = os.path.join(share, "launch", stack.launch_file)

    arguments = dict(stack.arguments)
    arguments.update({
        "with_input": LaunchConfiguration("with_input").perform(context),
        "allow_legacy_packets": LaunchConfiguration("allow_legacy_packets").perform(context),
    })
    if profile.robot_id == "nero" and stack.launch_file == "nero_xhand_stack.launch.py":
        arguments["enable_viewer"] = LaunchConfiguration("enable_viewer").perform(context)
        arguments["left_serial_port"] = LaunchConfiguration("left_serial_port").perform(context)
        arguments["right_serial_port"] = LaunchConfiguration("right_serial_port").perform(context)
    elif mode == "sim":
        arguments.update({
            "enable_viewer": LaunchConfiguration("enable_viewer").perform(context),
            "with_head_teleop": LaunchConfiguration("with_head_teleop").perform(context),
        })
    else:
        arguments.update({
            "with_video": LaunchConfiguration("with_video").perform(context),
            "with_head_teleop": LaunchConfiguration("with_head_teleop").perform(context),
        })

    print(
        f"[morla robot] profile={profile_id} robot={profile.robot_id} mode={mode} "
        f"stack={stack.package}/{stack.launch_file}"
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile_id", default_value="astral_left_gripper"),
        DeclareLaunchArgument("mode", default_value="sim", description="sim | real"),
        DeclareLaunchArgument("sdk_backend", default_value="fake", description="fake | hardware"),
        DeclareLaunchArgument("with_input", default_value="true"),
        DeclareLaunchArgument("allow_legacy_packets", default_value="false"),
        DeclareLaunchArgument("enable_viewer", default_value="false"),
        DeclareLaunchArgument("with_video", default_value="true"),
        DeclareLaunchArgument("with_head_teleop", default_value="true"),
        DeclareLaunchArgument("left_serial_port", default_value=""),
        DeclareLaunchArgument("right_serial_port", default_value=""),
        OpaqueFunction(function=_setup),
    ])
