"""Launch Astral dual-arm MuJoCo sim only."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("astral_mujoco_sim")
    cfg = os.path.join(pkg, "config", "astral_mujoco_sim.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_viewer", default_value="true"),
            Node(
                package="astral_mujoco_sim",
                executable="astral_mujoco_sim_node",
                name="astral_mujoco_sim_node",
                output="screen",
                parameters=[
                    cfg,
                    {
                        "enable_viewer": ParameterValue(
                            LaunchConfiguration("enable_viewer"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
