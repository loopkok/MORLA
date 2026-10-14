"""Launch the morla_web_manager (ROS node + FastAPI web server).

The monitor runs as a single executable that hosts the rclpy node in a
background thread and uvicorn on the main thread. This launch file simply
starts that executable with optional parameter overrides.

Usage:
  ros2 launch morla_web_manager web_manager.launch.py
  # override host / port / dist path via env:
  MORLA_WEB_MANAGER_PORT=9090 ros2 launch morla_web_manager web_manager.launch.py
  # explicit LAN access:
  ros2 launch morla_web_manager web_manager.launch.py web_host:=0.0.0.0
"""
from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _web_env(suffix: str, default: str) -> str:
    return os.environ.get(
        f"MORLA_WEB_MANAGER_{suffix}",
        os.environ.get(f"ASTRAL_WEB_MONITOR_{suffix}", default),
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "web_port",
                default_value=_web_env("PORT", "8080"),
                description="Web UI / API port",
            ),
            DeclareLaunchArgument(
                "web_host",
                default_value=_web_env("HOST", "127.0.0.1"),
                description="Web UI bind host",
            ),
            DeclareLaunchArgument(
                "web_dist",
                default_value=_web_env("DIST", ""),
                description="Path to built frontend (web/dist). Empty = no SPA.",
            ),
            Node(
                package="morla_web_manager",
                executable="morla_web_manager",
                name="morla_web_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"web_port": LaunchConfiguration("web_port")},
                    {"web_host": LaunchConfiguration("web_host")},
                    {"web_dist": LaunchConfiguration("web_dist")},
                ],
                additional_env={
                    "MORLA_WEB_MANAGER_PORT": LaunchConfiguration("web_port"),
                    "MORLA_WEB_MANAGER_HOST": LaunchConfiguration("web_host"),
                    "MORLA_WEB_MANAGER_DIST": LaunchConfiguration("web_dist"),
                },
            ),
        ]
    )
