"""Launch the astral_web_monitor (ROS node + FastAPI web server).

The monitor runs as a single executable that hosts the rclpy node in a
background thread and uvicorn on the main thread. This launch file simply
starts that executable with optional parameter overrides.

Usage:
  ros2 launch astral_web_monitor web_monitor.launch.py
  # override host / port / dist path via env:
  ASTRAL_WEB_MONITOR_PORT=9090 ros2 launch astral_web_monitor web_monitor.launch.py
  # explicit LAN access:
  ros2 launch astral_web_monitor web_monitor.launch.py web_host:=0.0.0.0
"""
from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "web_port",
                default_value=os.environ.get("ASTRAL_WEB_MONITOR_PORT", "8080"),
                description="Web UI / API port",
            ),
            DeclareLaunchArgument(
                "web_host",
                default_value=os.environ.get("ASTRAL_WEB_MONITOR_HOST", "127.0.0.1"),
                description="Web UI bind host",
            ),
            DeclareLaunchArgument(
                "web_dist",
                default_value=os.environ.get("ASTRAL_WEB_MONITOR_DIST", ""),
                description="Path to built frontend (web/dist). Empty = no SPA.",
            ),
            Node(
                package="astral_web_monitor",
                executable="astral_web_monitor",
                name="astral_web_monitor",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"web_port": LaunchConfiguration("web_port")},
                    {"web_host": LaunchConfiguration("web_host")},
                    {"web_dist": LaunchConfiguration("web_dist")},
                ],
                additional_env={
                    "ASTRAL_WEB_MONITOR_PORT": LaunchConfiguration("web_port"),
                    "ASTRAL_WEB_MONITOR_HOST": LaunchConfiguration("web_host"),
                    "ASTRAL_WEB_MONITOR_DIST": LaunchConfiguration("web_dist"),
                },
            ),
        ]
    )
