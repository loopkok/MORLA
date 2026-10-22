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
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _web_env(suffix: str, default: str) -> str:
    return os.environ.get(
        f"MORLA_WEB_MANAGER_{suffix}",
        os.environ.get(f"ASTRAL_WEB_MONITOR_{suffix}", default),
    )


def _default_web_dist() -> str:
    """Resolve the installed SPA bundle unless an explicit path is configured."""
    configured = _web_env("DIST", "").strip()
    if configured:
        return configured
    # Under colcon --symlink-install the installed asset files are symlinks
    # whose targets live outside the install share. Starlette's default
    # StaticFiles policy correctly rejects those paths, so serve the source
    # dist directly when the launch file resolves back to the source checkout.
    source_dist = Path(__file__).resolve().parents[1] / "web" / "dist"
    if (source_dist / "index.html").is_file():
        return str(source_dist)
    try:
        share = Path(get_package_share_directory("morla_web_manager"))
        return str(share / "web" / "dist")
    except Exception:
        # Keep source launch usable before colcon installation.
        return str(Path(__file__).resolve().parents[1] / "web" / "dist")


def _operator_inputs(context, web_node):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    voice = value("voice_source")
    if voice not in {"off", "host", "quest", "both"}:
        raise ValueError("voice_source must be off, host, quest or both")
    if voice == "off" and not value("pedal_device"):
        return []
    share = get_package_share_directory("morla_operator_control")
    arguments = {
        # The selected Profile owns the router, including across Profile restarts.
        "start_router": "false", "voice_source": voice,
        "pedal_device": value("pedal_device"),
        "microphone_device": value("microphone_device"),
        "dry_run": value("operator_dry_run"),
    }
    if value("operator_config"):
        arguments["config"] = value("operator_config")
    return [
        RegisterEventHandler(OnProcessExit(target_action=web_node, on_exit=[
            EmitEvent(event=Shutdown(reason="Web exited; stopping operator inputs")),
        ])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "operator_control.launch.py")),
            launch_arguments=arguments.items()),
    ]


def generate_launch_description() -> LaunchDescription:
    web_node = Node(
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
            "MORLA_QUEST_PREVIEW": LaunchConfiguration("quest_preview"),
        },
    )
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
                default_value=_default_web_dist(),
                description="Path to built frontend (web/dist). Empty = no SPA.",
            ),
            DeclareLaunchArgument("voice_source", default_value="off", description="off | host | quest | both"),
            DeclareLaunchArgument("quest_preview", default_value="false", description="Receive Quest tracking before a robot Profile starts"),
            DeclareLaunchArgument("microphone_device", default_value=""),
            DeclareLaunchArgument("pedal_device", default_value=""),
            DeclareLaunchArgument("operator_config", default_value=""),
            DeclareLaunchArgument("operator_dry_run", default_value="false", description="Recognize inputs without sending operator commands"),
            OpaqueFunction(function=_operator_inputs, args=[web_node]),
            web_node,
        ]
    )
