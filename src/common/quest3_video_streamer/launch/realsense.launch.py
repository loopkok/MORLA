"""Launch a single RealSense D435i color stream -> Quest 3 WebRTC push.

All tunables live in ``config/params.yaml``. This launch loads that yaml and
overrides the ``cameras`` list to ``["d435i"]`` so only the D435i streams. The
D435i source mode (v4l2 direct vs ros node) is read from the yaml
(``d435i.source``); realsense2_camera_node is started only in ``ros`` mode.

Usage:
    ros2 launch quest3_video_streamer realsense.launch.py
    ros2 launch quest3_video_streamer realsense.launch.py d435i_source:=ros
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PRESET_MAP = {
    "480p": (640, 480, 60), "480p30": (640, 480, 30),
    "720p": (1280, 720, 60), "720p30": (1280, 720, 30),
    "1080p": (1920, 1080, 60), "1080p30": (1920, 1080, 30),
}


def _load_params() -> dict:
    path = os.path.join(
        get_package_share_directory("quest3_video_streamer"), "config", "params.yaml"
    )
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data.get("quest3_video_streamer", {}).get("ros__parameters", data)


def _camera_field(params: dict, name: str, field: str, default):
    block = params.get(name, {})
    if isinstance(block, dict) and field in block:
        return block[field]
    return default


def generate_launch_description():
    signaling_port = LaunchConfiguration("signaling_port")
    enable_mocap_tcp = LaunchConfiguration("enable_mocap_tcp")
    verbose = LaunchConfiguration("verbose")
    d435i_source_arg = LaunchConfiguration("d435i_source")

    params_file = os.path.join(
        get_package_share_directory("quest3_video_streamer"), "config", "params.yaml"
    )

    def _build(context: LaunchContext):
        overrides = {
            "cameras": ["d435i"],  # single-camera mode
        }
        sp = signaling_port.perform(context)
        em = enable_mocap_tcp.perform(context)
        vb = verbose.perform(context)
        overrides["signaling_port"] = int(sp)
        overrides["enable_mocap_tcp"] = em.lower() == "true"
        overrides["verbose"] = vb.lower() == "true"

        cli_src = d435i_source_arg.perform(context)
        yaml_params = _load_params()
        if cli_src:
            d435i_src = cli_src
            overrides["d435i.source"] = cli_src
        else:
            d435i_src = str(_camera_field(yaml_params, "d435i", "source", "v4l2"))

        nodes = []
        if d435i_src != "v4l2":
            preset = str(_camera_field(yaml_params, "d435i", "preset", "1080p30"))
            w, h, fps = _PRESET_MAP.get(preset, (1920, 1080, 30))
            nodes.append(Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="camera",
                namespace="camera",
                parameters=[{
                    "enable_color": True,
                    "enable_depth": False,
                    "enable_infra1": False,
                    "enable_infra2": False,
                    "enable_infra": False,
                    "rgb_camera.color_format": "RGB8",
                    "rgb_camera.color_profile": f"{w}x{h}x{fps}",
                }],
                output="screen",
            ))

        nodes.append(Node(
            package="quest3_video_streamer",
            executable="quest3_video_streamer",
            name="quest3_video_streamer",
            parameters=[params_file, overrides],
            output="screen",
        ))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument("signaling_port", default_value="8765"),
        DeclareLaunchArgument("enable_mocap_tcp", default_value="false"),
        DeclareLaunchArgument("verbose", default_value="false"),
        DeclareLaunchArgument("d435i_source", default_value="",
                              description="Override d435i source: v4l2 | ros (empty = use yaml)"),
        GroupAction([OpaqueFunction(function=_build)]),
    ])
