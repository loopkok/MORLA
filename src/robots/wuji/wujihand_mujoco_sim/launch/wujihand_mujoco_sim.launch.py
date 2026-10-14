"""Launch Wuji Hand MuJoCo sim (subscribes to /{hand}/joint_commands).

Stand-in for the real hand driver while validating the teleop pipeline offline.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("wujihand_mujoco_sim")
    default_config = os.path.join(pkg, "config", "wujihand_mujoco_sim.yaml")

    hand_side_arg = DeclareLaunchArgument(
        "hand_side",
        default_value=os.environ.get("GLOVE_HAND_SIDE", "right"),
        description="left|right",
    )
    enable_viewer_arg = DeclareLaunchArgument(
        "enable_viewer",
        default_value="true",
        description="Open MuJoCo passive viewer",
    )
    realtime_arg = DeclareLaunchArgument(
        "realtime",
        default_value="true",
        description="Sleep model.opt.timestep each step",
    )
    mjcf_path_arg = DeclareLaunchArgument(
        "mjcf_path",
        default_value="",
        description="Override MJCF path; empty uses package assets",
    )
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML params file",
    )

    node = Node(
        package="wujihand_mujoco_sim",
        executable="wujihand_mujoco_sim_node",
        name="wujihand_mujoco_sim_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config"),
            {
                "hand_side": LaunchConfiguration("hand_side"),
                "mjcf_path": ParameterValue(
                    LaunchConfiguration("mjcf_path"), value_type=str
                ),
                "enable_viewer": ParameterValue(
                    LaunchConfiguration("enable_viewer"), value_type=bool
                ),
                "realtime": ParameterValue(
                    LaunchConfiguration("realtime"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription([
        hand_side_arg,
        enable_viewer_arg,
        realtime_arg,
        mjcf_path_arg,
        config_arg,
        node,
    ])
