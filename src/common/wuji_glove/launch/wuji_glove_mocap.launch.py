"""Launch Wuji Glove mocap → hand_landmarks/{left,right}.

Parameters mirror rob_station GLOVE_HAND_* where applicable.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("wuji_glove")
    default_config = os.path.join(pkg_share, "config", "wuji_glove.yaml")

    hand_side_arg = DeclareLaunchArgument(
        "hand_side",
        default_value=os.environ.get("GLOVE_HAND_SIDE", "right"),
        description="left|right|both (rob_station: GLOVE_HAND_SIDE)",
    )
    sn_arg = DeclareLaunchArgument(
        "sn",
        default_value=os.environ.get("GLOVE_HAND_SN", ""),
        description="Glove SN (rob_station: GLOVE_HAND_SN); empty=auto UDP scan",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="50.0",
        description="Publish rate Hz (rob_station loop-fps default 50)",
    )
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML params file",
    )

    node = Node(
        package="wuji_glove",
        executable="wuji_glove_mocap",
        name="wuji_glove_mocap",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config"),
            {
                "hand_side": LaunchConfiguration("hand_side"),
                "sn": ParameterValue(LaunchConfiguration("sn"), value_type=str),
                "publish_rate": ParameterValue(
                    LaunchConfiguration("publish_rate"), value_type=float
                ),
            },
        ],
    )

    return LaunchDescription([
        hand_side_arg,
        sn_arg,
        publish_rate_arg,
        config_arg,
        node,
    ])
