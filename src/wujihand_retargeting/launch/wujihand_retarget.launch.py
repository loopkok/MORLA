"""Launch Wuji Hand retargeting node.

Defaults align with rob_station GLOVE_HAND_* for the official backend.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("wujihand_retargeting")
    default_config = os.path.join(pkg, "config", "wujihand_retarget_params.yaml")

    hand_side_arg = DeclareLaunchArgument(
        "hand_side",
        default_value=os.environ.get("GLOVE_HAND_SIDE", "right"),
        description="left|right|both (rob_station: GLOVE_HAND_SIDE)",
    )
    backend_arg = DeclareLaunchArgument(
        "retarget_backend",
        default_value="official",
        description="official | wuji_retargeting | dexpilot",
    )
    nlopt_arg = DeclareLaunchArgument(
        "nlopt_max_eval",
        default_value="25",
        description="Only for wuji_retargeting; 0=library default",
    )
    model_arg = DeclareLaunchArgument(
        "hand_model",
        default_value=os.environ.get("GLOVE_HAND_MODEL", "wuji_hand"),
        description="rob_station GLOVE_HAND_MODEL: wuji_hand | wuji_hand_2",
    )
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML params file",
    )
    wuji_lib_left_arg = DeclareLaunchArgument(
        "wuji_lib_config_left",
        default_value="",
        description="wuji_retargeting yaml for left (empty=retarget_wuji_lib_left.yaml)",
    )
    wuji_lib_right_arg = DeclareLaunchArgument(
        "wuji_lib_config_right",
        default_value="",
        description="wuji_retargeting yaml for right (empty=retarget_wuji_lib_right.yaml)",
    )
    smoothing_arg = DeclareLaunchArgument(
        "smoothing_alpha",
        default_value="1.0",
        description="Post-retarget EMA; 1.0=off (rob_station official default)",
    )

    node = Node(
        package="wujihand_retargeting",
        executable="wujihand_retarget_node",
        name="wujihand_retarget_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config"),
            {
                "hand_side": LaunchConfiguration("hand_side"),
                "retarget_backend": LaunchConfiguration("retarget_backend"),
                "hand_model": LaunchConfiguration("hand_model"),
                "wuji_lib_config_left": LaunchConfiguration("wuji_lib_config_left"),
                "wuji_lib_config_right": LaunchConfiguration("wuji_lib_config_right"),
                "smoothing_alpha": ParameterValue(
                    LaunchConfiguration("smoothing_alpha"), value_type=float
                ),
                "nlopt_max_eval": ParameterValue(
                    LaunchConfiguration("nlopt_max_eval"), value_type=int
                ),
            },
        ],
    )

    return LaunchDescription([
        hand_side_arg,
        backend_arg,
        model_arg,
        config_arg,
        wuji_lib_left_arg,
        wuji_lib_right_arg,
        smoothing_arg,
        nlopt_arg,
        node,
    ])
