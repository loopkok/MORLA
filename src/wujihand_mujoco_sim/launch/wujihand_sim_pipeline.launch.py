"""Full offline pipeline: input → retarget → MuJoCo sim (no real hand).

Do not run together with wujihand_control/wujihand_real_pipeline on the same
joint_commands topic.

Usage:
  # Glove + official
  ros2 launch wujihand_mujoco_sim wujihand_sim_pipeline.launch.py \\
    input_source:=glove hand_side:=right retarget_backend:=official

  # Quest3 + wuji_retargeting
  ros2 launch wujihand_mujoco_sim wujihand_sim_pipeline.launch.py \\
    input_source:=quest3 hand_side:=right retarget_backend:=wuji_retargeting
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *args, **kwargs):
    glove_launch = os.path.join(
        get_package_share_directory("wuji_glove"),
        "launch",
        "wuji_glove_mocap.launch.py",
    )
    retarget_launch = os.path.join(
        get_package_share_directory("wujihand_retargeting"),
        "launch",
        "wujihand_retarget.launch.py",
    )
    sim_launch = os.path.join(
        get_package_share_directory("wujihand_mujoco_sim"),
        "launch",
        "wujihand_mujoco_sim.launch.py",
    )
    quest3_cfg = os.path.join(
        get_package_share_directory("quest3_hand_mocap"),
        "config",
        "quest3_mocap.yaml",
    )
    retarget_pkg = get_package_share_directory("wujihand_retargeting")

    input_source = LaunchConfiguration("input_source").perform(context)
    hand_side = LaunchConfiguration("hand_side")

    wuji_left = LaunchConfiguration("wuji_lib_config_left").perform(context).strip()
    wuji_right = LaunchConfiguration("wuji_lib_config_right").perform(context).strip()
    if input_source == "quest3":
        if not wuji_left:
            wuji_left = os.path.join(
                retarget_pkg, "config", "retarget_wuji_lib_quest3_left.yaml"
            )
        if not wuji_right:
            wuji_right = os.path.join(
                retarget_pkg, "config", "retarget_wuji_lib_quest3_right.yaml"
            )

    glove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(glove_launch),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("input_source"), "' == 'glove'"]
            )
        ),
        launch_arguments={
            "hand_side": hand_side,
            "sn": LaunchConfiguration("sn"),
        }.items(),
    )

    quest3 = Node(
        package="quest3_hand_mocap",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("input_source"), "' == 'quest3'"]
            )
        ),
        parameters=[
            quest3_cfg,
            {
                "protocol": LaunchConfiguration("quest3_protocol"),
                "udp_port": ParameterValue(
                    LaunchConfiguration("quest3_udp_port"), value_type=int
                ),
                "tcp_port": ParameterValue(
                    LaunchConfiguration("quest3_tcp_port"), value_type=int
                ),
                "arm_side": "both",
                "viz": False,
                "landmark_preprocess": "raw",
                "enable_xhand_pinky_adapt": False,
            },
        ],
    )

    retarget = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(retarget_launch),
        launch_arguments={
            "hand_side": hand_side,
            "retarget_backend": LaunchConfiguration("retarget_backend"),
            "hand_model": LaunchConfiguration("hand_model"),
            "wuji_lib_config_left": wuji_left,
            "wuji_lib_config_right": wuji_right,
        }.items(),
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "hand_side": hand_side,
            "enable_viewer": LaunchConfiguration("enable_viewer"),
        }.items(),
    )

    return [glove, quest3, retarget, sim]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_source",
            default_value="glove",
            description="glove | quest3 | none",
        ),
        DeclareLaunchArgument(
            "hand_side",
            default_value=os.environ.get("GLOVE_HAND_SIDE", "right"),
            description="left|right",
        ),
        DeclareLaunchArgument(
            "retarget_backend",
            default_value="official",
            description="official | wuji_retargeting | dexpilot",
        ),
        DeclareLaunchArgument(
            "hand_model",
            default_value=os.environ.get("GLOVE_HAND_MODEL", "wuji_hand"),
            description="official: wuji_hand | wuji_hand_2",
        ),
        DeclareLaunchArgument(
            "sn",
            default_value=os.environ.get("GLOVE_HAND_SN", ""),
            description="Glove SN",
        ),
        DeclareLaunchArgument("quest3_protocol", default_value="tcp_wired"),
        DeclareLaunchArgument("quest3_udp_port", default_value="9000"),
        DeclareLaunchArgument("quest3_tcp_port", default_value="8000"),
        DeclareLaunchArgument(
            "wuji_lib_config_left",
            default_value="",
            description="wuji_retargeting left yaml override",
        ),
        DeclareLaunchArgument(
            "wuji_lib_config_right",
            default_value="",
            description="wuji_retargeting right yaml override",
        ),
        DeclareLaunchArgument("enable_viewer", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
