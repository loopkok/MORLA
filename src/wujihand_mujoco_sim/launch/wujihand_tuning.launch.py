"""Launch Wuji Hand tuning viewer with selectable input + retarget backend.

Input (same topic hand_landmarks/{side}):
  input_source:=glove   — wuji_glove_mocap
  input_source:=quest3  — quest3_udp_mocap
  input_source:=none    — external publisher already running

Retarget (Wuji Hand MJCF):
  retarget_backend:=wuji_retargeting — 3-layer TuningViewer + YAML hot-reload
  retarget_backend:=official         — RetargetSession → mesh
  retarget_backend:=dexpilot         — DexPilot → mesh

Examples:
  ros2 launch wujihand_mujoco_sim wujihand_tuning.launch.py \\
    input_source:=quest3 hand_side:=right retarget_backend:=wuji_retargeting

  ros2 launch wujihand_mujoco_sim wujihand_tuning.launch.py \\
    input_source:=glove retarget_backend:=official
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
    sim_pkg = get_package_share_directory("wujihand_mujoco_sim")
    quest3_cfg = os.path.join(
        get_package_share_directory("quest3_hand_mocap"),
        "config",
        "quest3_mocap.yaml",
    )
    glove_launch = os.path.join(
        get_package_share_directory("wuji_glove"),
        "launch",
        "wuji_glove_mocap.launch.py",
    )

    hand_side = LaunchConfiguration("hand_side").perform(context)
    input_source = LaunchConfiguration("input_source").perform(context)
    backend = LaunchConfiguration("retarget_backend")

    # Quest3 needs its own yaml (no glove mediapipe_rotation; snap MCP).
    # Explicit retarget_config:= still wins.
    retarget_config = LaunchConfiguration("retarget_config").perform(context).strip()
    if not retarget_config and input_source == "quest3":
        retarget_pkg = get_package_share_directory("wujihand_retargeting")
        retarget_config = os.path.join(
            retarget_pkg,
            "config",
            f"retarget_wuji_lib_quest3_{hand_side}.yaml",
        )

    glove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(glove_launch),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("input_source"), "' == 'glove'"]
            )
        ),
        launch_arguments={
            "hand_side": LaunchConfiguration("hand_side"),
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
                # Match glove: Retargeter applies MANO once; avoid double-transform.
                "landmark_preprocess": "raw",
                "enable_xhand_pinky_adapt": False,
            },
        ],
    )

    tuning = Node(
        package="wujihand_mujoco_sim",
        executable="wujihand_tuning_node",
        name="wujihand_tuning_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "hand_side": LaunchConfiguration("hand_side"),
                "retarget_backend": backend,
                "hand_model": LaunchConfiguration("hand_model"),
                "retarget_config": retarget_config,
                "dexpilot_config": ParameterValue(
                    LaunchConfiguration("dexpilot_config"), value_type=str
                ),
                "viz_config": ParameterValue(
                    LaunchConfiguration("viz_config"), value_type=str
                ),
                "publish_joint_commands": ParameterValue(
                    LaunchConfiguration("publish_joint_commands"), value_type=bool
                ),
                "nlopt_max_eval": ParameterValue(
                    LaunchConfiguration("nlopt_max_eval"), value_type=int
                ),
            },
        ],
    )

    return [glove, quest3, tuning]


def generate_launch_description():
    sim_pkg = get_package_share_directory("wujihand_mujoco_sim")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_source",
            default_value="glove",
            description="glove | quest3 | none",
        ),
        DeclareLaunchArgument(
            "hand_side",
            default_value=os.environ.get("GLOVE_HAND_SIDE", "right"),
            description="left|right (which hand to retarget/view)",
        ),
        DeclareLaunchArgument(
            "retarget_backend",
            default_value="wuji_retargeting",
            description="official | wuji_retargeting | dexpilot",
        ),
        DeclareLaunchArgument(
            "hand_model",
            default_value=os.environ.get("GLOVE_HAND_MODEL", "wuji_hand"),
            description="official only: wuji_hand | wuji_hand_2",
        ),
        DeclareLaunchArgument(
            "sn",
            default_value=os.environ.get("GLOVE_HAND_SN", ""),
            description="Glove SN (input_source:=glove)",
        ),
        DeclareLaunchArgument(
            "quest3_protocol",
            default_value="tcp_wired",
            description="quest3: tcp_wired | udp | ...",
        ),
        DeclareLaunchArgument("quest3_udp_port", default_value="9000"),
        DeclareLaunchArgument("quest3_tcp_port", default_value="8000"),
        DeclareLaunchArgument(
            "retarget_config",
            default_value="",
            description=(
                "wuji_retargeting yaml override; empty + input_source:=quest3 "
                "uses retarget_wuji_lib_quest3_{side}.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "dexpilot_config",
            default_value="",
            description="dexpilot yaml override",
        ),
        DeclareLaunchArgument(
            "viz_config",
            default_value=os.path.join(sim_pkg, "config", "tuning_viz.yaml"),
        ),
        DeclareLaunchArgument("publish_joint_commands", default_value="true"),
        DeclareLaunchArgument("nlopt_max_eval", default_value="25"),
        OpaqueFunction(function=_setup),
    ])
