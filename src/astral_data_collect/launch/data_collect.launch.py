"""启动数据采集：采集节点 + 键盘控制器。

前置条件：
  1. 遥操作链路已运行（astral_teleop / astral_robot_control / ...）
  2. quest3_video_streamer 已运行且 collect_tap: true（默认开）

用法：
  ros2 launch astral_data_collect data_collect.launch.py \
      session:=pick_place end_effector_right:=wuji

参数来源约定（重要）：
  config/data_collect.yaml 是所有节点参数的唯一默认值来源。launch 参数
  默认空串 = 不覆盖；只有显式传入（CLI `xxx:=` 或 web 预设 args）才盖掉
  yaml 同名字段。此前 launch 默认值（如 arms 默认 "left,right"）会静默
  盖掉 yaml——改了 yaml 却采出旧 schema，即此机制所致。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


# 可被 launch 覆盖的节点参数（值一律空串默认；schema 相关在 yaml 里配置）
_OVERRIDABLE = [
    "session", "save_root",
    "arms", "end_effector_left", "end_effector_right",
    "include_waist", "include_head",
    "cameras", "camera_topic_prefix",
    "dataset_fps", "action_source", "hold_frames", "max_gap_ms",
    "jpeg_quality", "default_task",
]


def _build_nodes(context):
    share = get_package_share_directory("astral_data_collect")
    yaml_path = os.path.join(share, "config", "data_collect.yaml")

    overrides = {}
    for key in _OVERRIDABLE:
        val = LaunchConfiguration(key).perform(context).strip()
        if val:
            overrides[key] = val

    collect_node = Node(
        package="astral_data_collect",
        executable="data_collect_node",
        name="data_collect",
        output="screen",
        parameters=[yaml_path, overrides],
    )
    keyboard_node = Node(
        package="astral_data_collect",
        executable="keyboard_controller",
        name="data_collect_keyboard",
        output="screen",
        condition=IfCondition(LaunchConfiguration("keyboard")),
    )
    # VR 采集控制：右手柄 A=start / B=stop&save / 摇杆按下=discard（手不离手柄）。
    # 需遥操链路里的 quest3_hand_mocap 在发布 quest3/right_controller_joy；
    # mocap 没跑则收不到帧、自然无动作。与 web 卡片/键盘控制器并存。
    vr_control_node = Node(
        package="astral_data_collect",
        executable="vr_collect_control",
        name="data_collect_vr_control",
        output="screen",
        condition=IfCondition(LaunchConfiguration("vr_control")),
    )
    return [collect_node, keyboard_node, vr_control_node]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            key, default_value="",
            description="留空=用 config/data_collect.yaml 的值；显式传入才覆盖",
        )
        for key in _OVERRIDABLE
    ]
    args.append(
        DeclareLaunchArgument(
            "keyboard", default_value="true",
            description="是否同启键盘控制器节点",
        )
    )
    args.append(
        DeclareLaunchArgument(
            "vr_control", default_value="true",
            description="是否同启 VR 采集控制节点（右手柄：A=start / B=stop / 摇杆按下=discard）",
        )
    )
    return LaunchDescription([*args, OpaqueFunction(function=_build_nodes)])
