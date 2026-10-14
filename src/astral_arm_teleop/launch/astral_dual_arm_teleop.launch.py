"""Dual Astral arm teleop — 2× arm_node (+ optional driver).

  quest3_udp_mocap          (optional, with_mocap:=true)
  astral_arm_teleop_arm ×2
  astral_robot_control driver (optional, with_driver:=true)

Arm-only. Gripper / dexterous hand are composed by astral_teleop, not here.
Solver / protocol / convert_to_robot come from yaml unless you pass a launch override.

Usage:
  ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py dry_run:=true
  ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py \\
    with_driver:=true control_board_ip:=192.168.10.2
"""

from __future__ import annotations

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 按侧拆分共享日志基路径 + run 目录（生产路径用包内已单测的 side_log_path/run_log_dir）。
# 裸路径直接 `ros2 launch <launch 文件路径>` 时包不在 sys.path、导入会失败——
# 内联同逻辑兜底，避免 launch 解析期报 ModuleNotFoundError。
try:
    from astral_arm_teleop.teleop_log import run_log_dir, side_log_path
except ImportError:  # pragma: no cover - 生产安装路径走上面的 import
    def side_log_path(base, side):
        b = (base or "").strip()
        if not b:
            return ""
        if b.endswith("_left.jsonl") or b.endswith("_right.jsonl"):
            return b
        if b.endswith(".jsonl"):
            return f"{b[:-6]}_{side}.jsonl"
        return f"{b}_{side}.jsonl"

    def run_log_dir(root, tag=""):
        root = (root or "").strip()
        if not root:
            return ""
        tag = (tag or "").strip().replace("/", "_").replace(" ", "_")
        stamp = time.strftime("%Y%m%d-%H%M%S")
        name = f"{stamp}_{tag}" if tag else stamp
        d = os.path.join(root, name)
        os.makedirs(d, exist_ok=True)
        return d


def _opt(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _launch_setup(context, *args, **kwargs):
    teleop_pkg = get_package_share_directory("astral_arm_teleop")
    control_pkg = get_package_share_directory("astral_robot_control")
    quest_pkg = get_package_share_directory("quest3_hand_mocap")
    cfg_l = os.path.join(teleop_pkg, "config", "astral_arm_teleop_left.yaml")
    cfg_r = os.path.join(teleop_pkg, "config", "astral_arm_teleop_right.yaml")
    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")
    drivers_launch = os.path.join(control_pkg, "launch", "astral_drivers.launch.py")

    teleop_extra = {
        "dry_run": _opt(context, "dry_run").lower() in ("true", "1", "yes"),
    }
    rsig = _opt(context, "require_start_signal")
    if rsig:
        teleop_extra["require_start_signal"] = rsig.lower() in ("true", "1", "yes")
    solver_type = _opt(context, "solver_type")
    if solver_type:
        teleop_extra["solver_type"] = solver_type
    urdf_path = _opt(context, "urdf_path")
    if urdf_path:
        teleop_extra["urdf_path"] = urdf_path

    arm_side = _opt(context, "arm_side").lower() or "both"
    if arm_side not in ("left", "right", "both"):
        raise RuntimeError("arm_side must be left|right|both")
    want_l = arm_side in ("left", "both")
    want_r = arm_side in ("right", "both")

    # 遥操诊断 JSONL 日志（可选）：给共享基路径按侧拆 _left/_right（见 teleop_log.py）。
    # log_dir 模式：每次 launch 自动建 {log_dir}/{stamp}_{tag}/ 运行目录，基路径放里面
    # （不再重复写同一个 /tmp 文件、重启即丢）。显式 teleop_log_file 优先于 log_dir。
    tlog = _opt(context, "teleop_log_file")
    log_dir = _opt(context, "log_dir")
    driver_log = _opt(context, "driver_log_file")
    if not tlog and log_dir:
        run_dir = run_log_dir(log_dir, _opt(context, "log_tag"))
        if run_dir:
            tlog = os.path.join(run_dir, "teleop_teleop.jsonl")
            # 驱动层日志同运行目录（抓"电机抽"：cmd/send/state/srv/spike）
            if not driver_log:
                driver_log = os.path.join(run_dir, "driver.jsonl")

    mocap_extra = {"arm_side": arm_side}
    protocol = _opt(context, "protocol")
    if protocol:
        mocap_extra["protocol"] = protocol
    convert = _opt(context, "convert_to_robot")
    if convert:
        mocap_extra["convert_to_robot"] = convert.lower() in ("true", "1", "yes")

    with_mocap = _opt(context, "with_mocap").lower() in ("true", "1", "yes")
    actions = []
    if with_mocap:
        actions.append(
            Node(
                package="quest3_hand_mocap",
                executable="quest3_udp_mocap",
                name="quest3_udp_mocap",
                output="screen",
                parameters=[quest_cfg, mocap_extra],
            )
        )

    if want_l:
        left_extra = dict(teleop_extra)
        if tlog:
            left_extra["teleop_log_file"] = side_log_path(tlog, "left")
        actions.extend(
            [
                Node(
                    package="astral_arm_teleop",
                    executable="ik_solver_node",
                    name="ik_solver_left",
                    output="screen",
                    parameters=[{"arm_side": "left", "solver_type": "analytic_dh"}],
                ),
                Node(
                    package="astral_arm_teleop",
                    executable="astral_arm_teleop_node",
                    name="astral_arm_teleop_left",
                    output="screen",
                    parameters=[cfg_l, left_extra],
                ),
            ]
        )
    if want_r:
        right_extra = dict(teleop_extra)
        if tlog:
            right_extra["teleop_log_file"] = side_log_path(tlog, "right")
        actions.extend(
            [
                Node(
                    package="astral_arm_teleop",
                    executable="ik_solver_node",
                    name="ik_solver_right",
                    output="screen",
                    parameters=[{"arm_side": "right", "solver_type": "analytic_dh"}],
                ),
                Node(
                    package="astral_arm_teleop",
                    executable="astral_arm_teleop_node",
                    name="astral_arm_teleop_right",
                    output="screen",
                    parameters=[cfg_r, right_extra],
                ),
            ]
        )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drivers_launch),
            condition=IfCondition(LaunchConfiguration("with_driver")),
            launch_arguments={
                "dry_run": LaunchConfiguration("dry_run"),
                "control_board_ip": LaunchConfiguration("control_board_ip"),
                "driver_log_file": driver_log,
            }.items(),
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "with_mocap",
                default_value="true",
                description="Start quest3_udp_mocap (false when a parent launch owns it)",
            ),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument(
                "arm_side",
                default_value="both",
                description="left | right | both — which arm teleop nodes to start",
            ),
            DeclareLaunchArgument(
                "protocol",
                default_value="",
                description="empty → quest3_mocap.yaml",
            ),
            DeclareLaunchArgument(
                "convert_to_robot",
                default_value="",
                description="empty → yaml",
            ),
            DeclareLaunchArgument(
                "with_driver",
                default_value="false",
                description="Also launch astral_robot_control driver",
            ),
            DeclareLaunchArgument(
                "control_board_ip",
                default_value=os.environ.get("ASTRAL_BOARD_IP", "192.168.10.2"),
            ),
            DeclareLaunchArgument(
                "solver_type",
                default_value="",
                description="empty → astral_arm_teleop_{left,right}.yaml",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value="",
                description="empty → yaml / astral_robot.pin.urdf",
            ),
            DeclareLaunchArgument(
                "require_start_signal",
                default_value="",
                description=(
                    "empty → yaml. true → do not auto-capture vr_init/arm on first "
                    "VR pose; wait for /teleop/start (or ~/start) to capture the zero "
                    "from the current pose and arm."
                ),
            ),
            DeclareLaunchArgument(
                "teleop_log_file",
                default_value="",
                description=(
                    "empty → off (or log_dir). Non-empty shared base path → per-side "
                    "JSONL diagnostics log (loop/wrist/state/body/metrics records), "
                    "e.g. /tmp/teleop_teleop.jsonl → ..._left/_right.jsonl."
                ),
            ),
            DeclareLaunchArgument(
                "driver_log_file",
                default_value="",
                description=(
                    "empty → off (or log_dir). Non-empty JSONL path for the driver "
                    "diagnostics log (cmd/send/state/srv/spike records — 抓电机抽). "
                    "log_dir mode derives {run_dir}/driver.jsonl automatically."
                ),
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="",
                description=(
                    "empty → off. Non-empty root dir → create a per-run stamped "
                    "directory {log_dir}/{YYYYMMDD-HHMMSS}[_tag]/ and write the "
                    "per-side teleop JSONL logs there (avoids reusing one shared "
                    "file across runs / losing logs in /tmp on reboot). "
                    "teleop_log_file, when set, wins over this."
                ),
            ),
            DeclareLaunchArgument(
                "log_tag",
                default_value="",
                description="Event name appended to the run directory (log_dir mode).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
