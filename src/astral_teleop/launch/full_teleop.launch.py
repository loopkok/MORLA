"""Whole-robot teleop: dual arms + left gripper + right (Wuji hand or gripper).

One Quest3 mocap. Right hand input is selectable:
  quest3  — Wuji dexterous hand from Quest3 landmarks
  glove   — Wuji dexterous hand from Wuji Glove
  gripper — right pinch → right gripper (no dexterous hand)
  none    — no right-hand device

Video return (quest3_video_streamer) is launched by default (with_video:=true):
robot USB cameras → Quest 3 panels over WebRTC. The launch auto-runs
``adb reverse tcp:8000 tcp:8000`` (mocap) and ``adb reverse tcp:8765 tcp:8765``
(video signaling) when adb is available; you still need to enable the video
feed in the Quest app. Which cameras stream is configured in
quest3_video_streamer/config/params.yaml (default: wrist_left + wrist_right,
no RealSense) and can be gated at runtime (see that package's README).

Usage:
  # Quest3 wrists + left pinch + Quest3 right dexterous hand
  ros2 launch astral_teleop full_teleop.launch.py \\
    with_arm_driver:=true with_hand_driver:=true \\
    right_hand_source:=quest3 retarget_backend:=wuji_retargeting

  # Same, but right Wuji from glove
  ros2 launch astral_teleop full_teleop.launch.py \\
    with_arm_driver:=true with_hand_driver:=true \\
    right_hand_source:=glove

  # Dual grippers: left pinch → left gripper, right pinch → right gripper
  ros2 launch astral_teleop full_teleop.launch.py \\
    with_arm_driver:=true with_hand_driver:=false \\
    right_hand_source:=gripper with_gripper:=true

  # Left arm + left gripper only (no right-arm teleop)
  ros2 launch astral_teleop full_teleop.launch.py \\
    with_arm_driver:=true with_hand_driver:=false \\
    right_hand_source:=none with_gripper:=true arm_side:=left
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _opt(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _adb_reverse(ports: list[int]) -> None:
    """Best-effort `adb reverse` for the wired Quest link.

    The Quest app reaches host services through these reversals:
      tcp:8000 — mocap telemetry (quest3_hand_mocap TCP server)
      tcp:8765 — video signaling (quest3_video_streamer WebSocket)
    Non-fatal: missing adb / no device just logs a warning (wireless mode or
    reversal already done manually both keep working).
    """
    import shutil
    import subprocess

    if shutil.which("adb") is None:
        print("[full_teleop] video/mocap: adb not found; skipping adb reverse "
              "(wireless mode or run it manually: adb reverse tcp:8000 tcp:8000; "
              "adb reverse tcp:8765 tcp:8765)")
        return
    for port in ports:
        try:
            res = subprocess.run(
                ["adb", "reverse", f"tcp:{port}", f"tcp:{port}"],
                capture_output=True, text=True, timeout=5.0,
            )
            if res.returncode == 0:
                print(f"[full_teleop] adb reverse tcp:{port} OK")
            else:
                err = (res.stderr or res.stdout or "").strip()
                print(f"[full_teleop] WARN adb reverse tcp:{port} failed: {err}")
        except Exception as exc:  # timeout etc.
            print(f"[full_teleop] WARN adb reverse tcp:{port} error: {exc}")


def _setup(context, *args, **kwargs):
    quest_pkg = get_package_share_directory("quest3_hand_mocap")
    teleop_pkg = get_package_share_directory("astral_arm_teleop")
    retarget_pkg = get_package_share_directory("wujihand_retargeting")
    glove_pkg = get_package_share_directory("wuji_glove")
    hand_ctrl_pkg = get_package_share_directory("wujihand_control")

    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")
    right_src = _opt(context, "right_hand_source").lower()
    if right_src not in ("quest3", "glove", "gripper", "none"):
        raise RuntimeError("right_hand_source must be quest3|glove|gripper|none")

    backend = _opt(context, "retarget_backend")
    if not backend:
        backend = "wuji_retargeting" if right_src == "quest3" else "official"

    wuji_right = _opt(context, "wuji_lib_config_right")
    if not wuji_right and right_src == "quest3" and backend == "wuji_retargeting":
        wuji_right = os.path.join(
            retarget_pkg, "config", "retarget_wuji_lib_quest3_right.yaml"
        )

    arm_side = _opt(context, "arm_side").lower() or "both"
    if arm_side not in ("left", "right", "both"):
        raise RuntimeError("arm_side must be left|right|both")

    mocap_extra = {
        "arm_side": arm_side,
        "viz": False,
        "landmark_preprocess": "raw",
        "enable_xhand_pinky_adapt": False,
        "publish_landmarks_left": True,
        "publish_landmarks_right": right_src != "glove",
    }
    protocol = _opt(context, "protocol")
    if protocol:
        mocap_extra["protocol"] = protocol
    convert = _opt(context, "convert_to_robot")
    if convert:
        mocap_extra["convert_to_robot"] = convert.lower() in ("true", "1", "yes")

    mocap = Node(
        package="quest3_hand_mocap",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        emulate_tty=True,
        parameters=[quest_cfg, mocap_extra],
    )

    # Left Touch controller grip button (mask bit 5) → /teleop/start external
    # start gate. Rising-edge only; re-press re-captures vr_init (re-center).
    start_gate = Node(
        package="astral_teleop",
        executable="controller_start_gate",
        name="controller_start_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{"joy_topic": "quest3/left_controller_joy", "button_index": 5}],
    )

    # Left Touch controller X button (primary, mask bit 0) → 段间回位：停止跟随
    # VR 并直达回到工作位（/teleop/disarm + /teleop/init_direct，不经途径点）。
    # 与 start_gate 对称；录制中（RECORDING/PAUSED/SAVING）由闸门自行忽略。
    workpos_gate = Node(
        package="astral_teleop",
        executable="controller_workpos_gate",
        name="controller_workpos_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{"joy_topic": "quest3/left_controller_joy", "button_index": 0}],
    )

    # Right Touch thumbstick → head yaw/pitch (absolute, spring-return).
    # Reads /head/joint_states for head_init at start; publishes to the
    # existing /head/joint_commands that astral_robot_control consumes.
    astral_teleop_pkg = get_package_share_directory("astral_teleop")
    head_cfg = os.path.join(astral_teleop_pkg, "config", "head_teleop.yaml")
    head_extra = {}
    req_start = _opt(context, "require_start_signal")
    if req_start:
        head_extra["require_start_signal"] = req_start.lower() in ("true", "1", "yes")
    head_teleop = Node(
        package="astral_teleop",
        executable="head_teleop_node",
        name="head_teleop_node",
        output="screen",
        emulate_tty=True,
        parameters=[head_cfg, head_extra],
        condition=IfCondition(LaunchConfiguration("with_head_teleop")),
    )

    arms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_pkg, "launch", "astral_dual_arm_teleop.launch.py")
        ),
        launch_arguments={
            "with_mocap": "false",
            "with_driver": _opt(context, "with_arm_driver") or "false",
            "dry_run": _opt(context, "dry_run") or "false",
            "control_board_ip": _opt(context, "control_board_ip"),
            "solver_type": _opt(context, "solver_type"),
            "urdf_path": _opt(context, "urdf_path"),
            "protocol": protocol,
            "convert_to_robot": convert,
            "require_start_signal": _opt(context, "require_start_signal"),
            "arm_side": _opt(context, "arm_side") or "both",
            "teleop_log_file": _opt(context, "teleop_log_file"),
            "log_dir": _opt(context, "log_dir"),
            "log_tag": _opt(context, "log_tag"),
            "driver_log_file": _opt(context, "driver_log_file"),
        }.items(),
    )

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astral_gripper_teleop"),
                "launch",
                "gripper_teleop.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("with_gripper")),
        launch_arguments={
            "hand_side": "left",
            "controller_joy_topic": "quest3/left_controller_joy",
        }.items(),
    )

    # Right gripper: only when right_hand_source==gripper (right pinch → right gripper).
    right_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astral_gripper_teleop"),
                "launch",
                "gripper_teleop.launch.py",
            )
        ),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("right_hand_source"), "' == 'gripper'"]
            )
        ),
        launch_arguments={
            "hand_side": "right",
            "controller_joy_topic": "quest3/right_controller_joy",
        }.items(),
    )

    glove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(glove_pkg, "launch", "wuji_glove_mocap.launch.py")
        ),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("right_hand_source"), "' == 'glove'"]
            )
        ),
        launch_arguments={
            "hand_side": "right",
            "sn": LaunchConfiguration("glove_sn"),
        }.items(),
    )

    # Video return: robot cameras → Quest panels (WebRTC signaling :8765).
    # The 8000 reverse covers mocap (tcp_wired); 8765 covers video signaling.
    with_video = _opt(context, "with_video").lower() in ("true", "1", "yes")
    _adb_reverse([8000, 8765] if with_video else [8000])
    video = []
    if with_video:
        video_args = {}
        if _opt(context, "video_cameras"):
            video_args["cameras"] = _opt(context, "video_cameras")
        video.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("quest3_video_streamer"),
                        "launch",
                        "multi_camera.launch.py",
                    )
                ),
                launch_arguments=video_args.items(),
            )
        )

    use_wuji = right_src in ("quest3", "glove")
    wuji = []
    if use_wuji:
        wuji.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        retarget_pkg, "launch", "wujihand_retarget.launch.py"
                    )
                ),
                launch_arguments={
                    "hand_side": "right",
                    "retarget_backend": backend,
                    "hand_model": _opt(context, "hand_model") or "wuji_hand",
                    "wuji_lib_config_right": wuji_right,
                }.items(),
            )
        )
        wuji.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hand_ctrl_pkg, "launch", "wujihand_drivers.launch.py"
                    )
                ),
                condition=IfCondition(LaunchConfiguration("with_hand_driver")),
                launch_arguments={
                    "hand_side": "right",
                    "right_serial": LaunchConfiguration("right_serial"),
                    "filter_cutoff_freq": LaunchConfiguration("filter_cutoff_freq"),
                }.items(),
            )
        )

    return [mocap, start_gate, workpos_gate, head_teleop, arms, gripper, right_gripper, glove, *video, *wuji]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "right_hand_source",
                default_value="quest3",
                description="quest3 | glove | gripper | none — right hand: dexterous (quest3/glove) or right pinch gripper (gripper) or none",
            ),
            DeclareLaunchArgument(
                "retarget_backend",
                default_value="",
                description="empty → wuji_retargeting if quest3 else official",
            ),
            DeclareLaunchArgument(
                "hand_model",
                default_value=os.environ.get("GLOVE_HAND_MODEL", "wuji_hand"),
            ),
            DeclareLaunchArgument(
                "wuji_lib_config_right",
                default_value="",
                description="override wuji_retargeting yaml for the right hand",
            ),
            DeclareLaunchArgument(
                "glove_sn",
                default_value=os.environ.get("GLOVE_HAND_SN", ""),
            ),
            DeclareLaunchArgument("right_serial", default_value=""),
            DeclareLaunchArgument("filter_cutoff_freq", default_value="10.0"),
            DeclareLaunchArgument(
                "with_gripper",
                default_value="true",
                description="Left Quest pinch → left gripper",
            ),
            DeclareLaunchArgument(
                "with_arm_driver",
                default_value="false",
                description="astral_robot_control (arms + left gripper)",
            ),
            DeclareLaunchArgument(
                "with_hand_driver",
                default_value="false",
                description="wujihand_driver for the right Wuji hand",
            ),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument(
                "control_board_ip",
                default_value=os.environ.get("ASTRAL_BOARD_IP", "192.168.10.2"),
            ),
            DeclareLaunchArgument(
                "arm_side",
                default_value="both",
                description="left | right | both — which arm teleop nodes to start",
            ),
            DeclareLaunchArgument("protocol", default_value=""),
            DeclareLaunchArgument("convert_to_robot", default_value=""),
            DeclareLaunchArgument("solver_type", default_value=""),
            DeclareLaunchArgument("urdf_path", default_value=""),
            DeclareLaunchArgument(
                "require_start_signal",
                default_value="",
                description=(
                    "empty → yaml. true → wait for /teleop/start to capture vr_init "
                    "and arm (use after placing hand at initial pose)."
                ),
            ),
            DeclareLaunchArgument(
                "with_head_teleop",
                default_value="true",
                description=(
                    "launch head_teleop_node: right thumbstick → head yaw/pitch "
                    "(absolute, spring-return; gated by /teleop/start)."
                ),
            ),
            DeclareLaunchArgument(
                "with_video",
                default_value="true",
                description=(
                    "launch quest3_video_streamer (robot cameras → Quest panels). "
                    "Auto adb reverse 8000+8765; enable video feed on the Quest app."
                ),
            ),
            DeclareLaunchArgument(
                "video_cameras",
                default_value="",
                description=(
                    "comma-separated camera labels to stream (empty → "
                    "quest3_video_streamer/config/params.yaml default list)."
                ),
            ),
            DeclareLaunchArgument(
                "teleop_log_file",
                default_value="",
                description=(
                    "empty → off. Non-empty shared base path → per-side JSONL "
                    "diagnostics log (loop/wrist/state/body/metrics records), "
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
                    "empty → off. Non-empty root dir → per-run stamped directory "
                    "{log_dir}/{YYYYMMDD-HHMMSS}[_tag]/ for the teleop JSONL logs "
                    "(distinguishes each recording, survives reboot). "
                    "teleop_log_file, when set, wins over this."
                ),
            ),
            DeclareLaunchArgument(
                "log_tag",
                default_value="",
                description="Event name appended to the run directory (log_dir mode).",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
