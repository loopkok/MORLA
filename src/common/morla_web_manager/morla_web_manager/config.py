"""Centralised configuration: topic names, thresholds, ports, presets loader.

All topic names live here so the monitor never hardcodes ROS contracts in
multiple places. Values can be overridden via environment variables for
testing without touching code.
"""
from __future__ import annotations

import os
import re
from pathlib import Path
from dataclasses import dataclass, field


def _web_env(suffix: str, default: str) -> str:
    """Read MORLA names first while accepting deployed Astral environment names."""
    return os.environ.get(
        f"MORLA_WEB_MANAGER_{suffix}",
        os.environ.get(f"ASTRAL_WEB_MONITOR_{suffix}", default),
    )


# --- Web server -------------------------------------------------------------
WEB_HOST = _web_env("HOST", "127.0.0.1")
WEB_PORT = int(_web_env("PORT", "8080"))

# Offline processing uses the existing pipeline in its own Python process.
def _sdk_root() -> Path:
    explicit = os.environ.get("MORLA_SDK_ROOT") or os.environ.get("ASTRAL_SDK_ROOT")
    if explicit:
        return Path(explicit).expanduser().resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "common" / "morla_data_collect").is_dir() and parent.name == "MORLA":
            return parent.parent
        if (parent / "MORLA" / "src" / "common" / "morla_data_collect").is_dir():
            return parent
        # A source checkout may live alongside VLA before the first build.
        if (parent / "MORLA").is_dir() and (parent / "VLA").is_dir():
            return parent
    return Path.home() / "loopkok" / "sdk"


SDK_ROOT = _sdk_root()
MORLA_WS = Path(os.environ.get("MORLA_WS", str(SDK_ROOT / "MORLA"))).expanduser().resolve()
DATA_ROOT = Path(os.environ.get("MORLA_DATA_ROOT", os.environ.get("ASTRAL_DATA_ROOT", str(SDK_ROOT / "morla_data")))).expanduser().resolve()
REPLAY_BROWSE_ROOTS = list(dict.fromkeys([
    DATA_ROOT, (Path.home() / "morla_data").resolve(),
    *(Path(value).expanduser().resolve() for value in
      _web_env("REPLAY_ROOTS", "").split(os.pathsep) if value),
]))
DATA_PIPELINE_PACKAGE = MORLA_WS / "src/common/morla_data_collect"
DATA_PIPELINE_PYTHON = os.environ.get("MORLA_DATA_PYTHON", os.environ.get("ASTRAL_DATA_PYTHON", str(SDK_ROOT / "VLA/openpi/.venv/bin/python")))
DATA_CHECK_PYTHON = os.environ.get("MORLA_DATA_CHECK_PYTHON", os.environ.get("ASTRAL_DATA_CHECK_PYTHON", str(Path.home() / "miniconda3/envs/lerobot/bin/python")))

# --- WebSocket push ---------------------------------------------------------
WS_PUSH_HZ = float(_web_env("PUSH_HZ", "30"))
# 环形缓冲（全量，供 GET /api/v1/logs 下载）；WS ui_state 只推尾部以免撑爆帧。
WS_LOG_TAIL_LINES = int(_web_env("LOG_TAIL", "8000"))
WS_LOG_PUSH_LINES = int(_web_env("LOG_PUSH", "800"))


def log_tail_for_push(lines: list[str], n: int | None = None) -> list[str]:
    """WS 实时推送切片：只带环形缓冲的最后 n 行。"""
    keep = WS_LOG_PUSH_LINES if n is None else n
    if keep <= 0:
        return []
    return lines[-keep:]

# --- Stale detection --------------------------------------------------------
STALE_THRESHOLD_S = float(_web_env("STALE_S", "2.0"))

# --- Launch subprocess ------------------------------------------------------
STARTUP_WARMUP_S = float(_web_env("WARMUP_S", "2.0"))
STOP_SIGINT_TIMEOUT_S = float(_web_env("STOP_TIMEOUT_S", "30.0"))
STOP_SIGKILL_GRACE_S = float(_web_env("KILL_GRACE_S", "5.0"))

# --- Rate counter EWMA -----------------------------------------------------
RATE_EWMA_ALPHA = float(_web_env("RATE_ALPHA", "0.7"))

# --- Health inspection ------------------------------------------------------
# Expected command-rate floor (Hz) per entity, used by the health panel to flag
# a stream as "slow"/"dead". Read-only — the monitor never enforces these.
# Override via env as comma-separated name=hz pairs.
_DEFAULT_EXPECTED = {
    "left_arm_cmd": 30.0,
    "right_arm_cmd": 30.0,
    "left_gripper_cmd": 30.0,
    "right_hand_cmd": 30.0,
}


def _parse_expected() -> dict[str, float]:
    raw = _web_env("EXPECTED_HZ", "")
    out = dict(_DEFAULT_EXPECTED)
    if not raw:
        return out
    for part in raw.split(","):
        part = part.strip()
        if "=" in part:
            k, v = part.split("=", 1)
            try:
                out[k.strip()] = float(v)
            except ValueError:
                pass
    return out


EXPECTED_RATES_HZ: dict[str, float] = _parse_expected()

# --- ROS topic names (read-only subscriptions) -----------------------------
# Grouped by entity. Each entry: (state_topic, command_topic, cmd_kind).
# command_topic is "" when there is no command stream to monitor for that
# entity. cmd_kind: "joint_state" (JointState) or "float64" (std_msgs/Float64)
# — grippers stream a unitless close ratio on /{side}_gripper/command (the
# driver owns the rad mapping), so their command topic is Float64.
@dataclass(frozen=True)
class TopicPair:
    state: str
    command: str = ""
    cmd_kind: str = "joint_state"


TOPICS: dict[str, TopicPair] = {
    "left_arm":      TopicPair("/left_arm/joint_states",      "/left_arm/joint_commands"),
    "right_arm":     TopicPair("/right_arm/joint_states",     "/right_arm/joint_commands"),
    "left_gripper":  TopicPair("/left_gripper/joint_states", "/left_gripper/command", "float64"),
    "right_hand":    TopicPair("/right_hand/joint_states",    "/right_hand/joint_commands"),
    # Head (morla_bringup/head_teleop_node → driver). No cmd-rate floor on
    # purpose: head commands legitimately idle at 0 Hz before start / when the
    # right controller is not streaming, so a floor would flag false "slow".
    "head":          TopicPair("/head/joint_states",          "/head/joint_commands"),
    "full_body":     TopicPair("/astral/joint_states",        ""),
}

# Hand namespace is configurable because wujihand uses {hand_name} not "hand".
HAND_NAME = _web_env("HAND_NAME", "right_hand")
# Rebuild right_hand topics with the configured hand name.
_right_hand_state = f"/{HAND_NAME}/joint_states"
_right_hand_cmd = f"/{HAND_NAME}/joint_commands"
TOPICS["right_hand"] = TopicPair(_right_hand_state, _right_hand_cmd)

# --- Latency / pipeline metrics (read-only subscriptions) -------------------
# Mocap wrist poses: header.stamp = host wall-clock packet arrival. The monitor
# computes stamp age = now - stamp as a proxy for mocap→ROS pipeline delay.
MOCAP_WRIST_TOPICS = {
    "left":  "/quest3/left_wrist_pose",
    "right": "/quest3/right_wrist_pose",
}
# IK solver status: std_msgs/String with "OK dt=X.XXms" or "FAILED". The monitor
# parses the solve time for the latency panel.
IK_STATUS_TOPICS = {
    "left":  "/ik_solver_left/ik_status",
    "right": "/ik_solver_right/ik_status",
}
# Expected mocap wrist rate (Hz) for health "slow" flagging.
MOCAP_EXPECTED_HZ = float(_web_env("MOCAP_HZ", "60.0"))

# --- Control topics (publish, pause/resume) ---------------------------------
# These already exist in astral_arm_teleop; publishing to them is the only
# write path the monitor uses. It never touches hardware topics.
TOPIC_ARMED = "/teleop/armed"
TOPIC_DISARM = "/teleop/disarm"
# One-shot "capture vr_init + arm" trigger for astral_arm_teleop when
# require_start_signal is true. Deliberately volatile (not latched) so a
# late-joining arm node does not auto-start from a stale start signal.
TOPIC_START = "/teleop/start"
# One-shot HOME / park-to-zero trigger (both arm nodes): 当前位姿 →
# init_pose → init_waypoints → 零位。同样 volatile——晚启动节点不得被
# 历史 HOME 信号误触发（与 /teleop/start 同理）。
TOPIC_HOME = "/teleop/home"
# One-shot 工作位 / go-to-init trigger (both arm nodes): 当前位姿 →
# init_waypoints → init_pose。启动不再自动归位（move_to_init_pose=false），
# 回工作位靠本信号手动触发。同样 volatile——晚启动节点不得被历史信号误触发。
TOPIC_INIT = "/teleop/init"
# 段间回位直达（左 X / 数采卡片「段间回位」）：当前位姿 → **直接**到 init_pose，
# 不经 init_waypoints（与 TOPIC_INIT 的区别）。volatile 一次性，同契约。
TOPIC_INIT_DIRECT = "/teleop/init_direct"

# Unified profile-driven MORLA control coordinator. The monitor treats its
# latched state as the source of the active robot profile and publishes service
# requests only through this API when the coordinator is present.
CONTROL_TOPIC_STATE = "/morla/control/state"
CONTROL_SERVICE_PREFIX = "/morla/control"
CONTROL_OPERATIONS = (
    "pause", "resume", "start", "release", "ready", "enable", "position", "damping",
    "home", "workpos", "workpos_direct", "estop",
)

# --- Driver ROS services (call, robot hardware mode) -----------------------
# These services already exist on astral_robot_control's driver node. Calling
# them is non-intrusive (the driver owns the SDK/hardware; the monitor just
# invokes its already-exposed services). Override the node name via env if the
# driver is remapped.
DRIVER_NODE = _web_env("DRIVER_NODE", "astral_robot_driver")
DRIVER_SRV_READY = f"/{DRIVER_NODE}/ready"
# 上电不回零（遥操启动/恢复归位路径用，避免与 teleop 轨迹抢目标）
DRIVER_SRV_ENABLE = f"/{DRIVER_NODE}/enable"
DRIVER_SRV_HOME = f"/{DRIVER_NODE}/home"
DRIVER_SRV_ESTOP = f"/{DRIVER_NODE}/estop"
DRIVER_SRV_DAMPING = f"/{DRIVER_NODE}/damping"
DRIVER_SRV_POSITION = f"/{DRIVER_NODE}/position"

# --- Driver-enable wait after launch (start/restart/home) -------------------
# 启动带 driver 的预设后，monitor 等待 ~/enable 服务可用并调用，确保电机
# 使能不静默失败（急停后 driver auto_ready 可能没确认上电）。总预算上限。
DRIVER_ENABLE_WAIT_S = float(_web_env("ENABLE_WAIT_S", "12.0"))

# --- Video return (quest3_video_streamer) runtime gate ----------------------
# The streamer exposes a SetBool master switch and a latched String topic with
# the active camera subset (empty = all). It also publishes a latched JSON
# gate_state snapshot. The monitor only calls/publishes/subscribes these —
# the streamer owns the cameras. Override the node name via env if remapped.
VIDEO_NODE = _web_env("VIDEO_NODE", "quest3_video_streamer")
VIDEO_SRV_PUSH = f"/{VIDEO_NODE}/set_push_enabled"
VIDEO_TOPIC_CAMERAS = f"/{VIDEO_NODE}/active_cameras"
VIDEO_TOPIC_GATE_STATE = f"/{VIDEO_NODE}/gate_state"
# Per-camera JPEG preview topics published by the streamer (CompressedImage).
VIDEO_TOPIC_PREVIEW = f"/{VIDEO_NODE}/preview"  # + "/{label}"

# --- morla_data_collect（VLA 数据采集）-------------------------------------
# 录制控制（String 命令：start/stop/discard/next/pause/resume）+ 任务文本
# （latched，应用于下一段）+ 状态镜像（latched JSON，含状态/段号/时长/流率）。
DC_TOPIC_CONTROL = "/data_collect/control"
DC_TOPIC_TASK = "/data_collect/task"
DC_TOPIC_SESSION = "/data_collect/session"
DC_TOPIC_STATE = "/data_collect/state"
# 允许的录制命令（web 端按钮白名单，防注入任意字符串）
DC_COMMANDS = ("start", "stop", "discard", "next", "pause", "resume")

# --- morla_policy_inference（策略推理）--------------------------------------
# 控制面是纯话题：cmd（String 动词，同 policy_keyboard 契约）+ task（latched）+
# state 镜像（latched JSON）。web 按钮与键盘节点完全等价；CLI 启动的推理节点
# 同样可控。
PI_CMD_TOPIC = "/policy_inference/cmd"
PI_TASK_TOPIC = "/policy_inference/task"
PI_STATE_TOPIC = "/policy_inference/state"
# 允许的推理命令（对应 keyboard 全部功能：s/y/space/n/h/g/x）
PI_COMMANDS = ("policy", "playback", "pause", "resume", "takeover", "release", "stop")
# 记录日志开关 → 落盘根目录。每次 `ros2 launch` 由 launch 层自动建
# `{root}/{YYYYMMDD-HHMMSS}[_tag]/` 运行子目录（见 policy_inference/astral_dual_arm_teleop
# 的 log_dir 逻辑）：区分每次记录、持久化到测试归档、不复用同一个 /tmp 文件（重启即丢）。
# 默认测试归档 inference_test_logs/（推理 → inference/，遥操 → teleop/），
# env MORLA_WEB_MANAGER_LOG_ROOT 可覆盖（如 Jetson 工作区路径不同）。
_LOG_WS_ROOT = str(MORLA_WS)
LOG_ROOT_DEFAULT = _web_env("LOG_ROOT", os.path.join(_LOG_WS_ROOT, "inference_test_logs"))
# metrics=state 指标 JSON 行（pi_metrics.jsonl）；joint=关节指令流 JSON 行（pi_cmds.jsonl）
PI_LOG_ROOT = os.path.join(LOG_ROOT_DEFAULT, "inference")

# --- astral_arm_teleop（遥操诊断日志）-----------------------------------------
# 预设启动勾选「记录遥操日志」时注入的 log_dir 根目录；launch 每次按侧拆 _left/_right
# 并建运行子目录。节点记录 kind=loop/wrist/state/body/metrics 的 JSON 行（见 teleop_log.py）。
# 只对声明了 log_dir 参数的 launch 预设注入（launch_manager.preset_with_teleop_log）。
TELEOP_LOG_ROOT = os.path.join(LOG_ROOT_DEFAULT, "teleop")


def policy_launch_args(cfg: dict) -> dict[str, str]:
    """web 推理配置 → policy_inference.launch.py 显式参数（k:=v，str→str）。

    日志开关为 True 时注入 `log_dir`（launch 每次自动建运行子目录，metrics/cmd/
    control/camera diagnostics 四条流落到里面）——不再写 /tmp 固定文件。

    **消毒（对抗性审查）**：推理泳道的 launch 参数来自 web 用户输入，而
    LaunchManager 用 ``bash -c "exec ros2 launch ... k:=v"`` 拼装命令——host/
    backend_type/model/engine_mode 若含 shell 元字符（``;`` ``$`` ``()`` …）就是
    命令注入。这些值本应只含字母数字/_/./:/−（IPv4/IPv6/主机名），白名单清洗，
    非法字符直接剔除。port/camera_image_size 先 int 化再 str（天然安全），
    日志根目录是包内常量（含 `/`，不走 safe 清洗）。
    """
    _SAFE = re.compile(r"[^A-Za-z0-9_.:\-]")
    safe = lambda v: _SAFE.sub("", str(v))  # noqa: E731

    args: dict[str, str] = {
        "backend_type": safe(cfg.get("backend_type", "remote")),
        "model": safe(cfg.get("model", "pi05")),
        "profile_id": safe(cfg.get("profile_id", "")),
        "host": safe(cfg.get("host", "127.0.0.1")),
        "port": str(int(cfg.get("port", 8001))),
        "camera_image_size": str(int(cfg.get("camera_image_size", 224))),
        "engine_mode": safe(cfg.get("engine_mode", "queue_async")),
        "keyboard": "false",  # web 按钮取代键盘节点
    }
    if cfg.get("log"):
        args["log_dir"] = PI_LOG_ROOT
    return args


def normalize_infer_cmd(cmd: str) -> str:
    """Normalize command verbs while preserving case-sensitive source paths."""
    verb, sep, source = cmd.strip().partition(":")
    return verb.lower() + sep + source


def valid_infer_cmd(cmd: str) -> bool:
    """推理命令白名单：精确动词，或 playback 带源（"playback:<path>[:<ep>]"）。"""
    if cmd in PI_COMMANDS:
        return True
    return cmd.startswith("playback:")

# --- Orphan process detection ----------------------------------------------
# Regex fragment matched against the full command line of running processes
# to detect a leftover teleop launch before starting a new one.
BACKEND_MATCH = "ros2 launch"


def _presets_path() -> str:
    """Resolve presets.yaml from the installed share directory or source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("morla_web_manager")
        return os.path.join(share, "config", "presets.yaml")
    except Exception:
        # Fallback: source tree (dev mode before colcon install)
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "presets.yaml",
        )


def video_params_path() -> str | None:
    """Resolve quest3_video_streamer/config/params.yaml (share, else source tree).

    Deprecated for the video card (the streamer's gate_state is now the
    authoritative camera list); kept for external tooling.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("quest3_video_streamer")
        return os.path.join(share, "config", "params.yaml")
    except Exception:
        pass
    # Source-tree fallback: walk up from this file looking for a sibling
    # quest3_video_streamer package (workspace src/ layout).
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        candidate = os.path.join(here, "quest3_video_streamer", "config", "params.yaml")
        if os.path.isfile(candidate):
            return candidate
        here = os.path.dirname(here)
    return None
