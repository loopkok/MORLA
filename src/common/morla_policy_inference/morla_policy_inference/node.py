#!/usr/bin/env python3
"""ROS 2 policy node: deploy / replay / human-in-the-loop.

Modes (see :mod:`controller`): IDLE, POLICY(_PAUSED), PLAYBACK(_PAUSED), HUMAN.

Control flow (one timer at ``dataset_fps * control_interp``):

* POLICY   — latest obs (state + cameras) → engine ``tick()`` at the policy
  rate; each absolute action row is split per schema, safety-passed by the
  :class:`~morla_policy_inference.executor.SafeExecutor` and published on the
  existing arm/gripper/head command topics (last-writer-wins arbitration: we
  publish only in POLICY/PLAYBACK, and disarm teleop on entry).
* PLAYBACK — :class:`PlaybackSession` steps a recorded episode; human takeover
  re-anchors an offset to the robot's actual pose so the remainder continues
  without a jump.
* HUMAN    — VR teleop takes over through ``~/prepare_reanchor`` and a matching
  ownership token (robot origin ← FK(current joints), vr_init ← current VR pose).
  Returning to POLICY re-plans from the live state; returning to PLAYBACK
  re-anchors.

Commands arrive on ``~/cmd`` (String) — verbs: policy | playback | pause |
resume | takeover | release | stop, optionally ``playback:<source>:<episode>``.
The same verbs are sent by :mod:`keyboard`.

Robot schema parameters mirror ``morla_data_collect/data_collect.yaml`` and
build the shared :class:`~morla_core.schema.CollectSchema`, so any
upstream robot-config change propagates here with no code edits.
"""

from __future__ import annotations

import collections
import json
import queue
import threading
import time
import uuid
from typing import Optional

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from morla_core.schema import CollectSchema, upgrade_v1_schema
from morla_core.profile import ProfileRegistry
from morla_policy_inference.backend import ObsBatch, make_backend
from morla_policy_inference.controller import Controller, InvalidTransition
from morla_policy_inference.engine import ActionEngine, EngineStateError
from morla_policy_inference.executor import SafeExecutor
from morla_policy_inference.replay import PlaybackSession, load_replay
from morla_policy_inference.robot_io import (
    ObsLayout,
    decode_jpeg_rgb,
    letterbox,
    reorder_joint_positions,
)

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class PolicyNode(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__("policy_node", parameter_overrides=parameter_overrides)
        self._declare_params()
        self.declare_parameter("ownership_ack_required", True)
        self._load_robot_params()

        self.controller = Controller()
        # Serializes the arbitration/control layer: every *cmd_* handler and the
        # control timer _tick run under this lock (single-writer guarantee for
        # command topics + engine/session swaps). Sensor subscription callbacks
        # (_on_joints/_on_ratio/_on_image/_on_task) stay lock-free on purpose.
        self._lock = threading.RLock()
        # Do not put high-rate JPEG callbacks, the control timer, and commands in
        # Node's default MutuallyExclusiveCallbackGroup. That makes a nominally
        # MultiThreadedExecutor effectively single-threaded: base-image decoding
        # can then starve left_wrist *and* a reliable /policy_inference/cmd.
        # Each image source gets its own serial group (a camera cannot race with
        # itself); base and left_wrist can decode in parallel. State callbacks
        # are small and reentrant. Control/status remain mutually exclusive to
        # preserve one-writer arbitration, and commands get a dedicated lane.
        self._sensor_callback_group = ReentrantCallbackGroup()
        self._image_callback_groups: dict[str, MutuallyExclusiveCallbackGroup] = {}
        self._command_callback_group = MutuallyExclusiveCallbackGroup()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()

        # /~/cmd records are queued by the ROS callback and drained by the
        # control timer. Keep a receive sequence/timestamp so run logs prove
        # the full path: DDS receive -> queue -> controller execution.
        self._cmd_queue: "queue.Queue[tuple[str, int, float]]" = queue.Queue()
        self._cmd_rx_seq = 0
        self._engine: Optional[ActionEngine] = None
        max_vel = float(self.get_parameter("max_joint_vel").value)
        velocity_by_stream = {}
        limits_by_stream = {}
        if self._profile is not None:
            component_by_id = {
                item["component_id"]: item for item in self._profile.values["components"]
                if item.get("enabled")
            }
            for block in self._schema.action_blocks:
                if not block.writable or not block.command_stream:
                    continue
                component = component_by_id.get(block.block_id, {})
                velocity_by_stream[block.command_stream] = float(
                    component.get("max_joint_velocity_rad_s", max_vel)
                )
                if block.limits:
                    bounds = np.asarray(block.limits, dtype=np.float64)
                    limits_by_stream[block.command_stream] = (bounds[:, 0], bounds[:, 1])
        self._executor = SafeExecutor(
            max_joint_vel=max_vel, joint_limits=limits_by_stream,
            max_joint_vel_by_stream=velocity_by_stream,
        )
        self._exec_events: list[str] = []
        self._playback: Optional[PlaybackSession] = None
        self._last_cmds: list = []
        self._ratio_last: dict[str, tuple[float, float]] = {}  # topic -> (val, t)
        # driver /{side}_gripper/joint_states（last-commanded rad 回显，state
        # 定时器每拍都发）缓存：夹爪命令从未出现时的 ratio 种子来源
        self._grip_rad: dict[str, np.ndarray] = {}
        self._grip_open_rad = float(self.get_parameter("gripper_open_rad").value)
        self._grip_closed_rad = float(self.get_parameter("gripper_closed_rad").value)
        self._prompt = str(self.get_parameter("default_prompt").value or "")

        # 指标 log 文件句柄（metrics_log_file 为空则关闭此功能，零开销）
        self._metrics_fh: Optional[object] = None
        metrics_log = str(self.get_parameter("metrics_log_file").value or "")
        if metrics_log:
            try:
                self._metrics_fh = open(metrics_log, "a", buffering=1)  # 行缓冲
                self.get_logger().info(f"metrics log -> {metrics_log}")
            except OSError as exc:
                self.get_logger().warn(f"metrics_log_file {metrics_log!r} open failed: {exc}")

        # 关节指令流句柄（joint_stream_log_file 为空则关闭）
        self._js_fh: Optional[object] = None
        js_log = str(self.get_parameter("joint_stream_log_file").value or "")
        if js_log:
            try:
                self._js_fh = open(js_log, "a", buffering=1)
                self.get_logger().info(f"joint stream log -> {js_log}")
            except OSError as exc:
                self.get_logger().warn(f"joint_stream_log_file {js_log!r} open failed: {exc}")

        # 控制定时器逐 tick 诊断：与 pi_cmds 分流，确保 state gate 拒绝、IDLE、
        # HUMAN 等“没有下发指令”的周期也有记录，可区分 callback 阻塞和主动 hold。
        self._control_diag_fh: Optional[object] = None
        control_diag_log = str(
            self.get_parameter("control_diagnostics_log_file").value or ""
        )
        if control_diag_log:
            try:
                self._control_diag_fh = open(control_diag_log, "a", buffering=1)
                self.get_logger().info(f"control diagnostics log -> {control_diag_log}")
            except OSError as exc:
                self.get_logger().warn(
                    f"control_diagnostics_log_file {control_diag_log!r} "
                    f"open failed: {exc}"
                )
        self._control_diag_seq = 0
        self._last_tick_started: Optional[float] = None
        self._active_control_diag: Optional[dict] = None

        # 相机全链路诊断：本节点逐帧 rx/decode + streamer 每 5 秒发布的
        # capture/tap 统计写进同一个 JSONL。仅 log_dir/显式路径开启时有开销。
        self._camera_diag_fh: Optional[object] = None
        self._camera_diag_queue: "queue.Queue[str]" = queue.Queue(maxsize=4096)
        self._camera_diag_stop = threading.Event()
        self._camera_diag_thread: Optional[threading.Thread] = None
        camera_diag_log = str(
            self.get_parameter("camera_diagnostics_log_file").value or ""
        )
        if camera_diag_log:
            try:
                self._camera_diag_fh = open(camera_diag_log, "a", buffering=1)
                self._camera_diag_thread = threading.Thread(
                    target=self._camera_diagnostics_writer,
                    name="camera-diagnostics-writer",
                    daemon=True,
                )
                self._camera_diag_thread.start()
                self.get_logger().info(f"camera diagnostics log -> {camera_diag_log}")
            except OSError as exc:
                self.get_logger().warn(
                    f"camera_diagnostics_log_file {camera_diag_log!r} "
                    f"open failed: {exc}"
                )
        # 每次 plan 的请求快照、完整 server chunk、续播索引和旧/新 chunk 边界。
        # 采用独立 writer，不能把磁盘 IO 带进 planner 线程。
        self._plan_trace_fh: Optional[object] = None
        self._plan_trace_queue: "queue.Queue[str]" = queue.Queue(maxsize=256)
        self._plan_trace_stop = threading.Event()
        self._plan_trace_thread: Optional[threading.Thread] = None
        plan_trace_log = str(self.get_parameter("plan_trace_log_file").value or "")
        if plan_trace_log:
            try:
                self._plan_trace_fh = open(plan_trace_log, "a", buffering=1)
                self._plan_trace_thread = threading.Thread(
                    target=self._plan_trace_writer, name="plan-trace-writer", daemon=True,
                )
                self._plan_trace_thread.start()
                self.get_logger().info(f"plan trace log -> {plan_trace_log}")
            except OSError as exc:
                self.get_logger().warn(
                    f"plan_trace_log_file {plan_trace_log!r} open failed: {exc}"
                )
        self._image_rx_count: dict[str, int] = collections.defaultdict(int)
        self._image_rx_last: dict[str, float] = {}

        # observation buffers ---------------------------------------------------
        self._values: dict[str, np.ndarray] = {}
        self._stamps: dict[str, float] = {}
        self._images: dict[str, np.ndarray] = {}
        self._image_stamps: dict[str, float] = {}
        self._cam_frames = 0
        # 端到端控制延迟统计（新观测 → 指令发布）：loop_ms=节点处理耗时, obs_age=观测龄期
        self._loop_ms: "collections.deque[float]" = collections.deque(maxlen=50)
        self._obs_age_ms: "collections.deque[float]" = collections.deque(maxlen=50)

        self._ctrl_rate = float(self._dataset_fps) * int(self.get_parameter("control_interp").value)
        self._policy_dt = 1.0 / float(self._dataset_fps)
        self._acc = 0.0
        self._sub = 0
        self._hold_end: Optional[float] = None
        self._seg_prev: Optional[np.ndarray] = None
        self._seg_cur: Optional[np.ndarray] = None
        self._interp = int(self.get_parameter("control_interp").value)
        self._obs_stale_stop_s = float(self.get_parameter("obs_stale_stop_s").value)
        self._obs_lost_since: Optional[float] = None
        # Event-driven HUMAN-takeover handshake: re-anchor requests are sent
        # async and polled by the control timer, so the effect never blocks an
        # rclpy executor thread (which would starve the wait-set that completes
        # the very same service responses).
        self._takeover_pending: Optional[dict[str, object]] = None
        self._takeover_prepared = False
        self._takeover_deadline = 0.0
        self._takeover_tokens: list[str] = []
        self._ownership_epoch = uuid.uuid4().hex
        self._ownership_sequence = 0
        self._transition_id = None
        self._ownership_acks = set()
        self._policy_start_pending = None
        self.create_subscription(String, '/teleop/ownership_ack', self._on_ownership_ack, 10)

        self._setup_io()
        self._pubs: dict[tuple[str, str], object] = {}
        self._teleop_disarm = self.create_publisher(
            Bool, self.get_parameter("teleop_disarm_topic").value, _LATCHED_QOS
        )
        self._state_pub = self.create_publisher(
            String, self.get_parameter("state_topic").value, _LATCHED_QOS
        )
        self._cmd_sub = self.create_subscription(
            String,
            self.get_parameter("cmd_topic").value,
            self._on_cmd,
            10,
            callback_group=self._command_callback_group,
        )
        self._task_sub = self.create_subscription(
            String,
            self.get_parameter("task_topic").value,
            self._on_task,
            10,
            callback_group=self._command_callback_group,
        )
        reanchor_services = [
            str(row["prepare_reanchor_service"])
            for row in (self._profile.values.get("writers", []) if self._profile else [])
            if row.get("prepare_reanchor_service")
        ]
        if not reanchor_services:
            reanchor_services = [
                str(name).strip().removesuffix("/reanchor") + "/prepare_reanchor"
                for name in self.get_parameter("teleop_reanchor_services").value
            ]
        self._reanchor_clients = {
            name: self.create_client(Trigger, name) for name in reanchor_services
        }

        self._timer = self.create_timer(
            1.0 / max(1.0, self._ctrl_rate),
            self._tick,
            callback_group=self._control_callback_group,
        )
        self._status_timer = self.create_timer(
            1.0,
            self._publish_state,
            callback_group=self._control_callback_group,
        )
        self._last_publish = 0.0
        guided_rtc = (
            str(self.get_parameter("engine_mode").value) == "rtc"
            and str(self.get_parameter("model").value) == "pi05"
        )
        self.get_logger().info(
            f"policy_node up: profile={self._schema.profile_id} "
            f"layout={self._schema.layout_hash[:12]} state/action="
            f"{self._schema.state_dim}/{self._schema.action_dim}D fps={self._dataset_fps} "
            f"ctrl={self._ctrl_rate}Hz mode={self.get_parameter('engine_mode').value}"
            # 生效参数自报（防 yaml 被静默丢弃/launch 覆盖后无感）：
            # yaml 顶层键≠节点名时整份参数被 rclpy 忽略，节点按代码默认值跑。
            f" rtc_guided={guided_rtc} "
            f"rtc_queue={self.get_parameter('rtc_queue_threshold').value} "
            f"rtc_horizon={self.get_parameter('rtc_execution_horizon').value} "
            f"coeff={0.0 if guided_rtc else self.get_parameter('temporal_ensemble_coeff').value} "
            f"anchor_tol={0.0 if guided_rtc else self.get_parameter('chunk_anchor_tol').value} "
            f"anchor_blend={self.get_parameter('chunk_anchor_blend').value} "
            f"prefetch={self.get_parameter('async_prefetch_ahead').value} "
            f"mute={sorted(self._mute_cameras)} "
            f"jpeg={self.get_parameter('jpeg_transport').value} "
            f"backend={self.get_parameter('backend_type').value} "
            f"host={self.get_parameter('host').value}:{self.get_parameter('port').value} "
            f"ws_timeout={self.get_parameter('server_connect_timeout_s').value}/"
            f"{self.get_parameter('server_infer_timeout_s').value}s "
            f"cmd_topic={self.get_parameter('cmd_topic').value} "
            "callbacks=image-per-camera+command+control"
        )

    # ------------------------------------------------------------- parameters

    def _declare_params(self) -> None:
        defaults = {
            # robot / schema
            "profile_id": "",
            "model_capacity": 0,
            "require_layout_handshake": False,
            "require_kinematics_handshake": False,
            "arms": ["left"],
            "end_effector_left": "gripper",
            "end_effector_right": "none",
            "include_waist": False,
            "include_head": False,
            "cameras": ["base", "left_wrist"],
            # JSON-encoded {model_camera_name: quest3_collect_label} map
            "camera_map": (
                '{"base_0_rgb": "base", "left_wrist_0_rgb": "left_wrist"}'
            ),
            "dataset_fps": 30,
            # backend / engine
            "backend_type": "remote",  # remote | inproc | stub（传输方式）
            "model": "act",            # act | pi05 | ...（模型族）
            # 上行 JPEG：true 时 camera 槽位编码成 JPEG 字节（1.38MB→~0.2MB），
            # serve 端解码还原（image_codec）。false = 现状发原始 RGB。yaml 默认 true。
            "jpeg_transport": False,
            "checkpoint_dir": "",
            "host": "127.0.0.1",
            "port": 8000,
            "server_connect_timeout_s": 5.0,
            "server_infer_timeout_s": 3.0,
            "engine_mode": "queue_async",
            "action_chunk": 50,
            # LeRobot-style guided RTC for remote OpenPI pi05.  A value of 0
            # for the cadence fields keeps ActionEngine's chunk/fps defaults.
            "rtc_execution_horizon": 10,
            "rtc_queue_threshold": 30,
            "rtc_max_guidance_weight": 10.0,
            "rtc_prefix_schedule": "linear",
            "rtc_replan_interval_s": 0.0,
            "rtc_min_tail": 0,
            "control_interp": 1,
            # ACT 时序融合系数（借鉴 lerobot ACTTemporalEnsembler）：>0 时换 chunk
            # 做指数加权平均消除边界跳变（ACT 推荐 0.01）；0 = 关闭。yaml 默认 0.01。
            "temporal_ensemble_coeff": 0.0,
            # 换 chunk 切换平滑：安装时续播起点偏离"正在执行的旧 command" >tol rad →
            # 前 blend 行从旧值平滑过渡到新轨迹（消除收敛拉回/模型突变的切换尖峰）。
            # 0 = 关闭。yaml 默认 0.05（开启）。
            "chunk_anchor_tol": 0.0,
            "chunk_anchor_blend": 4,
            # 重规划/融合的重叠行数（剩余多少行时触发重规划，与旧尾段融合）。
            # 每 (action_chunk − async_prefetch_ahead) 步融合一次。0 = 自动 (chunk//2)。
            "async_prefetch_ahead": 0,
            "camera_image_size": 224,
            # JSON 数组：这些 collect label 不放入推理请求；pi0/pi05 服务端会
            # 对缺失槽补零并设 image_mask=False，用于相机消融诊断。
            # 例：["left_wrist"]。空 = 全部相机原样发送。
            "mute_cameras": "",
            "abs_action_min_scale": 0.5,
            "default_prompt": "",
            # replay
            "replay_source": "",
            "replay_episode": 0,
            "replay_hold_s": 1.0,
            # safety
            "max_joint_vel": 6.0,
            "obs_timeout_s": 0.5,
            "obs_stale_stop_s": 1.0,   # consecutive obs loss before POLICY auto-pauses
            "image_timeout_s": 1.0,
            "image_required": False,
            # 夹爪 rad↔ratio 线性映射（须与 driver 的 *_gripper_open/closed_rad
            # 一致）：首次进 POLICY 且 /{side}_gripper/command 从未出现时，用
            # driver /{side}_gripper/joint_states（last-commanded 回显）换算
            # ratio 作夹爪状态种子，避免"没命令过→obs 永远缺夹爪→进不了策略"
            "gripper_open_rad": 0.8,
            "gripper_closed_rad": 0.0,
            # hitl / topics
            "teleop_reanchor_services": ["/astral_arm_teleop_left/reanchor"],
            "teleop_disarm_topic": "/teleop/disarm",
            "teleop_start_topic": "/teleop/start",
            "cmd_topic": "/policy_inference/cmd",
            "state_topic": "/policy_inference/state",
            "task_topic": "/policy_inference/task",
            # 指标调试：非空时每次 _publish_state（1Hz + 状态变化）把带时间戳的
            # state JSON 追加写文件，并在终端打印一行延迟/引擎摘要（launch output=screen 可见）
            "metrics_log_file": "",
            # 关节指令流：非空时每次实际下发（30Hz）把带时间戳的各话题指令值
            # 追加写该文件（JSON 行，排障卡顿用；与 metrics_log_file 相互独立）
            "joint_stream_log_file": "",
            # 控制定时器逐 tick 诊断：调度间隔/回调耗时、state/image 龄期、
            # state gate 拒绝原因、resend_last 原因、engine 队列快照。
            "control_diagnostics_log_file": "",
            # 相机全链路 JSONL：policy 每帧到达间隔/大小/解码耗时，以及 streamer
            # /diagnostics 的 V4L2 capture + collect tap 五秒窗口统计。
            "camera_diagnostics_log_file": "",
            # 每次推理请求/返回/安装的完整 trace；log_dir 模式自动写 pi_plan_trace.jsonl。
            "plan_trace_log_file": "",
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)

    def _load_robot_params(self) -> None:
        profile_id = str(self.get_parameter("profile_id").value or "").strip()
        self._profile = ProfileRegistry.from_index().load(profile_id) if profile_id else None
        if self._profile is None:
            legacy = CollectSchema(
                arms=[str(s) for s in self.get_parameter("arms").value],
                end_effector_left=str(self.get_parameter("end_effector_left").value),
                end_effector_right=str(self.get_parameter("end_effector_right").value),
                include_waist=bool(self.get_parameter("include_waist").value),
                include_head=bool(self.get_parameter("include_head").value),
                cameras=[str(c) for c in self.get_parameter("cameras").value],
                dataset_fps=int(self.get_parameter("dataset_fps").value),
            )
            self._schema = upgrade_v1_schema(legacy.to_dict())
            self._legacy_collect_schema = legacy
        else:
            self._schema = self._profile.frozen_schema
            self._legacy_collect_schema = None
        schema = self._schema
        camera_map = self._parse_camera_map(self.get_parameter("camera_map").value)
        if self._profile is not None:
            configured = self._profile.values.get("backend_requirements", {}).get("camera_map", {})
            if not camera_map or camera_map == {"base_0_rgb": "base", "left_wrist_0_rgb": "left_wrist"}:
                camera_map = {str(k): str(v) for k, v in configured.items()}
        labels = {str(v) for v in camera_map.values()}
        unknown = labels - set(schema.cameras)
        if unknown:
            self.get_logger().warn(
                f"camera_map labels {sorted(unknown)} not in cameras list; "
                "still subscribing (streamer label may differ from recorded set)"
            )
        self._layout = ObsLayout(schema, strict_names=self._profile is not None)
        self._topic_to_key = {src.topic: k for k, src in self._layout.sources.items()}
        self._camera_map = {str(k): str(v) for k, v in camera_map.items()}
        self._mute_cameras = {
            str(s) for s in json.loads(self.get_parameter("mute_cameras").value or "[]")
        }
        unknown_mute = self._mute_cameras - set(self._camera_map.values())
        if unknown_mute:
            self.get_logger().warn(
                f"mute_cameras {sorted(unknown_mute)} 不在 camera_map labels 里；"
                "将不起作用（可 mute 的 label: "
                f"{sorted(set(self._camera_map.values()))}）"
            )
        self._image_size = int(self.get_parameter("camera_image_size").value)
        self._dataset_fps = int(schema.dataset_fps)
        self._obs_timeout = float(self.get_parameter("obs_timeout_s").value)
        self._image_timeout = float(self.get_parameter("image_timeout_s").value)
        self._image_required = bool(self.get_parameter("image_required").value)
        requirements = self._profile.values.get("backend_requirements", {}) if self._profile else {}
        self._backend_cfg = {
            "backend_type": str(self.get_parameter("backend_type").value),
            "model": str(self.get_parameter("model").value),
            "checkpoint_dir": str(self.get_parameter("checkpoint_dir").value),
            "host": str(self.get_parameter("host").value),
            "port": int(self.get_parameter("port").value),
            "connect_timeout_s": float(self.get_parameter("server_connect_timeout_s").value),
            "infer_timeout_s": float(self.get_parameter("server_infer_timeout_s").value),
            "default_prompt": str(self.get_parameter("default_prompt").value),
            "action_dim": schema.action_dim,
            "layout_hash": schema.layout_hash,
            "kinematics_hash": schema.kinematics_hash or "",
            "layout_id": str(requirements.get("model_layout_id", schema.profile_id)),
            "model_capacity": int(self.get_parameter("model_capacity").value or requirements.get("model_capacity", 0)),
            "require_layout_handshake": bool(
                self.get_parameter("require_layout_handshake").value
                or requirements.get("requires_model_layout_handshake", False)
            ),
            "require_kinematics_handshake": bool(
                self.get_parameter("require_kinematics_handshake").value
                or requirements.get("requires_model_kinematics_handshake", False)
            ),
            "camera_map": self._camera_map,
            "jpeg_transport": bool(self.get_parameter("jpeg_transport").value),
        }

    @staticmethod
    def _parse_camera_map(raw) -> dict:
        """camera_map arrives as JSON text (rclpy has no struct params)."""
        if isinstance(raw, dict):
            return {str(k): str(v) for k, v in raw.items()}
        text = str(raw or "").strip()
        if not text:
            return {}
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(
                "camera_map must be JSON text, e.g. "
                '{"base_0_rgb": "base"}'
            ) from exc
        if not isinstance(data, dict):
            raise ValueError("camera_map JSON must be an object of string pairs")
        return {str(k): str(v) for k, v in data.items()}

    def _setup_io(self) -> None:
        for key, src in self._layout.sources.items():
            if src.kind == "ratio":
                cb = lambda msg, k=key: self._on_ratio(msg, k)  # noqa: E731
                self.create_subscription(Float64, src.topic, cb, _SENSOR_QOS)
                side = src.key.split("_", 1)[0]
                self.create_subscription(
                    JointState,
                    f"/{side}_gripper/joint_states",
                    lambda msg, s=side: self._on_gripper_rad(msg, s),
                    _SENSOR_QOS,
                    callback_group=self._sensor_callback_group,
                )
            else:
                cb = lambda msg, k=key, d=src.dim, s=src: self._on_joints(  # noqa: E731
                    msg, k, d, s.joint_names, s.strict_names
                )
                self.create_subscription(
                    JointState,
                    src.topic,
                    cb,
                    _SENSOR_QOS,
                    callback_group=self._sensor_callback_group,
                )
        for label in sorted({v for v in self._camera_map.values()}):
            topic = f"/quest3_video_streamer/collect/{label}"
            group = MutuallyExclusiveCallbackGroup()
            self._image_callback_groups[label] = group
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, lab=label: self._on_image(msg, lab),
                _SENSOR_QOS,
                callback_group=group,
            )
        if self._camera_diag_fh is not None:
            self.create_subscription(
                String,
                "/quest3_video_streamer/diagnostics",
                self._on_streamer_diagnostics,
                _SENSOR_QOS,
                callback_group=self._sensor_callback_group,
            )

    # ------------------------------------------------------------ subscribers

    def _on_joints(
        self, msg: JointState, key: str, dim: int,
        joint_names: tuple[str, ...] = (), strict_names: bool = False,
    ) -> None:
        expected = joint_names if joint_names else tuple("" for _ in range(dim))
        arr = reorder_joint_positions(
            msg.name, msg.position, expected, strict=strict_names
        )
        if arr is None or arr.shape != (dim,):
            return
        self._values[key] = arr.copy()
        self._stamps[key] = time.monotonic()

    def _on_ratio(self, msg: Float64, key: str) -> None:
        self._values[key] = np.asarray([msg.data], dtype=np.float64)
        self._stamps[key] = time.monotonic()

    def _on_gripper_rad(self, msg: JointState, side: str) -> None:
        if not msg.position:
            return
        self._grip_rad[side] = np.asarray([msg.position[0]], dtype=np.float64)

    def _on_image(self, msg: CompressedImage, label: str) -> None:
        rx_mono = time.monotonic()
        rx_wall = time.time()
        previous = self._image_rx_last.get(label)
        self._image_rx_last[label] = rx_mono
        self._image_rx_count[label] += 1
        decode_t0 = time.perf_counter()
        img = decode_jpeg_rgb(msg.data)
        decode_ms = (time.perf_counter() - decode_t0) * 1000.0
        self._write_camera_diagnostics({
            "t": round(rx_wall, 4),
            "stage": "policy_rx",
            "label": label,
            "count": self._image_rx_count[label],
            "gap_ms": (
                round((rx_mono - previous) * 1000.0, 3)
                if previous is not None else None
            ),
            "payload_bytes": len(msg.data),
            "decode_ms": round(decode_ms, 3),
            "decoded": img is not None,
        })
        if img is None:
            self.get_logger().warn("collect image decode failed", throttle_duration_sec=5.0)
            return
        self._cam_frames += 1
        if self._image_size and (img.shape[0] != self._image_size or img.shape[1] != self._image_size):
            img = letterbox(img, self._image_size)
        self._images[label] = img
        self._image_stamps[label] = time.monotonic()

    def _on_streamer_diagnostics(self, msg: String) -> None:
        """Persist upstream capture/tap statistics on the policy run timeline."""
        try:
            upstream = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            upstream = {"stage": "streamer_invalid", "raw": str(msg.data)}
        if not isinstance(upstream, dict):
            upstream = {"stage": "streamer_invalid", "raw": str(upstream)}
        upstream["policy_received_t"] = round(time.time(), 4)
        self._write_camera_diagnostics(upstream)

    def _write_camera_diagnostics(self, rec: dict) -> None:
        if self._camera_diag_fh is None:
            return
        try:
            line = json.dumps(rec, ensure_ascii=False) + "\n"
            self._camera_diag_queue.put_nowait(line)
        except queue.Full:
            # Diagnostics must never block image callbacks. A full 4096-line
            # queue means the disk is badly stalled; pi_control still retains
            # receiver freshness evidence.
            pass
        except Exception:  # noqa: BLE001 - diagnostics must never affect control
            pass

    def _write_plan_trace(self, rec: dict) -> None:
        """Called by the planner thread; queue only, never block inference."""
        if self._plan_trace_fh is None:
            return
        try:
            self._plan_trace_queue.put_nowait(json.dumps(rec, ensure_ascii=False) + "\n")
        except (queue.Full, TypeError, ValueError):
            # Trace loss is preferable to adding a planning/control stall.
            pass
        except Exception:  # noqa: BLE001
            pass

    def _plan_trace_writer(self) -> None:
        while not self._plan_trace_stop.is_set() or not self._plan_trace_queue.empty():
            try:
                line = self._plan_trace_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if self._plan_trace_fh is not None:
                    self._plan_trace_fh.write(line)
            except Exception:  # noqa: BLE001
                pass
            finally:
                self._plan_trace_queue.task_done()
    def _camera_diagnostics_writer(self) -> None:
        """Drain camera diagnostics off callback threads (never block images)."""
        while not self._camera_diag_stop.is_set() or not self._camera_diag_queue.empty():
            try:
                line = self._camera_diag_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if self._camera_diag_fh is not None:
                    self._camera_diag_fh.write(line)
            except Exception:  # noqa: BLE001
                pass
            finally:
                self._camera_diag_queue.task_done()

    def _on_task(self, msg: String) -> None:
        self._prompt = msg.data
        self.get_logger().info(f"task set -> {msg.data!r}")

    # -------------------------------------------------------------- commands

    def _on_cmd(self, msg: String) -> None:
        """ROS callback: enqueue only. The control timer drains the queue while
        holding ``self._lock``, so slow transitions never run on an rclpy
        executor thread (where a blocking wait would starve the wait-set)."""
        text = msg.data.strip()
        if text:
            self._cmd_rx_seq += 1
            seq = self._cmd_rx_seq
            queued_at = time.monotonic()
            self._cmd_queue.put_nowait((text, seq, queued_at))
            self.get_logger().info(
                f"cmd_rx seq={seq} cmd={text!r} queued={self._cmd_queue.qsize()}"
            )

    def _handle_cmd(self, text: str) -> None:
        """Synchronous command entry (ROS-timer drain + tests). Caller must not
        hold ``self._lock`` when it is invoked from outside."""
        text = text.strip()
        if not text:
            return
        with self._lock:
            self._exec_cmd(text)

    def _exec_cmd(self, text: str) -> None:
        verb = text.split(":", 1)[0].split()[0]
        self.get_logger().info(f"cmd {text!r}")
        try:
            if verb == "policy":
                self._cmd_policy()
            elif verb == "playback":
                self._cmd_playback(text)
            elif verb == "pause":
                self._cmd_pause()
            elif verb == "resume":
                self._cmd_resume()
            elif verb == "takeover":
                self._cmd_takeover()
            elif verb == "release":
                self._cmd_release()
            elif verb == "stop":
                self._cmd_stop()
            else:
                self.get_logger().warn(f"unknown cmd verb {verb!r}")
        except InvalidTransition as exc:
            self.get_logger().warn(f"transition rejected: {exc}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cmd {verb!r} failed: {exc}")

    def _drain_cmds(self) -> None:
        """Run queued commands now (caller holds ``self._lock``)."""
        while True:
            try:
                text, rx_seq, queued_at = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            queue_delay_ms = (time.monotonic() - queued_at) * 1000.0
            self._mark_control_diag(
                "cmd_execute",
                cmd=text,
                cmd_rx_seq=rx_seq,
                cmd_queue_delay_ms=round(queue_delay_ms, 3),
            )
            # While a takeover effect is in flight only 'stop' is honoured; the
            # pending re-anchor owns the transition and other verbs would race it.
            if self._takeover_pending is not None or self._policy_start_pending is not None:
                verb = text.split(":", 1)[0].split()[0]
                if verb != "stop":
                    self.get_logger().warn(
                        f"cmd_exec seq={rx_seq} {verb!r} ignored during takeover effect"
                    )
                    continue
            try:
                self._exec_cmd(text)
                self.get_logger().info(
                    f"cmd_exec seq={rx_seq} state={self.controller.state} "
                    f"queue_delay_ms={queue_delay_ms:.1f}"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"cmd_exec seq={rx_seq} {text!r} failed: {exc}"
                )

    def _state_ok(self) -> tuple[Optional[np.ndarray], list[str]]:
        now = time.monotonic()
        vectors: dict[str, Optional[np.ndarray]] = {}
        for key, src in self._layout.sources.items():
            if key not in self._values:
                continue
            # Gripper has no true feedback: state == most recent commanded
            # ratio (collection convention). Once seen it stays valid; we also
            # seed from ratios this node itself commanded.
            if src.kind == "ratio":
                vectors[key] = self._values[key]
            elif now - self._stamps[key] <= self._obs_timeout:
                vectors[key] = self._values[key]
        for key, src in self._layout.sources.items():
            if key in vectors:
                continue
            if src.kind == "ratio":
                if src.topic in self._ratio_last:
                    val, _ts = self._ratio_last[src.topic]
                    vectors[key] = np.asarray([val], dtype=np.float64)
                elif src.key.split("_", 1)[0] in self._grip_rad:
                    # 命令从未出现且自身未发过：driver joint_states 的
                    # last-commanded rad 回显线性换算回 ratio（open↔closed）
                    side = src.key.split("_", 1)[0]
                    rad = float(self._grip_rad[side][0])
                    span = self._grip_open_rad - self._grip_closed_rad
                    if span <= 0.0:
                        continue
                    ratio = (self._grip_open_rad - rad) / span
                    vectors[key] = np.asarray(
                        [min(1.0, max(0.0, ratio))], dtype=np.float64
                    )
        state, missing = self._layout.assemble_state(vectors)
        images_missing = [
            lab for lab in self._camera_map.values()
            if lab not in self._mute_cameras
            and (lab not in self._images
                 or now - self._image_stamps[lab] > self._image_timeout)
        ]
        if self._image_required and images_missing:
            return None, [f"image:{m}" for m in images_missing]
        return state, missing + [f"image_stale:{m}" for m in images_missing]

    def _obs_batch(self, state: np.ndarray) -> ObsBatch:
        now = time.monotonic()
        images = {
            lab: arr
            for lab, arr in self._images.items()
            if now - self._image_stamps.get(lab, 0.0) <= self._image_timeout * 4
        }
        # mute_cameras：直接从请求省略；RemoteBackend 因取不到 label 不发送对应
        # observation/camera/<slot>，pi0/pi05 AstralInputs 在服务端补零且 mask=False。
        for lab in self._mute_cameras:
            images.pop(lab, None)
        active_labels = sorted(images.keys())
        metadata = {
            "obs_created_monotonic": round(now, 6),
            "obs_created_wall_time": round(time.time(), 6),
            "state_source_age_ms": {
                key: round((now - stamp) * 1000.0, 3)
                for key, stamp in self._stamps.items()
            },
            "image_age_ms": {
                label: round((now - self._image_stamps.get(label, 0.0)) * 1000.0, 3)
                for label in active_labels
            },
            "image_stamp_monotonic": {
                label: round(self._image_stamps.get(label, 0.0), 6)
                for label in active_labels
            },
            "muted_cameras": sorted(self._mute_cameras),
            "kinematics_hash": self._schema.kinematics_hash,
            "coordinate_contract": self._schema.metadata.get("coordinate_contract", {}),
        }
        return ObsBatch(state=state, images=images, prompt=self._prompt, metadata=metadata)

    def _disarm_teleop(self) -> None:
        msg = Bool()
        msg.data = True
        self._teleop_disarm.publish(msg)

    def _release_teleop_gate(self) -> None:
        """Open the arbitration gate (disarm=False): teleop drives the robot.

        Used on entering HUMAN so the pinch/trigger gripper teleop resumes
        publishing its ratio stream (arm teleop is armed by the reanchor call).
        """
        msg = Bool()
        msg.data = False
        self._teleop_disarm.publish(msg)

    def _stop_engine(self) -> None:
        """Disable planning/ticking but keep the engine open (cheap hold)."""
        if self._engine is not None:
            try:
                self._engine.set_enabled(False)
                self._engine.reset()
            except Exception:  # noqa: BLE001
                pass
        self._seg_prev = None
        self._seg_cur = None
        self._acc = 0.0
        self._sub = 0

    def _teardown_engine(self) -> None:
        engine, self._engine = self._engine, None
        if engine:
            engine.set_enabled(False)
            threading.Thread(target=engine.stop, daemon=True).start()
        self._seg_prev = self._seg_cur = None
        self._acc = 0.0
        self._sub = 0

    def _new_engine(self) -> ActionEngine:
        mode = str(self.get_parameter("engine_mode").value)
        guided_rtc = mode == "rtc" and str(self.get_parameter("model").value) == "pi05"
        if guided_rtc and str(self.get_parameter("backend_type").value) != "remote":
            raise ValueError("OpenPI pi05 guided RTC requires backend_type=remote")
        rtc_horizon = int(self.get_parameter("rtc_execution_horizon").value)
        return ActionEngine(
            make_backend(
                **self._backend_cfg, rtc_guided=guided_rtc, rtc_horizon=rtc_horizon,
            ),
            mode=mode,
            action_dim=self._schema.action_dim,
            chunk=int(self.get_parameter("action_chunk").value),
            policy_fps=self._dataset_fps,
            autostart=True,
            abs_action_min_scale=float(
                self.get_parameter("abs_action_min_scale").value
            ),
            # Guided denoising already conditions the overlap.  Keep the
            # queued-action smoothing path for the other engine modes.
            temporal_ensemble_coeff=(0.0 if guided_rtc else float(
                self.get_parameter("temporal_ensemble_coeff").value
            )),
            chunk_anchor_tol=(0.0 if guided_rtc else float(
                self.get_parameter("chunk_anchor_tol").value
            )),
            chunk_anchor_blend=int(
                self.get_parameter("chunk_anchor_blend").value
            ),
            async_prefetch_ahead=(
                int(self.get_parameter("async_prefetch_ahead").value)
                or None  # 0 = 引擎默认 chunk//2
            ),
            rtc_guided=guided_rtc,
            rtc_queue_threshold=int(self.get_parameter("rtc_queue_threshold").value),
            rtc_execution_horizon=rtc_horizon,
            rtc_max_guidance_weight=float(self.get_parameter("rtc_max_guidance_weight").value),
            rtc_prefix_schedule=str(self.get_parameter("rtc_prefix_schedule").value),
            rtc_replan_interval_s=(float(self.get_parameter("rtc_replan_interval_s").value) or None),
            rtc_min_tail=(int(self.get_parameter("rtc_min_tail").value) or None),
            plan_trace_hook=(self._write_plan_trace if self._plan_trace_fh is not None else None),
        )

    def _cmd_policy(self) -> None:
        self.controller.request("policy")
        self._start_policy()

    def _on_ownership_ack(self, msg):
        try:
            value = json.loads(msg.data)
            if value.get('transition_id') == self._transition_id:
                self._ownership_acks.add(value['writer'])
        except (ValueError, TypeError, KeyError):
            pass

    def _required_writers(self):
        if self._profile is not None:
            return {
                str(row["writer_id"]) for row in self._profile.values.get("writers", [])
                if row.get("ack_topic")
            }
        legacy = self._legacy_collect_schema
        writers = {f"{side}_arm" for side in legacy.arms}
        for side in legacy.arms:
            ee = legacy.end_effector_for(side)
            if ee == "gripper":
                writers.add(f"{side}_gripper")
            elif ee == "wuji":
                writers.add(f"{side}_hand")
        if legacy.include_head:
            writers.add("head")
        return writers

    def _start_policy(self) -> bool:
        """Gate teleop, confirm ownership, then plan off the control thread."""
        state, missing = self._state_ok()
        if state is None:
            self.controller.revert()
            self._publish_state()
            return False
        old = self._engine
        was_enabled = bool(old and old.enabled)
        if old: old.set_enabled(False)
        self._transition_id = uuid.uuid4().hex
        self._ownership_acks = set()
        self._policy_start_pending = {'old': old, 'was_enabled': was_enabled,
                                      'phase': 'ack', 'deadline': time.monotonic()+2.0,
                                      'cancelled': False, 'done': False, 'engine': None,
                                      'retire_lock': threading.Lock()}
        self._publish_state()
        return True

    def _begin_playback(self, session, reanchor=False):
        old = self._engine
        was_enabled = bool(old and old.enabled)
        if old: old.set_enabled(False)
        self._transition_id = uuid.uuid4().hex
        self._ownership_acks = set()
        self._policy_start_pending = {'old': old, 'was_enabled': was_enabled,
            'phase': 'ack', 'deadline': time.monotonic()+2.0,
            'cancelled': False, 'done': False, 'engine': None,
            'retire_lock': threading.Lock(),
            'kind': 'playback', 'session': session, 'reanchor': reanchor}
        self._publish_state()

    @staticmethod
    def _retire_candidate_once(pending):
        candidate = pending.get('engine')
        if candidate is None:
            return
        lock = pending.setdefault('retire_lock', threading.Lock())
        with lock:
            if pending.get('retirement_started'):
                return
            pending['retirement_started'] = True
            try:
                candidate.set_enabled(False)
            except Exception:  # noqa: BLE001 - stop still owns resource cleanup
                pass
            threading.Thread(target=candidate.stop, daemon=True).start()

    def _cancel_policy_start(self, rollback=False, reason='cancelled'):
        pending = self._policy_start_pending
        if pending is None: return
        pending['cancelled'] = True
        self._policy_start_pending = None
        candidate = pending.get('engine')
        if candidate is not None:
            try:
                candidate.set_enabled(False)
            except Exception:  # noqa: BLE001 - stop still owns resource cleanup
                pass
            if pending.get('done'):
                self._retire_candidate_once(pending)
        if rollback:
            self.controller.revert()
            if self.controller.activity == 'human':
                # Freeze was acknowledged before planning. A failed first
                # plan must not reopen an old VR anchor after the operator
                # moved during the wait: press grip to re-anchor explicitly.
                self._disarm_teleop()
            if pending['old']:
                pending['old'].set_enabled(pending['was_enabled'])
            if self.controller.activity in (None, 'human'):
                self._release_teleop_gate()
            self.get_logger().error(f'policy start failed: {reason}')
            self._publish_state()

    def _poll_policy_start(self, now):
        pending = self._policy_start_pending
        if pending is None: return False
        if now > pending['deadline']:
            self._cancel_policy_start(True, 'ownership/first-plan timeout')
            return True
        if pending['phase'] == 'ack':
            required = self._required_writers()
            if self.get_parameter('ownership_ack_required').value and not required <= self._ownership_acks:
                self._maybe_publish_state(now)
                return True
            state, missing = self._state_ok()
            if state is None:
                self._cancel_policy_start(True, 'feedback lost before planning')
                return True
            if pending.get('kind') == 'playback':
                self._disarm_teleop()
                old = pending['old']
                if old:
                    threading.Thread(target=old.stop, daemon=True).start()
                self._engine = None
                self._playback = pending['session']
                if pending['reanchor']: self._playback.reanchor(state)
                self._executor.seed(self._layout.split_action(state))
                self._last_cmds = []
                self._hold_end = None
                self._obs_lost_since = None
                self._policy_start_pending = None
                if self.controller.snapshot()['paused']: self._hold_current()
                self._publish_state()
                # Commit ownership at the end of this control tick. The next
                # tick emits the first playback command from the seeded state.
                return True
            try:
                candidate = self._new_engine()
                candidate.feed_obs(self._obs_batch(state))
                pending['engine'] = candidate
                pending['phase'] = 'planning'
                pending['deadline'] = now + max(5.0, float(self._backend_cfg.get('infer_timeout_s', 10.0))+2.0)
                self._hold_current()
                def plan():
                    try: candidate.start()
                    except Exception as exc: pending['error'] = str(exc)
                    finally:
                        pending['done'] = True
                        if pending['cancelled']:
                            self._retire_candidate_once(pending)
                threading.Thread(target=plan, name='policy-first-plan', daemon=True).start()
            except Exception as exc:
                self._cancel_policy_start(True, str(exc))
            return True
        if not pending['done']:
            self._resend_last('policy_start_wait')
            self._maybe_publish_state(now)
            return True
        if pending.get('error'):
            self._cancel_policy_start(True, pending['error'])
            return True
        state, missing = self._state_ok()
        if state is None:
            self._cancel_policy_start(True, 'feedback lost at commit')
            return True
        self._disarm_teleop()
        old = pending['old']
        if old:
            old.set_enabled(False)
            threading.Thread(target=old.stop, daemon=True).start()
        self._engine = pending['engine']
        self._policy_start_pending = None
        self._playback = None
        self._hold_end = None
        self._last_cmds = []
        self._obs_lost_since = None
        self._executor.seed(self._layout.split_action(state))
        self._seg_prev = self._seg_cur = None
        self._acc = 0.0
        self._sub = 0
        self._publish_state()
        # Do not emit a policy row in the same tick that ownership commits.
        return True

    def _cmd_playback(self, text: str) -> None:
        # Load + validate the source *before* touching the FSM or any resource
        # (adversarial-review F4): a load/dim/schema failure leaves the current
        # activity (e.g. a running POLICY engine) fully intact.
        parts = text.split(":")[1:]
        source = str(parts[0]) if parts and parts[0] else self.get_parameter("replay_source").value
        episode = int(parts[1]) if len(parts) > 1 and parts[1] else int(
            self.get_parameter("replay_episode").value
        )
        if not source:
            self.get_logger().error("playback needs replay_source param or playback:<path>")
            return
        try:
            ep = load_replay(source, episode)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"replay load failed: {exc}")
            return
        if ep.action_dim != self._schema.action_dim:
            self.get_logger().error(
                f"replay action_dim {ep.action_dim} != robot action_dim "
                f"{self._schema.action_dim}; schema mismatch"
            )
            return
        if ep.schema is not None:
            recorded = ep.schema.state_names()
            robot = self._schema.state_names()
            recorded_hash = getattr(ep.schema, "layout_hash", None)
            if (recorded_hash and recorded_hash != self._schema.layout_hash) or recorded != robot:
                self.get_logger().error(
                    "replay state layout mismatch (same total dim, different "
                    "blocks/order): recorded "
                    f"{recorded} != robot {robot}; refusing to replay"
                )
                return
            expected_kinematics = self._schema.kinematics_hash
            recorded_kinematics = ep.schema.kinematics_hash
            require_kinematics = bool(
                self._profile
                and self._profile.values.get("backend_requirements", {}).get(
                    "requires_model_kinematics_handshake", False
                )
            )
            if require_kinematics and (
                not expected_kinematics or recorded_kinematics != expected_kinematics
            ):
                self.get_logger().error(
                    "replay kinematics mismatch or unknown: recorded "
                    f"{recorded_kinematics!r} != robot {expected_kinematics!r}; refusing to replay"
                )
                return
        ep_fps = int(ep.fps)
        if ep_fps != self._dataset_fps:
            self.get_logger().warn(
                f"replay fps {ep_fps} != profile dataset_fps "
                f"{self._dataset_fps}; pacing uses the frozen profile rate"
            )
        self.controller.request("playback")
        self._begin_playback(PlaybackSession(ep))

    def _cmd_pause(self) -> None:
        self.controller.request("pause")
        # Freeze planning: the paused branch only re-sends the last target, so a
        # background planner must not keep re-inferring (or drifting the chunk).
        if self._engine is not None:
            try:
                self._engine.set_enabled(False)
            except Exception:  # noqa: BLE001
                pass
        if not self._last_cmds:
            # never commanded yet (e.g. paused right at playback start):
            # freeze the robot at its measured pose so resume has a base
            self._hold_current()
        self._publish_state()
        self.get_logger().info("paused (holding last target)")

    def _cmd_resume(self) -> None:
        prev = self.controller.snapshot()
        self.controller.request("resume")
        activity = self.controller.activity
        if prev["paused"] and activity == "policy":
            # After a pause the robot held its pose while the policy kept going
            # (real-time); the old chunk's remaining rows are stale by the hold
            # time. Re-plan fresh from the live state (reverts to *_PAUSED on
            # failure) to avoid a jump.
            self._start_policy()
            return
        self._disarm_teleop()
        if prev["paused"] and activity == "playback":
            if self._playback is None or self._playback.done:
                self._cmd_stop()
                return
            state, _ = self._state_ok()
            if state is not None:
                self._playback.reanchor(state)
            self._publish_state()
            self.get_logger().info("PLAYBACK resumed")
            return
        if self.controller.state == "POLICY" and self._engine is None:
            # defensive: an interrupted run had torn the engine down
            self._start_policy()
            return
        self._publish_state()
        self.get_logger().info("resumed")

    def _cmd_takeover(self) -> None:
        if self.controller.state == "HUMAN":
            self.get_logger().warn("already in HUMAN takeover")
            return
        if self._takeover_pending is not None:
            self.get_logger().warn("takeover already in progress")
            return
        state, missing = self._state_ok()
        if state is None:
            self.get_logger().error(
                f"takeover blocked: no fresh joint state for re-anchor "
                f"({missing[:5]}); refusing (stale pose would jump)"
            )
            self._publish_state()
            return
        if not self._reanchor_clients:
            self.get_logger().error("no teleop_reanchor_services configured")
            return
        # Freeze policy/playback emission while the effect is in flight so the
        # first teleop node that a re-anchor arms cannot race our writes.
        engine = self._engine
        if engine is not None:
            engine.set_enabled(False)
        pends: dict[str, object] = {}
        self._takeover_pending = pends
        self._takeover_prepared = False
        self._takeover_tokens = []
        self._takeover_deadline = time.monotonic() + 3.0
        self._transition_id = uuid.uuid4().hex
        self._ownership_acks = set()
        self._publish_state()
        try:
            for name, client in self._reanchor_clients.items():
                pends[name] = client.call_async(Trigger.Request())
        except Exception as exc:
            self._abort_takeover(f'prepare request failed: {exc}')
            return
        self.get_logger().info(
            f"takeover: re-anchoring VR teleop @ {time.strftime('%H:%M:%S.%f')[:-3]}"
        )

    def _poll_takeover(self, now: float) -> bool:
        """Advance the takeover handshake; returns True while still pending
        (caller must skip normal control dispatch)."""
        pends = self._takeover_pending
        if pends is None:
            return False
        pending = [f for f in pends.values() if not f.done()]
        if pending:
            if now > self._takeover_deadline:
                self._abort_takeover(
                    "re-anchor timeout ("
                    f"{', '.join(sorted(pends))})"
                )
                return False
            return True
        if not self._takeover_prepared:
            for name, fut in pends.items():
                try:
                    result = fut.result()
                except Exception as exc:  # noqa: BLE001
                    self._abort_takeover(f"re-anchor {name} error: {exc}")
                    return False
                if result is None or not result.success:
                    msg = (
                        "no response"
                        if result is None
                        else (getattr(result, "message", "") or "rejected")
                    )
                    self._abort_takeover(f"re-anchor {name} {msg}")
                    return False
                try:
                    token = json.loads(result.message)['token']
                    if not isinstance(token, str) or not token:
                        raise ValueError('empty token')
                    self._takeover_tokens.append(token)
                except (ValueError, KeyError, TypeError):
                    self._abort_takeover(f'prepare {name} returned no commit token')
                    return False
            self._takeover_prepared = True
        required = self._required_writers()
        if (self.get_parameter('ownership_ack_required').value
                and not required <= self._ownership_acks):
            if now > self._takeover_deadline:
                self._abort_takeover('ownership acknowledgement timeout')
                return False
            return True
        self.get_logger().info(
            "takeover: all reanchor done @ "
            f"{time.strftime('%H:%M:%S.%f')[:-3]} → committing"
        )
        self._commit_takeover()
        return False

    def _commit_takeover(self) -> None:
        # Teleop owns arm + gripper streams while the operator is in the loop:
        # open the arbitration gate (False). Arm/head teleop are data-sensitive
        # and only disarm on True, so this does NOT disarm what we just armed.
        try:
            self.controller.request("takeover")
        except InvalidTransition as exc:
            self._abort_takeover(f"takeover rejected: {exc}")
            return
        # 防御：FSM 一切 HUMAN 就立刻发布状态（teardown 之前）——即使
        # _teardown_engine 阻塞（join planner + 关后端，可达 ~2s），web 也
        # 立即显示 HUMAN，不把"接管反馈"拖到拆引擎之后。
        self._takeover_pending = None
        self._takeover_prepared = False
        self._publish_state()
        self._release_teleop_gate()
        self._teardown_engine()
        self._last_cmds = []
        self._obs_lost_since = None
        self._takeover_pending = None
        self._publish_state()
        self.get_logger().warn(
            "HUMAN takeover: VR teleop armed (incremental) @ "
            f"{time.strftime('%H:%M:%S.%f')[:-3]}"
        )

    def _abort_takeover(self, reason: str) -> None:
        # Roll back ownership effects, then pause the old activity. After a
        # timeout its queued plan is stale, so never resume that chunk blindly.
        for future in (self._takeover_pending or {}).values():
            if not future.done():
                future.cancel()
        self._takeover_pending = None
        self._takeover_prepared = False
        self._takeover_tokens = []
        self._disarm_teleop()
        engine = self._engine
        if engine is not None:
            engine.set_enabled(False)
        # A timeout makes the queued plan old. Keep ownership, but require a
        # fresh resume rather than executing that old plan after a long wait.
        if self.controller.state in ('POLICY', 'PLAYBACK'):
            self.controller.request('pause')
        self._hold_current()
        self._obs_lost_since = None
        self.get_logger().error(f"takeover failed ({reason}); staying put")
        self._publish_state()

    def _cmd_release(self) -> None:
        self.controller.request("release")
        if self.controller.state == "IDLE":
            # nothing was interrupted: leave manual teleop in charge
            self._teardown_engine()
            self._last_cmds = []
            self._release_teleop_gate()
            self._publish_state()
            return
        activity = self.controller.activity
        paused = bool(self.controller.snapshot()["paused"])
        if activity == "policy":
            if paused:
                # robot keeps whatever pose the human left it at until 'resume'
                self._disarm_teleop()
                self._hold_current()
                self._publish_state()
            else:
                # disarms + installs a fresh engine on success; on failure the
                # FSM reverts to HUMAN and the gate stays open (VR teleop alive)
                self._start_policy()
        elif activity == "playback":
            if self._playback is None or self._playback.done:
                self._cmd_stop()
                self.get_logger().warn("release: playback finished")
                return
            self._begin_playback(self._playback, reanchor=True)
        else:
            self._teardown_engine()
            self._release_teleop_gate()
            self._publish_state()

    def _hold_current(self) -> None:
        """Hold the robot at its measured pose (used entering a paused state)."""
        state, missing = self._state_ok()
        if state is None:
            self.get_logger().warn(f"hold_current: no state ({missing[:3]})")
            return
        self._executor.reset()
        self._send_targets_from(state)

    def _cmd_stop(self) -> None:
        self._cancel_policy_start()
        if self._takeover_pending is not None:
            for future in self._takeover_pending.values():
                if not future.done():
                    future.cancel()
            self._takeover_pending = None
            self._takeover_prepared = False
            self.get_logger().warn("stop during takeover effect; effect aborted")
        self.controller.request("stop")
        self._takeover_tokens = []
        self._teardown_engine()
        self._playback = None
        self._last_cmds = []
        self._hold_end = None
        self._obs_lost_since = None
        # Open the arbitration gate: teleop/pinch may drive again (latched True
        # would otherwise leave the gripper gate shut for the node's whole life).
        self._release_teleop_gate()
        self._publish_state()
        self.get_logger().info("stopped -> IDLE")

    # ---------------------------------------------------------- control loop

    def _tick(self) -> None:
        tick_started = time.monotonic()
        self._begin_control_diag(tick_started)
        try:
            with self._lock:
                now = time.monotonic()
                # Event-driven takeover handshake first: while the re-anchor effect is
                # in flight we neither drain commands (except stop, handled in
                # _drain_cmds) nor emit control targets — single-writer guarantee.
                self._drain_cmds()
                if getattr(self, '_policy_start_pending', None) is not None:
                    if self._poll_policy_start(now):
                        return
                if self._takeover_pending is not None:
                    if self._poll_takeover(now):
                        self._mark_control_diag("takeover_wait")
                        self._maybe_publish_state(now)
                        return
                now = time.monotonic()
                dt = 1.0 / max(1.0, self._ctrl_rate)
                st = self.controller.state
                if st == "POLICY":
                    if self._engine is None:
                        self._mark_control_diag("policy_no_engine")
                        self._maybe_publish_state(now)
                        return
                    if not self.controller.snapshot()["paused"]:
                        self._policy_tick(dt)
                elif st == "POLICY_PAUSED":
                    self._resend_last("policy_paused")
                elif st == "PLAYBACK":
                    self._playback_tick(dt)
                elif st == "PLAYBACK_PAUSED":
                    self._resend_last("playback_paused")
                elif st == "HUMAN":
                    self._mark_control_diag("human_silent")
                else:
                    self._mark_control_diag("idle_silent")
                self._maybe_publish_state(now)
        except Exception as exc:
            self._mark_control_diag(
                "tick_exception", exception=f"{type(exc).__name__}: {exc}"
            )
            raise
        finally:
            self._finish_control_diag(tick_started)

    def _policy_tick(self, dt: float) -> None:
        self._acc += dt
        t_loop0: Optional[float] = None
        if self._acc + 1e-9 >= self._policy_dt:
            self._acc = max(0.0, self._acc - self._policy_dt)
            t_loop0 = time.monotonic()
            state, missing = self._state_ok()
            if state is None:
                self._mark_control_diag("state_rejected", missing=missing)
                # Mid-run observation loss gate: never keep re-inferring (or
                # emitting) from a stale joint-state snapshot. After a grace
                # period, auto-pause so the robot holds instead of running a
                # blind plan (adversarial-review F6).
                now = time.monotonic()
                if self._obs_lost_since is None:
                    self._obs_lost_since = now
                    return
                if now - self._obs_lost_since > self._obs_stale_stop_s:
                    self.get_logger().error(
                        f"joint state lost > {self._obs_stale_stop_s:.1f}s — "
                        "auto-pausing POLICY (holding last target)"
                    )
                    self._cmd_pause()
                return
            self._obs_lost_since = None
            try:
                self._engine.feed_obs(self._obs_batch(state))
            except Exception as exc:  # noqa: BLE001
                self._mark_control_diag(
                    "feed_obs_error", exception=f"{type(exc).__name__}: {exc}"
                )
            try:
                row = self._engine.tick()
            except EngineStateError as exc:
                self.get_logger().error(f"engine failed: {exc}")
                self._cmd_stop()
                return
            if row is None:
                self._resend_last("engine_no_row")
                return
            if self._seg_cur is not None:
                self._seg_prev = self._seg_cur
            self._seg_cur = np.asarray(row, dtype=np.float64)
            self._sub = 0
        else:
            self._sub += 1
        self._emit_target()
        self._mark_control_diag("policy_emit", policy_row=t_loop0 is not None)
        # 端到端控制延迟（新观测 → 指令发布）与观测龄期；只测「消费了新观测」的 tick
        if t_loop0 is not None:
            now = time.monotonic()
            self._loop_ms.append((now - t_loop0) * 1000.0)
            # 关节反馈才是真观测（ratio 是自回显，发布时刷新 stamp，会污染龄期）
            joint_keys = [
                k for k, src in self._layout.sources.items()
                if src.kind != "ratio" and k in self._stamps
            ]
            if joint_keys:
                self._obs_age_ms.append(
                    (now - max(self._stamps[k] for k in joint_keys)) * 1000.0
                )

    def _emit_target(self) -> None:
        """Publish one control-rate target, linearly sub-dividing frame steps."""
        if self._seg_cur is None:
            return
        if self._interp <= 1 or self._seg_prev is None:
            self._send_targets_from(self._seg_cur)
            return
        f = min(1.0, (self._sub + 1) / float(self._interp))
        tgt = self._seg_prev * (1.0 - f) + self._seg_cur * f
        self._send_targets_from(tgt)

    def _playback_tick(self, dt: float) -> None:
        self._acc += dt
        if self._acc + 1e-9 < self._policy_dt:
            return
        self._acc = max(0.0, self._acc - self._policy_dt)
        if self._playback is None:
            self._cmd_stop()
            return
        row = self._playback.target()
        if row is None:
            if not self._last_cmds:
                self._cmd_stop()
                return
            # hold end pose then auto stop
            if self._hold_end is None:
                self._hold_end = time.monotonic()
            if time.monotonic() - self._hold_end > float(
                self.get_parameter("replay_hold_s").value
            ):
                self._cmd_stop()
                return
            self._resend_last("playback_end_hold")
            return
        self._playback.advance()
        self._send_targets_from(row)

    # -------------------------------------------------------- command sending

    def _send_targets_from(self, row: np.ndarray) -> None:
        try:
            targets = self._layout.split_action(row)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"split_action failed: {exc}")
            return
        safe = self._executor.step(targets, 1.0 / max(1.0, self._ctrl_rate))
        # Keep the model-side target separate from the safety-processed command.
        # Without this, a run log can show a gripper at 0/1 but cannot tell
        # whether pi0.5 predicted an invalid ratio or the driver changed it.
        # Reading ``events`` drains SafeExecutor's per-step event buffer.
        safety_events = self._executor.events
        event_codes = [f"{e.kind}:{e.stream}" for e in safety_events]
        self._exec_events += event_codes
        if safety_events and self._active_control_diag is not None:
            self._active_control_diag["safety_events"] = [
                {"stream": e.stream, "kind": e.kind, "detail": e.detail}
                for e in safety_events
            ]
            self._active_control_diag["events"].extend(
                f"safety:{code}" for code in event_codes
            )
        self._publish_targets(safe)
        self._last_cmds = safe
        if self._js_fh is not None:
            self._log_joint_stream(
                safe,
                send_kind="new_target",
                raw_targets=targets,
                safety_events=safety_events,
            )

    def _log_joint_stream(
        self,
        targets: list,
        *,
        send_kind: str,
        hold_reason: Optional[str] = None,
        raw_targets: Optional[list] = None,
        safety_events: Optional[list] = None,
    ) -> None:
        """joint_stream_log_file 开启时：每次实际下发（30Hz）记录各话题指令值 +
        同一时刻的观测 state（同时间轴，供绘图对比 state↔action 与错位时间）。"""
        rec: dict = {"t": round(time.time(), 4), "send_kind": send_kind}
        if hold_reason is not None:
            rec["hold_reason"] = hold_reason
        if self._active_control_diag is not None:
            rec["control_seq"] = self._active_control_diag["seq"]
        for tg in targets:
            rec[tg.topic] = [float(v) for v in tg.values]
        if raw_targets is not None:
            rec["raw_targets"] = {
                tg.topic: [float(v) for v in tg.values] for tg in raw_targets
            }
        if safety_events:
            rec["safety_events"] = [
                {"stream": e.stream, "kind": e.kind, "detail": e.detail}
                for e in safety_events
            ]
        # 观测 state（layout 顺序：本机 [left_arm(7), left_ee(1)]）。只读、无副作用。
        try:
            state, _ = self._state_ok()
            if state is not None:
                rec["state"] = [float(v) for v in state]
        except Exception:  # noqa: BLE001
            pass
        try:
            self._js_fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception:  # noqa: BLE001  写失败不干扰控制流
            pass

    def _resend_last(self, reason: str = "unspecified") -> None:
        self._mark_control_diag(
            "resend_last", hold_reason=reason, has_last_cmd=bool(self._last_cmds)
        )
        if self._last_cmds:
            self._publish_targets(self._last_cmds)
            if self._js_fh is not None:
                self._log_joint_stream(
                    self._last_cmds,
                    send_kind="resend_last",
                    hold_reason=reason,
                )

    def _begin_control_diag(self, tick_started: float) -> None:
        if self._control_diag_fh is None:
            return
        self._control_diag_seq += 1
        interval_ms = None
        if self._last_tick_started is not None:
            interval_ms = (tick_started - self._last_tick_started) * 1000.0
        self._last_tick_started = tick_started
        expected_ms = 1000.0 / max(1.0, self._ctrl_rate)
        self._active_control_diag = {
            "t": round(time.time(), 4),
            "seq": self._control_diag_seq,
            "tick_interval_ms": (
                round(interval_ms, 3) if interval_ms is not None else None
            ),
            "tick_late_ms": (
                round(max(0.0, interval_ms - expected_ms), 3)
                if interval_ms is not None else None
            ),
            "expected_interval_ms": round(expected_ms, 3),
            "action": "unclassified",
            "events": [],
        }

    def _mark_control_diag(self, action: str, **fields) -> None:
        rec = self._active_control_diag
        if rec is None:
            return
        rec["action"] = action
        rec["events"].append(action)
        rec.update(fields)

    def _observation_diagnostics(self, now: float) -> dict:
        source_age_ms: dict[str, Optional[float]] = {}
        source_status: dict[str, str] = {}
        joint_ages: list[float] = []
        for key, src in self._layout.sources.items():
            stamp = self._stamps.get(key)
            age_ms = (now - stamp) * 1000.0 if stamp is not None else None
            source_age_ms[key] = round(age_ms, 3) if age_ms is not None else None
            if src.kind == "ratio":
                if key in self._values:
                    source_status[key] = "latched_ratio"
                elif src.topic in self._ratio_last:
                    source_status[key] = "commanded_ratio_fallback"
                elif src.key.split("_", 1)[0] in self._grip_rad:
                    source_status[key] = "gripper_joint_fallback"
                else:
                    source_status[key] = "missing"
            elif stamp is None:
                source_status[key] = "missing"
            else:
                joint_ages.append(age_ms)
                source_status[key] = (
                    "fresh" if age_ms <= self._obs_timeout * 1000.0 else "stale"
                )
        image_age_ms = {
            lab: (
                round((now - self._image_stamps[lab]) * 1000.0, 3)
                if lab in self._image_stamps else None
            )
            for lab in sorted(set(self._camera_map.values()))
        }
        image_status = {
            lab: (
                "muted"
                if lab in self._mute_cameras
                else "missing"
                if age_ms is None
                else "fresh"
                if age_ms <= self._image_timeout * 1000.0
                else "stale"
            )
            for lab, age_ms in image_age_ms.items()
        }
        return {
            "source_age_ms": source_age_ms,
            "source_status": source_status,
            "joint_age_max_ms": round(max(joint_ages), 3) if joint_ages else None,
            "obs_timeout_ms": round(self._obs_timeout * 1000.0, 3),
            "image_age_ms": image_age_ms,
            "image_status": image_status,
            "image_timeout_ms": round(self._image_timeout * 1000.0, 3),
        }

    def _finish_control_diag(self, tick_started: float) -> None:
        rec = self._active_control_diag
        self._active_control_diag = None
        if rec is None or self._control_diag_fh is None:
            return
        try:
            # 诊断绝不能反向影响控制；快照或写盘任一失败都只丢本行。
            now = time.monotonic()
            rec["callback_ms"] = round((now - tick_started) * 1000.0, 3)
            rec["state"] = self.controller.state
            rec["cam_frames"] = self._cam_frames
            rec["observation"] = self._observation_diagnostics(now)
            engine = self._engine
            if engine is not None:
                stats = engine.stats
                rec["engine"] = {
                    key: stats.get(key)
                    for key in ("pops", "plans", "remaining", "last_plan_ms")
                }
            else:
                rec["engine"] = None
            self._control_diag_fh.write(
                json.dumps(rec, ensure_ascii=False) + "\n"
            )
        except Exception:  # noqa: BLE001  写失败不干扰控制流
            pass

    def _publish_targets(self, targets: list) -> None:
        for t in targets:
            if t.kind == "ratio":
                pub = self._pubs.get(("ratio", t.topic))
                if pub is None:
                    pub = self.create_publisher(Float64, t.topic, _COMMAND_QOS)
                    self._pubs[("ratio", t.topic)] = pub
                msg = Float64()
                msg.data = float(t.values[0])
                pub.publish(msg)
                self._ratio_last[t.topic] = (msg.data, time.monotonic())
                key = self._topic_to_key.get(t.topic)
                if key is not None:
                    self._values[key] = np.asarray([msg.data], dtype=np.float64)
                    self._stamps[key] = time.monotonic()
            else:
                pub = self._pubs.get(("js", t.topic))
                if pub is None:
                    pub = self.create_publisher(JointState, t.topic, _COMMAND_QOS)
                    self._pubs[("js", t.topic)] = pub
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = list(t.joint_names)
                msg.position = [float(v) for v in t.values]
                pub.publish(msg)

    # ---------------------------------------------------------------- state

    @staticmethod
    def _deque_stats(dq: "collections.deque[float]") -> dict | None:
        if not dq:
            return None
        xs = sorted(dq)
        n = len(xs)
        return {
            "avg": round(sum(xs) / n, 2),
            "p50": round(xs[n // 2], 2),
            "p95": round(xs[min(n - 1, int(n * 0.95))], 2),
            "max": round(xs[-1], 2),
        }

    def _state_payload(self) -> dict:
        with self._lock:
            return self._state_payload_locked()

    def _state_payload_locked(self) -> dict:
        engine_stats = self._engine.stats if self._engine is not None else {}
        self._ownership_sequence += 1
        return {
            "sent_at": time.time(),
            "epoch": self._ownership_epoch,
            "sequence": self._ownership_sequence,
            "takeover_pending": self._takeover_pending is not None,
            "policy_start_pending": self._policy_start_pending is not None,
            "transition_id": self._transition_id,
            "reanchor_tokens": self._takeover_tokens,
            "state": self.controller.state,
            "activity": self.controller.activity,
            "paused": self.controller.snapshot()["paused"],
            "prompt": self._prompt,
            "profile_id": self._schema.profile_id,
            "layout_hash": self._schema.layout_hash,
            "kinematics_hash": self._schema.kinematics_hash,
            "coordinate_contract": self._schema.metadata.get("coordinate_contract", {}),
            "state_dim": self._schema.state_dim,
            "action_dim": self._schema.action_dim,
            "engine": engine_stats,
            "playback": (
                {
                    "idx": self._playback.idx,
                    "frames": self._playback.episode.num_frames,
                    "remaining": self._playback.remaining,
                }
                if self._playback is not None
                else None
            ),
            "cam_frames": self._cam_frames,
            "latency_ms": {
                "loop": self._deque_stats(self._loop_ms),
                "obs_age": self._deque_stats(self._obs_age_ms),
            },
            "exec_events": self._exec_events[-20:],
            "error": getattr(self, "_last_error", None),
        }

    def _publish_state(self) -> None:
        payload = self._state_payload()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._state_pub.publish(msg)
        if self._metrics_fh is not None:
            self._write_metrics(payload)

    def _write_metrics(self, payload: dict) -> None:
        """metrics_log_file 开启时：state JSON + 终端一行延迟/引擎摘要。"""
        try:
            self._metrics_fh.write(
                json.dumps({"t": round(time.time(), 3), **payload}, ensure_ascii=False) + "\n"
            )
        except Exception:  # noqa: BLE001  写失败不干扰控制流
            pass
        eng = payload.get("engine") or {}
        lat = payload.get("latency_ms") or {}
        loop = lat.get("loop") or {}
        obs = lat.get("obs_age") or {}
        exec_ev = ",".join(payload.get("exec_events") or [])
        srv = (eng.get("server_timing") or {}).get("total_ms")
        print(
            f"[MET] {time.strftime('%H:%M:%S')} state={payload.get('state')} "
            f"loop_avg={loop.get('avg')}ms obs_avg={obs.get('avg')}ms "
            f"pops={eng.get('pops')} plans={eng.get('plans')} "
            f"plan_ms={eng.get('last_plan_ms')} srv={srv}ms "
            f"rem={eng.get('remaining')} exec=[{exec_ev}]",
            flush=True,
        )

    def destroy_node(self) -> None:
        self._cancel_policy_start()
        self._teardown_engine()
        if self._metrics_fh is not None:
            try:
                self._metrics_fh.close()
            except Exception:  # noqa: BLE001
                pass
            self._metrics_fh = None
        if self._js_fh is not None:
            try:
                self._js_fh.close()
            except Exception:  # noqa: BLE001
                pass
            self._js_fh = None
        if self._control_diag_fh is not None:
            try:
                self._control_diag_fh.close()
            except Exception:  # noqa: BLE001
                pass
            self._control_diag_fh = None
        if self._camera_diag_fh is not None:
            self._camera_diag_stop.set()
            if self._camera_diag_thread is not None:
                self._camera_diag_thread.join(timeout=2.0)
                self._camera_diag_thread = None
            try:
                self._camera_diag_fh.close()
            except Exception:  # noqa: BLE001
                pass
            self._camera_diag_fh = None
        if self._plan_trace_fh is not None:
            self._plan_trace_stop.set()
            if self._plan_trace_thread is not None:
                self._plan_trace_thread.join(timeout=2.0)
                self._plan_trace_thread = None
            try:
                self._plan_trace_fh.close()
            except Exception:  # noqa: BLE001
                pass
            self._plan_trace_fh = None
        super().destroy_node()

    def _maybe_publish_state(self, now: float) -> None:
        if now - self._last_publish > 0.5:
            self._last_publish = now
            self._publish_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor(num_threads=4))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
