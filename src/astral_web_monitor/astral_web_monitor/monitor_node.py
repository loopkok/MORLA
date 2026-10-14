"""ROS 2 node: read-only subscriptions + thread-safe snapshot + pause/resume.

This node is the only ROS surface of the monitor. It:
  * subscribes to joint state/command topics (read-only, never republishes them)
  * caches the latest values behind a lock (latest-wins, like a SHM ring)
  * exposes pause()/resume() which publish ONE latched Bool to the existing
    /teleop/disarm and /teleop/armed topics — no new control topics are created
  * runs rclpy.spin in a background thread so the FastAPI/uvicorn event loop
    can run on the main thread
  * optionally drives the quest3_video_streamer runtime gate (SetBool master
    switch + latched active-camera subset) and mirrors its latched gate_state

It deliberately does NOT subscribe to any hardware command topics. Hardware
mode changes go through the driver's already-exposed services. Start/stop of
the teleop stack is handled by LaunchManager via subprocess, not by this node.
"""
from __future__ import annotations

import json
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from .config import (
    STALE_THRESHOLD_S,
    TOPICS,
    TOPIC_ARMED,
    TOPIC_DISARM,
    TOPIC_START,
    TOPIC_HOME,
    TOPIC_INIT,
    TOPIC_INIT_DIRECT,
    EXPECTED_RATES_HZ,
    DRIVER_SRV_READY,
    DRIVER_SRV_ENABLE,
    DRIVER_SRV_HOME,
    DRIVER_SRV_ESTOP,
    DRIVER_SRV_DAMPING,
    DRIVER_SRV_POSITION,
    VIDEO_SRV_PUSH,
    VIDEO_TOPIC_CAMERAS,
    VIDEO_TOPIC_GATE_STATE,
    VIDEO_TOPIC_PREVIEW,
    MOCAP_WRIST_TOPICS,
    IK_STATUS_TOPICS,
    MOCAP_EXPECTED_HZ,
    DC_TOPIC_CONTROL,
    DC_TOPIC_TASK,
    DC_TOPIC_SESSION,
    DC_TOPIC_STATE,
    PI_CMD_TOPIC,
    PI_TASK_TOPIC,
    PI_STATE_TOPIC,
)
from .rate_counter import RateRegistry, RateCounter


def _qos_best_effort() -> QoSProfile:
    """监控订阅：只留最新一帧，不在 DDS 队列里堆旧指令/腕位。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


@dataclass
class JointSlot:
    values: list[float] = field(default_factory=list)
    ts: float = 0.0

    def is_stale(self, threshold: float = STALE_THRESHOLD_S, now: float | None = None) -> bool:
        if not self.ts:
            return True
        now = now if now is not None else time.time()
        return (now - self.ts) > threshold


@dataclass
class LatencySlot:
    """Latest latency sample for a pipeline stage (ms)."""
    value_ms: float = 0.0
    ts: float = 0.0  # when the sample was received (wall clock)
    ok: bool = True  # False for IK "FAILED"

    def is_stale(self, threshold: float = STALE_THRESHOLD_S, now: float | None = None) -> bool:
        if not self.ts:
            return True
        now = now if now is not None else time.time()
        return (now - self.ts) > threshold


class MonitorNode(Node):
    """Read-only monitor + pause/resume publisher."""

    def __init__(self) -> None:
        super().__init__("astral_web_monitor")
        self._lock = threading.Lock()
        self._state: dict[str, JointSlot] = {k: JointSlot() for k in TOPICS}
        self._rates = RateRegistry()
        # State-topic rate counters (one per entity) for health inspection.
        self._state_rates = RateRegistry.for_state_topics()
        # Cached driver service clients (created lazily on first call).
        self._driver_clients: dict[str, object] = {}

        # Pause/resume publishers (latched/transient so late arm nodes pick up).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_armed = self.create_publisher(Bool, TOPIC_ARMED, qos)
        self._pub_disarm = self.create_publisher(Bool, TOPIC_DISARM, qos)

        # One-shot start trigger (volatile, not latched): publishing here is the
        # only write path for /teleop/start. Used by require_start_signal to
        # capture vr_init from the current pose and arm both arms at once.
        start_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self._pub_start = self.create_publisher(Bool, TOPIC_START, start_qos)
        # One-shot HOME / park-to-zero trigger (volatile, same rationale as
        # TOPIC_START: a latched HOME would auto-park late-joining arm nodes).
        self._pub_home = self.create_publisher(Bool, TOPIC_HOME, start_qos)
        # One-shot 工作位 / go-to-init trigger (volatile, same rationale):
        # 启动不再自动归位，回 init_pose 靠 web「工作位」按钮手动触发。
        self._pub_init = self.create_publisher(Bool, TOPIC_INIT, start_qos)
        # 段间回位直达（左 X / 数采卡片「段间回位」）：volatile 一次性，
        # 不经 init_waypoints 直接到 init_pose（与 TOPIC_INIT 同契约）。
        self._pub_init_direct = self.create_publisher(Bool, TOPIC_INIT_DIRECT, start_qos)

        # Video gate: latched active-cameras publisher + gate_state mirror.
        self._pub_video_cameras = self.create_publisher(String, VIDEO_TOPIC_CAMERAS, qos)
        self._video_gate_state: dict[str, Any] | None = None
        self.create_subscription(
            String, VIDEO_TOPIC_GATE_STATE, self._on_video_gate_state, qos,
        )
        self._video_push_client: Any = None  # SetBool client, created lazily

        # astral_data_collect bridge: control (reliable, volatile) + task text
        # (latched) publishers, and a latched mirror of the recorder's state
        # JSON (state/episode/elapsed/rates/dropped). None = recorder offline.
        dc_ctrl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub_dc_control = self.create_publisher(String, DC_TOPIC_CONTROL, dc_ctrl_qos)
        self._pub_dc_task = self.create_publisher(String, DC_TOPIC_TASK, qos)
        self._pub_dc_session = self.create_publisher(String, DC_TOPIC_SESSION, qos)
        self._dc_state: dict[str, Any] | None = None
        self._dc_state_ts: float = 0.0
        self.create_subscription(String, DC_TOPIC_STATE, self._on_dc_state, qos)

        # astral_policy_inference bridge: cmd (String 动词，同 policy_keyboard
        # 契约) + task (latched) publishers, 以及 state JSON 镜像 (latched)。
        # None = 推理节点离线。web 按钮与键盘节点完全等价，CLI 启动的推理节点
        # 同样可控。
        pi_ctrl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub_pi_cmd = self.create_publisher(String, PI_CMD_TOPIC, pi_ctrl_qos)
        self._pub_pi_task = self.create_publisher(String, PI_TASK_TOPIC, qos)
        self._pi_state: dict[str, Any] | None = None
        self._pi_state_ts: float = 0.0
        self.create_subscription(String, PI_STATE_TOPIC, self._on_pi_state, qos)
        # Web preview: latest JPEG bytes per camera label + dynamic subscriptions
        # (created when gate_state reports the camera list).
        self._preview_jpeg: dict[str, bytes] = {}
        self._preview_seq: dict[str, int] = {}
        self._preview_subs: dict[str, Any] = {}

        # Read-only subscriptions.
        for entity, pair in TOPICS.items():
            self.create_subscription(
                JointState, pair.state,
                lambda msg, e=entity: self._on_state(e, msg),
                _qos_best_effort(),
            )
            # State-topic rate tick (health: is the source itself alive?).
            self.create_subscription(
                JointState, pair.state,
                lambda msg, e=entity: self._state_rates.tick(f"{e}_state"),
                _qos_best_effort(),
            )
            if pair.command:
                cmd_type = Float64 if pair.cmd_kind == "float64" else JointState
                # Arm command topics: also capture stamp age for e2e latency.
                arm_side = None
                if entity == "left_arm":
                    arm_side = "left"
                elif entity == "right_arm":
                    arm_side = "right"
                if arm_side is not None:
                    self.create_subscription(
                        JointState, pair.command,
                        lambda msg, e=entity, s=arm_side: (
                            self._rates.tick(f"{e}_cmd"),
                            self._on_arm_cmd_latency(s, msg),
                        ),
                        _qos_best_effort(),
                    )
                else:
                    self.create_subscription(
                        cmd_type, pair.command,
                        lambda msg, e=entity: self._rates.tick(f"{e}_cmd"),
                        _qos_best_effort(),
                    )

        # Latency / pipeline metrics state.
        #   mocap: stamp age of quest3/{side}_wrist_pose (ms) + arrival Hz
        #   ik: parsed "dt=X.XXms" from ik_solver_{side}/ik_status (String)
        #   e2e: stamp age of {side}_arm/joint_commands (ms) — VR→command delay
        self._latency: dict[str, LatencySlot] = {}
        self._mocap_rates = RateRegistry()
        for side in MOCAP_WRIST_TOPICS:
            self._latency[f"mocap_{side}"] = LatencySlot()
            self._mocap_rates._counters[f"mocap_{side}"] = RateCounter(f"mocap_{side}")
        for side in IK_STATUS_TOPICS:
            self._latency[f"ik_{side}"] = LatencySlot()
        self._latency["e2e_left"] = LatencySlot()
        self._latency["e2e_right"] = LatencySlot()

        # Mocap wrist pose subscriptions (PoseStamped) → stamp age = pipeline delay.
        from geometry_msgs.msg import PoseStamped
        for side, topic in MOCAP_WRIST_TOPICS.items():
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, s=side: self._on_mocap_wrist(s, msg),
                _qos_best_effort(),
            )
        # IK solver status subscriptions (String "OK dt=X.XXms" / "FAILED").
        for side, topic in IK_STATUS_TOPICS.items():
            self.create_subscription(
                String, topic,
                lambda msg, s=side: self._on_ik_status(s, msg),
                _qos_best_effort(),
            )

        self.get_logger().info(
            "astral_web_monitor ready (read-only subs + pause/resume pubs)"
        )

    # --- callbacks (write snapshot) ---------------------------------------
    def _on_state(self, entity: str, msg: JointState) -> None:
        now = time.time()
        with self._lock:
            slot = self._state[entity]
            slot.values = list(msg.position)
            slot.ts = now

    def _on_mocap_wrist(self, side: str, msg: Any) -> None:
        """Mocap wrist pose arrived → stamp age = pipeline delay (ms)."""
        now = time.time()
        self._mocap_rates.tick(f"mocap_{side}")
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_sec > 0:
            age_ms = max(0.0, (now - stamp_sec) * 1000.0)
            with self._lock:
                slot = self._latency[f"mocap_{side}"]
                slot.value_ms = age_ms
                slot.ts = now
                slot.ok = True

    def _on_ik_status(self, side: str, msg: String) -> None:
        """Parse 'OK dt=X.XXms' or 'FAILED' from ik_solver status."""
        now = time.time()
        data = msg.data.strip()
        ok = not data.upper().startswith("FAIL")
        dt_ms = 0.0
        if ok:
            # Extract dt=NN.NNms
            m = re.search(r"dt=([0-9.]+)", data)
            if m:
                dt_ms = float(m.group(1))
        with self._lock:
            slot = self._latency[f"ik_{side}"]
            slot.value_ms = dt_ms
            slot.ts = now
            slot.ok = ok

    def _on_arm_cmd_latency(self, side: str, msg: JointState) -> None:
        """Arm joint_commands arrived → stamp age = VR→command e2e delay (ms)."""
        now = time.time()
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_sec > 0:
            age_ms = max(0.0, (now - stamp_sec) * 1000.0)
            with self._lock:
                slot = self._latency[f"e2e_{side}"]
                slot.value_ms = age_ms
                slot.ts = now
                slot.ok = True

    def _on_video_gate_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            with self._lock:
                self._video_gate_state = data
            self._ensure_preview_subs(data)

    def _ensure_preview_subs(self, gate: dict[str, Any]) -> None:
        """Subscribe ~/preview/{label} for every camera the streamer reports."""
        from sensor_msgs.msg import CompressedImage

        labels = [
            str(c.get("label", ""))
            for c in (gate.get("cameras") or [])
            if c.get("label")
        ]
        if not labels:
            labels = [str(x) for x in (gate.get("configured") or [])]
        preview_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for label in labels:
            if not label or label in self._preview_subs:
                continue
            self._preview_subs[label] = self.create_subscription(
                CompressedImage,
                f"{VIDEO_TOPIC_PREVIEW}/{label}",
                lambda m, lb=label: self._on_preview(lb, m),
                preview_qos,
            )

    def _on_preview(self, label: str, msg: Any) -> None:
        data = bytes(msg.data)
        with self._lock:
            self._preview_jpeg[label] = data
            self._preview_seq[label] = self._preview_seq.get(label, 0) + 1

    def preview_frame(self, label: str) -> tuple[bytes | None, int]:
        """(latest JPEG bytes, sequence) for the web MJPEG relay."""
        with self._lock:
            return self._preview_jpeg.get(label), self._preview_seq.get(label, 0)

    def preview_labels(self) -> list[str]:
        """Labels that have a preview subscription (i.e. known cameras)."""
        return list(self._preview_subs.keys())

    # --- astral_data_collect bridge -----------------------------------------

    def _on_dc_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            # 多节点检测：/data_collect/state 的发布者 >1 即有残留/双开节点
            # （老进程不持单例锁，只能靠图查询发现）。消息驱动，~1Hz。
            try:
                data["node_count"] = self.count_publishers(DC_TOPIC_STATE)
            except Exception:
                pass
            with self._lock:
                self._dc_state = data
                self._dc_state_ts = time.time()

    def publish_dc_control(self, cmd: str) -> None:
        self._pub_dc_control.publish(String(data=cmd))

    def publish_dc_task(self, text: str) -> None:
        self._pub_dc_task.publish(String(data=text))

    def publish_dc_session(self, text: str) -> None:
        self._pub_dc_session.publish(String(data=text))

    # --- astral_policy_inference bridge --------------------------------------

    def _on_pi_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            # 多节点检测：/policy_inference/state 发布者 >1 = web 泳道与 CLI 双开
            # 推理节点（会双写 joint_commands，与数采双开同危害）。消息驱动。
            try:
                data["node_count"] = self.count_publishers(PI_STATE_TOPIC)
            except Exception:
                pass
            with self._lock:
                self._pi_state = data
                self._pi_state_ts = time.time()

    def publish_pi_cmd(
        self, cmd: str, timeout_s: float = 2.0, poll_s: float = 0.02
    ) -> bool:
        """Publish a volatile policy command only after DDS finds a subscriber.

        A one-shot command published immediately after the web lane starts the
        policy node is otherwise silently lost: command QoS must remain volatile
        so a stale ``policy`` command cannot auto-start motion after a restart.
        """
        deadline = time.monotonic() + max(0.0, timeout_s)
        while self._pub_pi_cmd.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                return False
            time.sleep(max(0.001, poll_s))
        self._pub_pi_cmd.publish(String(data=cmd))
        return True

    def publish_pi_task(self, text: str) -> None:
        self._pub_pi_task.publish(String(data=text))

    # --- snapshot read (web thread) ---------------------------------------
    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            joints = {
                e: {
                    "values": list(s.values),
                    "ts": s.ts,
                    "stale": s.is_stale(now=now),
                }
                for e, s in self._state.items()
            }
            latency_raw = {
                k: {
                    "value_ms": s.value_ms,
                    "ts": s.ts,
                    "stale": s.is_stale(now=now),
                    "ok": s.ok,
                }
                for k, s in self._latency.items()
            }
        cmd_rates = self._rates.snapshot()
        state_rates = self._state_rates.snapshot()
        mocap_rates = self._mocap_rates.snapshot()
        with self._lock:
            video_gate = dict(self._video_gate_state) if self._video_gate_state else None
            dc_state = dict(self._dc_state) if self._dc_state else None
            dc_state_ts = self._dc_state_ts
            pi_state = dict(self._pi_state) if self._pi_state else None
            pi_state_ts = self._pi_state_ts
        if dc_state is not None:
            # 采集节点状态龄期：stale 说明节点可能已死（latched 消息会残留）
            dc_state["stale"] = (now - dc_state_ts) > STALE_THRESHOLD_S
        if pi_state is not None:
            # 推理节点状态龄期：stale 说明节点可能已死（latched 消息会残留）
            pi_state["stale"] = (now - pi_state_ts) > STALE_THRESHOLD_S
        return {
            "joints": joints,
            "rates_hz": cmd_rates,
            "state_rates_hz": state_rates,
            "health": self._health_summary(joints, cmd_rates, state_rates, now),
            "video_gate": video_gate,
            "data_collect": dc_state,
            "infer": pi_state,
            "latency": self._latency_summary(latency_raw, mocap_rates, now),
        }

    @staticmethod
    def _latency_summary(
        latency_raw: dict[str, Any],
        mocap_rates: dict[str, float],
        now: float,
    ) -> dict[str, Any]:
        """Pipeline latency breakdown per stage (ms) + mocap arrival Hz.

        Stages: mocap_{side} (stamp age), ik_{side} (solve ms), e2e_{side}
        (VR→command stamp age). Each entry: value_ms, stale, ok, hz (mocap only).
        """
        stages: dict[str, Any] = {}
        for side in ("left", "right"):
            m = latency_raw.get(f"mocap_{side}", {})
            ik = latency_raw.get(f"ik_{side}", {})
            e2e = latency_raw.get(f"e2e_{side}", {})
            stages[f"mocap_{side}"] = {
                "value_ms": float(m.get("value_ms", 0.0)),
                "stale": bool(m.get("stale", True)),
                "ok": bool(m.get("ok", True)),
                "hz": float(mocap_rates.get(f"mocap_{side}", 0.0)),
                "expected_hz": MOCAP_EXPECTED_HZ,
            }
            stages[f"ik_{side}"] = {
                "value_ms": float(ik.get("value_ms", 0.0)),
                "stale": bool(ik.get("stale", True)),
                "ok": bool(ik.get("ok", True)),
            }
            stages[f"e2e_{side}"] = {
                "value_ms": float(e2e.get("value_ms", 0.0)),
                "stale": bool(e2e.get("stale", True)),
                "ok": bool(e2e.get("ok", True)),
            }
        return {"stages": stages}

    @staticmethod
    def _health_summary(
        joints: dict[str, Any],
        cmd_rates: dict[str, float],
        state_rates: dict[str, float],
        now: float,
    ) -> dict[str, Any]:
        """Per-entity health: alive (state stream fresh) + command rate vs floor."""
        entities: dict[str, Any] = {}
        any_stale = False
        any_slow = False
        for e, slot in joints.items():
            stale = bool(slot["stale"])
            state_hz = float(state_rates.get(f"{e}_state", 0.0))
            cmd_key = f"{e}_cmd"
            cmd_hz = float(cmd_rates.get(cmd_key, 0.0))
            expected = float(EXPECTED_RATES_HZ.get(cmd_key, 0.0))
            slow = expected > 0.0 and cmd_hz < expected and not stale
            if stale:
                any_stale = True
            if slow:
                any_slow = True
            entities[e] = {
                "stale": stale,
                "state_hz": state_hz,
                "cmd_hz": cmd_hz,
                "expected_hz": expected,
                "slow": slow,
                "status": "stale" if stale else ("slow" if slow else "ok"),
            }
        overall = "ok"
        if any_stale:
            overall = "stale"
        elif any_slow:
            overall = "slow"
        return {"overall": overall, "entities": entities}

    # --- pause/resume -----------------------------------------------------
    def publish_disarm(self) -> None:
        self._pub_disarm.publish(Bool(data=True))

    def publish_arm(self) -> None:
        self._pub_armed.publish(Bool(data=True))

    def publish_start(self) -> None:
        """One-shot /teleop/start: arm teleop captures vr_init and arms."""
        self._pub_start.publish(Bool(data=True))

    def publish_home(self) -> None:
        """One-shot /teleop/home: both arm nodes park to zero via the HOME path.

        The arm nodes are the authority: they reject while homing/busy and log a
        warning, so publishing unconditionally is safe (mirrors publish_start).
        """
        self._pub_home.publish(Bool(data=True))

    def publish_init(self) -> None:
        """One-shot /teleop/init: both arm nodes move to the work position
        (init_waypoints → init_pose) — 启动自动归位的按需替代（工作位按钮）。

        The arm nodes are the authority: they reject while homing/busy and log a
        warning, so publishing unconditionally is safe (mirrors publish_start).
        """
        self._pub_init.publish(Bool(data=True))

    def publish_init_direct(self) -> None:
        """One-shot /teleop/init_direct: both arm nodes return **directly** to
        the work position (关节空间直线插补，不经 init_waypoints) — 段间回位
        （左 X / 数采卡片「段间回位」）。

        Arm nodes are the authority (same contract as publish_init).
        """
        self._pub_init_direct.publish(Bool(data=True))

    # --- driver service calls (hardware mode) -----------------------------
    # The driver node (astral_robot_control) already exposes Trigger services
    # for one_click_ready / enable / home / e_stop / damping / position. The
    # monitor calls them as a client — non-intrusive (the driver owns the
    # hardware). Clients are cached; the background spin thread completes the
    # futures.
    _DRIVER_SERVICES = {
        "ready": DRIVER_SRV_READY,
        "enable": DRIVER_SRV_ENABLE,
        "home": DRIVER_SRV_HOME,
        "estop": DRIVER_SRV_ESTOP,
        "damping": DRIVER_SRV_DAMPING,
        "position": DRIVER_SRV_POSITION,
    }

    def call_driver_service(self, name: str, timeout_s: float = 4.0) -> tuple[bool, str]:
        """Invoke a driver Trigger service by short name (ready/home/estop/...).

        Returns (success, message). The driver is the hardware authority; this
        only calls its already-exposed service. Safe to call from the web
        thread — the background rclpy spin thread completes the future.
        """
        from std_srvs.srv import Trigger  # local import keeps module import light
        srv_name = self._DRIVER_SERVICES.get(name)
        if srv_name is None:
            return False, f"unknown driver service: {name}"
        cli = self._driver_clients.get(name)
        if cli is None:
            cli = self.create_client(Trigger, srv_name)
            self._driver_clients[name] = cli
        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=2.0):
                return False, f"driver service 未就绪: {srv_name}（driver 未启动？）"
        future = cli.call_async(Trigger.Request())
        deadline = time.time() + timeout_s
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            return False, f"driver service 超时: {srv_name}"
        res = future.result()
        return bool(res.success), str(res.message)

    # --- video return gate (quest3_video_streamer) --------------------------
    # Non-intrusive: the streamer exposes ~/set_push_enabled (SetBool) and
    # ~/active_cameras (latched String); the monitor only calls/publishes them.
    def set_video_push(self, enabled: bool, timeout_s: float = 4.0) -> tuple[bool, str]:
        """Master push switch on the video streamer. Returns (success, message)."""
        from std_srvs.srv import SetBool
        if self._video_push_client is None:
            self._video_push_client = self.create_client(SetBool, VIDEO_SRV_PUSH)
        cli = self._video_push_client
        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=2.0):
                return False, f"video service 未就绪: {VIDEO_SRV_PUSH}（streamer 未启动？）"
        req = SetBool.Request()
        req.data = bool(enabled)
        future = cli.call_async(req)
        deadline = time.time() + timeout_s
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            return False, f"video service 超时: {VIDEO_SRV_PUSH}"
        res = future.result()
        return bool(res.success), str(res.message)

    def publish_video_cameras(self, cameras: list[str]) -> None:
        """Latched active-camera subset (empty list = all configured cameras)."""
        msg = String()
        msg.data = ",".join(c.strip() for c in cameras if c.strip())
        self._pub_video_cameras.publish(msg)


# --- module-level singleton helpers -----------------------------------------
_node: MonitorNode | None = None
_spin_thread: threading.Thread | None = None


def init_node() -> MonitorNode:
    """Create the node and start a background rclpy spin thread."""
    global _node, _spin_thread
    if _node is not None:
        return _node
    rclpy.init(args=None)
    _node = MonitorNode()
    _spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    _spin_thread.start()
    return _node


def get_node() -> MonitorNode | None:
    return _node


def shutdown_node() -> None:
    global _node, _spin_thread
    if _node is not None:
        _node.publish_disarm()  # safety: disarm on monitor exit
    rclpy.shutdown()
    _node = None
    _spin_thread = None
