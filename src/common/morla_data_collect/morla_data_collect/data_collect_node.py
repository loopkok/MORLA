"""morla_data_collect 采集节点：与遥操作并行录制 raw HDF5。

职责边界：只负责"忠实落盘"——所有订阅流原样写入 episode 目录
（robot_data.h5 / camera_data.h5 / meta.json），不做在线对齐。

控制面（键盘与话题完全等价，见 keyboard_controller）：
  订阅 /data_collect/control (std_msgs/String, RELIABLE)：
      start | stop | discard | next | pause | resume
  订阅 /data_collect/task (std_msgs/String, latched)：
      下一段 episode 的任务文本
  发布 /data_collect/state (std_msgs/String, latched JSON)：
      状态/段号/时长/各流计数与频率/task，1Hz + 状态跃迁时

线程模型：rclpy 回调只入队（deque + 各自原子 append）；独立写盘线程
每 20ms 批量 drain 到 HDF5。丢弃段 = 关闭后删除目录，不落盘即无垃圾。
"""

from __future__ import annotations

import fcntl
import json
import os
import shutil
import tempfile
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from morla_data_collect import __version__
from morla_data_collect.data_writer import (
    CAMERA_H5,
    META_JSON,
    ROBOT_H5,
    CameraDataWriter,
    StreamDataWriter,
)
from morla_core.profile import ProfileRegistry
from morla_core.schema import EE_GRIPPER, STREAM_TOPICS, CollectSchema
from morla_core.resource_locator import morla_source_root
from morla_data_collect.schema import FrozenPipelineSchema

STATE_IDLE = "IDLE"
STATE_RECORDING = "RECORDING"
STATE_PAUSED = "PAUSED"
STATE_SAVING = "SAVING"

_CMDS = ("start", "stop", "discard", "next", "pause", "resume")

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)
_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=4,
)
_CONTROL_QOS = QoSProfile(
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
# 遥操事件订阅用 VOLATILE：DDS 订阅端 durability 只能 <= 发布端，VOLATILE
# 订阅可同收 latched 与 volatile 两种发布者（/teleop/start 在 web_monitor 里
# 是刻意的 volatile 一次性触发，latched 订阅会完全不兼容收不到）。代价仅是
# 收不到订阅前的历史 latched 值——事件本来就是跃迁语义，影响可忽略。
_EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# 遥操作事件话题 → 事件名（latched Bool，记录跃迁用于 armed 覆盖率统计）
_TELEOP_EVENTS = {
    "/teleop/armed": "teleop_armed",
    "/teleop/disarm": "teleop_disarmed",
    "/teleop/start": "teleop_start",
}

_STREAM_BUFFER_MAX = 20000   # 单流缓冲上限（1000Hz 手 ≈ 20s）
_IMAGE_BUFFER_MAX = 600      # 单相机缓冲上限（30fps ≈ 20s）


def _stamp_sec(header: Any, fallback: float) -> float:
    """header.stamp 有效（sec>0）用之，否则用 fallback（到达时刻）。"""
    try:
        sec = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
        if sec > 0.0:
            return sec
    except AttributeError:
        pass
    return fallback


def _to_bool(v: Any) -> bool:
    """宽松 bool 解析：launch 传入的是字符串 "true"/"false"，bool("false") 恒真。"""
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "yes", "on")
    return bool(v)


def _low_fps_warning(state: str, rates: dict[str, float], schema: CollectSchema) -> str | None:
    """录制期参考相机实率低于 dataset_fps 一半时给出告警文案，否则 None。

    对齐以 cameras[0] 为参考时钟——它塌了整段的有效帧率就跟着塌
    （实测 d435i 走未压缩 YUYV 时 30fps 目标只剩 3.3fps）。
    """
    if state != STATE_RECORDING or not schema.cameras:
        return None
    ref = schema.cameras[0]
    floor = schema.dataset_fps * 0.5
    ref_rate = float(rates.get(f"cam:{ref}", 0.0))
    if ref_rate >= floor:
        return None
    return (
        f"参考相机 {ref} 实率 {ref_rate:.0f}fps < {floor:.0f}fps"
        f"（目标 {schema.dataset_fps}fps 的一半）——检查相机格式/总线带宽"
    )


class DataCollectNode(Node):
    def __init__(self, **node_kwargs: Any) -> None:
        super().__init__(
            "data_collect",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            **node_kwargs,
        )
        p = lambda name, default: self._param(name, default)

        root_value = str(p("save_root", "")).strip()
        if root_value:
            default_root = root_value
        elif os.environ.get("MORLA_DATA_ROOT"):
            default_root = os.environ["MORLA_DATA_ROOT"]
        elif os.environ.get("MORLA_SDK_ROOT") or os.environ.get("SDK_ROOT"):
            default_root = os.path.join(os.environ.get("MORLA_SDK_ROOT") or os.environ["SDK_ROOT"], "morla_data", "raw")
        else:
            try:
                default_root = str(morla_source_root().parent / "morla_data" / "raw")
            except Exception:
                default_root = str(Path.home() / "morla_data" / "raw")
        self._save_root = os.path.expanduser(default_root)
        self._session = str(p("session", "default_task"))
        self._camera_prefix = str(p("camera_topic_prefix", "/quest3_video_streamer/collect"))
        self._profile_id = str(p("profile_id", "")).strip()
        self._runtime_mode = str(p("runtime_mode", "unknown")).strip().lower()
        if self._runtime_mode not in {"unknown", "sim", "real"}:
            raise ValueError("runtime_mode must be unknown, sim, or real")
        self._profile_values = None
        self._profile = None
        if self._profile_id:
            self._profile = ProfileRegistry.from_index().load(self._profile_id)
            self._profile_values = self._profile.values
            self._schema = FrozenPipelineSchema(self._profile.frozen_schema)
            self._stream_specs = dict(self._schema.stream_specs)
            self._sensor_specs = tuple(self._schema.sensor_specs)
        else:
            arms_param = p("arms", ["left", "right"])
            if isinstance(arms_param, str):
                arms_param = [s.strip() for s in arms_param.split(",") if s.strip()]
            cameras_param = p("cameras", ["d435i", "wrist_left", "wrist_right"])
            if isinstance(cameras_param, str):
                cameras_param = [s.strip() for s in cameras_param.split(",") if s.strip()]
            self._schema = CollectSchema(
                arms=[str(s) for s in arms_param],
                end_effector_left=str(p("end_effector_left", "gripper")),
                end_effector_right=str(p("end_effector_right", "gripper")),
                include_waist=_to_bool(p("include_waist", False)),
                include_head=_to_bool(p("include_head", False)),
                cameras=[str(c) for c in cameras_param],
                dataset_fps=int(p("dataset_fps", 30)),
                action_source=str(p("action_source", "next_state")),
                hold_frames=int(p("hold_frames", 10)),
                max_gap_ms=float(p("max_gap_ms", 100.0)),
                jpeg_quality=int(p("jpeg_quality", 90)),
            )
            self._stream_specs = {}
            self._sensor_specs = ()
        self._streams = (
            self._schema.numeric_streams()
            if self._profile is not None else self._schema.required_streams()
        )

        # 单例锁：/data_collect/control 是全局单例控制面——任何第二个采集
        # 节点（哪怕 session 不同）都会响应同一条 start 各录一份。锁按
        # ROS_DOMAIN_ID 落在 tmp，随进程退出自动释放。段目录完整性另由
        # _claim_episode_dir 的原子 mkdir 兜底（对无锁的老残留节点也互斥）。
        self._singleton_lock_fd = self._acquire_singleton_lock()

        self._state = STATE_IDLE
        self._state_lock = threading.Lock()
        self._episode_dir: str | None = None
        self._episode_index = -1
        self._episode_start_wall = 0.0
        self._pause_started = None
        self._pending_task = str(p("default_task", ""))
        self._episode_task = self._pending_task
        # 可观测性：状态不合法被忽略的指令计数（SAVING 期按 start 等；
        # web 按钮有 disabled 视觉，键盘/VR 没有——计数进 state JSON 兜底）
        self._ignored_cmds: dict[str, int] = {}
        # 空录检查（段级）：录制 2s 后数值/图像仍全 0 → 告警文案，每段独立判定
        self._empty_warning: str | None = None
        self._quiet_start_checked = False
        self._events: list[dict[str, Any]] = []
        self._robot_writer: StreamDataWriter | None = None
        self._camera_writer: CameraDataWriter | None = None
        self._stream_counts: dict[str, int] = {}
        self._cam_counts: dict[str, int] = {}
        self._drop_counts: dict[str, int] = {}
        # 本次 session 累计磁盘占用（段结束 close 时增量累加，避免 1Hz 全目录重扫）
        self._session_bytes_accum = 0

        # 采集缓冲：回调 append，写盘线程 drain
        self._buf_lock = threading.Lock()
        self._stream_buf: dict[str, deque] = {
            name: deque(maxlen=_STREAM_BUFFER_MAX) for name in self._streams
        }
        self._sensor_buf: dict[str, deque] = {
            str(spec["sensor_key"]): deque(maxlen=_STREAM_BUFFER_MAX)
            for spec in self._sensor_specs
        }
        self._cam_buf: dict[str, deque] = {
            cam: deque(maxlen=_IMAGE_BUFFER_MAX) for cam in self._schema.cameras
        }
        # 遥操作当前状态（latched 话题持续跟踪，段开始时记初始值）
        self._teleop_state: dict[str, bool] = {v: False for v in _TELEOP_EVENTS.values()}

        self._mk_subscriptions()
        self._state_pub = self.create_publisher(String, "/data_collect/state", _LATCHED_QOS)
        self.create_subscription(
            String, "/data_collect/control", self._on_control, _CONTROL_QOS
        )
        self.create_subscription(String, "/data_collect/task", self._on_task, _LATCHED_QOS)
        self.create_subscription(
            String, "/data_collect/session", self._on_session, _LATCHED_QOS
        )

        self._writer_stop = threading.Event()
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="data-collect-writer", daemon=True
        )
        self._writer_thread.start()
        self._status_timer = self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"data_collect ready: profile={self._profile_id or 'legacy-astra'} "
            f"save_root={self._save_root} session={self._session} "
            f"schema={self._schema.to_json()}"
        )
        self._publish_state()

    @staticmethod
    def _singleton_lock_path() -> str:
        domain = os.environ.get("ROS_DOMAIN_ID", "0").strip() or "0"
        return os.path.join(
            tempfile.gettempdir(), f"morla_data_collect_domain{domain}.lock"
        )

    def _acquire_singleton_lock(self) -> int:
        """对 domain 级锁文件取非阻塞排他锁；拿到即本 domain 唯一采集节点。"""
        fd = os.open(
            self._singleton_lock_path(), os.O_CREAT | os.O_RDWR, 0o644
        )
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(fd)
            msg = (
                "另一个 data_collect 节点已在运行（/data_collect/control 是单例"
                "控制面，双开会把每段录成两份）——请先停止残留节点："
                "ros2 node list 检查 /data_collect 数量，多余者 kill"
            )
            self.get_logger().fatal(msg)
            raise RuntimeError(msg) from exc
        return fd

    # -- 参数与订阅 ------------------------------------------------------------

    def _param(self, name: str, default: Any) -> Any:
        try:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            val = self.get_parameter(name).value
            return default if val is None else val
        except Exception:
            return default

    def _mk_subscriptions(self) -> None:
        from sensor_msgs.msg import CompressedImage, JointState

        if self._profile is not None:
            from rosidl_runtime_py.utilities import get_message

            sensor_by_stream = {
                str(sensor["source_stream"]): sensor for sensor in self._sensor_specs
            }
            for name, spec in self._stream_specs.items():
                if spec.source_kind == "camera":
                    cam = name.removeprefix("camera:")
                    self.create_subscription(
                        CompressedImage, spec.topic, self._mk_cam_cb(cam), _IMAGE_QOS
                    )
                elif spec.source_kind == "sensor":
                    sensor = sensor_by_stream.get(name)
                    if sensor is None:
                        raise ValueError(f"profile sensor stream {name!r} has no sensor spec")
                    message_type = get_message(spec.message_type)
                    qos = QoSProfile(
                        reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=1,
                    )
                    self.create_subscription(
                        message_type, spec.topic, self._mk_sensor_cb(sensor), qos
                    )
                elif spec.message_type == "std_msgs/msg/Float64":
                    self.create_subscription(
                        Float64, spec.topic, self._mk_float_cb(name, spec), _SENSOR_QOS
                    )
                elif spec.message_type == "sensor_msgs/msg/JointState":
                    dim = int(spec.shape[-1])
                    self.create_subscription(
                        JointState, spec.topic, self._mk_joint_cb(name, dim, spec), _SENSOR_QOS
                    )
                else:
                    raise ValueError(
                        f"profile stream {name!r} has unsupported collector type {spec.message_type!r}"
                    )
        else:
            for name, dim in self._streams.items():
                topic = STREAM_TOPICS[name][0]
                if name.endswith("_gripper_ratio"):
                    self.create_subscription(
                        Float64, topic, self._mk_float_cb(name), _SENSOR_QOS,
                    )
                else:
                    self.create_subscription(
                        JointState, topic, self._mk_joint_cb(name, dim), _SENSOR_QOS,
                    )
            for cam in self._schema.cameras:
                topic = f"{self._camera_prefix}/{cam}"
                self.create_subscription(
                    CompressedImage, topic, self._mk_cam_cb(cam), _IMAGE_QOS
                )
        for topic, event in _TELEOP_EVENTS.items():
            self.create_subscription(
                Bool, topic, self._mk_event_cb(event), _EVENT_QOS
            )
        from morla_core.ownership import PolicyLease
        self._policy_lease = PolicyLease()
        self.create_subscription(String, '/policy_inference/state', self._on_policy_state, _EVENT_QOS)

    def _on_policy_state(self, msg):
        with self._state_lock:
            self._policy_lease.update(msg.data)
            if self._state in (STATE_RECORDING, STATE_PAUSED):
                value = dict(self._policy_lease.payload) if self._policy_lease.fresh else {'unknown': True}
                self._events.append({'t': time.time(), 'name': 'control_owner', 'data': value})

    def _accepting(self) -> bool:
        """仅 RECORDING 才接收样本：IDLE 期数据不污染下一段，PAUSED 期真正暂停。"""
        return self._state == STATE_RECORDING

    def _mk_joint_cb(self, name: str, dim: int, stream_spec=None):
        # *_cmd 流用到达时刻而非 header.stamp：teleop 端把 joint_commands 的
        # stamp 打成上游 VR 输入戳（供延迟面板算 E2E 延迟），IK 定时器比 VR
        # 快时会多次复用同一戳——实测 ~45% cmd 样本戳完全重复（值不同），
        # validate F4 严格递增检查必然失败。指令的样本时刻语义本就应是
        # 「发出时刻」；且 stamp 比真实发出时间早一个管线延迟，混用两种
        # 时钟还会给 command 模式的 action 对齐引入交错偏差。
        use_arrival = name.endswith("_cmd")

        def _cb(msg: Any) -> None:
            if not self._accepting():
                return
            now = time.time()
            arrival_timestamp = use_arrival
            pos = list(msg.position)
            if stream_spec is not None:
                expected = tuple(stream_spec.joint_names)
                names = tuple(str(v) for v in msg.name)
                if (len(pos) != dim or len(names) != dim or len(set(names)) != dim
                        or set(names) != set(expected)):
                    self._drop_counts[name] = self._drop_counts.get(name, 0) + 1
                    return
                by_name = dict(zip(names, pos))
                pos = [by_name[joint] for joint in expected]
                if not np.isfinite(np.asarray(pos, dtype=float)).all():
                    self._drop_counts[name] = self._drop_counts.get(name, 0) + 1
                    return
                arrival_timestamp = stream_spec.timestamp_mode == "arrival"
            else:
                if len(pos) > dim:
                    pos = pos[:dim]
                elif len(pos) < dim:
                    return
            ts = now if arrival_timestamp else _stamp_sec(msg.header, now)
            with self._buf_lock:
                dq = self._stream_buf[name]
                if len(dq) == dq.maxlen:
                    self._drop_counts[name] = self._drop_counts.get(name, 0) + 1
                dq.append((ts, np.asarray(pos, dtype=np.float32)))
        return _cb

    def _mk_float_cb(self, name: str, stream_spec=None):
        def _cb(msg: Any) -> None:
            if not self._accepting():
                return
            value = float(msg.data)
            if not np.isfinite(value):
                self._drop_counts[name] = self._drop_counts.get(name, 0) + 1
                return
            ts = time.time()  # Float64 无 header
            with self._buf_lock:
                dq = self._stream_buf[name]
                if len(dq) == dq.maxlen:
                    self._drop_counts[name] = self._drop_counts.get(name, 0) + 1
                dq.append((ts, np.asarray([value], dtype=np.float32)))
        return _cb

    def _mk_sensor_cb(self, sensor_spec: dict[str, Any]):
        key = str(sensor_spec["sensor_key"])
        side = str(sensor_spec.get("side", ""))
        finger_order = tuple(sensor_spec.get("finger_order", ()))
        point_count = int(sensor_spec.get("point_count_per_finger", 120))
        hardware = (self._profile_values or {}).get("hardware", {}).get("xhand", {})
        hand_id = int(hardware.get(side, {}).get("hand_id", 0))
        source_stream = str(sensor_spec["source_stream"])
        stream_spec = self._stream_specs[source_stream]

        def _cb(msg: Any) -> None:
            if not self._accepting():
                return
            now = time.time()
            ids = [int(value) for value in msg.hand_id]
            if len(set(ids)) != len(ids) or hand_id not in ids:
                self._drop_counts[f"sensor:{key}"] = self._drop_counts.get(f"sensor:{key}", 0) + 1
                return
            hand_index = ids.index(hand_id)
            if len(msg.sensor_states) != len(ids):
                self._drop_counts[f"sensor:{key}"] = self._drop_counts.get(f"sensor:{key}", 0) + 1
                return
            sensor_state = msg.sensor_states[hand_index]
            by_finger = {}
            for finger_state in sensor_state.finger_sensor_states:
                label = str(finger_state.location).strip().lower()
                if label == "mid":
                    label = "middle"
                if label in by_finger:
                    self._drop_counts[f"sensor:{key}"] = self._drop_counts.get(f"sensor:{key}", 0) + 1
                    return
                by_finger[label] = finger_state

            forces = np.zeros((len(finger_order), point_count, 3), dtype=np.float32)
            sums = np.zeros((len(finger_order), 3), dtype=np.float32)
            temperatures = np.zeros((len(finger_order), 20), dtype=np.int32)
            temperature_means = np.zeros(len(finger_order), dtype=np.float32)
            valid = np.zeros(len(finger_order), dtype=np.bool_)
            temperature_valid = np.zeros(len(finger_order), dtype=np.bool_)
            for index, finger in enumerate(finger_order):
                item = by_finger.get(str(finger))
                if item is None:
                    continue
                raw = np.asarray([(v.x, v.y, v.z) for v in item.raw_force], dtype=np.float64)
                calc = np.asarray((item.calc_force.x, item.calc_force.y, item.calc_force.z), dtype=np.float64)
                if raw.shape == (point_count, 3) and np.isfinite(raw).all() and np.isfinite(calc).all():
                    forces[index] = raw.astype(np.float32)
                    sums[index] = calc.astype(np.float32)
                    valid[index] = True
                raw_temp = np.asarray(item.raw_temperature, dtype=np.float64)
                calc_temp = float(item.calc_temperature)
                if (raw_temp.shape == (20,) and np.isfinite(raw_temp).all()
                        and np.equal(raw_temp, np.rint(raw_temp)).all()
                        and raw_temp.min(initial=0) >= np.iinfo(np.int32).min
                        and raw_temp.max(initial=0) <= np.iinfo(np.int32).max
                        and np.isfinite(calc_temp)):
                    temperatures[index] = raw_temp.astype(np.int32)
                    temperature_means[index] = calc_temp
                    temperature_valid[index] = self._runtime_mode != "sim"

            sample = {
                "force_raw": forces, "force_sum": sums,
                "temperature_raw": temperatures, "temperature_mean": temperature_means,
                "valid": valid, "temperature_valid": temperature_valid,
            }
            ts = _stamp_sec(msg.header, now) if stream_spec.timestamp_mode == "source_header" else now
            with self._buf_lock:
                dq = self._sensor_buf[key]
                if len(dq) == dq.maxlen:
                    self._drop_counts[f"sensor:{key}"] = self._drop_counts.get(f"sensor:{key}", 0) + 1
                dq.append((ts, now, sample))
        return _cb

    def _mk_cam_cb(self, cam: str):
        def _cb(msg: Any) -> None:
            if not self._accepting():
                return
            now = time.time()
            ts = _stamp_sec(msg.header, now)
            with self._buf_lock:
                dq = self._cam_buf[cam]
                if len(dq) == dq.maxlen:
                    self._drop_counts[f"cam:{cam}"] = self._drop_counts.get(f"cam:{cam}", 0) + 1
                dq.append((ts, bytes(msg.data), now))
        return _cb

    def _mk_event_cb(self, event: str):
        def _cb(msg: Any) -> None:
            value = bool(msg.data)
            with self._state_lock:
                # arm/disarm are commands updating one effective state.
                if event == "teleop_disarmed":
                    if not value:
                        return
                    armed = False
                elif event == "teleop_armed":
                    armed = value
                else:
                    if self._state in (STATE_RECORDING, STATE_PAUSED):
                        self._events.append({"t": time.time(), "name": event, "data": value})
                    return
                previous = self._teleop_state.get("teleop_armed", False)
                self._teleop_state["teleop_armed"] = armed
                self._teleop_state["teleop_disarmed"] = not armed
                if previous != armed and self._state in (STATE_RECORDING, STATE_PAUSED):
                    self._events.append({"t": time.time(), "name": "teleop_armed", "data": armed})
        return _cb

    # -- 控制面 ------------------------------------------------------------------

    def _on_task(self, msg: String) -> None:
        self._pending_task = msg.data
        self.get_logger().info(f"task for next episode <- {msg.data!r}")
        self._publish_state()

    def _on_session(self, msg: String) -> None:
        """运行期切换 session 目录（仅 IDLE 生效，录制/保存中拒绝）。

        会话目录 = {save_root}/{session}，切换只影响下一段。目录名安全
        规则：非空、无路径分隔符、不含 ".."、不以 "." 开头、长度 ≤64。
        """
        name = msg.data.strip()
        if (
            not name
            or "/" in name or "\\" in name
            or name == ".." or name.startswith(".")
            or len(name) > 64
        ):
            self.get_logger().warning(f"session name rejected: {msg.data!r}")
            self._ignored_cmds["set_session"] = self._ignored_cmds.get("set_session", 0) + 1
            return
        with self._state_lock:
            if self._state != STATE_IDLE:
                self.get_logger().warning(
                    f"set_session ignored in state {self._state}（仅 IDLE 可切换目录）"
                )
                self._ignored_cmds["set_session"] = self._ignored_cmds.get("set_session", 0) + 1
                return
        self._session = name
        self._session_bytes_accum = 0  # 换 session：累计磁盘占用重新计
        self.get_logger().info(f"session -> {name!r}（下一段写入 {self._session_dir}）")
        self._publish_state()

    def _on_control(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd not in _CMDS:
            self.get_logger().warning(f"unknown control cmd {cmd!r} (expect {_CMDS})")
            return
        self.get_logger().info(f"control <- {cmd}")
        with self._state_lock:
            try:
                self._apply(cmd)
            except Exception as exc:  # 状态机错误不致命
                self.get_logger().error(f"cmd {cmd} failed: {exc}")
        self._publish_state()

    def _reject(self, cmd: str, st: str) -> None:
        """状态不合法被忽略的指令：计数入 state JSON（混用控制面时被吞的
        按键可见）+ WARN。IDLE 期的误按不算数据问题，但 SAVING 期被吞的
        start 会让操作者以为已开录——需要留痕。"""
        self._ignored_cmds[cmd] = self._ignored_cmds.get(cmd, 0) + 1
        self.get_logger().warning(f"{cmd} ignored in state {st}")

    def _apply(self, cmd: str) -> None:
        st = self._state
        if cmd == "start":
            if st != STATE_IDLE:
                self._reject("start", st)
                return
            self._begin_episode()
        elif cmd == "stop":
            if st not in (STATE_RECORDING, STATE_PAUSED):
                self._reject("stop", st)
                return
            self._end_episode(save=True)
        elif cmd == "discard":
            if st not in (STATE_RECORDING, STATE_PAUSED):
                self._reject("discard", st)
                return
            self._end_episode(save=False)
        elif cmd == "next":
            if st not in (STATE_RECORDING, STATE_PAUSED):
                self._reject("next", st)
                return
            self._end_episode(save=True)
            self._begin_episode()
        elif cmd == "pause":
            if st == STATE_RECORDING:
                self._drain_buffers()  # 暂停前落盘已缓冲数据
                self._state = STATE_PAUSED
                self._pause_started = time.time()
                self._events.append({"t": self._pause_started, "name": "pause", "data": True})
            else:
                self._reject("pause", st)
        elif cmd == "resume":
            if st == STATE_PAUSED:
                # Resume into a separate episode: no action crosses a recording gap.
                self._end_episode(save=True)
                self._begin_episode()
            else:
                self._reject("resume", st)

    # -- episode 生命周期 ---------------------------------------------------------

    @property
    def _session_dir(self) -> str:
        if self._profile_id:
            return os.path.join(self._save_root, self._profile_id, self._session)
        return os.path.join(self._save_root, self._session)

    @staticmethod
    def _dir_bytes(path: str) -> int:
        """目录磁盘占用（文件 size 求和，浅递归一层 episode 子目录）。

        注：录制中 HDF5 是 chunk 预分配（数值流 1000 行/块），故返回的磁盘占用
        含预分配、会按块粒度取整——语义是"占了多少磁盘"，不是"数据字节数"
        （数据字节请用 _robot_writer.counts() / _camera_writer.counts()）。
        """
        total = 0
        try:
            with os.scandir(path) as it:
                for e in it:
                    try:
                        if e.is_file(follow_symlinks=False):
                            total += e.stat().st_size
                        elif e.is_dir(follow_symlinks=False):
                            with os.scandir(e.path) as sub:
                                total += sum(
                                    f.stat().st_size
                                    for f in sub
                                    if f.is_file(follow_symlinks=False)
                                )
                    except OSError:
                        continue
        except OSError:
            pass
        return total

    def _next_episode_index(self) -> int:
        d = self._session_dir
        if not os.path.isdir(d):
            return 0
        best = -1
        for name in os.listdir(d):
            if name.startswith("episode") and name[len("episode"):].isdigit():
                best = max(best, int(name[len("episode"):]))
        return best + 1

    def _claim_episode_dir(self) -> tuple[int, str]:
        """原子占位段号：扫目录给起点，mkdir 成功才占有该号。

        scan-then-create 不是原子的——残留/并发节点会撞号互写（实测一个
        start 在两个节点里分别写出 episode000000 和 episode000001）。
        mkdir 是原子操作，撞号即让位到下一号。
        """
        os.makedirs(self._session_dir, exist_ok=True)
        idx = self._next_episode_index()
        while True:
            ep_dir = os.path.join(self._session_dir, f"episode{idx:06d}")
            try:
                os.mkdir(ep_dir)
                return idx, ep_dir
            except FileExistsError:
                idx += 1

    def _begin_episode(self) -> None:
        self._episode_index, self._episode_dir = self._claim_episode_dir()
        self._episode_task = self._pending_task
        # 空录检查每段独立：告警复位、检查旗标复位
        self._empty_warning = None
        self._quiet_start_checked = False
        # 防御性清空（回调已被 _accepting 门控，此处防状态竞态残留）；
        # 丢弃计数按段归零（meta.json 的 dropped 是段级 provenance）
        with self._buf_lock:
            for dq in (*self._stream_buf.values(), *self._sensor_buf.values(), *self._cam_buf.values()):
                dq.clear()
            self._drop_counts = {}
        self._prev_counts = {}
        self._robot_writer = StreamDataWriter(
            os.path.join(self._episode_dir, ROBOT_H5), self._streams,
            sensor_specs=self._sensor_specs,
        ).open()
        self._camera_writer = CameraDataWriter(
            os.path.join(self._episode_dir, CAMERA_H5), self._schema.cameras
        ).open()
        self._events = [
            {"t": time.time(), "name": "episode_start", "data": True},
            {'t': time.time(), 'name': 'control_owner',
             'data': dict(self._policy_lease.payload) if self._policy_lease.fresh else {'unknown': True}},
            # 段开始时的遥操作状态快照（armed 覆盖率统计的起点）
            *(
                {"t": time.time(), "name": k, "data": v}
                for k, v in self._teleop_state.items()
                if k == "teleop_armed"
            ),
        ]
        self._episode_start_wall = time.time()
        self._pause_started = None
        self._state = STATE_RECORDING
        self.get_logger().info(
            f"REC start episode{self._episode_index:06d} task={self._pending_task!r} "
            f"-> {self._episode_dir}"
        )

    def _end_episode(self, *, save: bool) -> None:
        self._state = STATE_SAVING
        ep_dir = self._episode_dir
        ep_idx = self._episode_index
        # A paused episode ends at the last recorded sample boundary, not when
        # the operator later resumes or stops it.
        end = self._pause_started if self._pause_started is not None else time.time()
        duration = end - self._episode_start_wall
        self._drain_buffers()
        writer = self._robot_writer
        stream_counts = writer.close() if writer else {}
        sensor_valid_counts = writer.sensor_valid_counts() if writer else {}
        sensor_temperature_valid_counts = writer.sensor_temperature_valid_counts() if writer else {}
        cam_counts = self._camera_writer.close() if self._camera_writer else {}
        self._robot_writer = None
        self._camera_writer = None

        if save and ep_dir is not None:
            self._session_bytes_accum += self._dir_bytes(ep_dir)
            self._events.append({"t": end, "name": "episode_end", "data": True})
            meta = {
                "package": "morla_data_collect",
                "package_version": __version__,
                "format": "raw_hdf5_v1",
                "timestamp_semantics": {"camera": "source_stamp_or_host_read_time",
                                        "camera_arrival": "host_wall_time",
                                        "command": "host_wall_arrival",
                                        "state": "ros_header_stamp_or_host_arrival"},
                "task": self._episode_task,
                "event_format": "effective_armed_v2",
                "control_event_format": "policy_owner_v1",
                "session": self._session,
                "schema": self._schema.to_dict(),
                "profile_id": self._profile_id or None,
                "robot_id": getattr(self._schema, "robot_id", "astral"),
                "schema_hash": getattr(self._schema, "schema_hash", None),
                "layout_hash": getattr(self._schema, "layout_hash", None),
                "kinematics_hash": getattr(getattr(self._schema, "frozen", None), "kinematics_hash", None),
                "coordinate_contract": (
                    getattr(self._schema, "frozen", None).metadata.get("coordinate_contract", {})
                    if getattr(self._schema, "frozen", None) is not None else {}
                ),
                "recording_mode": self._runtime_mode,
                "source_revisions": {
                    "description_revision": getattr(self._schema, "frozen", None).metadata.get("description_revision")
                    if getattr(self._schema, "frozen", None) is not None else None,
                    "kinematics_hash": getattr(getattr(self._schema, "frozen", None), "kinematics_hash", None),
                },
                "episode_index": ep_idx,
                "start_time_wall": self._episode_start_wall,
                "end_time_wall": end,
                "duration_s": duration,
                "events": self._events,
                "streams": {
                    name: {"dim": dim, "count": int(stream_counts.get(name, 0))}
                    for name, dim in self._streams.items()
                },
                "sensors": {
                    key: {
                        "count": int(stream_counts.get(f"sensor:{key}", 0)),
                        "finger_order": list(spec.get("finger_order", ())),
                        "valid_samples_by_finger": sensor_valid_counts.get(key, []),
                        "temperature_valid_samples_by_finger": sensor_temperature_valid_counts.get(key, []),
                        "force_unit": "N" if self._runtime_mode == "sim" else "unknown_sdk" if self._runtime_mode == "real" else "unknown",
                        "force_frame": "finger_link2" if self._runtime_mode == "sim" else "sensor_local" if self._runtime_mode == "real" else str(spec.get("force_frame", "unknown")),
                        "source_kind": "synthetic" if self._runtime_mode == "sim" else "real" if self._runtime_mode == "real" else "unknown",
                        "temperature_source_kind": "synthetic_zero" if self._runtime_mode == "sim" else "sdk_native" if self._runtime_mode == "real" else "unknown",
                        "temperature_scale": str(spec.get("temperature_scale", "unknown")),
                        "action_observation_only": bool(spec.get("action_observation_only", True)),
                        "point_count_per_finger": int(spec.get("point_count_per_finger", 120)),
                        "temperature_count_per_finger": 20,
                        "calibration_revision": str(spec.get("calibration_revision", "unknown")),
                    }
                    for key, spec in ((str(item["sensor_key"]), item) for item in self._sensor_specs)
                },
                "cameras": {
                    cam: {"count": int(cam_counts.get(cam, 0))}
                    for cam in self._schema.cameras
                },
                "dropped": dict(self._drop_counts),
            }
            with open(os.path.join(ep_dir, META_JSON), "w", encoding="utf-8") as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)
            self.get_logger().info(
                f"REC saved episode{ep_idx:06d} ({duration:.1f}s, "
                f"streams={sum(stream_counts.values())} samples, "
                f"images={sum(cam_counts.values())}) -> {ep_dir}"
            )
        else:
            if ep_dir and os.path.isdir(ep_dir):
                shutil.rmtree(ep_dir, ignore_errors=True)
            self.get_logger().info(f"REC discarded episode{ep_idx:06d} (deleted {ep_dir})")

        self._episode_dir = None
        self._episode_index = -1
        self._empty_warning = None  # 段结束清空告警（否则 IDLE 期卡片仍挂红条）
        self._state = STATE_IDLE

    # -- 写盘线程 ------------------------------------------------------------------

    def _writer_loop(self) -> None:
        while not self._writer_stop.is_set():
            with self._state_lock:
                recording = self._state == STATE_RECORDING
                if recording:
                    self._drain_buffers()
            time.sleep(0.02)

    def _drain_buffers(self) -> None:
        """把缓冲写进 HDF5。调用方须持 _state_lock 或处于写盘线程+RECORDING。"""
        if self._robot_writer is None:
            return
        with self._buf_lock:
            stream_items = {
                name: [dq.popleft() for _ in range(len(dq))]
                for name, dq in self._stream_buf.items()
                if dq
            }
            cam_items = {
                cam: [dq.popleft() for _ in range(len(dq))]
                for cam, dq in self._cam_buf.items()
                if dq
            }
        rw = self._robot_writer
        cw = self._camera_writer
        for name, items in stream_items.items():
            for ts, values in items:
                rw.write(name, values, ts)
        sensor_items = {}
        with self._buf_lock:
            sensor_items = {
                name: [dq.popleft() for _ in range(len(dq))]
                for name, dq in self._sensor_buf.items() if dq
            }
        for name, items in sensor_items.items():
            for ts, arrival, sample in items:
                rw.write_sensor(name, sample, ts, arrival)
        if cw is not None:
            for cam, items in cam_items.items():
                for ts, jpeg, arrival in items:
                    cw.write_image(cam, jpeg, ts, arrival)

    # -- 状态发布 ------------------------------------------------------------------

    def _publish_state(self) -> None:
        with self._state_lock:
            st = self._state
            ep_idx = self._episode_index
            elapsed = (
                time.time() - self._episode_start_wall
                if st in (STATE_RECORDING, STATE_PAUSED)
                else 0.0
            )
            stream_counts = (
                self._robot_writer.counts() if self._robot_writer else {}
            )
            cam_counts = (
                self._camera_writer.counts() if self._camera_writer else {}
            )
        # writer 每段新建（计数归零），delta 对段边界取 max(0,..) 防负速率
        prev = getattr(self, "_prev_counts", {})
        rates = {}
        for name, c in stream_counts.items():
            rates[name] = round(max(0.0, c - prev.get(name, 0)), 1)
        for cam, c in cam_counts.items():
            rates[f"cam:{cam}"] = round(max(0.0, c - prev.get(f"cam:{cam}", 0)), 1)
        self._prev_counts = {
            **{k: v for k, v in stream_counts.items()},
            **{f"cam:{k}": v for k, v in cam_counts.items()},
        }
        low_fps = _low_fps_warning(st, rates, self._schema)
        if (
            st == STATE_RECORDING
            and elapsed >= 2.0
            and not self._quiet_start_checked
        ):
            # 启动 2s 空录检查（段级一次性，阈值对齐 validate F5 的 2s）：
            # low_fps 只盯参考相机实率半速，管不到"源未就绪全 0"的空录。
            self._quiet_start_checked = True
            n_stream = sum(stream_counts.values())
            n_cam = sum(cam_counts.values())
            if n_stream == 0 and n_cam == 0:
                self._empty_warning = (
                    "录制 2s 未收到任何样本（数值/图像均 0）——"
                    "遥操作与相机链路可能都未就绪（是否忘了 armed？）"
                )
            elif n_stream == 0:
                self._empty_warning = (
                    "录制 2s 无数值样本（关节/末端流全 0）——teleop 未在发布？"
                )
            elif n_cam == 0:
                self._empty_warning = (
                    "录制 2s 无图像帧（相机流全 0）——相机抽头未在发布？"
                )
            if self._empty_warning:
                self.get_logger().warning(f"EMPTY-REC: {self._empty_warning}")
        # 实时采集量（web「本次采集数据」下拉）：写盘器存在（录制中）才有当前段
        # 计数；磁盘占用含 HDF5 chunk 预分配（"占了多少磁盘"口径）。
        rw = self._robot_writer
        cw = self._camera_writer
        recording = rw is not None
        ep_dir = self._episode_dir
        ep_bytes = (
            self._dir_bytes(ep_dir)
            if recording and ep_dir and os.path.isdir(ep_dir)
            else 0
        )
        payload = {
            "state": st,
            "profile_id": self._profile_id or None,
            "robot_id": getattr(self._schema, "robot_id", "astral"),
            "runtime_mode": self._runtime_mode,
            "session": self._session,
            "episode_index": ep_idx,
            "elapsed_s": round(elapsed, 1),
            "task_next": self._pending_task,
            "samples_per_s": rates,
            "dropped": dict(self._drop_counts),
            "schema": self._schema.to_dict(),
            "low_fps_warning": low_fps,
            "empty_warning": self._empty_warning,
            "ignored": dict(self._ignored_cmds),
            "folder": self._session_dir,
            "episode_counts": rw.counts() if recording else {},
            "camera_counts": cw.counts() if cw is not None else {},
            "episode_bytes": ep_bytes,
            "session_bytes": self._session_bytes_accum + ep_bytes,
        }
        if low_fps:
            self.get_logger().warning(
                f"LOW-FPS {low_fps}", throttle_duration_sec=5.0
            )
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._state_pub.publish(msg)

    # -- 关闭 ------------------------------------------------------------------

    def destroy_node(self) -> bool:
        with self._state_lock:
            if self._state in (STATE_RECORDING, STATE_PAUSED):
                self.get_logger().warning("shutdown while recording — discarding episode")
                self._end_episode(save=False)
        self._writer_stop.set()
        self._writer_thread.join(timeout=2.0)
        lock_fd = getattr(self, "_singleton_lock_fd", None)
        if lock_fd is not None:
            os.close(lock_fd)
            self._singleton_lock_fd = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DataCollectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
