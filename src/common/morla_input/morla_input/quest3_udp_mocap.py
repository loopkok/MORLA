#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseArray
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time as RosTime
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from morla_core.coordinates import UNITY_TO_QUEST3_CANONICAL

import json
import socket
import threading
import time
import numpy as np
import re
from typing import Optional, Tuple

from morla_input.latency_meter import LatencyMeter

# Quest opens one TCP socket per streamer (hands / head / controller / body).
_TCP_LISTEN_BACKLOG = 8
# No body packet for this long → head/wrist/controller are world again.
_IOBT_TIMEOUT_S = 0.4
# Hold head/wrist/controller publish until the frame is known: either a body
# packet arrives (→ operator body) or this many seconds pass with none (→ tracking world).
# Stops the startup world→body flip from reaching downstream (arm IK would
# capture its zero point in world then receive body-frame poses → cross-frame
# delta). Landmarks are wrist-local and not gated.
_WRIST_SETTLE_S = 1.0


def _stream_qos() -> QoSProfile:
    """腕/头/身体/Joy：只留最新一帧，避免下游 IK 吃到排队旧样本。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


def _latched_event_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class FPSCounter:
    """Simple sliding-window FPS counter."""
    def __init__(self, window=100, print_interval=5.0):
        self._window = window
        self._print_interval = print_interval
        self._times = []
        self._last_print = time.time()
    def tick(self):
        now = time.time()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times.pop(0)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / max(1e-9, self._times[-1] - self._times[0])
    def should_print(self):
        now = time.time()
        if now - self._last_print >= self._print_interval:
            self._last_print = now
            return True
        return False


class BandwidthMeter:
    """Bytes-per-second meter for a network link, printed on an interval."""
    def __init__(self, print_interval=2.0):
        self._print_interval = print_interval
        self._bytes = 0
        self._last_print = time.time()

    def add(self, n: int) -> None:
        self._bytes += int(n)

    def should_print(self):
        now = time.time()
        if now - self._last_print >= self._print_interval:
            return True
        return False

    def take_kbps(self) -> float:
        now = time.time()
        dt = max(1e-9, now - self._last_print)
        kbps = (self._bytes * 8.0 / dt) / 1000.0
        self._bytes = 0
        self._last_print = now
        return kbps


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion ``[x, y, z, w]``; return identity if near-zero."""
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Return conjugate of ``[x, y, z, w]`` (inverse for unit quaternions)."""
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product ``q1 ⊗ q2`` for ``[x, y, z, w]``."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector ``v`` by unit quaternion ``q`` (``[x, y, z, w]``)."""
    qv = np.array([v[0], v[1], v[2], 0.0], dtype=float)
    return _quat_multiply(_quat_multiply(q, qv), _quat_conjugate(q))[:3]


def pose_in_parent_frame(
    parent_pos: np.ndarray,
    parent_quat: np.ndarray,
    child_pos: np.ndarray,
    child_quat: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Express child world pose in parent frame: ``T_p_c = inv(T_w_p) @ T_w_c``.

    Computed in Unity axes; convert to robot with :func:`unity_pose_to_robot` after.
    """
    parent_quat = _quat_normalize(parent_quat)
    child_quat = _quat_normalize(child_quat)
    parent_inv = _quat_conjugate(parent_quat)
    rel_pos = _quat_rotate(parent_inv, child_pos - parent_pos)
    rel_quat = _quat_normalize(_quat_multiply(parent_inv, child_quat))
    return rel_pos, rel_quat


# Unity LH (X right, Y up, Z forward) → robot (X left, Y back, Z up):
#   (x, y, z)_r = (-x_u, -z_u, y_u)
# Compatibility alias; this is a handedness-changing basis conversion, not TF.
UNITY_TO_ROBOT = UNITY_TO_QUEST3_CANONICAL


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Unit quaternion ``[x, y, z, w]`` → 3×3 rotation matrix."""
    q = _quat_normalize(q)
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _matrix_to_quat(m: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → quaternion ``[x, y, z, w]``."""
    t = float(np.trace(m))
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize(np.array([x, y, z, w], dtype=float))


def unity_vec_to_robot(v: np.ndarray) -> np.ndarray:
    """Map Unity LH vector/point to robot (X left, Y back, Z up)."""
    return UNITY_TO_ROBOT @ np.asarray(v, dtype=float)


def unity_quat_to_robot(q: np.ndarray) -> np.ndarray:
    """Map Unity LH orientation to robot axes: ``R_r = M R_u M^T``."""
    r_u = _quat_to_matrix(q)
    r_r = UNITY_TO_ROBOT @ r_u @ UNITY_TO_ROBOT.T
    return _matrix_to_quat(r_r)


def unity_pose_to_robot(
    pos: np.ndarray, quat: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert Unity LH pose to robot (X left, Y back, Z up)."""
    return unity_vec_to_robot(pos), unity_quat_to_robot(quat)


def unity_landmarks_to_robot(points: np.ndarray) -> np.ndarray:
    """Convert (N,3) Unity LH points to robot axes."""
    return (UNITY_TO_ROBOT @ np.asarray(points, dtype=float).T).T


class Quest3UDPMocap(Node):
    def __init__(self):
        super().__init__("quest3_udp_mocap")
        self.get_logger().info("Quest 3 Hand Mocap Node Initialized")

        # 参数配置
        self.declare_parameter("protocol", "tcp_wired")  # "udp" | "tcp_wired" | "tcp_wireless"
        self.declare_parameter("udp_port", 9000)
        self.declare_parameter("tcp_port", 8000)
        self.declare_parameter("ema_alpha", 0.7)
        self.declare_parameter("viz", True)
        self.declare_parameter("arm_side", "right")
        # Unified input is wrist-local raw in robot axes. Robot-specific MANO
        # and finger-length adaptation belongs in that robot's retargeter.
        self.declare_parameter("landmark_preprocess", "raw")
        # Unity LH → robot (X left, Y back, Z up). If false: keep Unity axes
        # (landmarks only apply legacy X flip).
        self.declare_parameter("convert_to_robot", True)
        self.declare_parameter("coordinate_basis_id", "quest3_canonical_legacy_v1")
        self.declare_parameter("coordinate_contract_version", 0)
        self.declare_parameter("kinematics_hash", "")
        # Mixed/IOBT: Touch 6DoF also published on quest3/{side}_wrist_pose so
        # existing arm IK keeps tracking the held controller.
        self.declare_parameter("controller_as_wrist", True)
        self.declare_parameter("max_packet_age_s", 0.5)
        self.declare_parameter("allow_legacy_packets", False)
        # When glove owns a side, skip Quest landmarks on that topic (wrist still published).
        self.declare_parameter("publish_landmarks_left", True)
        self.declare_parameter("publish_landmarks_right", True)

        self.protocol = self.get_parameter("protocol").value.lower()
        self.udp_port = self.get_parameter("udp_port").value
        self.tcp_port = self.get_parameter("tcp_port").value
        self.ema_alpha = self.get_parameter("ema_alpha").value
        self.viz = self.get_parameter("viz").value
        self.arm_side = self.get_parameter("arm_side").value.lower()
        self.both = self.arm_side == "both"
        self.landmark_preprocess = str(
            self.get_parameter("landmark_preprocess").value
        ).strip().lower()
        if self.landmark_preprocess != "raw":
            raise ValueError(
                "morla_input publishes only unified raw landmarks; run MANO "
                "preprocessing in the robot-specific retargeter"
            )
        self.convert_to_robot = bool(self.get_parameter("convert_to_robot").value)
        self.coordinate_basis_id = str(self.get_parameter("coordinate_basis_id").value).strip()
        self.coordinate_contract_version = int(self.get_parameter("coordinate_contract_version").value)
        self.kinematics_hash = str(self.get_parameter("kinematics_hash").value)
        expected_basis = (
            "quest3_canonical_legacy_v1" if self.convert_to_robot else "unity_left_handed_v1"
        )
        if self.coordinate_basis_id != expected_basis:
            raise ValueError(
                f"coordinate_basis_id={self.coordinate_basis_id!r} conflicts with "
                f"convert_to_robot={self.convert_to_robot}; expected {expected_basis!r}"
            )
        from morla_input.packet_guard import PacketGuard
        self._packet_guard = PacketGuard(float(self.get_parameter("max_packet_age_s").value),
                                         bool(self.get_parameter("allow_legacy_packets").value))
        if self._packet_guard.allow_legacy:
            self.get_logger().warning(
                "LEGACY QUEST PACKETS ENABLED: source age, sequence and session "
                "checks are bypassed for unwrapped packets"
            )
        self.controller_as_wrist = bool(self.get_parameter("controller_as_wrist").value)
        self.publish_landmarks_left = bool(
            self.get_parameter("publish_landmarks_left").value
        )
        self.publish_landmarks_right = bool(
            self.get_parameter("publish_landmarks_right").value
        )
        self.declare_parameter("print_latency", True)
        self.declare_parameter("latency_print_interval", 2.0)
        self._print_latency = bool(self.get_parameter("print_latency").value)
        self._lat = LatencyMeter(
            float(self.get_parameter("latency_print_interval").value)
        )
        self._last_wrist_arrival = {"left": 0.0, "right": 0.0}
        self._bw = BandwidthMeter(
            print_interval=float(self.get_parameter("latency_print_interval").value)
        )

        self.get_logger().info(
            f"Quest 3 Hand Mocap: side=[{self.arm_side.upper()}], "
            f"protocol={self.protocol}, "
            f"preprocess={self.landmark_preprocess}, "
            f"convert_to_robot={self.convert_to_robot}, basis={self.coordinate_basis_id}, "
            f"contract={self.coordinate_contract_version}, kinematics={self.kinematics_hash[:12]}, "
            f"controller_as_wrist={self.controller_as_wrist}, "
            f"landmarks_L={self.publish_landmarks_left}, "
            f"landmarks_R={self.publish_landmarks_right}"
        )

        # 缓存用于 EMA 平滑滤波
        self.landmark_cache_right = np.zeros((21, 3))
        self.landmark_cache_left = np.zeros((21, 3))

        # World frame for hips / (head,wrist,controller when IOBT is off).
        # When IOBT is on, Quest already expressed head/hand/controller in the
        # hips frame — those use quest3_operator_body_* instead.
        self._world_frame_id = (
            "quest3_tracking_world_canonical" if self.convert_to_robot
            else "quest3_tracking_world_unity"
        )
        self._body_frame_id = (
            "quest3_operator_body_canonical" if self.convert_to_robot
            else "quest3_operator_body_unity"
        )
        self._iobt_lock = threading.Lock()
        self._last_iobt_time = 0.0
        # Sticky body-frame latch: once a body packet has been seen, head/wrist/
        # controller stay in the hips (torso) frame for the rest of the run, so
        # transient UDP packet loss does not flip them back to the world frame.
        self._body_ever_seen = False
        self._logged_iobt_fidelity = ""
        self._wrist_src = {"left": "none", "right": "none"}
        self._published_pose_frame = None
        # Hold gate: 0 until the first head/wrist/controller packet, then the
        # wall-clock time of that first packet. Pose publish is suppressed until
        # _pose_frame_settled() is True (body seen, or _WRIST_SETTLE_S elapsed).
        self._first_pose_time = 0.0

        # 发布者：始终按左右分 topic
        # Frame tree (IOBT off):
        #   quest3_tracking_world_* ── head / wrist|controller
        # Frame tree (IOBT on):
        #   quest3_tracking_world_* ── hips
        #   quest3_operator_body_* ── head / wrist|controller / body_joints
        #                                └── landmarks (wrist-local, unchanged)
        stream_qos = _stream_qos()
        self.mocap_pub_right = self.create_publisher(
            PoseArray, "hand_landmarks/right", stream_qos
        )
        self.mocap_pub_left = self.create_publisher(
            PoseArray, "hand_landmarks/left", stream_qos
        )
        self.wrist_pub_right = self.create_publisher(
            PoseStamped, "quest3/right_wrist_pose", stream_qos
        )
        self.wrist_pub_left = self.create_publisher(
            PoseStamped, "quest3/left_wrist_pose", stream_qos
        )
        self.ctrl_pub_right = self.create_publisher(
            PoseStamped, "quest3/right_controller_pose", stream_qos
        )
        self.ctrl_pub_left = self.create_publisher(
            PoseStamped, "quest3/left_controller_pose", stream_qos
        )
        self.head_pub = self.create_publisher(PoseStamped, "quest3/head_pose", stream_qos)
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.body_pub = self.create_publisher(
            PoseArray, "quest3/body_joints", stream_qos
        )
        self.body_names_pub = self.create_publisher(
            String, "quest3/body_joint_names", latch_qos
        )
        self.hips_pub = self.create_publisher(
            PoseStamped, "quest3/hips_pose", stream_qos
        )
        self.mix_pub = self.create_publisher(String, "quest3/input_mix", latch_qos)
        # A session/frame change invalidates motion anchors in all consumers.
        self.input_epoch_pub = self.create_publisher(
            String, "quest3/input_epoch", _latched_event_qos()
        )
        # Touch controller button/thumbstick state (sensor_msgs/Joy).
        # axes = [trigger, grip, stickX, stickY]; buttons = [primary, secondary,
        # stickPress, menu, triggerClick, gripClick].
        self.joy_pub_right = self.create_publisher(
            Joy, "quest3/right_controller_joy", stream_qos
        )
        self.joy_pub_left = self.create_publisher(
            Joy, "quest3/left_controller_joy", stream_qos
        )
        
        # 新增：RViz 可视化发布者
        if self.viz:
            self.right_marker_pub = self.create_publisher(MarkerArray, "quest3/right_hand_markers", 10)
            self.left_marker_pub = self.create_publisher(MarkerArray, "quest3/left_hand_markers", 10)
            self.marker_pub = self.create_publisher(MarkerArray, f"quest3/{self.arm_side}_hand_markers", 10)

        # 腕部数据的正则解析器
        self.wrist_pattern = re.compile(r'wrist:,?\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)')

        self.is_running = True

        # ---- Network listener (UDP / TCP wired / TCP wireless) ----
        if self.protocol == "udp":
            self._start_udp_listener()
        elif self.protocol in ("tcp_wired", "tcp_wireless"):
            self._start_tcp_listener()
        else:
            self.get_logger().error(f"Unknown protocol: {self.protocol}")
            raise ValueError(f"Unknown protocol: {self.protocol}")

    def _start_udp_listener(self):
        """Start UDP listener thread."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('0.0.0.0', self.udp_port))
        self.socket.settimeout(0.5)
        self.get_logger().info(f"Listening for UDP data on port {self.udp_port}...")
        self.thread = threading.Thread(target=self.udp_listener, daemon=True)
        self.thread.start()

    def _start_tcp_listener(self):
        """Start TCP server listener thread."""
        host = "localhost" if self.protocol == "tcp_wired" else "0.0.0.0"
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_server.bind((host, self.tcp_port))
        self.tcp_server.listen(_TCP_LISTEN_BACKLOG)
        self.tcp_server.settimeout(1.0)
        mode = "wired (adb reverse)" if self.protocol == "tcp_wired" else "wireless (WiFi)"
        self.get_logger().info(
            f"TCP server listening on {host}:{self.tcp_port} ({mode})"
        )
        if self.protocol == "tcp_wired":
            self.get_logger().info(
                "  On Quest3: adb reverse tcp:{0} tcp:{0}".format(self.tcp_port)
            )
        self.thread = threading.Thread(target=self.tcp_listener, daemon=True)
        self.thread.start()

    def process_landmarks(self, landmarks, hand_label):
        """Center the one unified raw 21-point stream on its wrist."""
        points = np.asarray(landmarks, dtype=float)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError("Quest landmarks must be finite with shape (21, 3)")
        return points - points[0:1, :]

    def _process_network_line(self, line, arrival_time, arrival_monotonic=None):
        accepted = self._packet_guard.accept(
            line, received=arrival_time, received_monotonic=arrival_monotonic
        )
        if accepted:
            changed = self._packet_guard.last_session_change
            if changed:
                self._signal_input_epoch("session_changed", changed)
            self.process_line(*accepted)
        elif time.monotonic() - getattr(self, '_last_packet_warning', 0) > 5.0:
            self._last_packet_warning = time.monotonic()
            self.get_logger().warn(
                f'Quest input rejected ({self._packet_guard.rejected}): unsynchronized, '
                'stale, duplicate/out-of-order, or legacy packet; install the monotonic-clock APK')

    def _handle_peer_line(self, line, arrival_time, arrival_monotonic, peer_key, send_bytes):
        """Handle Quest sync control lines and timestamped telemetry per peer."""
        if line.startswith('@q3sync_ack '):
            if self._packet_guard.handle_sync_ack(
                line, peer_key, received_ns=int(arrival_monotonic * 1e9)
            ):
                self.get_logger().info(f'Quest monotonic clock synchronized for {peer_key}')
            return
        if line.startswith('@q3m '):
            challenge = self._packet_guard.make_sync_challenge(
                peer_key, now_ns=int(arrival_monotonic * 1e9)
            )
            if challenge:
                send_bytes(challenge.encode('ascii'))
        self._process_network_line(line, arrival_time, arrival_monotonic)

    def udp_listener(self):
        """UDP listener thread."""
        while self.is_running and rclpy.ok():
            try:
                data, remote = self.socket.recvfrom(65536)
                arrival_time = time.time()
                arrival_monotonic = time.monotonic()
                self._bw.add(len(data))
                message = data.decode('utf-8')
                peer_key = f'udp:{remote[0]}:{remote[1]}'
                for line in message.splitlines():
                    if not line:
                        continue
                    self._handle_peer_line(
                        line, arrival_time, arrival_monotonic, peer_key,
                        lambda payload: self.socket.sendto(payload, remote),
                    )
                if self._bw.should_print():
                    self.get_logger().info(
                        f"[Mocap Downlink] {self._bw.take_kbps():.1f} kbps"
                    )
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP Listener Error: {e}")

    def tcp_listener(self):
        """TCP server thread — accept connections and handle each."""
        while self.is_running and rclpy.ok():
            try:
                conn, addr = self.tcp_server.accept()
                self.get_logger().info(f"TCP connection from {addr}")
                t = threading.Thread(
                    target=self._tcp_connection_handler,
                    args=(conn, addr), daemon=True,
                )
                t.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f"TCP accept error: {e}")

    def _tcp_connection_handler(self, conn, addr):
        """Handle a single TCP connection — read lines and process."""
        buf = ""
        peer_key = f'tcp:{addr[0]}:{addr[1]}'
        with conn:
            try:
                challenge = self._packet_guard.make_sync_challenge(peer_key, force=True)
                conn.sendall(challenge.encode('ascii'))
            except OSError:
                return
            while self.is_running and rclpy.ok():
                try:
                    data = conn.recv(65536)
                    if not data:
                        self.get_logger().info(f"TCP {addr} disconnected")
                        break
                    arrival_time = time.time()
                    arrival_monotonic = time.monotonic()
                    self._bw.add(len(data))
                    buf += data.decode('utf-8')
                    if len(buf) > 1048576:
                        raise ValueError("oversized Quest frame")
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        if line.strip():
                            self._handle_peer_line(
                                line.strip(), arrival_time, arrival_monotonic, peer_key,
                                conn.sendall,
                            )
                    if self._bw.should_print():
                        self.get_logger().info(
                            f"[Mocap Downlink] {self._bw.take_kbps():.1f} kbps"
                        )
                except Exception as e:
                    self.get_logger().error(f"TCP read error from {addr}: {e}")
                    break

    def _float_to_ros_time(self, t: float) -> RosTime:
        """Convert wall-clock float seconds to ROS Time message."""
        stamp = RosTime()
        stamp.sec = int(t)
        stamp.nanosec = int((t - int(t)) * 1e9)
        return stamp

    def _maybe_log_latency(self) -> None:
        if not self._print_latency or not self._lat.has_samples():
            return
        if not self._lat.should_print():
            return
        self.get_logger().info(f"[Latency][VR] {self._lat.format_and_reset()}")

    def _iobt_active(self) -> bool:
        with self._iobt_lock:
            # Sticky: once IOBT body data has been seen, treat IOBT as active for
            # the rest of the run so transient UDP packet loss does not flip
            # head/wrist/controller back to the world frame mid-stream.
            return self._body_ever_seen or (time.time() - self._last_iobt_time) < _IOBT_TIMEOUT_S

    def _mark_iobt(self) -> None:
        with self._iobt_lock:
            self._last_iobt_time = time.time()
            self._body_ever_seen = True

    def _signal_input_epoch(self, reason: str, details: dict | None = None) -> None:
        """Invalidate receiver caches and notify the common control gate."""
        self.landmark_cache_right.fill(0.0)
        self.landmark_cache_left.fill(0.0)
        self._last_wrist_arrival = {"left": 0.0, "right": 0.0}
        self._wrist_src = {"left": "none", "right": "none"}
        self._published_pose_frame = None
        with self._iobt_lock:
            self._last_iobt_time = 0.0
            self._body_ever_seen = False
            self._first_pose_time = 0.0
        payload = {"reason": reason, "time": time.time()}
        if details:
            payload.update(details)
        self.input_epoch_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.get_logger().warning(
            f"Quest input epoch changed ({reason}); cleared input caches and "
            "requested downstream disarm/restart"
        )

    def _pose_frame_settled(self) -> bool:
        """True once the head/wrist/controller frame is known.

        Suppresses pose publish until either a body packet has arrived (frame is
        quest3_operator_body_*) or _WRIST_SETTLE_S has elapsed with no body packet (frame is
        world). Prevents the startup world→body flip from reaching downstream.
        """
        with self._iobt_lock:
            if self._body_ever_seen:
                return True
            if self._first_pose_time == 0.0:
                self._first_pose_time = time.time()
                return False
            return (time.time() - self._first_pose_time) >= _WRIST_SETTLE_S

    def _pose_frame_id(self) -> str:
        """Parent of head / wrist / controller (and hips-relative body joints)."""
        return self._body_frame_id if self._iobt_active() else self._world_frame_id

    def _unity_to_out(
        self, pos_u: np.ndarray, quat_u: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        if self.convert_to_robot:
            return unity_pose_to_robot(pos_u, quat_u)
        return pos_u, quat_u

    @staticmethod
    def _parse_pose7(line: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        parts = line.split(",")
        if len(parts) < 7:
            return None
        try:
            vals = [float(p.strip()) for p in parts[-7:]]
        except ValueError:
            return None
        pos = np.array(vals[0:3], dtype=float)
        quat = _quat_normalize(np.array(vals[3:7], dtype=float))
        return pos, quat

    def _make_pose_stamped(
        self,
        pos: np.ndarray,
        quat: np.ndarray,
        arrival_time: float,
        frame_id: str,
    ) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self._float_to_ros_time(arrival_time)
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def _note_mix(self, side: str, src: str) -> None:
        tag = "ctrl" if src == "controller" else src
        last = self._wrist_src.get(side)
        if last == tag:
            return
        self._wrist_src[side] = tag
        mix = f"left={self._wrist_src['left']} right={self._wrist_src['right']}"
        self.get_logger().info(f"[HTS] mix {mix}")
        msg = String()
        msg.data = mix
        self.mix_pub.publish(msg)

    def _note_wrist_gap(self, side: str, arrival_time: float) -> None:
        if not self._print_latency or arrival_time <= 0.0:
            return
        self._lat.add("recv_to_pub", (time.time() - arrival_time) * 1000.0)
        prev = self._last_wrist_arrival[side]
        if prev > 0.0:
            self._lat.add(f"wrist_gap.{side}", (arrival_time - prev) * 1000.0)
        self._last_wrist_arrival[side] = arrival_time
        self._maybe_log_latency()

    def _publish_side_pose(
        self,
        side: str,
        pos: np.ndarray,
        quat: np.ndarray,
        arrival_time: float,
        src: str,
    ) -> None:
        if not self._pose_frame_settled():
            return  # hold until head/wrist/controller frame is known
        frame_id = self._pose_frame_id()
        if self._published_pose_frame and frame_id != self._published_pose_frame:
            old_frame = self._published_pose_frame
            self._signal_input_epoch(
                "coordinate_frame_changed", {
                    "old_frame": old_frame, "new_frame": frame_id,
                    "coordinate_basis_id": self.coordinate_basis_id,
                    "coordinate_contract_version": self.coordinate_contract_version,
                    "kinematics_hash": self.kinematics_hash,
                }
            )
            # Drop the transition sample; a fresh sample must follow the event.
            return
        self._published_pose_frame = frame_id
        pose_msg = self._make_pose_stamped(
            pos, quat, arrival_time, frame_id
        )
        if src == "controller":
            (
                self.ctrl_pub_right if side == "right" else self.ctrl_pub_left
            ).publish(pose_msg)
            if self.controller_as_wrist:
                (
                    self.wrist_pub_right if side == "right" else self.wrist_pub_left
                ).publish(pose_msg)
        else:
            (
                self.wrist_pub_right if side == "right" else self.wrist_pub_left
            ).publish(pose_msg)
        self._note_mix(side, src)
        self._note_wrist_gap(side, arrival_time)

    def _process_buttons_line(self, line: str, arrival_time: float) -> None:
        """Parse ``Left buttons:, trigger, grip, stickX, stickY, mask`` → Joy.

        mask bits: 0=primary(X/A) 1=secondary(Y/B) 2=thumbstick press
                    3=menu 4=trigger click 5=grip click
        axes=[trigger, grip, stickX, stickY], buttons=[each mask bit 0/1].
        """
        try:
            _, _, payload = line.partition(":")
            parts = [p.strip() for p in payload.split(",") if p.strip()]
            if len(parts) < 5:
                return
            trigger = float(parts[0])
            grip = float(parts[1])
            stick_x = float(parts[2])
            stick_y = float(parts[3])
            # parts = [trigger, grip, stickX, stickY, mask] (5 fields after the
            # "buttons:" tag). mask is parts[4]; the earlier `len < 5` guard
            # already ensured at least 5 fields, so read it unconditionally.
            mask = int(float(parts[4])) if len(parts) >= 5 else 0
        except (ValueError, IndexError):
            return

        if not np.isfinite([trigger, grip, stick_x, stick_y]).all() or not 0 <= mask <= 63:
            return
        side = "right" if "right" in line.lower() else "left"
        msg = Joy()
        msg.header.stamp = self._float_to_ros_time(arrival_time)
        msg.header.frame_id = self._pose_frame_id()
        msg.axes = [trigger, grip, stick_x, stick_y]
        msg.buttons = [(mask >> i) & 1 for i in range(6)]
        (self.joy_pub_right if side == "right" else self.joy_pub_left).publish(msg)

    def _process_body_line(self, line: str, arrival_time: float) -> None:
        """Parse ``body iobt | fid=High: hips,x,y,z,qx,qy,qz,qw spine-lower,...``.

        Quest streams hips in Unity world (re-axed torso frame) and every other
        joint already expressed in the hips frame — matching the body-relative
        head / hand / controller packets. Hips is published in world; the
        PoseArray (hips as identity root + body joints) shares the Quest operator-body frame.
        """
        self._mark_iobt()
        prefix, _, rest = line.partition(":")
        fid = ""
        if "fid=" in prefix.lower():
            fid = prefix.lower().split("fid=", 1)[1].split()[0].strip(" |")
        if fid and fid != self._logged_iobt_fidelity:
            self._logged_iobt_fidelity = fid
            self.get_logger().info(f"[HTS] IOBT fidelity={fid}")

        names = []
        unity_poses = []  # (name, pos_u, quat_u)
        hips_u = None
        for tok in rest.split():
            parts = tok.split(",")
            if len(parts) != 8:
                continue
            name = parts[0]
            try:
                pos_u = np.array(
                    [float(parts[1]), float(parts[2]), float(parts[3])], dtype=float
                )
                quat_u = _quat_normalize(
                    np.array(
                        [
                            float(parts[4]),
                            float(parts[5]),
                            float(parts[6]),
                            float(parts[7]),
                        ],
                        dtype=float,
                    )
                )
            except ValueError:
                continue
            unity_poses.append((name, pos_u, quat_u))
            if name == "hips":
                hips_u = (pos_u, quat_u)

        if not unity_poses:
            return

        stamp = self._float_to_ros_time(arrival_time)
        if hips_u is not None:
            hips_pos, hips_quat = self._unity_to_out(*hips_u)
            hips = PoseStamped()
            hips.header.stamp = stamp
            hips.header.frame_id = self._world_frame_id
            hips.pose.position.x = float(hips_pos[0])
            hips.pose.position.y = float(hips_pos[1])
            hips.pose.position.z = float(hips_pos[2])
            hips.pose.orientation.x = float(hips_quat[0])
            hips.pose.orientation.y = float(hips_quat[1])
            hips.pose.orientation.z = float(hips_quat[2])
            hips.pose.orientation.w = float(hips_quat[3])
            self.hips_pub.publish(hips)

        poses = []
        for name, pos_u, quat_u in unity_poses:
            if name == "hips":
                # Hips is streamed in world; in the body frame it is the root (identity).
                pos_u = np.zeros(3, dtype=float)
                quat_u = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
            # Non-hips joints are already hips-relative on the wire (Quest side
            # converted them), so no pose_in_parent_frame here — just axis remap.
            pos, quat = self._unity_to_out(pos_u, quat_u)
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            names.append(name)
            poses.append(pose)

        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = (
            self._body_frame_id if hips_u is not None else self._world_frame_id
        )
        msg.poses = poses
        self.body_pub.publish(msg)
        names_msg = String()
        names_msg.data = json.dumps(names)
        self.body_names_pub.publish(names_msg)

    def process_line(self, line, arrival_time: float = 0.0):
        # FPS tracking
        if not hasattr(self, '_fps'):
            self._fps = FPSCounter(window=100, print_interval=5.0)
        fps = self._fps.tick()
        if self._fps.should_print():
            self.get_logger().info(f"[VR Data Rate] {fps:.0f} Hz")

        line_lower = line.lower()

        # IOBT body packet contains a "head" joint — must run before head match.
        if line_lower.startswith("body"):
            self._process_body_line(line, arrival_time)
            return

        # Touch controller buttons/thumbstick: "Left buttons:, trigger, grip,
        # stickX, stickY, mask". Published as sensor_msgs/Joy. Handled before
        # the side-filter so both controllers are always forwarded.
        if "buttons" in line_lower:
            self._process_buttons_line(line, arrival_time)
            return

        # Head / HMD. IOBT on: already hips-relative on the wire.
        if "head" in line_lower or "hmd" in line_lower:
            parsed = self._parse_pose7(line)
            if parsed is not None and self._pose_frame_settled():
                try:
                    pos, quat = self._unity_to_out(*parsed)
                    self.head_pub.publish(
                        self._make_pose_stamped(
                            pos, quat, arrival_time, self._pose_frame_id()
                        )
                    )
                except Exception:
                    pass
            return

        side = "right" if "right" in line_lower else "left"

        # 过滤：非 both 模式下只处理指定手
        if not self.both and side != self.arm_side:
            return

        # Wrist or Touch controller (same 7-float pose). Mixed: one side can be
        # controller while the other is hand. Same side: Quest sends only one.
        if "wrist" in line_lower or "controller" in line_lower:
            parsed = self._parse_pose7(line)
            if parsed is not None:
                try:
                    pos, quat = self._unity_to_out(*parsed)
                    src = "controller" if "controller" in line_lower else "hand"
                    self._publish_side_pose(side, pos, quat, arrival_time, src)
                except Exception as e:
                    self.get_logger().error(f"解析浮点数失败: {e}")
        # 2. 手指：Unity 腕局部 → robot 轴，再 raw/mano + EMA
        if "landmarks" in line.lower():
            parts = line.split(":")
            if len(parts) < 2: return
            values_str = parts[1].split(",")
            values = [float(v.strip()) for v in values_str if v.strip()]

            if len(values) == 63:
                landmarks = np.array(values).reshape(21, 3)
                if self.convert_to_robot:
                    # Full Unity→robot axis map (X left, Y back, Z up).
                    mapped = unity_landmarks_to_robot(landmarks)
                else:
                    # Legacy: X-only flip, keep Unity Y/Z.
                    mapped = np.zeros_like(landmarks)
                    mapped[:, 0] = -landmarks[:, 0]
                    mapped[:, 1] = landmarks[:, 1]
                    mapped[:, 2] = landmarks[:, 2]
                processed_landmarks = self.process_landmarks(mapped, side)
                self.publish_mocap_data(processed_landmarks, side, arrival_time)
    def publish_hand_markers(self, hand_data, hand):
        """新增：将关节点转化为 RViz 的球体列表"""
        m = Marker()
        m.header.frame_id = 'world'  # RViz 的固定坐标系
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f"hand_{hand}"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.015; m.scale.y = 0.015; m.scale.z = 0.015
        m.color.a = 1.0
        
        if hand == "right":
            m.color.r = 0.9; m.color.g = 0.2; m.color.b = 0.2 # 右手红色
            publisher = self.right_marker_pub
        else:
            m.color.r = 0.2; m.color.g = 0.6; m.color.b = 0.9 # 左手蓝色
            publisher = self.left_marker_pub
            
        for i in range(hand_data.shape[0]):
            point = Point(x=float(hand_data[i, 0]), y=float(hand_data[i, 1]), z=float(hand_data[i, 2]))
            m.points.append(point)
            
        ma = MarkerArray()
        ma.markers.append(m)
        publisher.publish(ma)

    def publish_mocap_data(self, landmarks, side, arrival_time: float = 0.0):
        if side == "right" and not self.publish_landmarks_right:
            return
        if side == "left" and not self.publish_landmarks_left:
            return
        if side == 'right':
            self.landmark_cache_right = self.ema_alpha * landmarks + (1 - self.ema_alpha) * self.landmark_cache_right
            landmarks = self.landmark_cache_right
        elif side == 'left':
            self.landmark_cache_left = self.ema_alpha * landmarks + (1 - self.ema_alpha) * self.landmark_cache_left
            landmarks = self.landmark_cache_left
        else:
            return

        msg = PoseArray()
        msg.header.stamp = self._float_to_ros_time(arrival_time)
        msg.header.frame_id = f"hand_{side}"
        msg.poses = [
            Pose(
                position=Point(x=float(lm[0]), y=float(lm[1]), z=float(lm[2])),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ) for lm in landmarks
        ]

        if side == "right":
            self.mocap_pub_right.publish(msg)
        else:
            self.mocap_pub_left.publish(msg)

        if self.viz:
            self.publish_hand_markers(landmarks, side)

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'socket') and self.socket:
            self.socket.close()
        if hasattr(self, 'tcp_server') and self.tcp_server:
            self.tcp_server.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Quest3UDPMocap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
