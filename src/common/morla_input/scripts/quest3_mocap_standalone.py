#!/usr/bin/env python3
"""Standalone Quest3 HTS mocap receiver (no ROS).

Mirrors ``morla_input.quest3_udp_mocap`` processing:

Frame tree (Unity LH → optional robot: X left, Y back, Z up)::

    IOBT off:
        quest3_tracking_world_* ── head / wrist|controller
                                     └── landmarks  (wrist-local + raw|mano + EMA)
    IOBT on:
        quest3_tracking_world_* ── hips
        quest3_operator_body_* ── head / wrist|controller / body joints
                                     └── landmarks  (still wrist-local)

Examples::

    python scripts/quest3_mocap_standalone.py --protocol tcp_wired --arm-side both
    python scripts/quest3_mocap_standalone.py --protocol udp --port 9000 --viz
    python scripts/quest3_mocap_standalone.py --landmark-preprocess raw --jsonl out.jsonl
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

import numpy as np

from morla_core.coordinates import UNITY_TO_QUEST3_CANONICAL

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from morla_input.packet_guard import PacketGuard

LOG = logging.getLogger("quest3_mocap")

OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=float,
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ],
    dtype=float,
)

# MediaPipe / OpenXR-style finger connections for viz
HAND_BONES = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
)

BODY_BONES = (
    ("hips", "spine-lower"),
    ("spine-lower", "spine-middle"),
    ("spine-middle", "spine-upper"),
    ("spine-upper", "chest"),
    ("chest", "neck"),
    ("neck", "head"),
    ("chest", "left-shoulder"),
    ("left-shoulder", "left-scapula"),
    ("left-scapula", "left-arm-upper"),
    ("left-arm-upper", "left-arm-lower"),
    ("chest", "right-shoulder"),
    ("right-shoulder", "right-scapula"),
    ("right-scapula", "right-arm-upper"),
    ("right-arm-upper", "right-arm-lower"),
    ("hips", "left-upper-leg"),
    ("left-upper-leg", "left-lower-leg"),
    ("left-lower-leg", "left-foot-ankle"),
    ("left-foot-ankle", "left-foot-ball"),
    ("hips", "right-upper-leg"),
    ("right-upper-leg", "right-lower-leg"),
    ("right-lower-leg", "right-foot-ankle"),
    ("right-foot-ankle", "right-foot-ball"),
)

_TCP_LISTEN_BACKLOG = 8
_IOBT_TIMEOUT_S = 0.4


# ---------------------------------------------------------------------------
# Math helpers (same as ROS node)
# ---------------------------------------------------------------------------


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
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
    qv = np.array([v[0], v[1], v[2], 0.0], dtype=float)
    return _quat_multiply(_quat_multiply(q, qv), _quat_conjugate(q))[:3]


def pose_in_parent_frame(
    parent_pos: np.ndarray,
    parent_quat: np.ndarray,
    child_pos: np.ndarray,
    child_quat: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """``T_p_c = inv(T_w_p) @ T_w_c`` in Unity axes; convert with unity_pose_to_robot after."""
    parent_quat = _quat_normalize(parent_quat)
    child_quat = _quat_normalize(child_quat)
    parent_inv = _quat_conjugate(parent_quat)
    rel_pos = _quat_rotate(parent_inv, child_pos - parent_pos)
    rel_quat = _quat_normalize(_quat_multiply(parent_inv, child_quat))
    return rel_pos, rel_quat


# Unity LH (X right, Y up, Z forward) → robot (X left, Y back, Z up):
#   (x, y, z)_r = (-x_u, -z_u, y_u)
UNITY_TO_ROBOT = UNITY_TO_QUEST3_CANONICAL


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
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
    return UNITY_TO_ROBOT @ np.asarray(v, dtype=float)


def unity_quat_to_robot(q: np.ndarray) -> np.ndarray:
    r_u = _quat_to_matrix(q)
    r_r = UNITY_TO_ROBOT @ r_u @ UNITY_TO_ROBOT.T
    return _matrix_to_quat(r_r)


def unity_pose_to_robot(
    pos: np.ndarray, quat: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    return unity_vec_to_robot(pos), unity_quat_to_robot(quat)


def unity_landmarks_to_robot(points: np.ndarray) -> np.ndarray:
    return (UNITY_TO_ROBOT @ np.asarray(points, dtype=float).T).T


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


def adaptive_retargeting_xhand(landmarks: np.ndarray) -> np.ndarray:
    """XHand-only pinky stretch (keep off for Wuji)."""
    landmarks = landmarks.copy()
    pinky_mcp, pinky_pip, pinky_dip, pinky_tip = 17, 18, 19, 20
    pinky_extension = np.linalg.norm(landmarks[pinky_tip] - landmarks[pinky_mcp])
    extension_ratio = np.clip((pinky_extension - 0.03) / (0.10 - 0.03), 0.0, 1.0)
    adaptive_scale = 1.2 + (2.2 - 1.2) * extension_ratio

    landmarks[pinky_pip] = landmarks[pinky_mcp] + (
        landmarks[pinky_pip] - landmarks[pinky_mcp]
    ) * adaptive_scale
    landmarks[pinky_dip] = landmarks[pinky_pip] + (
        landmarks[pinky_dip] - landmarks[pinky_pip]
    ) * adaptive_scale
    landmarks[pinky_tip] = landmarks[pinky_dip] + (
        landmarks[pinky_tip] - landmarks[pinky_dip]
    ) * adaptive_scale
    return landmarks


def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
    assert keypoint_3d_array.shape == (21, 3)
    points = keypoint_3d_array[[0, 5, 9], :]
    x_vector = points[0] - points[2]
    centered = points - np.mean(points, axis=0, keepdims=True)
    _, _, v = np.linalg.svd(centered)
    normal = v[2, :]
    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / np.linalg.norm(x)
    z = np.cross(x, normal)
    if np.sum(z * (centered[1] - centered[2])) < 0:
        normal *= -1
        z *= -1
    return np.stack([x, normal, z], axis=1)


class FPSCounter:
    def __init__(self, window: int = 100, print_interval: float = 5.0):
        self._window = window
        self._print_interval = print_interval
        self._times: list[float] = []
        self._last_print = time.time()

    def tick(self) -> float:
        now = time.time()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times.pop(0)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / max(1e-9, self._times[-1] - self._times[0])

    def should_print(self) -> bool:
        now = time.time()
        if now - self._last_print >= self._print_interval:
            self._last_print = now
            return True
        return False


# ---------------------------------------------------------------------------
# Output payloads
# ---------------------------------------------------------------------------


@dataclass
class Pose6D:
    """Position + quaternion ``[x, y, z, w]``."""

    position: np.ndarray  # (3,)
    orientation: np.ndarray  # (4,) xyzw
    frame_id: str
    stamp: float

    def to_dict(self) -> dict:
        return {
            "frame_id": self.frame_id,
            "stamp": self.stamp,
            "position": self.position.tolist(),
            "orientation_xyzw": self.orientation.tolist(),
        }


@dataclass
class HandLandmarks:
    points: np.ndarray  # (21, 3)
    side: str
    frame_id: str
    stamp: float

    def to_dict(self) -> dict:
        return {
            "side": self.side,
            "frame_id": self.frame_id,
            "stamp": self.stamp,
            "points": self.points.tolist(),
        }


@dataclass
class BodyJoints:
    names: list
    poses: list  # list[Pose6D], hips-relative when IOBT hips present
    hips_world: Optional[Pose6D]
    stamp: float

    def to_dict(self) -> dict:
        return {
            "stamp": self.stamp,
            "names": self.names,
            "poses": [p.to_dict() for p in self.poses],
            "hips_world": None if self.hips_world is None else self.hips_world.to_dict(),
        }


@dataclass
class MocapConfig:
    max_packet_age_s: float = 0.5
    allow_legacy_packets: bool = False
    protocol: str = "tcp_wired"  # udp | tcp_wired | tcp_wireless
    udp_port: int = 9000
    tcp_port: int = 8000
    arm_side: str = "both"  # left | right | both
    ema_alpha: float = 0.7
    landmark_preprocess: str = "raw"  # raw is the unified default; mano is diagnostics-only
    enable_xhand_pinky_adapt: bool = False
    convert_to_robot: bool = True  # Unity LH → X left, Y back, Z up
    controller_as_wrist: bool = True
    viz: bool = False
    print_every: float = 1.0  # seconds; 0 = print every event
    jsonl_path: Optional[str] = None


# ---------------------------------------------------------------------------
# Core receiver
# ---------------------------------------------------------------------------


class Quest3MocapStandalone:
    """Non-ROS Quest3 HTS receiver with the same processing as the ROS node."""

    def __init__(
        self,
        config: MocapConfig,
        on_head: Optional[Callable[[Pose6D], None]] = None,
        on_wrist: Optional[Callable[[str, Pose6D], None]] = None,
        on_landmarks: Optional[Callable[[HandLandmarks], None]] = None,
        on_controller: Optional[Callable[[str, Pose6D], None]] = None,
        on_body: Optional[Callable[[BodyJoints], None]] = None,
    ):
        protocol = config.protocol.lower()
        if protocol not in ("udp", "tcp_wired", "tcp_wireless"):
            raise ValueError(f"Unknown protocol: {protocol}")
        preprocess = config.landmark_preprocess.strip().lower()
        if preprocess not in ("mano", "raw"):
            raise ValueError("landmark_preprocess must be 'mano' or 'raw'")
        arm = config.arm_side.lower()
        if arm not in ("left", "right", "both"):
            raise ValueError("arm_side must be left|right|both")

        self.config = config
        self._packet_guard = PacketGuard(config.max_packet_age_s, config.allow_legacy_packets)
        self.protocol = protocol
        self.arm_side = arm
        self.both = arm == "both"
        self.ema_alpha = float(config.ema_alpha)
        self.landmark_preprocess = preprocess
        self.enable_xhand_pinky_adapt = bool(config.enable_xhand_pinky_adapt)
        self.convert_to_robot = bool(config.convert_to_robot)
        self.controller_as_wrist = bool(config.controller_as_wrist)

        self.on_head = on_head
        self.on_wrist = on_wrist
        self.on_landmarks = on_landmarks
        self.on_controller = on_controller
        self.on_body = on_body

        self.landmark_cache_right = np.zeros((21, 3), dtype=float)
        self.landmark_cache_left = np.zeros((21, 3), dtype=float)

        self.world_frame_id = (
            "quest3_tracking_world_canonical" if self.convert_to_robot
            else "quest3_tracking_world_unity"
        )
        self.body_frame_id = (
            "quest3_operator_body_canonical" if self.convert_to_robot
            else "quest3_operator_body_unity"
        )
        self.head_frame_id = self.world_frame_id
        self.wrist_frame_id = self.world_frame_id
        self._last_iobt_time = 0.0
        # Sticky body-frame latch: once a body packet has been seen, head/wrist/
        # controller stay in the hips (torso) frame for the rest of the run.
        # This avoids frame_id flipping back to world on transient UDP packet
        # loss (Quest keeps IOBT at High fidelity, but UDP broadcast drops body
        # packets, which previously made the wrist jump between frames).
        self._body_ever_seen = False
        self._wrist_src = {"left": "none", "right": "none"}

        self._latest: Dict[str, object] = {
            "head": None,
            "left_wrist": None,
            "right_wrist": None,
            "left_controller": None,
            "right_controller": None,
            "left_landmarks": None,
            "right_landmarks": None,
            "body": None,
            "hips": None,
            "input_mix": "left=none right=none",
        }
        self._latest_lock = threading.Lock()

        self._fps = FPSCounter()
        self._jsonl_fp = None
        if config.jsonl_path:
            path = Path(config.jsonl_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._jsonl_fp = path.open("a", encoding="utf-8")

        self.is_running = False
        self._socket: Optional[socket.socket] = None
        self._tcp_server: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None

        self._viz_state = {
            "left": None,
            "right": None,
            "head": None,
            "left_wrist": None,
            "right_wrist": None,
            "left_controller": None,
            "right_controller": None,
            "body": None,
        }
        self._viz_lock = threading.Lock()

    # ---- public API -------------------------------------------------------

    def start(self) -> None:
        if self.is_running:
            return
        self.is_running = True
        if self.protocol == "udp":
            self._start_udp()
        else:
            self._start_tcp()
        LOG.info(
            "Quest3 mocap started: protocol=%s side=%s preprocess=%s pinky=%s convert_to_robot=%s",
            self.protocol,
            self.arm_side,
            self.landmark_preprocess,
            self.enable_xhand_pinky_adapt,
            self.convert_to_robot,
        )

    def stop(self) -> None:
        self.is_running = False
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        if self._tcp_server is not None:
            try:
                self._tcp_server.close()
            except OSError:
                pass
            self._tcp_server = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._jsonl_fp is not None:
            self._jsonl_fp.close()
            self._jsonl_fp = None
        LOG.info("Quest3 mocap stopped")

    def get_latest(self) -> dict:
        with self._latest_lock:
            return dict(self._latest)

    # ---- network ----------------------------------------------------------

    def _start_udp(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.config.udp_port))
        sock.settimeout(0.5)
        self._socket = sock
        LOG.info("Listening for UDP on 0.0.0.0:%d", self.config.udp_port)
        self._thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()

    def _start_tcp(self) -> None:
        host = "localhost" if self.protocol == "tcp_wired" else "0.0.0.0"
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, self.config.tcp_port))
        srv.listen(_TCP_LISTEN_BACKLOG)
        srv.settimeout(1.0)
        self._tcp_server = srv
        mode = "wired (adb reverse)" if self.protocol == "tcp_wired" else "wireless"
        LOG.info("TCP server on %s:%d (%s)", host, self.config.tcp_port, mode)
        if self.protocol == "tcp_wired":
            LOG.info("  adb reverse tcp:%d tcp:%d", self.config.tcp_port, self.config.tcp_port)
        self._thread = threading.Thread(target=self._tcp_accept_loop, daemon=True)
        self._thread.start()

    def _udp_loop(self) -> None:
        assert self._socket is not None
        while self.is_running:
            try:
                data, remote = self._socket.recvfrom(65536)
                arrival = time.time()
                arrival_mono = time.monotonic()
                peer_key = f"udp:{remote[0]}:{remote[1]}"
                for line in data.decode("utf-8").splitlines():
                    if line:
                        self._handle_peer_line(
                            line, arrival, arrival_mono, peer_key,
                            lambda payload: self._socket.sendto(payload, remote),
                        )
            except socket.timeout:
                continue
            except OSError:
                if self.is_running:
                    LOG.exception("UDP error")
                break

    def _tcp_accept_loop(self) -> None:
        assert self._tcp_server is not None
        while self.is_running:
            try:
                conn, addr = self._tcp_server.accept()
                LOG.info("TCP connection from %s", addr)
                threading.Thread(
                    target=self._tcp_conn_loop,
                    args=(conn, addr),
                    daemon=True,
                ).start()
            except socket.timeout:
                continue
            except OSError:
                if self.is_running:
                    LOG.exception("TCP accept error")
                break

    def _tcp_conn_loop(self, conn: socket.socket, addr) -> None:
        buf = ""
        peer_key = f"tcp:{addr[0]}:{addr[1]}"
        with conn:
            try:
                challenge = self._packet_guard.make_sync_challenge(peer_key, force=True)
                conn.sendall(challenge.encode("ascii"))
            except OSError:
                return
            while self.is_running:
                try:
                    data = conn.recv(65536)
                    if not data:
                        LOG.info("TCP %s disconnected", addr)
                        break
                    arrival = time.time()
                    arrival_mono = time.monotonic()
                    buf += data.decode("utf-8")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        if line.strip():
                            self._handle_peer_line(
                                line.strip(), arrival, arrival_mono, peer_key,
                                conn.sendall,
                            )
                except OSError as exc:
                    LOG.error("TCP read error from %s: %s", addr, exc)
                    break

    def _handle_peer_line(self, line, arrival, arrival_mono, peer_key, send_bytes) -> None:
        """Mirror the ROS receiver's per-peer sync and telemetry handling."""
        if line.startswith("@q3sync_ack "):
            if self._packet_guard.handle_sync_ack(
                line, peer_key, received_ns=int(arrival_mono * 1e9)
            ):
                LOG.info("Quest monotonic clock synchronized for %s", peer_key)
            return
        if line.startswith("@q3m "):
            challenge = self._packet_guard.make_sync_challenge(
                peer_key, now_ns=int(arrival_mono * 1e9)
            )
            if challenge:
                send_bytes(challenge.encode("ascii"))
        accepted = self._packet_guard.accept(
            line, received=arrival, received_monotonic=arrival_mono
        )
        if accepted:
            self.process_line(*accepted)

    # ---- processing (same as ROS node) ------------------------------------

    def _iobt_active(self) -> bool:
        # Sticky: once IOBT body data has been seen, treat IOBT as active for the
        # rest of the run so transient UDP packet loss does not flip head/wrist/
        # controller back to the world frame mid-stream.
        return self._body_ever_seen or (time.time() - self._last_iobt_time) < _IOBT_TIMEOUT_S

    def _pose_frame_id(self) -> str:
        return self.body_frame_id if self._iobt_active() else self.world_frame_id

    def _unity_to_out(
        self, pos_u: np.ndarray, quat_u: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        if self.convert_to_robot:
            return unity_pose_to_robot(pos_u, quat_u)
        return pos_u, quat_u

    def _note_mix(self, side: str, src: str) -> None:
        tag = "ctrl" if src == "controller" else src
        if self._wrist_src.get(side) == tag:
            return
        self._wrist_src[side] = tag
        mix = f"left={self._wrist_src['left']} right={self._wrist_src['right']}"
        with self._latest_lock:
            self._latest["input_mix"] = mix
        LOG.info("[HTS] mix %s", mix)

    def _process_body_line(self, line: str, arrival_time: float) -> None:
        self._last_iobt_time = time.time()
        self._body_ever_seen = True
        _, _, rest = line.partition(":")
        names = []
        unity_poses = []
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

        hips_world = None
        if hips_u is not None:
            hp, hq = self._unity_to_out(*hips_u)
            hips_world = Pose6D(
                position=hp,
                orientation=hq,
                frame_id=self.world_frame_id,
                stamp=arrival_time,
            )

        out_poses = []
        for name, pos_u, quat_u in unity_poses:
            if name == "hips":
                # Hips is streamed in world; in the body frame it is the root.
                pos_u = np.zeros(3, dtype=float)
                quat_u = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
            # Non-hips joints are already hips-relative on the wire.
            pos, quat = self._unity_to_out(pos_u, quat_u)
            names.append(name)
            out_poses.append(
                Pose6D(
                    position=pos,
                    orientation=quat,
                    frame_id=self.body_frame_id if hips_u is not None else self.world_frame_id,
                    stamp=arrival_time,
                )
            )
        body = BodyJoints(
            names=names,
            poses=out_poses,
            hips_world=hips_world,
            stamp=arrival_time,
        )
        self._emit_body(body)

    def process_landmarks(self, landmarks: np.ndarray, hand_label: str) -> np.ndarray:
        keypoint_3d_array = landmarks.copy()
        if self.landmark_preprocess == "raw":
            return keypoint_3d_array - keypoint_3d_array[0:1, :]

        operator2mano = (
            OPERATOR2MANO_RIGHT if hand_label == "right" else OPERATOR2MANO_LEFT
        )
        keypoint_3d_array *= 1.05
        keypoint_3d_array = keypoint_3d_array - keypoint_3d_array[0:1, :]
        wrist_rot = estimate_frame_from_hand_points(keypoint_3d_array)
        joint_pos = keypoint_3d_array @ wrist_rot @ operator2mano
        if self.enable_xhand_pinky_adapt:
            joint_pos = adaptive_retargeting_xhand(joint_pos)
        return joint_pos

    def process_line(self, line: str, arrival_time: float = 0.0) -> None:
        fps = self._fps.tick()
        if self._fps.should_print():
            LOG.info("[VR Data Rate] %.0f Hz", fps)

        line_lower = line.lower()

        if line_lower.startswith("body"):
            self._process_body_line(line, arrival_time)
            return

        if "head" in line_lower or "hmd" in line_lower:
            parsed = _parse_pose7(line)
            if parsed is not None:
                try:
                    pos, quat = self._unity_to_out(*parsed)
                    pose = Pose6D(
                        position=pos,
                        orientation=quat,
                        frame_id=self._pose_frame_id(),
                        stamp=arrival_time,
                    )
                    self._emit_head(pose)
                except Exception:
                    pass
            return

        side = "right" if "right" in line_lower else "left"
        if not self.both and side != self.arm_side:
            return

        if "wrist" in line_lower or "controller" in line_lower:
            parsed = _parse_pose7(line)
            if parsed is not None:
                try:
                    pos, quat = self._unity_to_out(*parsed)
                    pose = Pose6D(
                        position=pos,
                        orientation=quat,
                        frame_id=self._pose_frame_id(),
                        stamp=arrival_time,
                    )
                    src = "controller" if "controller" in line_lower else "hand"
                    self._emit_side_pose(side, pose, src)
                except Exception as exc:
                    LOG.error("Wrist/controller parse failed: %s", exc)

        if "landmarks" in line.lower():
            parts = line.split(":")
            if len(parts) < 2:
                return
            values = [float(v.strip()) for v in parts[1].split(",") if v.strip()]
            if len(values) != 63:
                return
            landmarks = np.array(values, dtype=float).reshape(21, 3)
            if self.convert_to_robot:
                mapped = unity_landmarks_to_robot(landmarks)
            else:
                mapped = np.zeros_like(landmarks)
                mapped[:, 0] = -landmarks[:, 0]
                mapped[:, 1] = landmarks[:, 1]
                mapped[:, 2] = landmarks[:, 2]
            processed = self.process_landmarks(mapped, side)
            smoothed = self._apply_ema(processed, side)
            msg = HandLandmarks(
                points=smoothed,
                side=side,
                frame_id=f"hand_{side}",
                stamp=arrival_time,
            )
            self._emit_landmarks(msg)

    def _apply_ema(self, landmarks: np.ndarray, side: str) -> np.ndarray:
        if side == "right":
            self.landmark_cache_right = (
                self.ema_alpha * landmarks
                + (1.0 - self.ema_alpha) * self.landmark_cache_right
            )
            return self.landmark_cache_right.copy()
        self.landmark_cache_left = (
            self.ema_alpha * landmarks
            + (1.0 - self.ema_alpha) * self.landmark_cache_left
        )
        return self.landmark_cache_left.copy()

    # ---- emit -------------------------------------------------------------

    def _write_jsonl(self, kind: str, payload: dict) -> None:
        if self._jsonl_fp is None:
            return
        rec = {"type": kind, **payload}
        self._jsonl_fp.write(json.dumps(rec) + "\n")
        self._jsonl_fp.flush()

    def _emit_head(self, pose: Pose6D) -> None:
        with self._latest_lock:
            self._latest["head"] = pose
        with self._viz_lock:
            self._viz_state["head"] = pose
        self._write_jsonl("head", pose.to_dict())
        if self.on_head:
            self.on_head(pose)

    def _emit_side_pose(self, side: str, pose: Pose6D, src: str) -> None:
        self._note_mix(side, src)
        if src == "controller":
            key_c = f"{side}_controller"
            with self._latest_lock:
                self._latest[key_c] = pose
            with self._viz_lock:
                self._viz_state[key_c] = pose
            self._write_jsonl("controller", {"side": side, **pose.to_dict()})
            if self.on_controller:
                self.on_controller(side, pose)
            if self.controller_as_wrist:
                self._emit_wrist(side, pose)
            return
        self._emit_wrist(side, pose)

    def _emit_wrist(self, side: str, pose: Pose6D) -> None:
        key = f"{side}_wrist"
        with self._latest_lock:
            self._latest[key] = pose
        with self._viz_lock:
            self._viz_state[key] = pose
        self._write_jsonl("wrist", {"side": side, **pose.to_dict()})
        if self.on_wrist:
            self.on_wrist(side, pose)

    def _emit_landmarks(self, msg: HandLandmarks) -> None:
        key = f"{msg.side}_landmarks"
        with self._latest_lock:
            self._latest[key] = msg
        with self._viz_lock:
            self._viz_state[msg.side] = msg.points
        self._write_jsonl("landmarks", msg.to_dict())
        if self.on_landmarks:
            self.on_landmarks(msg)

    def _emit_body(self, body: BodyJoints) -> None:
        with self._latest_lock:
            self._latest["body"] = body
            self._latest["hips"] = body.hips_world
        with self._viz_lock:
            self._viz_state["body"] = body
        self._write_jsonl("body", body.to_dict())
        if self.on_body:
            self.on_body(body)


# ---------------------------------------------------------------------------
# Optional matplotlib viz (replaces RViz markers)
# ---------------------------------------------------------------------------


_AXIS_COLORS = ("#e74c3c", "#2ecc71", "#3498db")  # X red, Y green, Z blue
_AXIS_NAMES = ("X", "Y", "Z")


def _draw_pose_axes(
    ax,
    position: np.ndarray,
    quat_xyzw: np.ndarray,
    scale: float = 0.12,
    label: Optional[str] = None,
    point_color: str = "k",
    point_size: float = 36,
) -> None:
    """Draw origin point + RGB triad for a pose (updates every frame with data)."""
    p = np.asarray(position, dtype=float).reshape(3)
    r = _quat_to_matrix(quat_xyzw)
    ax.scatter([p[0]], [p[1]], [p[2]], c=point_color, s=point_size, label=label)
    for i, (color, name) in enumerate(zip(_AXIS_COLORS, _AXIS_NAMES)):
        d = r[:, i] * scale
        ax.plot(
            [p[0], p[0] + d[0]],
            [p[1], p[1] + d[1]],
            [p[2], p[2] + d[2]],
            color=color,
            linewidth=2.0,
        )
        ax.text(
            p[0] + d[0],
            p[1] + d[1],
            p[2] + d[2],
            f"{label + '.' if label else ''}{name}",
            color=color,
            fontsize=7,
        )


def _set_axes_equal_lim(ax, points: list[np.ndarray], pad: float = 0.15) -> None:
    """Set cubic limits covering all points (plus pad)."""
    if not points:
        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(-0.3, 0.3)
        return
    pts = np.vstack(points)
    c = pts.mean(axis=0)
    span = float(np.max(np.abs(pts - c))) + pad
    span = max(span, 0.2)
    ax.set_xlim(c[0] - span, c[0] + span)
    ax.set_ylim(c[1] - span, c[1] + span)
    ax.set_zlim(c[2] - span, c[2] + span)


def run_matplotlib_viz(receiver: Quest3MocapStandalone) -> None:
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required for --viz: pip install matplotlib"
        ) from exc

    plt.ion()
    fig = plt.figure(figsize=(12, 5))
    ax_hand = fig.add_subplot(121, projection="3d")
    ax_pose = fig.add_subplot(122, projection="3d")
    axis_note = (
        "robot: X=left Y=back Z=up"
        if receiver.convert_to_robot
        else "Unity: X=right Y=up Z=forward"
    )
    fig.suptitle(f"Quest3 mocap (standalone)  |  {axis_note}")

    while receiver.is_running and plt.fignum_exists(fig.number):
        with receiver._viz_lock:
            left = None if receiver._viz_state["left"] is None else receiver._viz_state["left"].copy()
            right = (
                None
                if receiver._viz_state["right"] is None
                else receiver._viz_state["right"].copy()
            )
            head = receiver._viz_state["head"]
            lw = receiver._viz_state["left_wrist"]
            rw = receiver._viz_state["right_wrist"]
            lc = receiver._viz_state["left_controller"]
            rc = receiver._viz_state["right_controller"]
            body = receiver._viz_state["body"]

        ax_hand.cla()
        ax_hand.set_title("landmarks (wrist-local)")
        ax_hand.set_xlabel("X")
        ax_hand.set_ylabel("Y")
        ax_hand.set_zlabel("Z")
        hand_pts: list[np.ndarray] = []
        for pts, color, label in (
            (left, "#3399e6", "left"),
            (right, "#e63333", "right"),
        ):
            if pts is None:
                continue
            hand_pts.append(pts)
            ax_hand.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=color, s=12, label=label)
            for a, b in HAND_BONES:
                ax_hand.plot(
                    [pts[a, 0], pts[b, 0]],
                    [pts[a, 1], pts[b, 1]],
                    [pts[a, 2], pts[b, 2]],
                    color=color,
                    linewidth=1.0,
                )
        if hand_pts:
            ax_hand.legend(loc="upper right")
        _set_axes_equal_lim(ax_hand, hand_pts, pad=0.05)
        try:
            ax_hand.set_box_aspect((1, 1, 1))
        except Exception:
            pass

        # World or body frame: head + wrists + controllers + IOBT skeleton.
        ax_pose.cla()
        pose_frame = receiver.body_frame_id if body is not None else receiver.world_frame_id
        ax_pose.set_title(f"head / wrists / body @{pose_frame} (RGB=XYZ)")
        if receiver.convert_to_robot:
            ax_pose.set_xlabel("X (left)")
            ax_pose.set_ylabel("Y (back)")
            ax_pose.set_zlabel("Z (up)")
        else:
            ax_pose.set_xlabel("X")
            ax_pose.set_ylabel("Y")
            ax_pose.set_zlabel("Z")

        pose_pts: list[np.ndarray] = []
        if body is not None and body.poses:
            named = {n: p for n, p in zip(body.names, body.poses)}
            xs, ys, zs = [], [], []
            for p in body.poses:
                pose_pts.append(p.position.copy())
                xs.append(p.position[0])
                ys.append(p.position[1])
                zs.append(p.position[2])
            ax_pose.scatter(xs, ys, zs, c="#ff8c00", s=18, label="body")
            for a, b in BODY_BONES:
                if a not in named or b not in named:
                    continue
                pa, pb = named[a].position, named[b].position
                ax_pose.plot(
                    [pa[0], pb[0]],
                    [pa[1], pb[1]],
                    [pa[2], pb[2]],
                    color="#ffb347",
                    linewidth=1.5,
                )
        for pose, color, label, scale in (
            (head, "k", "head", 0.15),
            (lw, "#3399e6", "L_wrist", 0.10),
            (rw, "#e63333", "R_wrist", 0.10),
            (lc, "#ff1744", "L_ctrl", 0.08),
            (rc, "#ff1744", "R_ctrl", 0.08),
        ):
            if pose is None:
                continue
            pose_pts.append(pose.position.copy())
            r = _quat_to_matrix(pose.orientation)
            for i in range(3):
                pose_pts.append(pose.position + r[:, i] * scale)
            _draw_pose_axes(
                ax_pose,
                pose.position,
                pose.orientation,
                scale=scale,
                label=label,
                point_color=color,
                point_size=40 if label != "head" else 50,
            )

        if pose_pts:
            ax_pose.legend(loc="upper right", fontsize=8)
        _set_axes_equal_lim(ax_pose, pose_pts, pad=0.15)
        try:
            ax_pose.set_box_aspect((1, 1, 1))
        except Exception:
            pass

        plt.pause(0.03)

    plt.ioff()
    try:
        plt.close(fig)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Standalone Quest3 HTS mocap (same pipeline as morla_input, no ROS)."
    )
    p.add_argument(
        "--protocol",
        choices=("udp", "tcp_wired", "tcp_wireless"),
        default="tcp_wired",
    )
    p.add_argument("--udp-port", type=int, default=9000)
    p.add_argument("--tcp-port", type=int, default=8000)
    p.add_argument("--port", type=int, default=None, help="Override udp-port or tcp-port by protocol.")
    p.add_argument("--arm-side", choices=("left", "right", "both"), default="both")
    p.add_argument("--ema-alpha", type=float, default=0.7)
    p.add_argument(
        "--landmark-preprocess",
        choices=("mano", "raw"),
        default="raw",
    )
    p.add_argument(
        "--enable-xhand-pinky-adapt",
        action="store_true",
        help="XHand pinky stretch (off by default; do not use for Wuji).",
    )
    p.add_argument(
        "--convert-to-robot",
        dest="convert_to_robot",
        action="store_true",
        default=True,
        help="Unity LH → robot axes (X left, Y back, Z up). Default: on.",
    )
    p.add_argument(
        "--no-convert-to-robot",
        dest="convert_to_robot",
        action="store_false",
        help="Keep Unity axes (landmarks only legacy X flip).",
    )
    p.add_argument("--allow-legacy-packets", action="store_true", help="Compatibility only: no source-age protection")
    p.add_argument("--max-packet-age-s", type=float, default=0.5)
    p.add_argument("--viz", action="store_true", help="Matplotlib 3D visualization.")
    p.add_argument(
        "--print-every",
        type=float,
        default=1.0,
        help="Seconds between console summaries (0 = every event).",
    )
    p.add_argument("--jsonl", default=None, help="Append processed events as JSONL.")
    p.add_argument("-v", "--verbose", action="store_true")
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    udp_port = args.udp_port
    tcp_port = args.tcp_port
    if args.port is not None:
        if args.protocol == "udp":
            udp_port = args.port
        else:
            tcp_port = args.port

    config = MocapConfig(
        protocol=args.protocol,
        max_packet_age_s=args.max_packet_age_s,
        allow_legacy_packets=args.allow_legacy_packets,
        udp_port=udp_port,
        tcp_port=tcp_port,
        arm_side=args.arm_side,
        ema_alpha=args.ema_alpha,
        landmark_preprocess=args.landmark_preprocess,
        enable_xhand_pinky_adapt=args.enable_xhand_pinky_adapt,
        convert_to_robot=args.convert_to_robot,
        viz=args.viz,
        print_every=args.print_every,
        jsonl_path=args.jsonl,
    )

    last_print = {"t": 0.0}

    def maybe_print(tag: str, text: str) -> None:
        now = time.time()
        if args.print_every <= 0 or now - last_print["t"] >= args.print_every:
            last_print["t"] = now
            print(f"[{tag}] {text}", flush=True)

    def on_head(pose: Pose6D) -> None:
        p, q = pose.position, pose.orientation
        maybe_print(
            "head",
            f"frame={pose.frame_id} "
            f"pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) "
            f"quat=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})",
        )

    def on_wrist(side: str, pose: Pose6D) -> None:
        p = pose.position
        maybe_print(
            f"wrist/{side}",
            f"frame={pose.frame_id} "
            f"pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})",
        )

    def on_controller(side: str, pose: Pose6D) -> None:
        p = pose.position
        maybe_print(
            f"controller/{side}",
            f"frame={pose.frame_id} "
            f"pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})",
        )

    def on_body(body: BodyJoints) -> None:
        maybe_print("body", f"joints={len(body.names)} frame={body.poses[0].frame_id if body.poses else '-'}")

    def on_landmarks(msg: HandLandmarks) -> None:
        tip = msg.points[8]
        maybe_print(
            f"landmarks/{msg.side}",
            f"frame={msg.frame_id} index_tip=({tip[0]:.3f},{tip[1]:.3f},{tip[2]:.3f})",
        )

    receiver = Quest3MocapStandalone(
        config,
        on_head=on_head,
        on_wrist=on_wrist,
        on_landmarks=on_landmarks,
        on_controller=on_controller,
        on_body=on_body,
    )

    stop_event = threading.Event()

    def _handle_sig(_signum, _frame) -> None:
        stop_event.set()
        receiver.stop()

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    receiver.start()
    print(
        "Running. Mixed: controller+hand; IOBT: body + head/hand in hips frame. "
        f"convert_to_robot={args.convert_to_robot} "
        f"({'quest3_tracking_world_canonical X=left Y=back Z=up' if args.convert_to_robot else 'quest3_tracking_world_unity'}). "
        "Ctrl+C to stop.",
        flush=True,
    )

    try:
        if args.viz:
            run_matplotlib_viz(receiver)
        else:
            while not stop_event.is_set() and receiver.is_running:
                time.sleep(0.2)
    finally:
        receiver.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
