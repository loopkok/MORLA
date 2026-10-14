"""Timestamp-based latency and accuracy tracker for teleoperation pipeline.

Pipeline stages (arm):
  UDP arrival (header.stamp)
    → [ROS2 transport] → VR callback
    → [IK solve] → [safety filter]
    → move_js sent  ←── software pipeline ends
    → [CAN + motor execution + encoder readback]
    → feedback received  ←── physical control ends

Pipeline stages (hand):
  UDP arrival (header.stamp)
    → [ROS2 transport] → retargeting callback
    → [retargeting compute] → [post-process]
    → XHandCommand published  ←── software pipeline ends
    → [ROS2 + USB serial + hand execution]
    → XHandState received  ←── physical control ends
"""

import time
from typing import Dict, Optional
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class ArmMetrics:
    """Per-frame arm latency + accuracy metrics."""
    timestamp: float = 0.0
    # Software pipeline (VR arrival → command sent)
    vr_to_cmd_ms: float = 0.0        # UDP arrival → move_js via CAN sent
    ik_ms: float = 0.0               # IK computation time
    safety_ms: float = 0.0            # Safety filter time
    # Physical control (command → execution)
    control_ms: float = 0.0           # move_js sent → encoder feedback received
    # End-to-end
    e2e_ms: float = 0.0              # UDP arrival → encoder feedback
    # Accuracy
    pos_error_mm: float = 0.0         # |target - actual| flange position
    ori_error_deg: float = 0.0        # |target - actual| flange orientation


@dataclass
class HandMetrics:
    """Per-frame hand retargeting latency metrics."""
    timestamp: float = 0.0
    # Software pipeline (landmark arrival → command published)
    landmark_to_cmd_ms: float = 0.0   # UDP arrival → XHandCommand published
    retarget_ms: float = 0.0          # DexPilot retargeting computation
    postprocess_ms: float = 0.0       # Thumb fix + EMA smoothing
    # Physical control (command → execution)
    control_ms: float = 0.0           # XHandCommand sent → XHandState received
    # End-to-end
    e2e_ms: float = 0.0              # UDP arrival → XHandState received


class LatencyTracker:
    """Lightweight timestamp ring for latency computation."""

    def __init__(self, name: str, print_interval: float = 2.0, window_size: int = 100):
        self.name = name
        self._stamps: Dict[str, float] = {}
        self._last_print = 0.0
        self._print_interval = print_interval
        self._frame_count = 0
        self._window_size = window_size
        self._recent: list = []

    def stamp(self, key: str, ts: Optional[float] = None):
        self._stamps[key] = ts if ts is not None else time.time()

    def get(self, key: str) -> Optional[float]:
        return self._stamps.get(key)

    # ==================================================================
    # Arm metrics
    # ==================================================================

    def compute_arm_metrics(
        self, vr_header_stamp: float, target_pos: np.ndarray,
        actual_pos: np.ndarray, target_quat=None, actual_quat=None,
    ) -> ArmMetrics:
        stamps = self._stamps
        m = ArmMetrics(timestamp=time.time())

        # Software pipeline: VR → cmd_sent
        if "cmd_sent" in stamps and vr_header_stamp > 0:
            m.vr_to_cmd_ms = (stamps["cmd_sent"] - vr_header_stamp) * 1000

        # IK time
        if "ik_start" in stamps and "ik_end" in stamps:
            m.ik_ms = (stamps["ik_end"] - stamps["ik_start"]) * 1000

        # Safety filter time
        if "safety_start" in stamps and "safety_end" in stamps:
            m.safety_ms = (stamps["safety_end"] - stamps["safety_start"]) * 1000

        # Physical control: cmd_sent → feedback
        if "cmd_sent" in stamps and "feedback" in stamps:
            m.control_ms = (stamps["feedback"] - stamps["cmd_sent"]) * 1000

        # E2E: VR → feedback
        if "feedback" in stamps and vr_header_stamp > 0:
            m.e2e_ms = (stamps["feedback"] - vr_header_stamp) * 1000

        # Position accuracy
        if target_pos is not None and actual_pos is not None:
            m.pos_error_mm = float(np.linalg.norm(target_pos - actual_pos) * 1000)

        # Orientation accuracy
        if target_quat is not None and actual_quat is not None:
            try:
                R1 = Rotation.from_quat(target_quat).as_matrix()
                R2 = Rotation.from_quat(actual_quat).as_matrix()
                R_diff = R1.T @ R2
                trace = np.clip((np.trace(R_diff) - 1) / 2, -1, 1)
                m.ori_error_deg = float(np.arccos(trace) * 180 / np.pi)
            except Exception:
                pass

        self._recent.append(m)
        if len(self._recent) > self._window_size:
            self._recent.pop(0)
        self._frame_count += 1
        return m

    # ==================================================================
    # Hand metrics
    # ==================================================================

    def compute_hand_metrics(
        self, landmark_header_stamp: float,
        cmd_published_stamp: float, state_received_stamp: float = 0.0,
    ) -> HandMetrics:
        stamps = self._stamps
        m = HandMetrics(timestamp=time.time())

        # Software pipeline: landmark → cmd published
        if cmd_published_stamp > 0 and landmark_header_stamp > 0:
            m.landmark_to_cmd_ms = (cmd_published_stamp - landmark_header_stamp) * 1000

        # Retargeting computation
        if "retarget_start" in stamps and "retarget_done" in stamps:
            m.retarget_ms = (stamps["retarget_done"] - stamps["retarget_start"]) * 1000

        # Post-processing
        if "postprocess_start" in stamps and "postprocess_done" in stamps:
            m.postprocess_ms = (stamps["postprocess_done"] - stamps["postprocess_start"]) * 1000

        # Physical control: cmd published → state received
        if state_received_stamp > 0 and cmd_published_stamp > 0:
            m.control_ms = (state_received_stamp - cmd_published_stamp) * 1000

        # E2E: landmark → state received
        if state_received_stamp > 0 and landmark_header_stamp > 0:
            m.e2e_ms = (state_received_stamp - landmark_header_stamp) * 1000

        return m

    # ==================================================================
    # Stats helpers
    # ==================================================================

    def should_print(self) -> bool:
        now = time.time()
        if now - self._last_print >= self._print_interval:
            self._last_print = now
            return True
        return False

    @property
    def recent_stats(self) -> dict:
        if not self._recent:
            return {}
        first = self._recent[0]
        if isinstance(first, ArmMetrics):
            fields = [
                ("vr_to_cmd_ms", "VR→Cmd"),
                ("ik_ms", "IK"),
                ("safety_ms", "Safety"),
                ("control_ms", "Control"),
                ("e2e_ms", "E2E"),
                ("pos_error_mm", "PosErr"),
                ("ori_error_deg", "OriErr"),
            ]
        else:
            fields = [
                ("landmark_to_cmd_ms", "LM→Cmd"),
                ("retarget_ms", "Retarget"),
                ("postprocess_ms", "PostProc"),
                ("control_ms", "Control"),
                ("e2e_ms", "E2E"),
            ]
        stats = {}
        for attr, _label in fields:
            vals = [getattr(m, attr) for m in self._recent]
            stats[f"{attr}_avg"] = float(np.mean(vals))
            stats[f"{attr}_max"] = float(np.max(vals))
        stats["frames"] = self._frame_count
        stats["window"] = len(self._recent)
        return stats

    @property
    def frame_count(self) -> int:
        return self._frame_count


# ==================================================================
# Serialization helpers
# ==================================================================

ARM_METRICS_FIELDS = [
    "timestamp", "vr_to_cmd_ms", "ik_ms", "safety_ms",
    "control_ms", "e2e_ms", "pos_error_mm", "ori_error_deg",
]

HAND_METRICS_FIELDS = [
    "timestamp", "landmark_to_cmd_ms", "retarget_ms",
    "postprocess_ms", "control_ms", "e2e_ms",
]


def arm_metrics_to_array(m: ArmMetrics) -> list:
    return [getattr(m, f) for f in ARM_METRICS_FIELDS]


def hand_metrics_to_array(m: HandMetrics) -> list:
    return [getattr(m, f) for f in HAND_METRICS_FIELDS]
