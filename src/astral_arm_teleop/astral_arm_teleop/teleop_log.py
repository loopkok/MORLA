"""Append-only JSONL diagnostics log for the teleop node + per-side path helper.

Mirrors ``astral_policy_inference``'s ``metrics_log_file`` / ``joint_stream_log_file``:
empty path disables the log with zero overhead, records are one JSON line each,
line-buffered, and a write failure never touches the control flow.

Records written by ``astral_arm_teleop_node`` (discriminated by ``kind``):
  loop    — every armed control tick: raw VR / filtered / commanded TCP pos,
            q / q_state, psi_ref, per-frame timings, IK/workspace flags,
            and tracking-quality fields ``pos_err``/``ori_err`` (command FK vs
            filtered target — geometric≈0, numeric larger on orientation),
            ``psi_err`` (arm-angle follow error, null when no human-elbow
            prior = numeric/DH), ``vr_vel`` (VR target speed mm/s for
            slow/fast tagging)
  wrist   — every Quest wrist pose arrival (upstream staircase / stream health)
  state   — every joint_states arrival (measured exec chain)
  body    — every body_joints arrival (human elbow dir, EMA'd)
  metrics — every ~2s: LatencyMeter window (ms stats + counters) + armed/homing
            + ``track`` summary (slow_frac + pos/ori_err p95 on slow vs fast
            segments — the solver-comparison metrics)
"""

from __future__ import annotations

import json
import os
import time


def run_log_dir(root: str, tag: str = "") -> str:
    """Create a per-run directory ``{root}/{stamp}[_tag]`` and return it.

    ``stamp`` is the local launch time ``YYYYMMDD-HHMMSS``; ``tag`` (event name,
    e.g. the test case) is sanitized (``/`` `` `` → ``_``) and appended when
    non-empty. Empty ``root`` → ``""`` (disabled).

    Used by the launch files so each ``ros2 launch`` scopes its JSONL logs into
    a fresh directory (stamped by time + event) instead of appending to a shared
    file that is lost on reboot (old behaviour: ``/tmp/pi_*.jsonl`` etc.).
    """
    root = (root or "").strip()
    if not root:
        return ""
    tag = (tag or "").strip().replace("/", "_").replace(" ", "_")
    stamp = time.strftime("%Y%m%d-%H%M%S")
    name = f"{stamp}_{tag}" if tag else stamp
    d = os.path.join(root, name)
    os.makedirs(d, exist_ok=True)
    return d


def teleop_event_record(event: str, side: str, **fields) -> dict:
    """Build a ``kind=event`` record for takeover/lifecycle diagnostics.

    One record per lifecycle transition (``armed``/``disarm``/``reanchor``/
    ``start``/``fault``); ``side`` and free-form ``fields`` (e.g.
    ``disarm_reason``, ``ok``) ride along so a single jsonl timeline can
    reconstruct the full HITL takeover sequence without cross-referencing the
    policy-side metrics file. ``t`` is the local wall-clock arrival time.
    """
    rec = {"kind": "event", "t": time.time(), "side": side, "event": event}
    rec.update(fields)
    return rec


def side_log_path(base: str, side: str) -> str:
    """Per-side JSONL path from a shared launch base path.

    ``/tmp/teleop_teleop.jsonl`` + ``left`` → ``/tmp/teleop_teleop_left.jsonl``.
    A base already ending in ``_left.jsonl`` / ``_right.jsonl`` passes through
    unchanged (lets a caller pin an exact per-side path). Empty base → ``""``
    (disabled).
    """
    b = (base or "").strip()
    if not b:
        return ""
    if b.endswith("_left.jsonl") or b.endswith("_right.jsonl"):
        return b
    if b.endswith(".jsonl"):
        return f"{b[:-6]}_{side}.jsonl"
    return f"{b}_{side}.jsonl"


class TeleopJsonlLog:
    """Line-buffered append-only JSONL writer; empty/None path → disabled.

    Not thread-safe by design: the teleop node writes from the single ROS
    executor thread (callbacks + timer), same thread as everything else.
    """

    def __init__(self, path: str = ""):
        self.path = ""
        self._fh = None
        p = (path or "").strip()
        if p:
            try:
                self._fh = open(p, "a", buffering=1)  # line buffered
                self.path = p
            except OSError:
                self._fh = None
                self.path = ""

    @property
    def enabled(self) -> bool:
        return self._fh is not None

    def write(self, rec: dict) -> None:
        if self._fh is None:
            return
        try:
            self._fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception:  # noqa: BLE001  logging must never break control
            pass

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            except Exception:  # noqa: BLE001
                pass
            self._fh = None
