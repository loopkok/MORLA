"""Append-only JSONL diagnostics log for the driver node + spike helper.

Mirrors ``astral_arm_teleop.teleop_log`` (kept local to avoid a cross-package
dependency). Empty path disables with zero overhead; write failures never
touch the control flow.

Records written by ``astral_robot_driver`` (``kind``):
  cmd   — every incoming joint_commands (left/right/full/head/gripper): who
          published what, with arrival time (排"上游谁发坏指令")
  send  — every control tick, what was actually sent to the board + whether
          each side is fresh or a stale cache re-send (排"driver 陈旧重发")
  state — every measured joint_states publish (物理臂实际位置)
  srv   — every driver service call (~/ready|enable|home|estop|damping|position):
          motion-mode switches / cache clears that can re-seed targets and jerk
  spike — one-line event when a commanded OR measured jump exceeds the
          threshold (电机"抽一下"的直接记录；含跳变前后值)
"""

from __future__ import annotations

import json

import numpy as np


class DriverJsonlLog:
    """Line-buffered append-only JSONL writer; empty/None path → disabled."""

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


def spike_mrad(q_new, q_prev, thresh_mrad: float = 30.0) -> float:
    """最大单拍关节跳变 (mrad)；超阈值返回值，否则 0。

    ``q_*`` 为 np.ndarray；任一为 None → 0（无判断）。
    """
    if q_new is None or q_prev is None:
        return 0.0
    dq = np.abs(np.asarray(q_new, dtype=float) - np.asarray(q_prev, dtype=float))
    mx = float(np.max(dq)) if dq.size else 0.0
    return mx * 1e3 if mx * 1e3 > float(thresh_mrad) else 0.0
