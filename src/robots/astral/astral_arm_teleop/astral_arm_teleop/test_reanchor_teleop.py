#!/usr/bin/env python3
"""Regression tests for the HITL ``~/reanchor`` service on AstralTeleopArmNode.

The service must re-derive robot_init_pos/rot from the *current* measured joint
state (fallback: last commanded q) and re-capture vr_init from the current VR
pose, then arm — all without a jump. Tests instantiate the node class via
``__new__`` (no rclpy node spin) and stub ``pose``/``ik``/``safety``/logger.

Runs only where rclpy is importable (ROS-sourced system python); otherwise the
whole suite skips with exit 0. Style matches the other astral_arm_teleop
script-style tests.
"""

import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import rclpy  # noqa: F401
except ImportError:
    print("SKIP: rclpy not importable (source ROS first)")
    sys.exit(0)

from astral_arm_teleop.astral_arm_teleop_node import AstralTeleopArmNode  # noqa: E402


class _FakePose:
    def __init__(self):
        self.vr_current_pos = np.zeros(3)
        self.vr_current_rot = Rotation.identity()
        self.calibrated = 0
        self.R_vr_to_arm = np.eye(3)

    def calibrate_from_current(self):
        self.calibrated += 1
        return self.vr_current_pos is not None and self.vr_current_rot is not None


class _FakeIk:
    def __init__(self, t0):
        self.lower_limits = np.array([-3.14] * 7)
        self.upper_limits = np.array([3.14] * 7)
        self._t0 = t0
        self.synced = None

    def sync_state(self, q):
        self.synced = np.asarray(q)

    def fk(self, q):
        return self._t0

    def elbow_direction(self, q):
        # Upper-arm direction of the current config (arbitrary but deterministic).
        return np.array([1.0, 0.0, 0.0])


class _FakeSafety:
    def __init__(self):
        self.initial = None

    def set_initial_state(self, q, pos):
        self.initial = (np.asarray(q), np.asarray(pos).copy())


class _FakeLog:
    def __init__(self):
        self.messages = []

    def warn(self, msg, *a, **k):
        self.messages.append(msg)

    def info(self, msg, *a, **k):
        self.messages.append(msg)


class _FakeTlog:
    enabled = False

    def write(self, rec):
        pass


def _make_node():
    node = AstralTeleopArmNode.__new__(AstralTeleopArmNode)
    node.side = "left"
    node._homing = False
    node._got_state = True
    node._last_vr_t = time.monotonic()
    node._state_t = time.monotonic()
    node.data_timeout = 1.5
    node.state_q = np.array([0.1, 0.2, 0.3, -0.4, 0.5, 0.0, 0.0])
    node.q_cmd = np.array([-0.1, -0.2, -0.3, -0.9, -0.5, 0.0, 0.0])
    t0 = np.eye(4)
    t0[:3, 3] = [0.30, 0.10, -0.40]
    node.ik = _FakeIk(t0)
    node.pose = _FakePose()
    node.safety = _FakeSafety()
    node._armed = False
    node._disarm_reason = None
    node._flip_needed = False
    node._elbow_dir_vr = np.array([0.0, 1.0, 0.0])  # conflicting human-elbow dir
    node._elbow_dir_t = time.monotonic()
    node._human_elbow_tau = 0.15
    node._reanchor_elbow_hold = True
    node._reanchor_elbow_release_thresh = 0.2
    node._elbow_held = False
    node._elbow_reanchor_dir = None
    node._tlog = _FakeTlog()
    node._log = _FakeLog()
    node.get_logger = lambda: node._log
    return node


def _fk_result():
    t0 = np.eye(4)
    t0[:3, 3] = [0.30, 0.10, -0.40]
    return t0


def test_reanchor_blocks_while_homing():
    node = _make_node()
    node._homing = True
    ok, _ = node._reanchor_teleop()
    assert ok is False
    assert node._armed is False


def test_reanchor_requires_vr():
    node = _make_node()
    node.pose.vr_current_pos = None
    node.pose.vr_current_rot = None
    ok, msg = node._reanchor_teleop()
    assert ok is False
    assert "VR" in msg


def test_reanchor_uses_fresh_measured_state():
    node = _make_node()
    ok, _ = node._reanchor_teleop()
    assert ok is True
    # Anchor came from FK(state_q), not the fallback q_cmd.
    assert np.allclose(node.robot_init_pos, _fk_result()[:3, 3])
    assert np.allclose(node.ik.synced, node.state_q)
    assert node.pose.calibrated == 1
    assert node._armed is True
    assert node._disarm_reason is None
    assert np.allclose(node.q_cmd, node.state_q)
    assert node.safety.initial[0] is not None


def test_reanchor_rejects_missing_state():
    node = _make_node()
    node._got_state = False
    ok, _ = node._reanchor_teleop()
    assert not ok and not node._armed


def test_reanchor_rejects_stale_state():
    node = _make_node()
    node._state_t = time.monotonic() - 60
    ok, _ = node._reanchor_teleop()
    assert not ok and not node._armed


def test_reanchor_reseeds_elbow_to_current_config():
    """HITL re-anchor must ALSO re-anchor the arm-angle reference: the human
    elbow EMA is reset to the arm's current upper-arm direction, so the first
    solve_hard keeps the current config (no elbow snap — regression: reanchor
    后 0.26-0.32 rad 肘部重构)."""
    node = _make_node()
    ok, _ = node._reanchor_teleop()
    assert ok is True
    # _elbow_dir_vr is rotated into the VR frame: R_vr_to_arm.T @ u_se.
    expected = node.pose.R_vr_to_arm.T @ node.ik.elbow_direction(node.state_q)
    assert np.allclose(node._elbow_dir_vr, expected)
    assert node._elbow_dir_t > 0.0
    # A conflicting prior (the operator's arm pointing elsewhere) is gone.
    assert not np.allclose(node._elbow_dir_vr, [0.0, 1.0, 0.0])
    # 方案B：reanchor 后臂角保持开启（直到操作者手臂方向变化超阈值）。
    assert node._elbow_held is True
    assert np.allclose(node._elbow_reanchor_dir, node._elbow_dir_vr)


class _FakePos:
    def __init__(self, x, y, z):
        self.position = type("_P", (), {"x": x, "y": y, "z": z})()


class _FakeBodyMsg:
    """Shoulder at origin, elbow at ``d`` (unit upper-arm direction)."""

    def __init__(self, d):
        self.poses = [_FakePos(0.0, 0.0, 0.0), _FakePos(*d)]


def test_elbow_hold_keeps_config_until_operator_moves():
    """方案B：reanchor 后肘保持接管时刻臂角；操作者手臂方向变化超阈值才
    切回人肘 EMA 跟随（不自行摆动 = 不"卡/怪"）。"""
    node = _make_node()
    node._body_names = ["left-arm-upper", "left-arm-lower"]
    ok, _ = node._reanchor_teleop()
    assert ok is True and node._elbow_held is True
    held = node._elbow_dir_vr.copy()
    # 操作者手臂方向与 reanchor 时刻接近（阈值内）→ 保持，不混合
    near = np.array([1.0, 0.05, 0.0])
    near /= np.linalg.norm(near)
    node._on_body_joints(_FakeBodyMsg(near))
    assert node._elbow_held is True
    assert np.allclose(node._elbow_dir_vr, held)
    # 操作者手臂方向明显变化（超阈值）→ 释放，从保持值平滑 EMA 跟随
    far = np.array([0.7, 0.7, 0.1])
    far /= np.linalg.norm(far)
    node._on_body_joints(_FakeBodyMsg(far))
    assert node._elbow_held is False
    assert not np.allclose(node._elbow_dir_vr, far)  # EMA 已开始混合
    assert np.allclose(node._elbow_dir_vr, held, atol=0.5)  # 但仍接近保持值


def _run_all():
    tests = [
        test_reanchor_blocks_while_homing,
        test_reanchor_requires_vr,
        test_reanchor_uses_fresh_measured_state,
        test_reanchor_rejects_missing_state,
        test_reanchor_rejects_stale_state,
        test_reanchor_reseeds_elbow_to_current_config,
        test_elbow_hold_keeps_config_until_operator_moves,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL {t.__name__}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return failed


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)
