"""Landmarks are monitored while unarmed, but cannot silently resume motion."""

import time
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, String
from morla_core.ownership import PolicyLease
from xhand_retargeting.xhand_dex_retargeting_node import XHandDexRetargetingNode as Retarget


def fake_node():
    node = SimpleNamespace(
        _last_landmarks_at=0.0, _input_valid_after=0.0, input_timeout_s=0.5,
        _armed=False, _last_target=None, _policy_lease=PolicyLease(), require_armed=True,
        side="left", get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        get_logger=lambda: SimpleNamespace(warning=lambda msg: None),
    )
    node._input_fresh = lambda: Retarget._input_fresh(node)
    node.status = []
    node._status_pub = SimpleNamespace(publish=node.status.append)
    return node


def landmarks(stamp=100):
    msg = PoseArray()
    msg.header.stamp.sec = stamp
    msg.poses = [Pose() for _ in range(21)]
    return msg


def test_unarmed_input_is_monitored_and_missing_input_cannot_arm():
    node = fake_node()
    Retarget._on_armed(node, Bool(data=True))
    assert not node._armed
    Retarget._on_landmarks(node, landmarks())
    assert node._input_fresh() and not node._armed
    Retarget._on_armed(node, Bool(data=True))
    assert node._armed


def test_gap_detected_on_receive_does_not_silently_resume():
    node = fake_node()
    node._armed = True
    node._last_landmarks_at = time.monotonic() - 0.6
    Retarget._on_landmarks(node, landmarks())
    assert node._input_fresh() and not node._armed


@pytest.mark.parametrize("expired", ["input", "lease"])
def test_status_timer_disarms_expired_input_or_ownership(expired):
    node = fake_node()
    node._last_landmarks_at = time.monotonic()
    node._armed = True
    if expired == "input":
        node._last_landmarks_at -= 0.6
    else:
        node._policy_lease.update('{"activity":"human"}')
        node._policy_lease.received -= 3.0
    Retarget._publish_runtime_status(node)
    assert not node._armed


def test_epoch_invalidates_landmarks_and_queued_pre_epoch_samples():
    node = fake_node()
    node._last_landmarks_at = time.monotonic()
    node._armed = True
    Retarget._on_epoch(node, String(data="disconnected"))
    Retarget._on_landmarks(node, landmarks())
    assert not node._input_fresh() and not node._armed


def test_old_or_incomplete_landmarks_cannot_renew_readiness():
    node = fake_node()
    Retarget._on_landmarks(node, landmarks(99))
    incomplete = landmarks()
    incomplete.poses.pop()
    Retarget._on_landmarks(node, incomplete)
    assert not node._input_fresh()


def test_human_heartbeat_cannot_rearm_after_input_loss_or_expired_lease():
    node = fake_node()
    node._policy_lease.update('{"activity":"human"}')
    node._last_landmarks_at = time.monotonic()
    Retarget._on_ownership(node, String(data='{"activity":"human"}'))
    assert not node._armed
    node._policy_lease.received -= 3.0
    Retarget._on_ownership(node, String(data='{"activity":"human"}'))
    assert not node._armed


def test_idle_heartbeat_preserves_explicit_manual_start():
    node = fake_node()
    node._last_landmarks_at = time.monotonic()
    Retarget._on_armed(node, Bool(data=True))
    Retarget._on_ownership(node, String(data='{"activity":null}'))
    assert node._armed


def test_explicit_policy_to_human_transition_arms_with_fresh_landmarks():
    node = fake_node()
    node._policy_lease.update('{"activity":"policy"}')
    node._last_landmarks_at = time.monotonic()
    Retarget._on_ownership(node, String(data='{"activity":"human"}'))
    assert node._armed
