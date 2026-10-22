"""Run the real IK publisher with in-memory outputs; no SDK or robot driver."""

import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from rclpy.qos import DurabilityPolicy
from morla_core.ownership import PolicyLease
from nero_arm_teleop.teleop_node import NeroArmTeleopNode


@pytest.fixture
def teleop():
    rclpy.init(args=["--ros-args", "-r", "__ns:=/left_arm", "-p",
                    "vr_to_arm_rot_after_unity_to_robot:=[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]"])
    node = NeroArmTeleopNode()
    node.commands = []
    node._cmd_pub = SimpleNamespace(publish=node.commands.append)
    node._target_pub = SimpleNamespace(publish=lambda msg: None)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def pose(node, age=0.0):
    msg = PoseStamped()
    ns = node.get_clock().now().nanoseconds - int(age * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    msg.header.frame_id = "quest3_tracking_world_robot"
    msg.pose.orientation.w = 1.0
    return msg


def seed(node, q=None):
    if q is None:
        q = np.deg2rad([-29, 31, -42, 103, 120, -24, -10])
    state = JointState(name=node.joint_names, position=q.tolist())
    node._on_state(state)
    node._on_wrist(pose(node))
    assert node._capture_anchor(node._wrist, arm=True)
    return q


def test_epoch_subscription_uses_absolute_latched_topic(teleop):
    subs = [s for s in teleop.subscriptions if "input_epoch" in s.topic_name]
    assert len(subs) == 1
    assert subs[0].topic_name == "/quest3/input_epoch"
    assert subs[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_input_timeout_stops_commands_and_requires_explicit_restart(teleop):
    seed(teleop)
    teleop._wrist_received_at -= 0.6
    teleop._control_tick()
    assert not teleop._armed and not teleop.commands
    teleop._on_wrist(pose(teleop))
    teleop._control_tick()
    assert not teleop._armed and not teleop.commands
    teleop._on_start(Bool(data=True))
    teleop._control_tick()
    assert teleop._armed and len(teleop.commands) == 1


def test_late_wrist_cannot_hide_a_gap_before_the_timer_runs(teleop):
    seed(teleop)
    teleop._wrist_received_at -= 0.6
    teleop._on_wrist(pose(teleop))
    teleop._control_tick()
    assert not teleop._armed and not teleop.commands


def test_expired_human_lease_stops_commands_without_another_callback(teleop):
    seed(teleop)
    teleop._policy_lease.update('{"activity":"human"}')
    teleop._policy_lease.received -= 3.0
    teleop._control_tick()
    assert not teleop._armed and not teleop.commands
    assert teleop._prepared_token is None


def test_human_heartbeat_cannot_resume_but_explicit_start_reanchors_fresh_input(teleop):
    seed(teleop)
    teleop._on_disarm(Bool(data=True))
    teleop._on_wrist(pose(teleop))
    teleop._on_policy_state(String(data='{"activity":"human"}'))
    assert not teleop._armed
    teleop._on_start(Bool(data=True))
    assert teleop._armed


def test_explicit_start_cannot_bypass_uncommitted_prepared_human_takeover(teleop):
    seed(teleop)
    teleop._armed = False
    teleop._prepared_token = "prepared"
    teleop._policy_lease.update('{"activity":"human"}')
    teleop._on_start(Bool(data=True))
    assert not teleop._armed


@pytest.mark.parametrize("event", ["epoch", "disarm"])
def test_restart_clears_cache_token_and_ignores_pre_stop_queued_pose(teleop, event):
    seed(teleop)
    old = pose(teleop)
    teleop._prepared_token = "obsolete"
    if event == "epoch":
        teleop._on_epoch(String(data="reconnected"))
    else:
        teleop._on_disarm(Bool(data=True))
    assert teleop._wrist is None and teleop._prepared_token is None
    teleop._on_wrist(old)
    teleop._on_start(Bool(data=True))
    assert not teleop._armed
    teleop._on_wrist(pose(teleop))
    teleop._on_start(Bool(data=True))
    assert teleop._armed


def test_source_age_counts_toward_timeout_and_old_or_future_packets_are_rejected(teleop):
    seed(teleop)
    teleop._on_disarm(Bool(data=True))
    # Use a fresh node boundary to test source-age accounting independently.
    teleop._input_valid_after = 0.0
    teleop._on_wrist(pose(teleop, age=0.4))
    assert 0.39 <= time.monotonic() - teleop._wrist_received_at < 0.5
    stored = teleop._wrist
    teleop._on_wrist(pose(teleop, age=0.6))
    teleop._on_wrist(pose(teleop, age=-1.0))
    assert teleop._wrist is stored


@pytest.mark.parametrize("degrees", [
    [-29, 31, -42, 103, 120, -24, -10], [0, 85, 0, 0, 0, 0, 0],
])
def test_stationary_hand_preserves_all_measured_joints_without_redundant_ik_drift(teleop, degrees):
    q = seed(teleop, np.deg2rad(degrees))
    teleop.ik.solve = Mock(side_effect=AssertionError("zero input must reuse measured seed"))
    for _ in range(20):
        teleop._control_tick()
    assert len(teleop.commands) == 20
    for command in teleop.commands:
        np.testing.assert_allclose(command.position, q, atol=1e-12, rtol=0.0)


def test_new_target_is_solved_once_and_cached_while_joint_filter_catches_up(teleop):
    seed(teleop)
    teleop.pose_processor.pos_smoothing = 0.0
    moved = pose(teleop)
    moved.pose.position.x = 0.005
    teleop._on_wrist(moved)
    teleop.ik.solve = Mock(wraps=teleop.ik.solve)
    for _ in range(10):
        teleop._control_tick()
    assert teleop.ik.solve.call_count == 1
    assert len(teleop.commands) == 10
