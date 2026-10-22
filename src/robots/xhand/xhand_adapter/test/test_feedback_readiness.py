"""No hardware SDK is involved: exercise the ROS adapter with native messages."""

from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from xhand_control_interfaces.msg import XHandState, XHandStateArray
from xhand_adapter.adapter_node import XHandAdapterNode
from xhand_adapter.core import HAND_JOINT_NAMES


@pytest.fixture
def adapter():
    rclpy.init(args=["--ros-args", "-p", "joint_lower_limits:=" + str([-3.0] * 12),
                    "-p", "joint_upper_limits:=" + str([3.0] * 12)])
    node = XHandAdapterNode()
    node.native = []
    node._native_pub = SimpleNamespace(publish=node.native.append)
    node._standard_state_pub = SimpleNamespace(publish=lambda msg: None)
    node._on_armed(Bool(data=True))
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def feedback(node, age=0):
    msg = XHandStateArray()
    ns = node.get_clock().now().nanoseconds - int(age * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    msg.hand_id = [node.hand_id]
    state = XHandState()
    state.name = list(HAND_JOINT_NAMES)
    state.position = [0.0] * 12
    msg.hand_states = [state]
    return msg


def command(node):
    msg = JointState(name=list(HAND_JOINT_NAMES), position=[0.0] * 12)
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


def test_commands_require_fresh_measured_feedback(adapter):
    adapter._on_standard_command(command(adapter))
    assert not adapter.native
    adapter._on_native_state(feedback(adapter))
    adapter._on_standard_command(command(adapter))
    assert len(adapter.native) == 1
    adapter._last_feedback_at -= 0.6
    adapter._on_standard_command(command(adapter))
    assert len(adapter.native) == 1


def test_stale_or_wrong_hand_feedback_does_not_renew_health(adapter):
    adapter._on_native_state(feedback(adapter, age=0.6))
    wrong = feedback(adapter)
    wrong.hand_id = [99]
    adapter._on_native_state(wrong)
    assert not adapter._feedback_fresh()


def test_source_age_is_preserved_in_feedback_health(adapter):
    adapter._on_native_state(feedback(adapter, age=0.4))
    assert adapter._feedback_fresh()
    adapter._last_feedback_at -= 0.2
    assert not adapter._feedback_fresh()
