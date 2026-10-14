import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import time
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from morla_data_collect.ownership import PolicyLease
from astral_gripper_teleop.pinch_gripper_node import PinchGripperNode
from morla_bringup.controller_workpos_gate import ControllerWorkposGate
from morla_bringup.head_teleop_node import HeadTeleopNode


def arm():
    morla_root = Path(__file__).resolve().parents[4]
    path = morla_root/'src/robots/astral/astral_arm_teleop/astral_arm_teleop/test_reanchor_teleop.py'
    spec = importlib.util.spec_from_file_location('arm_helpers', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    node = mod._make_node()
    node._policy_lease = PolicyLease()
    node._prepared_token = None
    return node


def test_prepare_never_arms_and_only_matching_commit_grants_control():
    node = arm()
    node._policy_lease.update(json.dumps({'activity': 'policy'}))
    result = node._prepare_reanchor_srv(None, Trigger.Response())
    assert result.success and not node._armed
    token = json.loads(result.message)['token']
    node._on_policy_state(String(data=json.dumps({'activity': 'human', 'reanchor_tokens': ['wrong']})))
    assert not node._armed
    node._on_policy_state(String(data=json.dumps({'activity': 'human', 'reanchor_tokens': [token]})))
    assert node._armed


def test_late_prepare_after_human_commit_is_rejected():
    node = arm()
    node._policy_lease.update(json.dumps({'activity': 'human'}))
    node._armed = True
    result = node._prepare_reanchor_srv(None, Trigger.Response())
    assert not result.success and node._armed


def test_expired_or_invalid_policy_lease_disarms_arm_writer():
    node = arm()
    node._armed = True
    node._tlog_event = Mock()
    stale = json.dumps({'activity': 'human', 'sent_at': time.time() - 10})
    node._on_policy_state(String(data=stale))
    assert not node._armed
    assert node._disarm_reason == 'policy'


def test_head_writer_waits_for_fresh_joy_after_human_commit():
    class FakeClock:
        def now(self):
            value = time.time_ns()
            return SimpleNamespace(
                nanoseconds=value,
                to_msg=lambda: Time(sec=value // 1_000_000_000,
                                    nanosec=value % 1_000_000_000),
            )

    class Publisher:
        def __init__(self):
            self.messages = []
        def publish(self, msg):
            self.messages.append(msg)

    node = HeadTeleopNode.__new__(HeadTeleopNode)
    node.get_clock = lambda: FakeClock()
    node.get_logger = lambda: Mock()
    node._policy_lease = PolicyLease()
    node._armed = False
    node._human_wait_for_joy = False
    node._head_init = [0.0, 0.0]
    node._have_state = False
    node._head_state_t = 0.0
    node._last_joy_t = 0.0
    node._stick_yaw = node._stick_pitch = 0.0
    node._init_fallback = [0.0, 0.0]
    node._require_start = True
    node.yaw_axis, node.pitch_axis = 2, 3
    node.deadzone = 0.05
    node.yaw_scale, node.pitch_scale = 0.8, 0.4
    node.yaw_min, node.yaw_max = -0.9, 0.9
    node.pitch_min, node.pitch_max = -0.5, 0.5
    node.input_timeout_s = 0.4
    node._pub_cmd = Publisher()
    node._on_policy_state(String(data=json.dumps({
        'activity': 'policy', 'sent_at': time.time(), 'epoch': 'x', 'sequence': 1,
    })))
    state = JointState()
    state.header.stamp = node.get_clock().now().to_msg()
    state.name, state.position = ['head_yaw', 'head_pitch'], [0.1, -0.1]
    node._on_head_state(state)
    node._on_policy_state(String(data=json.dumps({
        'activity': 'human', 'sent_at': time.time(), 'epoch': 'x', 'sequence': 2,
    })))
    node._on_timer()
    assert not node._armed and not node._pub_cmd.messages
    joy = Joy()
    joy.header.stamp = node.get_clock().now().to_msg()
    joy.axes = [0.0, 0.0, 0.5, 0.0]
    node._on_joy(joy)
    node._on_timer()
    assert node._armed and node._pub_cmd.messages
    assert abs(node._pub_cmd.messages[-1].position[0] - 0.5) < 1e-6


def test_reanchor_rejects_old_wrist_before_changing_origin():
    node = arm()
    node._last_vr_t -= 60
    assert not node._reanchor_teleop()[0]
    assert node.ik.synced is None


def test_gripper_preserves_grasp_until_trigger_matches():
    node = PinchGripperNode.__new__(PinchGripperNode)
    node.side = "left"
    node._gripper_disarmed = True
    node._external_ratio = 1.0
    node._ratio_cmd = 0.0
    node.get_logger = lambda: Mock()
    node._policy_lease = PolicyLease()
    node._policy_lease.update(json.dumps({'activity': 'human'}))
    node._set_gate(False)
    node._takeover_tolerance = .05
    node._last_lm_t = 0.0
    node.input_timeout_s = .4
    node.on_timeout = 'hold'
    node._ratio = 0.0
    node.controller_joy_topic = 'joy'
    node._last_joy_t = time.monotonic()
    node._trigger_ratio = 0.0
    node.max_ratio_rate = 2.5
    node.log_interval_s = 0.0
    emitted = []
    node._pub_cmd = SimpleNamespace(publish=lambda msg: emitted.append(msg.data))
    node._on_timer()
    assert emitted[-1] == 1.0 and node._soft_takeover
    node._trigger_ratio = 1.0
    node._on_timer()
    assert not node._soft_takeover
    node._trigger_ratio = 0.0
    node._on_timer()
    assert emitted[-1] > .9


def test_delayed_workpos_rechecks_policy_and_recording():
    node = ControllerWorkposGate.__new__(ControllerWorkposGate)
    node._policy_lease = PolicyLease()
    node._policy_lease.update(json.dumps({'activity': 'policy'}))
    node._collect_state = 'IDLE'
    node._init_timer = Mock()
    node._pub_init = Mock()
    node._fire_init()
    node._pub_init.publish.assert_not_called()
    node._policy_lease.update(json.dumps({'activity': None}))
    node._collect_state = 'RECORDING'
    node._fire_init()
    node._pub_init.publish.assert_not_called()
