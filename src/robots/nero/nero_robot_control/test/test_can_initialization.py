"""Cold CAN startup enables motors but never grants software motion permission."""
import copy
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("rclpy")
import rclpy
from std_srvs.srv import Trigger

from test_enable_confirmation import driver, request_enable, spin_until  # noqa: F401
from nero_robot_control.driver_node import NeroRobotDriverNode
from nero_robot_control.sdk_adapter import PyAgxArmAdapter


@pytest.fixture
def cold_driver():
    rclpy.init(args=[
        "--ros-args", "-p", "sdk_backend:=fake",
        "-p", "initial_position:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
        "-p", "position_pose:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
        "-p", "can_feedback_timeout_s:=0.2", "-p", "state_rate_hz:=50.0",
    ])
    node = None
    try:
        node = NeroRobotDriverNode()
        state = {"push": False, "permit_push": True}
        read = node.sdk.read_state

        def read_state():
            if not state["push"]:
                raise RuntimeError("fake CAN push is off")
            return read()

        def initialize_can():
            node.sdk.calls.append(("initialize_can",))
            node.sdk.calls.append(("enable",))
            node.sdk.enabled = True
            state["push"] = state["permit_push"]
            return True

        node.sdk.read_state = read_state
        node.sdk.initialize_can = initialize_can
        spin_until(rclpy, node, lambda: node.core.connected)
        yield node, state
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_cold_can_enables_motors_without_feedback_or_motion_permission(cold_driver):
    node, state = cold_driver
    assert node.core.feedback is None
    assert ("mode", "js") not in node.sdk.calls
    assert not node._on_enable(Trigger.Request(), Trigger.Response()).success
    assert node._on_initialize_can(Trigger.Request(), Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._can_init_status == "ready")
    assert state["push"]
    assert node.core.feedback is not None
    assert not node.core.enabled
    assert node._can_init_seq == 1
    assert node.sdk.enabled and node._can_motor_enable_confirmed
    assert not any(call[0] in {"move_j", "move_js", "mode"} for call in node.sdk.calls)


def test_missing_can_feedback_times_out_and_requests_stop(cold_driver):
    node, state = cold_driver
    state["permit_push"] = False
    assert node._on_initialize_can(Trigger.Request(), Trigger.Response()).success
    assert not node._on_enable(Trigger.Request(), Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._can_init_status == "timed_out")
    assert "no fresh" in node._can_init_error
    assert node.core.feedback is None and not node.core.enabled
    spin_until(rclpy, node, lambda: node._estop_status == "sent")
    assert ("estop",) in node.sdk.calls
    assert not node._can_motor_enable_confirmed
    assert not any(call[0] in {"mode", "move_js", "move_j"} for call in node.sdk.calls)


def test_can_handover_rejected_when_driver_enabled(driver):
    ros, node = driver
    request_enable(node)
    spin_until(ros, node, lambda: node.core.enabled)
    assert not node._on_initialize_can(Trigger.Request(), Trigger.Response()).success
    assert ("initialize_can",) not in node.sdk.calls


def test_preinitialization_read_cannot_confirm_can_ready(driver):
    ros, node = driver
    node._can_init_status = "waiting_feedback"
    node._can_init_started = node._can_init_baseline = time.monotonic()
    node._last_feedback_read_started = node._can_init_baseline - 1.0
    node.core.update_feedback(np.zeros(7))
    node._check_can_feedback()
    assert node._can_init_status == "waiting_feedback"
    spin_until(ros, node, lambda: node._can_init_status == "ready")


def test_estop_cancels_can_initialization_without_late_success(cold_driver):
    node, _state = cold_driver
    initialize = node.sdk.initialize_can

    def delayed():
        time.sleep(0.05)
        return initialize()

    node.sdk.initialize_can = delayed
    assert node._on_initialize_can(Trigger.Request(), Trigger.Response()).success
    assert node._on_estop(Trigger.Request(), Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._estop_status == "sent")
    assert node._can_init_status == "cancelled"
    assert not node.core.enabled
    spin_until(rclpy, node, lambda: node._estop_status == "sent")
    assert ("estop",) in node.sdk.calls
    assert not node._can_motor_enable_confirmed
    assert not any(call[0] in {"mode", "move_js", "move_j"} for call in node.sdk.calls)


def test_sdk_cold_start_sends_motor_enable_and_normal_mode_without_targets():
    from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

    # Construct a protocol object but never connect or open a CAN interface.
    adapter = PyAgxArmAdapter.__new__(PyAgxArmAdapter)
    adapter._robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT,
        channel="test_unused_can", interface="socketcan"))
    messages = []
    adapter._robot._send_msg = lambda message: messages.append(copy.deepcopy(message))
    adapter._robot._msg_mode.move_mode = 0x04
    adapter._robot._msg_mode.mit_mode = 0xAD
    previous = adapter._robot._msg_mode
    adapter.initialize_can()
    assert [type(message).__name__ for message in messages] == [
        "ArmMsgMotorEnableDisableConfig", "ArmMsgLeaderFollowerModeConfig", "ArmMsgModeCtrl",
        "ArmMsgLeaderFollowerModeConfig", "ArmMsgModeCtrl"]
    assert messages[0].enable_flag == 2
    mode = messages[2]
    assert mode.ctrl_mode == 0x01
    assert mode.move_mode == 0xFF
    assert mode.enable_can_push == 0x01
    assert mode.mit_mode == 0x00
    for packet in messages[2::2]:
        raw = adapter._robot._parser.pack(packet)
        assert list(raw.data)[:4] == [1, 255, packet.move_spd_rate_ctrl, 0]
        assert raw.data[6] == 1
    assert adapter._robot._msg_mode is previous
    assert previous.mit_mode == 0xAD


def test_adapter_rejects_cached_samples_before_initialization_or_stale():
    now = time.time()
    joints = SimpleNamespace(msg=[0.] * 7, timestamp=now)
    flange = SimpleNamespace(msg=[0.] * 6, timestamp=now)
    adapter = PyAgxArmAdapter.__new__(PyAgxArmAdapter)
    adapter._robot = SimpleNamespace(get_joint_angles=lambda: joints, get_flange_pose=lambda: flange)
    adapter._max_feedback_age_s = 0.5
    adapter._feedback_not_before = now - 0.01
    adapter.read_state()
    adapter._feedback_not_before = now + 0.01
    with pytest.raises(RuntimeError, match="predates"):
        adapter.read_state()
    adapter._feedback_not_before = 0.
    joints.timestamp = now - 2.
    with pytest.raises(RuntimeError, match="stale"):
        adapter.read_state()


def test_sdk_initialization_restores_cached_configuration_on_error():
    previous = SimpleNamespace(ctrl_mode=1, move_mode=4, mit_mode=0xAD)

    def fail():
        assert adapter._robot._msg_mode.mit_mode == 0x00
        raise RuntimeError("injected CAN configuration failure")

    adapter = PyAgxArmAdapter.__new__(PyAgxArmAdapter)
    adapter._robot = SimpleNamespace(_msg_mode=previous, enable=lambda: False, set_normal_mode=fail)
    with pytest.raises(RuntimeError, match="injected CAN"):
        adapter.initialize_can()
    assert adapter._robot._msg_mode is previous
    assert previous.mit_mode == 0xAD


def test_cold_start_retries_until_motor_enable_acknowledgement(cold_driver):
    node, state = cold_driver
    initialize = node.sdk.initialize_can
    attempts = []

    def delayed_ack():
        attempts.append(True)
        return False if len(attempts) < 3 else initialize()

    node.sdk.initialize_can = delayed_ack
    assert node._on_initialize_can(Trigger.Request(), Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._can_init_status == "ready")
    assert len(attempts) == 3
    assert not node.core.enabled and not node.core.command_active


def test_unacknowledged_motor_enable_times_out_and_stops(cold_driver):
    node, _state = cold_driver
    node._can_init_started = time.monotonic()
    node.sdk.initialize_can = lambda: False
    assert node._on_initialize_can(Trigger.Request(), Trigger.Response()).success
    # Exercise the original SDK operation deadline without slowing the test.
    node._can_init_started -= 3.0
    spin_until(rclpy, node, lambda: node._can_init_status == "timed_out")
    spin_until(rclpy, node, lambda: node._estop_status == "sent")
    assert "motor enable" in node._can_init_error
    assert not node.core.enabled
    assert ("estop",) in node.sdk.calls


def test_reset_restarts_can_feedback_push_without_motor_enable_or_motion():
    from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
    adapter=PyAgxArmAdapter.__new__(PyAgxArmAdapter)
    adapter._robot=AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO,firmeware_version=NeroFW.DEFAULT,channel='test_unused_can',interface='socketcan'))
    messages=[]
    adapter._robot._send_msg=lambda message:messages.append(copy.deepcopy(message))
    previous=adapter._robot._msg_mode
    previous.move_mode=4
    previous.mit_mode=0xAD
    before=time.time()
    adapter.reset()
    assert adapter._feedback_not_before>=before
    assert [type(message).__name__ for message in messages]==[
        'ArmMsgMotionCtrl','ArmMsgLeaderFollowerModeConfig','ArmMsgModeCtrl']
    mode=messages[-1]
    assert mode.ctrl_mode==1 and mode.move_mode==0xFF and mode.mit_mode==0 and mode.enable_can_push==1
    assert adapter._robot._msg_mode is previous and previous.mit_mode==0xAD


def test_reset_restores_cached_mode_if_feedback_request_fails():
    from types import SimpleNamespace
    calls=[]
    previous=SimpleNamespace(ctrl_mode=1,move_mode=4,mit_mode=0xAD)
    adapter=PyAgxArmAdapter.__new__(PyAgxArmAdapter)
    def fail():raise RuntimeError('injected feedback request failure')
    adapter._robot=SimpleNamespace(_msg_mode=previous,reset=lambda:calls.append('reset'),set_normal_mode=fail)
    with pytest.raises(RuntimeError,match='feedback request failure'):adapter.reset()
    assert calls==['reset'] and adapter._robot._msg_mode is previous
