import time

import pytest

from morla_web_manager.piano_maintenance import maintenance_allowed, validate_maintenance, check_estop_before_motion
from morla_web_manager.piano_manager import PianoManager
from morla_web_manager.piano_maintenance import wait_maintenance
from morla_web_manager.piano_maintenance import maintenance_interface_ready
from types import SimpleNamespace
from test_piano_lane import store, manager, open_software, wait_job, use_virtual_clock


@pytest.mark.parametrize("side", ["left", "right"])
def test_known_estop_is_rejected_before_claim_and_does_not_require_a_fault_state(side):
    state = {"state": "CONNECTED", "motion_claimed": False}
    with pytest.raises(ValueError, match="未取得运动控制权"):
        check_estop_before_motion("hardware", {side+"_arm": {"controller_arm_status": 1}})
    assert maintenance_allowed("hardware", state["state"], state["motion_claimed"])


@pytest.mark.parametrize("mode", ["hardware", "software", "ros_fake"])
def test_motion_precheck_preserves_normal_and_simulation_paths(mode):
    check_estop_before_motion(mode, {"left_arm": {"controller_arm_status": 0}})


@pytest.mark.parametrize("mode,state,claimed,allowed", [
    ("hardware", "CONNECTED", False, True),
    ("hardware", "WAITING_FEEDBACK", False, True),
    ("hardware", "FAULT", False, False),
    ("hardware", "CONNECTED", True, False),
    ("hardware", "CONNECTED", None, False),
    ("hardware", "PLAYING", True, False),
    ("hardware", "READY", True, False),
    ("hardware", "STOPPED", True, False),
    ("hardware", "STARTING", False, False),
    ("hardware", "DISCONNECTED", False, False),
    ("software", "CONNECTED", False, False),
    ("ros_fake", "CONNECTED", False, False),
])
def test_reset_gate_never_restores_a_fault_or_motion_session(mode, state, claimed, allowed):
    assert maintenance_allowed(mode, state, claimed) is allowed
    if allowed:
        validate_maintenance(mode, state, claimed, True)
    else:
        with pytest.raises(ValueError):
            validate_maintenance(mode, state, claimed, True)


@pytest.mark.parametrize("confirmed", [False, None, "true", 1])
def test_reset_needs_explicit_boolean_confirmation(confirmed):
    with pytest.raises(ValueError, match="明确"):
        validate_maintenance("hardware", "CONNECTED", False, confirmed)


def test_manager_validates_before_enqueue_and_enqueues_only_reset(store):
    manager = PianoManager(store)
    manager._active = True
    manager._token = "owner"
    manager._lease_seen = time.monotonic()
    manager.mode = "hardware"
    manager._state = {"state": "CONNECTED", "motion_claimed": False}
    sent = []
    manager._write = sent.append
    try:
        for state in ({"state": "FAULT", "motion_claimed": False}, {"state": "CONNECTED"},
                      {"state": "READY", "motion_claimed": True}):
            manager._state = state
            with pytest.raises(ValueError):
                manager.command("owner", "reset", calibration_confirmed=True)
        assert sent == []
        manager._state = {"state": "CONNECTED", "motion_claimed": False}
        manager.command("owner", "reset", calibration_confirmed=True)
        assert len(sent) == 1 and sent[0]["operation"] == "reset"
        assert "target_deg" not in sent[0] and "score_path" not in sent[0]
    finally:
        manager._active = False


def test_live_worker_reports_maintenance_disabled_for_software_and_claimed_session(manager, tmp_path):
    use_virtual_clock(manager, tmp_path)
    token = open_software(manager)
    state = manager.snapshot()
    assert state["motion_claimed"] is False and state["maintenance_allowed"] is False
    assert wait_job(manager, manager.command(token, "prepare"), token)["ok"]
    state = manager.snapshot()
    assert state["motion_claimed"] is True and state["maintenance_allowed"] is False


def maintenance_backend(operation, update):
    prefix = 'reset' if operation == 'reset' else 'can_init'
    states = {side+'_arm': {prefix+'_seq': 4, prefix+'_status': 'cleared' if operation == 'reset' else 'ready',
              'controller_status_fresh': True, 'controller_arm_status': 0, 'feedback_fresh': True,
              'motor_status_fresh': True, 'motors_enabled': True} for side in ('left','right')}
    backend = SimpleNamespace(statuses=states, status_at={k: time.monotonic() for k in states})
    backend.pump = lambda: update(backend)
    return backend


@pytest.mark.parametrize('operation', ['reset', 'initialize_can'])
def test_maintenance_rejects_old_success_and_requires_both_new_sequences(operation):
    calls = []
    def update(b):
        calls.append(1)
        if len(calls) >= 3:
            prefix = 'reset' if operation == 'reset' else 'can_init'
            for state in b.statuses.values(): state[prefix+'_seq'] = 5
    b = maintenance_backend(operation, update)
    wait_maintenance(b, operation, dict(left=4,right=4), lambda: None, timeout=.5)
    assert len(calls) >= 3


def test_maintenance_does_not_report_success_from_rpc_ack_or_one_arm():
    b = maintenance_backend('reset', lambda b: b.statuses['left_arm'].update(reset_seq=5))
    with pytest.raises(RuntimeError, match='双臂实测完成'):
        wait_maintenance(b, 'reset', dict(left=4,right=4), lambda: None, timeout=.04)


def test_maintenance_propagates_controller_failure_and_cancellation():
    b = maintenance_backend('reset', lambda b: b.statuses['left_arm'].update(reset_seq=5,reset_status='timed_out',reset_error='still stopped'))
    with pytest.raises(RuntimeError, match='still stopped'):
        wait_maintenance(b, 'reset', dict(left=4,right=4), lambda: None)
    def cancel(): raise RuntimeError('operator cancelled')
    with pytest.raises(RuntimeError, match='operator cancelled'):
        wait_maintenance(b, 'reset', dict(left=4,right=4), cancel)


@pytest.mark.parametrize('patch', [dict(controller_status_fresh=False),dict(controller_arm_status=1),dict(motor_status_fresh=False),dict(motors_enabled=False)])
def test_can_ready_does_not_override_bad_hardware_status(patch):
    def update(b):
        for state in b.statuses.values(): state.update(can_init_seq=5, **patch)
    b = maintenance_backend('initialize_can', update)
    with pytest.raises(RuntimeError, match='双臂实测完成'):
        wait_maintenance(b, 'initialize_can', dict(left=4,right=4), lambda: None, timeout=.04)


def test_reset_interface_does_not_depend_on_any_joint_or_hand_feedback():
    b=SimpleNamespace(statuses={side+'_arm':dict(connected=True,backend='hardware',reset_seq=0,can_init_seq=0)
                      for side in ('left','right')},status_at={side+'_arm':time.monotonic() for side in ('left','right')})
    assert maintenance_interface_ready(b)
    b.status_at['left_arm']=time.monotonic()-3
    assert not maintenance_interface_ready(b)
