"""Planned J motion uses measured motor enable, independently of JS permission."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from morla_bringup.coordinator_logic import nero_home_blockers


def ready_status():
    return dict(connected=True, feedback_fresh=True, enabled=True,
                motors_enabled=True, motor_status_fresh=True, position_permitted=True,
                controller_status_fresh=True, controller_arm_status=0)


def test_post_reset_brake_state_tells_operator_to_prepare_not_reset_again():
    status = ready_status()
    status.update(enabled=False, motors_enabled=False, controller_arm_status=6, reset_status='cleared')
    reasons = nero_home_blockers(status)
    assert len(reasons) == 1
    assert '抱闸未打开' in reasons[0]
    assert '准备并使能' in reasons[0]
    assert '复位急停' not in reasons[0]


def test_arrived_workpos_can_home_with_streaming_revoked_and_motors_enabled():
    status = ready_status()
    status.update(enabled=False, streaming_enabled=False,
                  position_kind='workpos', position_status='arrived')
    assert not nero_home_blockers(status)


def test_cached_streaming_permission_cannot_authorize_disabled_motors():
    status = ready_status()
    status.update(enabled=True, motors_enabled=False)
    assert '电机未全部使能' in nero_home_blockers(status)[0]


def test_stale_motor_feedback_cannot_authorize_motion_even_with_cached_enable():
    status = ready_status()
    status.update(motors_enabled=True, motor_status_fresh=False)
    assert '使能反馈缺失或过期' in nero_home_blockers(status)[0]


@pytest.mark.parametrize('key, value', [('connected', False), ('feedback_fresh', False),
                                       ('controller_status_fresh', False), ('position_permitted', False),
                                       ('controller_arm_status', 1), ('controller_arm_status', 5)])
def test_each_existing_safety_gate_remains_blocking(key, value):
    status = ready_status()
    status[key] = value
    assert nero_home_blockers(status)


def test_missing_driver_state_and_old_driver_without_motor_feedback_are_not_ready():
    assert '过期' in nero_home_blockers({})[0]
    status = ready_status()
    del status['motor_status_fresh']
    assert nero_home_blockers(status)
    assert not nero_home_blockers(ready_status())
