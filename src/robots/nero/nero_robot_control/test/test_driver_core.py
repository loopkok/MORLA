import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nero_robot_control.driver_core import NeroDriverCore  # noqa: E402
from nero_robot_control.sdk_adapter import FakeNeroSDK  # noqa: E402


LIMITS = np.array([[-1.0, 1.0]] * 7)
SEED = np.array([0.2, -0.2, 0.1, 0.0, 0.3, -0.1, 0.4])


def make_core(clock=lambda: 1.0):
    return NeroDriverCore(
        "left",
        LIMITS,
        max_velocity_rad_s=2.0,
        command_timeout_s=0.2,
        max_feedback_age_s=0.1,
        command_period_s=0.01,
        clock=clock,
    )


def test_fake_sdk_requires_explicit_seed_and_has_no_hardware_side_effects():
    with pytest.raises(ValueError):
        FakeNeroSDK([])
    fake = FakeNeroSDK(SEED)
    assert fake.connect() is True
    assert fake.enable() is True
    np.testing.assert_array_equal(fake.read_state()[0], SEED)
    fake.move_js(SEED + 0.01)
    np.testing.assert_allclose(fake.read_state()[0], SEED + 0.01)
    fake.estop()
    assert fake.enabled is False


def test_fake_sdk_injects_connection_enable_feedback_and_command_failures():
    failing = FakeNeroSDK(SEED, fail_connect=True)
    with pytest.raises(RuntimeError, match="connect"):
        failing.connect()

    disabled = FakeNeroSDK(SEED, enable_result=False)
    disabled.connect()
    assert disabled.enable() is False

    feedback = FakeNeroSDK(SEED, read_error=True)
    feedback.connect()
    with pytest.raises(RuntimeError, match="feedback"):
        feedback.read_state()

    command = FakeNeroSDK(SEED, command_error=True)
    command.connect()
    command.enable()
    with pytest.raises(RuntimeError, match="command"):
        command.move_js(SEED)


def test_left_and_right_driver_cores_keep_independent_names_and_state():
    left = NeroDriverCore("left", LIMITS, clock=lambda: 5.0)
    right = NeroDriverCore("right", LIMITS, clock=lambda: 5.0)
    left.set_connected(True)
    right.set_connected(True)
    left.update_feedback(SEED, 5.0)
    right.update_feedback(-SEED, 5.0)
    left.set_enabled(True)
    right.set_enabled(True)
    assert left.joint_names[0] == "left_joint1"
    assert right.joint_names[0] == "right_joint1"
    left.accept_command(left.joint_names, SEED, 5.0)
    right.accept_command(right.joint_names, -SEED, 5.0)
    np.testing.assert_array_equal(left.feedback, SEED)
    np.testing.assert_array_equal(right.feedback, -SEED)


def test_enable_and_fresh_measured_feedback_are_required_before_command():
    core = make_core()
    with pytest.raises(RuntimeError, match="connected"):
        core.accept_command(core.joint_names, SEED)
    core.set_connected(True)
    with pytest.raises(RuntimeError, match="explicitly enabled"):
        core.accept_command(core.joint_names, SEED)
    with pytest.raises(RuntimeError, match="measured joint state"):
        core.set_enabled(True)
    core.update_feedback(SEED, received_at=1.0)
    core.set_enabled(True)
    result = core.accept_command(core.joint_names, SEED, received_at=1.0)
    np.testing.assert_array_equal(result.target, SEED)
    assert result.rate_limited is False


def test_names_are_reordered_by_contract_and_never_positionally_guessed():
    core = make_core()
    core.set_connected(True)
    core.update_feedback(SEED, 1.0)
    core.set_enabled(True)
    names = list(reversed(core.joint_names))
    values = list(reversed(SEED))
    ordered = core.reorder_command(names, values)
    np.testing.assert_array_equal(ordered, SEED)
    for invalid_names, invalid_values in (
        (core.joint_names[:6], SEED[:6]),
        ((*core.joint_names[:-1], "joint7"), SEED),
        ((*core.joint_names[:-1], core.joint_names[0]), SEED),
    ):
        with pytest.raises(ValueError):
            core.accept_command(invalid_names, invalid_values, 1.0)


@pytest.mark.parametrize(
    "target",
    [
        np.array([2.0] * 7),
        np.array([np.nan] + [0.0] * 6),
    ],
)
def test_bad_numeric_commands_are_rejected(target):
    core = make_core()
    core.set_connected(True)
    core.update_feedback(SEED, 1.0)
    core.set_enabled(True)
    with pytest.raises(ValueError):
        core.accept_command(core.joint_names, target, 1.0)


def test_rate_limit_uses_rad_per_second_and_measured_state_seed():
    core = make_core()
    core.set_connected(True)
    core.update_feedback(SEED, 1.0)
    core.set_enabled(True)
    target = np.clip(SEED + 0.5, -1.0, 1.0)
    result = core.accept_command(core.joint_names, target, 1.0)
    np.testing.assert_allclose(result.target - SEED, np.full(7, 0.02))
    assert result.rate_limited
    assert np.max(np.abs(result.target - SEED)) <= 2.0 * 0.01 + 1e-12


def test_stale_feedback_and_timeout_hold_revoke_motion_eligibility():
    core = make_core()
    core.set_connected(True)
    core.update_feedback(SEED, 1.0)
    core.set_enabled(True)
    core.accept_command(core.joint_names, SEED + 0.05, received_at=1.0)
    assert core.watchdog_hold(now=1.1) is None
    hold = core.watchdog_hold(now=1.21)
    assert hold is not None
    assert core.watchdog_hold(now=1.22) is None
    core.revoke()
    with pytest.raises(RuntimeError, match="explicitly enabled"):
        core.accept_command(core.joint_names, SEED, received_at=1.22)
    core.set_enabled(True)
    with pytest.raises(RuntimeError, match="stale"):
        core.accept_command(core.joint_names, SEED, received_at=1.3)
