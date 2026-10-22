"""Motor readiness requires seven current SDK driver-enable samples."""
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nero_robot_control.sdk_adapter import PyAgxArmAdapter


def adapter_with_samples():
    samples = [SimpleNamespace(timestamp=time.time(), msg=SimpleNamespace(
        foc_status=SimpleNamespace(driver_enable_status=True))) for _ in range(7)]
    adapter = PyAgxArmAdapter.__new__(PyAgxArmAdapter)
    adapter._max_feedback_age_s = .5
    adapter._feedback_not_before = 0.
    adapter._robot = SimpleNamespace(get_driver_states=lambda joint: samples[joint - 1])
    return adapter, samples


def test_all_seven_motor_enable_bits_are_required():
    adapter, samples = adapter_with_samples()
    status = adapter._read_motor_enable_status()
    assert status['motors_enabled'] is True
    assert status['motor_status_fresh'] is True
    assert status['joint_enable_status'] == [True] * 7
    # A normal arm-status code must not hide a single disabled joint motor.
    samples[3].msg.foc_status.driver_enable_status = False
    status = adapter._read_motor_enable_status()
    assert status['motor_status_fresh'] is True
    assert status['motors_enabled'] is False
    assert status['joint_enable_status'][3] is False


@pytest.mark.parametrize('joint', range(7))
def test_one_stale_motor_sample_blocks_readiness(joint):
    adapter, samples = adapter_with_samples()
    samples[joint].timestamp = time.time() - 2.
    status = adapter._read_motor_enable_status()
    # The cached bits may still say enabled, but freshness must block motion.
    assert status['motors_enabled'] is True
    assert status['motor_status_fresh'] is False
    assert status['motor_status_timestamp'] is None
    assert f'J{joint + 1}' in status['motor_status_error']


@pytest.mark.parametrize('stamp', [float('nan'), float('inf'), 0., time.time() + 100.])
def test_invalid_or_future_motor_timestamp_is_not_fresh(stamp):
    adapter, samples = adapter_with_samples()
    samples[0].timestamp = stamp
    assert not adapter._read_motor_enable_status()['motor_status_fresh']


def test_missing_motor_feedback_is_unknown_not_enabled():
    adapter, samples = adapter_with_samples()
    samples[6] = None
    status = adapter._read_motor_enable_status()
    assert status['motors_enabled'] is None
    assert status['joint_enable_status'][6] is None
    assert not status['motor_status_fresh']


def test_can_init_invalidates_old_motor_feedback_even_when_otherwise_recent():
    adapter, samples = adapter_with_samples()
    adapter._feedback_not_before = time.time() + .01
    assert not adapter._read_motor_enable_status()['motor_status_fresh']
