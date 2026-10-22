"""Nonblocking move_j must never restore JS while a target is in flight."""
import time

import numpy as np
import pytest

pytest.importorskip("rclpy")
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from test_enable_confirmation import driver, request_enable, spin_until  # noqa: F401


def start_delayed_position(rclpy, node):
    request_enable(node)
    spin_until(rclpy, node, lambda: node.core.enabled)
    node.position_pose = np.array([0.3, 0., 0., 0., 0., 0., 0.])
    speeds = []
    set_mode = node.sdk.set_position_mode

    def set_position_mode(speed):
        speeds.append(speed)
        set_mode(speed)

    node.sdk.set_position_mode = set_position_mode
    # Emulate the real SDK: send the target, but feedback has not arrived yet.
    node.sdk.move_j = lambda q: node.sdk.calls.append(("move_j", tuple(q)))
    mark = len(node.sdk.calls)
    response = node._on_position(Trigger.Request(), Trigger.Response())
    assert response.success
    spin_until(rclpy, node, lambda: node._position_status == "waiting_arrival")
    assert speeds == [5]
    return mark


def complete_workpos(rclpy, node):
    for stage in range(1, 5):
        spin_until(rclpy, node, lambda: node._position_stage == stage and node._position_status in {'waiting_arrival', 'stage_arrived', 'arrived'})
        with node.sdk._lock:
            node.sdk.q = node._position_target.copy()
        spin_until(rclpy, node, lambda: node._position_status == ('arrived' if stage == 4 else 'stage_arrived'))
        if stage < 4:
            assert not node._advance_position('workpos', stage + 2, Trigger.Response()).success
            assert node._advance_position('workpos', stage + 1, Trigger.Response()).success


def assert_no_streaming(node, mark):
    assert node.sdk.motion_mode == "j"
    assert ("mode", "js") not in node.sdk.calls[mark:]
    assert not any(call[0] == "move_js" for call in node.sdk.calls[mark:])
    assert not node.core.enabled


def test_position_stays_in_j_until_arrival_and_explicit_enable(driver):
    rclpy, node = driver
    mark = start_delayed_position(rclpy, node)
    assert node._position_active
    assert_no_streaming(node, mark)
    reads = sum(call[0] == "read" for call in node.sdk.calls)
    spin_until(rclpy, node, lambda: sum(call[0] == "read" for call in node.sdk.calls) >= reads + 3)
    assert node._position_active
    assert_no_streaming(node, mark)
    response = node._on_enable(Trigger.Request(), Trigger.Response())
    assert not response.success
    response = node._on_position(Trigger.Request(), Trigger.Response())
    assert not response.success
    complete_workpos(rclpy, node)
    assert not node._position_active
    assert_no_streaming(node, mark)
    request_enable(node)
    spin_until(rclpy, node, lambda: node.core.enabled)
    assert node.sdk.motion_mode == "js"


def test_position_timeout_requests_estop_without_js_restore(driver):
    rclpy, node = driver
    node.set_parameters([Parameter("position_timeout_s", value=0.1)])
    mark = start_delayed_position(rclpy, node)
    spin_until(rclpy, node, lambda: node._estop_status == "sent")
    assert node._position_status == "timed_out"
    assert not node._position_active
    assert ("estop",) in node.sdk.calls[mark:]
    assert "arrival timed out" in node._last_error
    assert_no_streaming(node, mark)


def test_position_lost_feedback_requests_estop_without_js_restore(driver):
    rclpy, node = driver
    mark = start_delayed_position(rclpy, node)
    node.sdk.read_error = True
    spin_until(rclpy, node, lambda: node._estop_status == "sent")
    assert node._position_status == "failed"
    assert not node._position_active
    assert ("estop",) in node.sdk.calls[mark:]
    assert_no_streaming(node, mark)


def test_estop_cancels_position_and_never_restores_js(driver):
    rclpy, node = driver
    mark = start_delayed_position(rclpy, node)
    assert node._on_estop(Trigger.Request(), Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._estop_status == "sent")
    assert node._position_status == "cancelled"
    assert not node._position_active
    with node.sdk._lock:
        node.sdk.q = node.position_pose.copy()
    deadline = time.monotonic() + 0.25
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    assert node._position_status == "cancelled"
    assert_no_streaming(node, mark)


def test_position_send_failure_never_restores_js(driver):
    rclpy, node = driver
    request_enable(node)
    spin_until(rclpy, node, lambda: node.core.enabled)
    node.position_pose = np.array([0.3, 0., 0., 0., 0., 0., 0.])

    def fail(_q):
        raise RuntimeError("injected move_j failure")

    node.sdk.move_j = fail
    mark = len(node.sdk.calls)
    assert node._on_position(Trigger.Request(), Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._estop_status == "sent")
    assert node._position_status == "failed"
    assert "injected move_j failure" in node._last_error
    assert_no_streaming(node, mark)


def test_boundary_measurement_noise_does_not_turn_into_stale_feedback_estop(driver):
    rclpy,node=driver
    start_delayed_position(rclpy,node)
    raw=node._position_target.copy()
    raw[3]+=np.deg2rad(.05)
    assert raw[3]>node.core.limits[3,1]
    with node.sdk._lock:node.sdk.q=raw.copy()
    spin_until(rclpy,node,lambda: node._position_status=='stage_arrived')
    assert node._feedback_read_ok
    assert not node._feedback_read_error
    np.testing.assert_allclose(node.core.feedback,raw)
    assert node._estop_status=='idle'


def test_true_feedback_limit_fault_stops_immediately_with_detailed_cause(driver):
    rclpy,node=driver
    start_delayed_position(rclpy,node)
    raw=node._position_target.copy();raw[3]+=np.deg2rad(.2)
    with node.sdk._lock:node.sdk.q=raw.copy()
    spin_until(rclpy,node,lambda: node._estop_status=='sent')
    assert node._position_status=='failed'
    assert 'position feedback limit violation' in node._last_error
    assert 'J4 measured=' in node._last_error and 'excess=' in node._last_error
    assert node._last_feedback_limit_violation['joints'][0]['joint']==4
    assert node._position_stage==1
    assert not node._feedback_read_ok
    assert not node._advance_position('workpos',2,Trigger.Response()).success
    from concurrent.futures import Future
    failure=node._last_error
    for _ in range(2):
        rejected=Future();rejected.set_result((raw.copy(),np.zeros(6)))
        node._read_future=rejected
        node._poll_operation('read')
        assert node._last_error==failure
    recovered=Future();recovered.set_result((node._position_target.copy(),np.zeros(6)))
    node._read_future=recovered
    node._poll_operation('read')
    assert node._last_error==failure
    assert node._feedback_read_ok



def test_bad_latest_read_cannot_confirm_arrival_using_cached_good_sample(driver):
    from concurrent.futures import Future
    rclpy,node=driver
    start_delayed_position(rclpy,node)
    # The cached sample is at target and would otherwise satisfy the dwell test.
    node.core.update_feedback(node._position_target)
    node._position_arrival_since=time.monotonic()-1.
    node._last_feedback_read_started=time.monotonic()+1.
    failed=Future();failed.set_exception(RuntimeError('injected missing flange sample'))
    node._read_future=failed
    node._poll_operation('read')
    node._check_position_progress()
    assert not node._feedback_read_ok
    assert node._position_status=='waiting_arrival'
    assert node._position_arrival_since is None
    assert node._position_active
    assert not node._advance_position('workpos',2,Trigger.Response()).success
