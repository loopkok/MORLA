"""HOME stage barriers, safe mode retention, and failure interruption."""
import time

import numpy as np
import pytest

pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from test_enable_confirmation import driver, request_enable, spin_until  # noqa: F401
from test_position_mode import assert_no_streaming


def start_home(rclpy, node):
    seed = np.array([.25, .5, -.6, 1.1, .8, -.3, .2])
    with node.sdk._lock:
        node.sdk.q = seed.copy()
    spin_until(rclpy, node, lambda: np.allclose(node.core.feedback, seed))
    request_enable(node)
    spin_until(rclpy, node, lambda: node.core.enabled)
    moves, speeds = [], []
    node.sdk.move_j = lambda q: moves.append(q.copy())
    set_mode = node.sdk.set_position_mode

    def position_mode(speed):
        speeds.append(speed)
        set_mode(speed)

    node.sdk.set_position_mode = position_mode
    mark = len(node.sdk.calls)
    assert node._on_home(Trigger.Request(), Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._position_status == 'waiting_arrival')
    return seed, moves, speeds, mark


def arrive(rclpy, node):
    with node.sdk._lock:
        node.sdk.q = node._position_target.copy()
    spin_until(rclpy, node, lambda: node._position_status in {'stage_arrived', 'arrived'})


def test_home_holds_each_stage_until_explicit_coordinator_transition(driver):
    from nero_robot_control.home_trajectory import home_targets
    rclpy,node=driver
    seed,moves,speeds,mark=start_home(rclpy,node)
    expected=home_targets(node.side)
    for stage in range(1,6):
        np.testing.assert_allclose(moves[-1],expected[stage-1])
        assert not node._advance_home(stage+1,Trigger.Response()).success
        arrive(rclpy,node)
        deadline=time.monotonic()+.1
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
        assert len(moves)==stage
        assert not node._on_enable(None,Trigger.Response()).success if stage<5 else True
        if stage<5:
            assert not node._advance_home(stage+2,Trigger.Response()).success
            assert node._advance_home(stage+1,Trigger.Response()).success
            spin_until(rclpy,node,lambda: node._position_status=='waiting_arrival')
    assert node._position_status=='arrived' and not node._position_active
    assert speeds==[5]*5
    assert node._position_seq==5
    assert_no_streaming(node,mark)
    request_enable(node)
    spin_until(rclpy,node,lambda: node.core.enabled)


@pytest.mark.parametrize('stage', [1, 2, 3, 4, 5])
def test_estop_cancels_home_at_any_stage(driver, stage):
    rclpy, node = driver
    _seed, moves, _speeds, mark = start_home(rclpy, node)
    for next_stage in range(2, stage + 1):
        arrive(rclpy, node)
        assert node._advance_home(next_stage, Trigger.Response()).success
        spin_until(rclpy, node, lambda: node._position_status == 'waiting_arrival')
    assert node._on_estop(None, Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._estop_status == 'sent')
    assert node._position_status == 'cancelled'
    assert not node._advance_home(2, Trigger.Response()).success
    assert not node._advance_home(3, Trigger.Response()).success
    assert len(moves) == stage
    assert_no_streaming(node, mark)


@pytest.mark.parametrize('failure', ['timeout', 'feedback', 'controller', 'sdk'])
def test_home_failure_stops_and_never_advances(driver, failure):
    rclpy, node = driver
    if failure == 'sdk':
        # The SDK sends no successful target and never restores JS on error.
        request_enable(node)
        spin_until(rclpy, node, lambda: node.core.enabled)
        node.sdk.move_j = lambda q: (_ for _ in ()).throw(RuntimeError('injected HOME write failure'))
        mark = len(node.sdk.calls)
        assert node._on_home(None, Trigger.Response()).success
    else:
        _seed, _moves, _speeds, mark = start_home(rclpy, node)
        if failure == 'timeout':
            node.set_parameters([Parameter('home_stage_timeout_s', value=.1)])
        elif failure == 'feedback':
            # Feedback loss at a bilateral barrier is also a stop condition.
            arrive(rclpy, node)
            node.sdk.read_error = True
        else:
            node.backend = 'hardware'
            node._controller_status = node.sdk.read_controller_status()
            node._controller_status['arm_status'] = 6
            node._controller_status_received = time.monotonic()
    spin_until(rclpy, node, lambda: node._estop_status == 'sent')
    assert node._position_status in {'timed_out', 'failed'}
    assert not node._position_active
    assert not node._advance_home(2, Trigger.Response()).success
    assert_no_streaming(node, mark)


def test_home_rejects_out_of_limits_target_without_motion(driver):
    rclpy, node = driver
    request_enable(node)
    spin_until(rclpy, node, lambda: node.core.enabled)
    points = list(node.get_parameter('workpos_waypoints_deg').value)
    points[0] = -180.
    node.set_parameters([Parameter('workpos_waypoints_deg', value=points)])
    mark = len(node.sdk.calls)
    response = node._on_home(None, Trigger.Response())
    assert not response.success and 'limits' in response.message
    assert not any(call[0] in {'move_j', 'move_js'} for call in node.sdk.calls[mark:])


def test_home_requires_post_command_feedback_and_stable_arrival(driver):
    rclpy, node = driver
    start_home(rclpy, node)
    # A cached pre-send sample cannot satisfy arrival, even if near target.
    node.core.update_feedback(node._position_target)
    node._position_arrival_since = time.monotonic() - 1.
    node._last_feedback_read_started = node._position_feedback_baseline - .1
    node._check_position_progress()
    assert node._position_status == 'waiting_arrival'
    node._last_feedback_read_started = time.monotonic()
    node.core.update_feedback(node._position_target + .1)
    node._check_position_progress()
    assert node._position_arrival_since is None


def test_home_disturbance_requires_arrival_to_settle_again(driver):
    rclpy, node = driver
    start_home(rclpy, node)
    arrive(rclpy, node)
    node.core.update_feedback(node._position_target + .1)
    node._check_position_progress()
    assert node._position_status == 'waiting_arrival'
    assert node._position_arrival_since is None
    assert not node._advance_home(2, Trigger.Response()).success
    arrive(rclpy, node)
    assert node._advance_home(2, Trigger.Response()).success


def test_home_advance_cannot_bypass_expired_stage_timeout(driver):
    rclpy, node = driver
    start_home(rclpy, node)
    arrive(rclpy, node)
    node._position_started = time.monotonic() - 100.
    assert not node._advance_home(2, Trigger.Response()).success
    spin_until(rclpy, node, lambda: node._estop_status == 'sent')
    assert node._position_status == 'timed_out'


def enable_fake_hardware(rclpy, node):
    # Exercise the production gates; the SDK remains an in-memory fake.
    node.backend = 'hardware'
    spin_until(rclpy, node, node._controller_status_is_fresh)
    request_enable(node)
    spin_until(rclpy, node, lambda: node.core.enabled and node._motor_status_is_fresh()
               and node._motors_enabled() is True)


from test_position_mode import complete_workpos


def arrive_workpos(rclpy, node):
    enable_fake_hardware(rclpy, node)
    node.position_pose = np.array([.2, .4, -.3, .8, .6, -.2, .1])
    assert node._on_position(None, Trigger.Response()).success
    complete_workpos(rclpy, node)
    assert not node.core.enabled
    assert node._position_permitted
    assert node.sdk.enabled


def test_workpos_home_workpos_chain_needs_no_js_restore_or_reenable(driver):
    rclpy, node = driver
    arrive_workpos(rclpy, node)
    mark = len(node.sdk.calls)
    assert node._on_home(None, Trigger.Response()).success
    for stage in range(1,6):
        spin_until(rclpy,node,lambda: node._position_status==('arrived' if stage==5 else 'stage_arrived'))
        if stage<5:assert node._advance_home(stage+1,Trigger.Response()).success
    np.testing.assert_allclose(node.core.feedback,np.deg2rad([0,85,0,0,0,0,0]))
    assert not node.core.enabled
    assert node._position_permitted
    # Returning to workpos from HOME is the same planned-J control session.
    assert node._on_position(None, Trigger.Response()).success
    complete_workpos(rclpy, node)
    assert_no_streaming(node, mark)
    assert not any(call[0] == 'enable' for call in node.sdk.calls[mark:])
    assert node._motor_status_is_fresh() and node._motors_enabled()


@pytest.mark.parametrize('failure', ['disabled', 'stale', 'estop', 'reset', 'permission'])
def test_chained_home_cannot_use_cached_enable_after_revocation(driver, failure):
    rclpy, node = driver
    arrive_workpos(rclpy, node)
    if failure == 'disabled':
        node.sdk.enabled = False
        spin_until(rclpy, node, lambda: node._motors_enabled() is False)
    elif failure == 'stale':
        node._controller_status['motor_status_timestamp'] = time.time() - 2.
    elif failure == 'estop':
        assert node._on_estop(None, Trigger.Response()).success
        spin_until(rclpy, node, lambda: node._estop_status == 'sent')
    elif failure == 'reset':
        node.sdk.arm_status = 1
        node._controller_status = node.sdk.read_controller_status()
        node._controller_status_received = time.monotonic()
        assert node._on_reset(None, Trigger.Response()).success
        spin_until(rclpy, node, lambda: node._reset_status == 'cleared')
    else:
        # Known local stop/revocation cannot be bypassed by older true bits.
        node._position_permitted = False
    mark = len(node.sdk.calls)
    response = node._on_home(None, Trigger.Response())
    assert not response.success
    assert not any(call[0] in {'move_j', 'move_js', 'enable'} for call in node.sdk.calls[mark:])
    if failure in {'estop', 'reset', 'permission'}:
        assert not node._position_permitted


@pytest.mark.parametrize('value',[200.,float('nan')])
def test_home_invalid_path_rejects_before_first_stage(driver,value):
    rclpy,node=driver
    request_enable(node)
    spin_until(rclpy,node,lambda: node.core.enabled)
    points=list(node.get_parameter('workpos_waypoints_deg').value)
    points[7]=value
    node.set_parameters([Parameter('workpos_waypoints_deg',value=points)])
    mark=len(node.sdk.calls)
    assert not node._on_home(None,Trigger.Response()).success
    assert not any(call[0] in {'move_j','move_js'} for call in node.sdk.calls[mark:])
