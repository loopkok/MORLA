"""SDK enable acknowledgement regression tests using the fake hardware backend."""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def spin_until(rclpy, node, condition, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    assert condition(), "condition did not become true before deadline"


@pytest.fixture
def driver():
    rclpy = pytest.importorskip("rclpy")
    from nero_robot_control.driver_node import NeroRobotDriverNode

    rclpy.init(args=[
        "--ros-args", "-p", "sdk_backend:=fake",
        "-p", "initial_position:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
        "-p", "position_pose:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
        "-p", "sdk_timeout_s:=0.3", "-p", "state_rate_hz:=50.0",
    ])
    node = None
    try:
        node = NeroRobotDriverNode()
        spin_until(rclpy, node, lambda: node.core.connected and node.core.feedback is not None)
        yield rclpy, node
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def request_enable(node):
    from std_srvs.srv import Trigger
    response = node._on_enable(Trigger.Request(), Trigger.Response())
    assert response.success


def test_enable_retries_until_motor_feedback_confirms(driver):
    rclpy, node = driver
    attempts = []

    def enable():
        attempts.append(time.monotonic())
        node.sdk.enabled = len(attempts) >= 3
        return node.sdk.enabled

    node.sdk.enable = enable
    request_enable(node)
    spin_until(rclpy, node, lambda: node._enable_status == "enabled")
    assert len(attempts) == 3
    assert node.core.enabled
    assert node.core.feedback_age() < node.core.max_feedback_age_s
    assert not node.core.command_active
    assert node.sdk.motion_mode == "js"
    assert not any(call[0] == "move_js" for call in node.sdk.calls)


def test_enable_timeout_stops_retries_and_survives_healthy_reads(driver):
    rclpy, node = driver
    attempts = []

    def enable():
        attempts.append(time.monotonic())
        return False

    node.sdk.enable = enable
    request_enable(node)
    spin_until(rclpy, node, lambda: node._enable_status == "timed_out")
    assert 1 < len(attempts) < 40
    assert not node.core.enabled
    error = node._last_error
    assert "not confirmed" in error
    attempt_count = len(attempts)
    reads_before = sum(call[0] == "read" for call in node.sdk.calls)
    spin_until(rclpy, node, lambda: sum(call[0] == "read" for call in node.sdk.calls) >= reads_before + 3)
    assert node._last_error == error
    assert len(attempts) == attempt_count


def test_sdk_exception_fails_without_retry_and_is_not_erased_by_feedback(driver):
    rclpy, node = driver
    attempts = []

    def enable():
        attempts.append(True)
        raise RuntimeError("injected enable failure")

    node.sdk.enable = enable
    request_enable(node)
    spin_until(rclpy, node, lambda: node._enable_status == "failed")
    assert not node.core.enabled
    reads_before = sum(call[0] == "read" for call in node.sdk.calls)
    spin_until(rclpy, node, lambda: sum(call[0] == "read" for call in node.sdk.calls) >= reads_before + 3)
    assert "injected enable failure" in node._last_error
    assert len(attempts) == 1


def test_explicit_controller_reset_confirms_status_without_enabling(driver):
    rclpy, node = driver
    from std_srvs.srv import Trigger

    # Exercise the hardware gate while keeping all SDK traffic on the fake.
    node.backend = "hardware"
    node.sdk.arm_status = 1
    node.core.revoke()
    node._controller_status = node.sdk.read_controller_status()
    node._controller_status_received = time.monotonic()
    node._controller_status_generation = 1
    node._controller_status_last_reset_seq = 0

    enable = node._on_enable(Trigger.Request(), Trigger.Response())
    assert not enable.success
    assert "emergency stop" in enable.message

    reset = node._on_reset(Trigger.Request(), Trigger.Response())
    assert reset.success
    spin_until(rclpy, node, lambda: node._reset_status == "cleared")
    assert node.sdk.arm_status == 0
    assert not node.sdk.enabled
    assert not node.core.enabled
    assert any(call[0] == "reset" for call in node.sdk.calls)

    # Clearing the vendor latch is not a motor-enable request; enable remains
    # a separate, explicit operation after the normal status is observed.
    enable = node._on_enable(Trigger.Request(), Trigger.Response())
    assert enable.success


def test_can_initialization_cannot_bypass_known_estop_latch(driver):
    _rclpy, node = driver
    from std_srvs.srv import Trigger

    node.backend = "hardware"
    node._controller_status = {"arm_status": 1, "timestamp": time.time() - 2.0}
    node._controller_status_received = time.monotonic() - 2.0
    response = node._on_initialize_can(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert "emergency stop" in response.message
    assert not any(call[0] == "initialize_can" for call in node.sdk.calls)


def test_unconfirmed_reset_blocks_enable(driver):
    _rclpy, node = driver
    from std_srvs.srv import Trigger

    node.backend = "hardware"
    node.sdk.arm_status = 1
    node._controller_status = node.sdk.read_controller_status()
    node._controller_status_received = time.monotonic()
    node._reset_status = "timed_out"
    response = node._on_enable(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert "emergency stop" in response.message

def test_brake_not_released_is_not_estop_and_prepare_can_enable_it(driver):
    rclpy, node = driver
    from std_srvs.srv import Trigger

    node.backend = "hardware"
    node.sdk.arm_status = 6
    node._controller_status = node.sdk.read_controller_status()
    node._controller_status_received = time.monotonic()
    node._controller_status_generation = 1
    node._reset_status = "timed_out"

    reset = node._on_reset(Trigger.Request(), Trigger.Response())
    assert reset.success
    assert "no reset command sent" in reset.message
    assert node._reset_status == "cleared" and node._reset_mode == "not_needed"
    assert not any(call[0] == "reset" for call in node.sdk.calls)

    def enable_and_release_brake():
        node.sdk.calls.append(("enable",))
        node.sdk.enabled = True
        node.sdk.arm_status = 0
        return True

    node.sdk.enable = enable_and_release_brake
    enable = node._on_enable(Trigger.Request(), Trigger.Response())
    assert enable.success
    spin_until(rclpy, node, lambda: node._enable_status == "enabled")
    assert node.core.enabled
    assert not any(call[0] == "move_js" for call in node.sdk.calls)


@pytest.mark.parametrize('cached', [None, 0, 1, 6])
def test_unknown_or_stale_status_allows_explicit_reset_without_enabling(driver, cached):
    rclpy,node=driver
    from std_srvs.srv import Trigger
    node.backend='hardware'
    node.sdk.arm_status=1
    node._controller_status=None if cached is None else {'arm_status':cached,'timestamp':time.time()-10}
    node._controller_status_received=0.
    before=len(node.sdk.calls)
    response=node._on_reset(None,Trigger.Response())
    assert response.success and node._reset_mode=='unknown_status_recovery'
    assert not node.core.enabled and not node._position_permitted
    spin_until(rclpy,node,lambda: node._reset_status=='cleared')
    calls=node.sdk.calls[before:]
    assert sum(call[0]=='reset' for call in calls)==1
    assert not any(call[0] in {'enable','initialize_can','move_j','move_js'} for call in calls)


def test_unknown_reset_without_returned_status_times_out_instead_of_reporting_success(driver):
    rclpy,node=driver
    from std_srvs.srv import Trigger
    from rclpy.parameter import Parameter
    node.backend='hardware'
    node._controller_status=None
    node.sdk.read_controller_status=lambda: None
    node.set_parameters([Parameter('reset_timeout_s',value=.1)])
    assert node._on_reset(None,Trigger.Response()).success
    spin_until(rclpy,node,lambda: node._reset_status=='timed_out')
    assert 'no fresh controller status returned' in node._reset_error
    assert not node.core.enabled and not node._position_permitted
    assert not node._on_enable(None,Trigger.Response()).success


def test_reset_accepts_fresh_brake_status_after_command_without_enabling(driver):
    rclpy,node=driver
    from std_srvs.srv import Trigger
    node.backend='hardware'
    node.sdk.arm_status=1
    node._controller_status=node.sdk.read_controller_status()
    node._controller_status_received=time.monotonic()
    def reset_to_brake():
        node.sdk.calls.append(('reset',))
        node.sdk.arm_status=6
        node.sdk.enabled=False
    node.sdk.reset=reset_to_brake
    assert node._on_reset(None,Trigger.Response()).success
    spin_until(rclpy,node,lambda: node._reset_status=='cleared')
    assert node._controller_status['arm_status']==6
    assert not node.core.enabled and not node._position_permitted and not node.sdk.enabled


@pytest.mark.parametrize('code',[0,6])
def test_normal_peer_reset_is_idempotent_and_sends_no_reset_packet(driver,code):
    _rclpy,node=driver
    from std_srvs.srv import Trigger
    node.backend='hardware'
    node.sdk.arm_status=code
    node._controller_status=node.sdk.read_controller_status()
    node._controller_status_received=time.monotonic()
    baseline=node._reset_seq
    assert node._on_reset(None,Trigger.Response()).success
    assert node._reset_seq==baseline+1 and node._reset_status=='cleared'
    assert node._reset_mode=='not_needed'
    assert not any(call[0] in {'reset','enable','move_j','move_js'} for call in node.sdk.calls)


@pytest.mark.parametrize('busy',['enabled','position','command','can_init'])
def test_unknown_recovery_cannot_bypass_motion_or_pending_operation(driver,busy):
    _rclpy,node=driver
    from std_srvs.srv import Trigger
    from concurrent.futures import Future
    node.backend='hardware'
    node._controller_status=None
    if busy=='enabled':node.core.set_enabled(True)
    elif busy=='position':node._position_active=True
    elif busy=='command':node._command_future=Future()
    else:node._can_init_status='queued'
    response=node._on_reset(None,Trigger.Response())
    assert not response.success and 'no pending driver operation' in response.message
    assert not any(call[0]=='reset' for call in node.sdk.calls)


def test_recovery_requires_controller_sample_newer_than_reset_request(driver):
    rclpy,node=driver
    from std_srvs.srv import Trigger
    node.backend='hardware'
    original=node.sdk.read_controller_status
    cached=original()
    cached['arm_status']=0
    node.sdk.read_controller_status=lambda: dict(cached)
    node._controller_status=None
    assert node._on_reset(None,Trigger.Response()).success
    spin_until(rclpy,node,lambda: node._reset_status=='waiting_status' and node._controller_status_generation>node._reset_baseline_generation)
    assert cached['timestamp']<node._reset_status_not_before
    assert node._reset_status!='cleared'
    node.sdk.read_controller_status=original
    spin_until(rclpy,node,lambda: node._reset_status=='cleared')
    assert not node.core.enabled and not node._position_permitted
