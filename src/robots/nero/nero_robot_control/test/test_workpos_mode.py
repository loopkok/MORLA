"""Four-stage driver progression never skips the measured-arrival barrier."""
import time
import numpy as np
import pytest
pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from test_enable_confirmation import driver, request_enable, spin_until
from test_position_mode import assert_no_streaming
from test_home_mode import arrive
from nero_robot_control.workpos_trajectory import workpos_targets


def start(rclpy, node):
    seed = np.array([.25,.5,-.6,1.1,.8,-.3,.2])
    with node.sdk._lock:
        node.sdk.q = seed.copy()
    spin_until(rclpy,node,lambda: np.allclose(node.core.feedback,seed))
    request_enable(node)
    spin_until(rclpy,node,lambda: node.core.enabled)
    node.position_pose = np.array([-.4,1.28,-.95,1.31,2.68,-.31,-.16])
    expected = workpos_targets(node.side)
    moves, speeds = [], []
    node.sdk.move_j = lambda q: moves.append(q.copy())
    set_mode = node.sdk.set_position_mode
    def mode(speed):
        speeds.append(speed)
        set_mode(speed)
    node.sdk.set_position_mode = mode
    mark = len(node.sdk.calls)
    assert node._on_position(None,Trigger.Response()).success
    spin_until(rclpy,node,lambda: node._position_status == 'waiting_arrival')
    return expected,moves,speeds,mark


def test_workpos_four_measured_stages_at_configured_speed(driver):
    rclpy,node = driver
    expected,moves,speeds,mark = start(rclpy,node)
    for stage in range(1,5):
        np.testing.assert_allclose(moves[-1],expected[stage-1])
        assert not node._advance_position('home',2,Trigger.Response()).success
        assert not node._advance_position('workpos',stage+1,Trigger.Response()).success
        arrive(rclpy,node)
        deadline = time.monotonic()+.1
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
        assert len(moves)==stage
        if stage<4:
            assert not node._advance_position('workpos',stage+2,Trigger.Response()).success
            assert node._advance_position('workpos',stage+1,Trigger.Response()).success
            spin_until(rclpy,node,lambda: node._position_status=='waiting_arrival')
    assert node._position_status=='arrived' and not node._position_active
    assert speeds==[5]*4
    assert_no_streaming(node,mark)


@pytest.mark.parametrize('stage',range(1,5))
def test_estop_interrupts_workpos_at_every_stage(driver,stage):
    rclpy,node=driver
    expected,moves,speeds,mark=start(rclpy,node)
    for n in range(2,stage+1):
        arrive(rclpy,node)
        assert node._advance_position('workpos',n,Trigger.Response()).success
        spin_until(rclpy,node,lambda: node._position_status=='waiting_arrival')
    assert node._on_estop(None,Trigger.Response()).success
    spin_until(rclpy,node,lambda: node._estop_status=='sent')
    assert len(moves)==stage and node._position_status=='cancelled'
    assert not node._advance_position('workpos',stage+1,Trigger.Response()).success
    assert_no_streaming(node,mark)


def test_all_workpos_targets_checked_before_first_command(driver):
    rclpy,node=driver
    request_enable(node)
    spin_until(rclpy,node,lambda: node.core.enabled)
    points=list(node.get_parameter("workpos_waypoints_deg").value)
    points[-1]=10000.
    node.set_parameters([Parameter("workpos_waypoints_deg",value=points)])
    mark=len(node.sdk.calls)
    response=node._on_position(None,Trigger.Response())
    assert not response.success and 'limits' in response.message
    assert not any(call[0]=='move_j' for call in node.sdk.calls[mark:])


def test_piano_workpos_accepts_two_and_half_degree_error_and_advances(driver):
    rclpy,node=driver
    node.set_parameters([Parameter('workpos_arrival_tolerance_deg',value=3.)])
    expected,moves,_,_=start(rclpy,node)
    with node.sdk._lock:
        node.sdk.q=expected[0].copy();node.sdk.q[0]+=np.deg2rad(2.5)
    spin_until(rclpy,node,lambda:node._position_status=='stage_arrived')
    assert node._position_arrival_tolerance_rad==pytest.approx(np.deg2rad(3.))
    assert node._advance_position('workpos',2,Trigger.Response()).success
    spin_until(rclpy,node,lambda:len(moves)==2)


def test_piano_workpos_rejects_error_beyond_three_degrees(driver):
    rclpy,node=driver
    node.set_parameters([Parameter('workpos_arrival_tolerance_deg',value=3.)])
    expected,moves,_,_=start(rclpy,node)
    with node.sdk._lock:
        node.sdk.q=expected[0].copy();node.sdk.q[0]+=np.deg2rad(3.1)
    deadline=time.monotonic()+.4
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    assert node._position_status=='waiting_arrival'
    assert not node._advance_position('workpos',2,Trigger.Response()).success
    assert len(moves)==1


def test_piano_tolerance_does_not_change_home_or_original_default(driver):
    from test_home_mode import start_home
    rclpy,node=driver
    assert node.get_parameter('workpos_arrival_tolerance_deg').value==pytest.approx(np.rad2deg(.02))
    node.set_parameters([Parameter('workpos_arrival_tolerance_deg',value=3.)])
    start_home(rclpy,node)
    assert node._position_arrival_tolerance_rad==pytest.approx(.02)
    with node.sdk._lock:
        node.sdk.q=node._position_target.copy();node.sdk.q[0]+=np.deg2rad(2.5)
    deadline=time.monotonic()+.4
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    assert node._position_status=='waiting_arrival'


def test_invalid_workpos_tolerance_rejects_before_motion(driver):
    rclpy,node=driver
    request_enable(node);spin_until(rclpy,node,lambda:node.core.enabled)
    node.set_parameters([Parameter('workpos_arrival_tolerance_deg',value=6.)])
    mark=len(node.sdk.calls)
    result=node._on_position(None,Trigger.Response())
    assert not result.success and 'tolerance' in result.message
    assert not any(call[0]=='move_j' for call in node.sdk.calls[mark:])


def test_ros_stage_services_execute_forward_and_reverse_path(driver):
    from nero_robot_control.home_trajectory import home_targets
    rclpy, node = driver
    request_enable(node)
    spin_until(rclpy, node, lambda: node.core.enabled)
    for names, targets in [
        (('position','position_point2','position_point3','position_point4'), workpos_targets(node.side)),
        (('home','home_point3','home_point2','home_point1','home_final_pose'), home_targets(node.side)),
    ]:
        for i, (name, target) in enumerate(zip(names, targets)):
            client = node.create_client(Trigger, node.get_fully_qualified_name() + '/' + name)
            assert client.wait_for_service(timeout_sec=1.)
            future = client.call_async(Trigger.Request())
            spin_until(rclpy, node, future.done)
            assert future.result().success, future.result().message
            spin_until(rclpy, node, lambda: node._position_status == ('arrived' if i == len(targets)-1 else 'stage_arrived'))
            np.testing.assert_allclose(node.core.feedback, target)
            node.destroy_client(client)
    assert not node.core.enabled and node._position_permitted
    assert node.sdk.motion_mode == 'j'
