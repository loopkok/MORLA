"""Real ROS service dispatch with simulated, independently delayed arm feedback."""
import json
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.mark.parametrize('fail_stage', [None, 1, 2, 3, 4])
def test_nero_workpos_has_bilateral_arrival_barriers_and_stops_both_on_failure(fail_stage):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from morla_bringup.control_coordinator_node import ControlCoordinator

    coordinator = peer = executor = None
    rclpy.init(args=['--ros-args', '-p', 'profile_id:=nero_dual_xhand', '-p', 'operating_mode:=real'])
    try:
        coordinator = ControlCoordinator()
        peer = Node('nero_home_test_drivers')
        # Writer lifecycle is covered separately; this test exercises actual
        # service dispatch and the independent driver-status arrival barriers.
        coordinator._writers_frozen = lambda: True
        coordinator._all_writers_manual_or_idle = lambda: True
        coordinator._effective_collector_state = lambda: 'IDLE'
        states = {side: dict(connected=True, enabled=False, feedback_fresh=True,
                            motors_enabled=True, motor_status_fresh=True, position_permitted=True,
                            controller_status_fresh=True, controller_arm_status=0,
                            position_seq=8, position_stage=1, position_kind='workpos',
                            position_stage_count=1, position_status='arrived',
                            position_active=False, position_error_rad=0.)
                  for side in ('left', 'right')}
        calls, stops, pubs = [], [], {}
        for side in states:
            pubs[side] = peer.create_publisher(String, f'/{side}_arm/nero_robot_driver/status', 10)
            for stage, service in enumerate(('position', 'position_point2', 'position_point3', 'position_point4'), 1):
                def move(_req, response, s=side, n=stage):
                    calls.append((s, n))
                    states[s].update(position_seq=states[s]['position_seq'] + 1,
                                     position_stage=n, position_kind='workpos', position_stage_count=4,
                                     position_status='waiting_arrival', position_active=True,
                                     position_error_rad=.4, enabled=False)
                    response.success = True
                    response.message = f'workpos stage {n}/4 queued'
                    return response
                peer.create_service(Trigger, f'/{side}_arm/nero_robot_driver/{service}', move)

            def stop(_req, response, s=side):
                stops.append(s)
                response.success = True
                return response
            peer.create_service(Trigger, f'/{side}_arm/nero_robot_driver/estop', stop)

        def publish():
            for side, state in states.items():
                pubs[side].publish(String(data=json.dumps(state)))
        peer.create_timer(.02, publish)
        executor = SingleThreadedExecutor()
        executor.add_node(coordinator)
        executor.add_node(peer)

        def spin_until(condition, timeout=3.):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and not condition():
                executor.spin_once(timeout_sec=.01)
            assert condition(), (calls, coordinator._active.message if coordinator._active else 'no operation')

        spin_until(lambda: len(coordinator._driver_status) == 2)
        assert coordinator._on_request('workpos', Trigger.Request(), Trigger.Response()).success
        for stage in range(1, 5):
            spin_until(lambda: len(calls) == stage * 2)
            assert set(calls[-2:]) == {('left', stage), ('right', stage)}
            arrived = 'arrived' if stage == 4 else 'stage_arrived'
            states['left'].update(position_status=arrived, position_active=stage < 4,
                                  position_error_rad=0.)
            # The left arm cannot advance while the right arm is still moving.
            until = time.monotonic() + .2
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.01)
            assert len(calls) == stage * 2
            assert coordinator._active.phase == 'executing'
            if stage == fail_stage:
                states['right'].update(position_status='timed_out', last_error='injected HOME stage timeout')
                spin_until(lambda: coordinator._active.phase == 'failed' and len(stops) == 2)
                assert set(stops) == {'left', 'right'}
                assert len(calls) == stage * 2
                assert 'injected HOME stage timeout' in coordinator._active.message
                return
            states['right'].update(position_status=arrived, position_active=stage < 4,
                                   position_error_rad=0.)
        spin_until(lambda: coordinator._active.phase == 'completed')
        assert len(calls) == 8
        assert not stops
        assert coordinator._active.result is True
        assert all(not state['enabled'] for state in states.values())
    finally:
        if executor:
            executor.shutdown()
        if peer:
            peer.destroy_node()
        if coordinator:
            coordinator.destroy_node()
        rclpy.shutdown()
