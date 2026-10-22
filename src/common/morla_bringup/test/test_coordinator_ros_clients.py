"""Exercise real rclpy client registration without sending hardware requests."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.mark.parametrize("profile, mode, expected", [
    ("nero_dual_xhand", "real", True),
    ("nero_dual_xhand", "sim", False),
    ("astral_left_gripper", "real", False),
])
def test_can_initialization_capability_is_specific_to_real_nero(profile, mode, expected):
    rclpy = pytest.importorskip("rclpy")
    from morla_bringup.control_coordinator_node import ControlCoordinator
    rclpy.init(args=["--ros-args", "-p", f"profile_id:={profile}", "-p", f"operating_mode:={mode}"])
    node = None
    try:
        node = ControlCoordinator()
        assert node.capabilities["initialize_can"] is expected
        assert node.capabilities["prepare"] is (mode == "real")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_coordinator_keeps_rclpy_client_registry_and_reuses_robot_clients():
    rclpy = pytest.importorskip("rclpy")
    from morla_bringup.control_coordinator_node import ControlCoordinator

    # Run in a separate test process/domain; clients never send requests.
    node = None
    try:
        rclpy.init(args=["--ros-args", "-p", "profile_id:=nero_dual_xhand",
                         "-p", "operating_mode:=real"])
        node = ControlCoordinator()
        before = list(node.clients)
        left = node._dispatch_service("left", "enable")
        assert left is node._dispatch_service("left", "enable")
        right = node._dispatch_service("right", "enable")
        assert left is not right
        assert list(node.clients) == before + [left, right]
        assert left.srv_name == "/left_arm/nero_robot_driver/enable"
        assert right.srv_name == "/right_arm/nero_robot_driver/enable"
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize("operation, fail_side", [
    ("enable", None), ("initialize_can", None), ("initialize_can", "right"),
    ("prepare", None), ("prepare", "right"), ("prepare_warm", None), ("prepare_mixed", None),
])
def test_robot_operation_reaches_confirmation_with_simulated_driver_status(operation, fail_side):
    rclpy = pytest.importorskip("rclpy")
    import json
    import time
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from morla_bringup.control_coordinator_node import ControlCoordinator

    coordinator = peer = executor = None
    rclpy.init(args=["--ros-args", "-p", "profile_id:=nero_dual_xhand",
                     "-p", "operating_mode:=real"])
    try:
        coordinator = ControlCoordinator()
        peer = Node("coordinator_test_drivers")
        enabled = {"left": False, "right": False}
        can_seq = {"left": 0, "right": 0}
        can_status = {"left": "idle", "right": "idle"}
        calls = []
        stops = []
        pubs = {}
        for side in enabled:
            pubs[f"{side}_driver"] = peer.create_publisher(
                String, f"/{side}_arm/nero_robot_driver/status", 10)
            pubs[f"{side}_arm"] = peer.create_publisher(
                String, f"/{side}_arm/nero_arm_teleop/status", 10)
            pubs[f"{side}_ee"] = peer.create_publisher(
                String, f"/teleop/writer_status/{side}_ee", 10)

            def enable(_request, response, s=side):
                calls.append(s)
                enabled[s] = True
                response.success = True
                response.message = "simulated driver accepted enable"
                return response

            peer.create_service(Trigger, f"/{side}_arm/nero_robot_driver/enable", enable)

            def initialize_can(_request, response, s=side):
                calls.append(s)
                can_seq[s] += 1
                can_status[s] = "timed_out" if s == fail_side else "ready"
                response.success = True
                response.message = "simulated CAN initialization queued"
                return response

            peer.create_service(Trigger, f"/{side}_arm/nero_robot_driver/initialize_can", initialize_can)

            def estop(_request, response, s=side):
                stops.append(s)
                response.success = True
                response.message = "simulated stop accepted"
                return response

            peer.create_service(Trigger, f"/{side}_arm/nero_robot_driver/estop", estop)

        def publish_status():
            for side in enabled:
                pubs[f"{side}_driver"].publish(String(data=json.dumps({
                    "connected": True, "enabled": enabled[side],
                    "motors_enabled": enabled[side], "motor_status_fresh": True,
                    "position_permitted": enabled[side],
                    "controller_status_fresh": True, "controller_arm_status": 0,
                    "feedback_fresh": (operation not in {"initialize_can", "prepare"} and not (operation == "prepare_mixed" and side == "left")) or can_status[side] == "ready",
                    "can_init_seq": can_seq[side], "can_init_status": can_status[side],
                    "can_init_error": "simulated CAN feedback timeout" if side == fail_side else "",
                })))
                pubs[f"{side}_arm"].publish(String(data=json.dumps({
                    "armed": False, "ownership": None,
                })))
                pubs[f"{side}_ee"].publish(String(data=json.dumps({
                    "writer_id": f"{side}_ee", "armed": False, "ownership": None,
                })))

        peer.create_timer(0.05, publish_status)
        executor = SingleThreadedExecutor()
        executor.add_node(coordinator)
        executor.add_node(peer)
        client = peer.create_client(Trigger, f"/morla/control/{'prepare' if operation.startswith('prepare') else operation}")
        assert client.wait_for_service(timeout_sec=3.0)
        # Teleop-only mode intentionally waits two seconds for graph discovery
        # before accepting an absent collector as NO_COLLECTOR.
        discovery_deadline = time.monotonic() + 4.0
        while time.monotonic() < discovery_deadline:
            executor.spin_once(timeout_sec=0.02)
            if (coordinator._effective_collector_state() == "NO_COLLECTOR"
                    and coordinator._writers_frozen()):
                break
        assert coordinator._effective_collector_state() == "NO_COLLECTOR"
        assert coordinator._writers_frozen()
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if coordinator._active is not None and coordinator._active.phase in {"completed", "failed"}:
                break
        assert future.done() and future.result().success
        assert coordinator._active is not None
        assert coordinator._active.phase == ("failed" if fail_side else "completed"), coordinator._active.message
        expected_calls = ["left", "right"]
        if operation == "prepare_mixed":
            expected_calls = ["left", "left", "right"]
        elif operation == "prepare" and not fail_side:
            expected_calls = ["left", "left", "right", "right"]
        assert sorted(calls) == expected_calls
        expected_clients = 3 if operation == "prepare_mixed" else 4 if fail_side or operation == "prepare" else 2
        assert len(list(coordinator.clients)) == expected_clients
        if operation.startswith("prepare"):
            assert all(enabled.values()) if not fail_side else not any(enabled.values())
        if fail_side:
            stop_deadline = time.monotonic() + 1.0
            while len(stops) < 2 and time.monotonic() < stop_deadline:
                executor.spin_once(timeout_sec=0.02)
            assert sorted(stops) == ["left", "right"]
            assert coordinator._active.components["left"]["result"] == "cancelled"
        if operation == "initialize_can":
            assert not any(enabled.values())
            if fail_side:
                assert "simulated CAN feedback timeout" in coordinator._active.message
    finally:
        if executor is not None:
            executor.shutdown()
        if peer is not None:
            peer.destroy_node()
        if coordinator is not None:
            coordinator.destroy_node()
        rclpy.shutdown()


def test_position_confirmation_requires_new_measured_arrival():
    pytest.importorskip("rclpy")
    from types import SimpleNamespace
    from morla_bringup.control_coordinator_node import ControlCoordinator

    status = {"position_seq": 4, "position_active": False,
              "position_status": "arrived", "position_error_rad": 0.01,
              "position_kind": "workpos", "position_stage": 4, "position_stage_count": 4,
              "feedback_fresh": True, "controller_status_fresh": True, "controller_arm_status": 0,
              "motor_status_fresh": True, "motors_enabled": True, "position_permitted": True}
    fake = SimpleNamespace(profile=SimpleNamespace(robot_id="nero"),
                           arms=[{"side": "left"}],
                           _fresh_driver_status=lambda side: status)
    execution = {"workpos_stage": 4, "status_baseline": {"left": 3}}
    for operation in ("position", "workpos", "workpos_direct"):
        assert ControlCoordinator._operation_confirmed(fake, operation, execution)
        status["position_status"] = "sent_arrival_unverified"
        assert not ControlCoordinator._operation_confirmed(fake, operation, execution)
        status["position_status"] = "arrived"
        status["position_active"] = True
        assert not ControlCoordinator._operation_confirmed(fake, operation, execution)
        status["position_active"] = False
        status["position_seq"] = 3
        assert not ControlCoordinator._operation_confirmed(fake, operation, execution)
        status["position_seq"] = 4


def test_position_failure_is_scoped_to_the_new_driver_sequence():
    pytest.importorskip("rclpy")
    import time
    from types import SimpleNamespace
    from morla_bringup.control_coordinator_node import ControlCoordinator

    failures = []
    status = {"position_seq": 3, "position_status": "failed", "last_error": "test failure"}
    fake = SimpleNamespace(
        profile=SimpleNamespace(robot_id="nero"), arms=[{"side": "left"}],
        _active=SimpleNamespace(components={}),
        _execution={"operation": "position", "phase": "executing",
                    "deadline": time.monotonic() + 10., "status_baseline": {"left": 3}},
        _fresh_driver_status=lambda side: status,
        _operation_confirmed=lambda operation, execution: False,
        _fail=failures.append,
    )
    ControlCoordinator._tick(fake)
    assert not failures
    status["position_seq"] = 4
    ControlCoordinator._tick(fake)
    assert failures == ["left: test failure"]
    assert fake._active.components["left"]["result"] == "failed"


def test_motion_failure_requests_bilateral_stop_and_does_not_leave_peer_accepted():
    pytest.importorskip("rclpy")
    from types import SimpleNamespace
    from std_srvs.srv import Trigger
    from morla_bringup.control_coordinator_node import ControlCoordinator

    class StopClient:
        def __init__(self):
            self.requests = []

        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.requests.append(request)
            return object()

    clients = {side: StopClient() for side in ("left", "right")}
    active = SimpleNamespace(
        operation="workpos",
        components={
            "left": {"result": "failed", "message": "position arrival timed out"},
            "right": {"result": "accepted", "message": "request queued"},
        },
    )
    phases = []
    fake = SimpleNamespace(
        _active=active,
        _execution={"futures": [("left", object()), ("right", object())]},
        _operation_clients=lambda operation: list(clients.items()) if operation == "estop" else [],
        _set_phase=lambda phase, message, result=None: phases.append((phase, message, result)),
    )

    ControlCoordinator._fail(fake, "left: position arrival timed out")

    assert all(len(client.requests) == 1 for client in clients.values())
    assert all(isinstance(client.requests[0], Trigger.Request) for client in clients.values())
    assert active.components["left"] == {
        "result": "failed", "message": "position arrival timed out",
    }
    assert active.components["right"]["result"] == "cancelled"
    assert "electronic stop requested" in active.components["right"]["message"]
    assert phases[-1][0] == "failed"
    assert "verify driver status" in phases[-1][1]
    assert fake._execution is None


def test_astral_prepare_uses_shared_enable_service_without_homing():
    rclpy = pytest.importorskip('rclpy')
    import time
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger
    from morla_bringup.control_coordinator_node import ControlCoordinator

    coordinator = peer = executor = None
    rclpy.init(args=['--ros-args', '-p', 'profile_id:=astral_dual_gripper', '-p', 'operating_mode:=real'])
    try:
        coordinator = ControlCoordinator()
        peer = Node('astral_prepare_test_driver')
        calls = []

        def enable(_request, response):
            calls.append('enable')
            response.success = True
            response.message = 'simulated enable completed'
            return response

        peer.create_service(Trigger, '/astral_robot_driver/enable', enable)
        # Only writer telemetry is simulated; the real request, plan, ROS
        # service dispatch and final-response handling run through the node.
        coordinator._writers_frozen = lambda: True
        coordinator._all_writers_manual_or_idle = lambda: True
        coordinator._started_monotonic -= 3.0
        response = coordinator._on_request('prepare', Trigger.Request(), Trigger.Response())
        assert response.success
        executor = SingleThreadedExecutor()
        executor.add_node(coordinator)
        executor.add_node(peer)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and coordinator._active.phase not in {'completed', 'failed'}:
            executor.spin_once(timeout_sec=0.02)
        assert coordinator._active.phase == 'completed', coordinator._active.message
        assert calls == ['enable']
        assert [client.srv_name for client in coordinator.clients] == ['/astral_robot_driver/enable']
    finally:
        if executor is not None:
            executor.shutdown()
        if peer is not None:
            peer.destroy_node()
        if coordinator is not None:
            coordinator.destroy_node()
        rclpy.shutdown()

def test_nero_enable_confirmation_requires_normal_controller_status():
    pytest.importorskip("rclpy")
    from types import SimpleNamespace
    from morla_bringup.control_coordinator_node import ControlCoordinator

    status = {"enabled": True, "controller_status_fresh": True,
              "controller_arm_status": 6, "motor_status_fresh": True, "motors_enabled": True, "position_permitted": True}
    fake = SimpleNamespace(
        profile=SimpleNamespace(robot_id="nero"), arms=[{"side": "left"}],
        _fresh_driver_status=lambda side: status,
    )
    assert not ControlCoordinator._operation_confirmed(fake, "enable", {})
    status["controller_arm_status"] = 0
    assert ControlCoordinator._operation_confirmed(fake, "enable", {})
    status["motors_enabled"] = False
    assert not ControlCoordinator._operation_confirmed(fake, "enable", {})
    status["motors_enabled"] = True
    status["motor_status_fresh"] = False
    assert not ControlCoordinator._operation_confirmed(fake, "enable", {})
    status["motor_status_fresh"] = True
    status["controller_status_fresh"] = False
    assert not ControlCoordinator._operation_confirmed(fake, "enable", {})


def test_reset_confirmation_requires_new_fresh_no_estop_status_without_streaming():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from morla_bringup.control_coordinator_node import ControlCoordinator
    status=dict(reset_seq=2,reset_status='cleared',controller_status_fresh=True,controller_arm_status=0,enabled=False)
    fake=SimpleNamespace(profile=SimpleNamespace(robot_id='nero'),arms=[{'side':'left'}],_fresh_driver_status=lambda side:status)
    execution=dict(status_baseline={'left':1})
    for code in (0,6):
        status['controller_arm_status']=code
        assert ControlCoordinator._operation_confirmed(fake,'reset',execution)
    for key,value in [('reset_seq',1),('reset_status','waiting_status'),('controller_status_fresh',False),('controller_arm_status',1),('controller_arm_status',2),('enabled',True)]:
        previous=status[key];status[key]=value
        assert not ControlCoordinator._operation_confirmed(fake,'reset',execution)
        status[key]=previous
