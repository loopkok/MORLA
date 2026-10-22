"""Start must not confuse an armed command writer with measured readiness."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from morla_bringup.coordinator_logic import nero_start_blockers


def ready_status():
    return dict(connected=True, feedback_fresh=True, feedback_read_ok=True,
                streaming_enabled=True, enabled=True, motors_enabled=True,
                motor_status_fresh=True, controller_status_fresh=True,
                controller_arm_status=0, position_active=False)


@pytest.mark.parametrize("field,value", [
    ("connected", False), ("feedback_fresh", False), ("feedback_read_ok", False),
    ("streaming_enabled", False), ("motors_enabled", False),
    ("motor_status_fresh", False), ("controller_status_fresh", False),
    ("controller_arm_status", 1), ("controller_arm_status", 6), ("position_active", True),
])
def test_each_measured_hardware_gate_blocks_start(field, value):
    status = ready_status()
    status[field] = value
    assert nero_start_blockers(status)


def test_ready_streaming_and_missing_or_old_driver_contract():
    assert not nero_start_blockers(ready_status())
    assert nero_start_blockers({})
    status = ready_status()
    del status["streaming_enabled"]
    assert nero_start_blockers(status)


@pytest.fixture
def coordinator():
    rclpy = pytest.importorskip("rclpy")
    from types import SimpleNamespace
    from morla_bringup.control_coordinator_node import ControlCoordinator
    rclpy.init(args=["--ros-args", "-p", "profile_id:=nero_dual_xhand", "-p", "operating_mode:=real"])
    node = ControlCoordinator()
    node.signals = []
    for field in ("_teleop_start", "_teleop_armed", "_teleop_disarm", "_policy_cmd"):
        setattr(node, field, SimpleNamespace(publish=lambda msg, f=field: node.signals.append((f, msg.data))))
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def seed_coordinator(node):
    import json
    from std_msgs.msg import String
    for side in ("left", "right"):
        node._on_driver_status(side, String(data=json.dumps(ready_status())))
        node._on_arm_teleop_status(side, node._arm_writer_ids[side], String(data=json.dumps(
            dict(armed=True, ownership=None, feedback_fresh=True, wrist_fresh=True))))
        node._on_hand_status(side, String(data=json.dumps(dict(side=side, feedback_fresh=True))))
        node._on_writer_status(f"{side}_ee", String(data=json.dumps(
            dict(writer_id=f"{side}_ee", armed=True, ownership=None, input_fresh=True))))


def start(node):
    from std_srvs.srv import Trigger
    assert node._on_request("start", Trigger.Request(), Trigger.Response()).success
    node._tick()


@pytest.mark.parametrize("broken", ["driver", "motor", "hand", "landmarks", "wrist", "old_hand_state"])
def test_start_rejects_partial_readiness_without_publishing_arm_signal(coordinator, broken):
    seed_coordinator(coordinator)
    if broken == "driver":
        coordinator._driver_status["right"]["streaming_enabled"] = False
    elif broken == "motor":
        coordinator._driver_status["left"]["motor_status_fresh"] = False
    elif broken == "hand":
        coordinator._hand_status["right"]["feedback_fresh"] = False
    elif broken == "landmarks":
        coordinator._writer_status["left_ee"]["input_fresh"] = False
    elif broken == "wrist":
        coordinator._teleop_status["right"]["wrist_fresh"] = False
    else:
        coordinator._hand_received["right"] -= 2.0
    start(coordinator)
    assert coordinator._active.phase == "failed"
    assert ("_teleop_start", True) not in coordinator.signals
    assert ("_teleop_armed", False) in coordinator.signals
    assert ("_teleop_disarm", True) in coordinator.signals


def test_ready_start_requires_new_writer_acknowledgements_not_cached_armed(coordinator):
    seed_coordinator(coordinator)
    assert not coordinator._start_blockers()
    start(coordinator)
    coordinator._tick()
    assert coordinator._active.phase == "executing"
    seed_coordinator(coordinator)
    coordinator._tick()
    assert coordinator._active.phase == "completed"
    assert ("_teleop_start", True) in coordinator.signals


def test_hardware_failure_during_start_freezes_all_writers(coordinator):
    seed_coordinator(coordinator)
    start(coordinator)
    coordinator._driver_status["right"]["motors_enabled"] = False
    coordinator._tick()
    assert coordinator._active.phase == "failed"
    assert coordinator.signals[-2:] == [("_teleop_armed", False), ("_teleop_disarm", True)]


def test_start_timeout_freezes_all_writers(coordinator):
    import time
    seed_coordinator(coordinator)
    start(coordinator)
    coordinator._execution["deadline"] = time.monotonic() - 1.0
    coordinator._tick()
    assert coordinator._active.phase == "failed"
    assert ("_teleop_disarm", True) in coordinator.signals


def test_readiness_is_visible_to_web_and_hand_adapter_topics_are_subscribed(coordinator):
    import json
    from types import SimpleNamespace
    payloads = []
    coordinator._state_pub = SimpleNamespace(publish=lambda msg: payloads.append(json.loads(msg.data)))
    coordinator._publish_state()
    assert payloads[-1]["start_readiness"]["ready"] is False
    topics = {sub.topic_name for sub in coordinator.subscriptions}
    assert {"/left_hand/adapter/status", "/right_hand/adapter/status"} <= topics
    seed_coordinator(coordinator)
    coordinator._publish_state()
    assert payloads[-1]["start_readiness"] == {"ready": True, "blockers": []}


def test_pause_preempts_an_accepted_start_without_an_estop_or_motor_operation(coordinator):
    from std_srvs.srv import Trigger
    seed_coordinator(coordinator)
    start(coordinator)
    old_id = coordinator._active.operation_id
    assert coordinator._on_request("pause", Trigger.Request(), Trigger.Response()).success
    assert coordinator._active.operation_id != old_id and coordinator._active.operation == "pause"
    coordinator._tick()
    assert ("_teleop_disarm", True) in coordinator.signals
