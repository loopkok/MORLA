import json
import time
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
from std_msgs.msg import String
from morla_operator_control.commands import request
from morla_operator_control.router import OperatorRouter


@pytest.fixture
def router():
    rclpy.init()
    node = OperatorRouter()
    node.events, node.collect_commands = [], []
    node._event_pub = SimpleNamespace(publish=lambda msg: node.events.append(json.loads(msg.data)))
    node._collect_pub = SimpleNamespace(publish=lambda msg: node.collect_commands.append(json.loads(msg.data)))
    node.count_publishers = lambda topic: 1
    node._control = dict(profile_id="nero_dual_xhand", operating_mode="real", teleop_armed=False)
    node._collect = dict(profile_id="nero_dual_xhand", runtime_mode="real", state="IDLE", episode_index=-1)
    node._control_at = node._collect_at = time.monotonic()
    node.futures = []
    def call(req):
        future = Future()
        node.futures.append(future)
        return future
    for op in ("start", "pause", "release"):
        node._service_clients[op] = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    try:
        yield node
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()


def issue(node, command, source="pedal", **extra):
    row = request(command, source, **extra)
    node._on_command(String(data=json.dumps(row)))
    return row


def test_accepted_trigger_is_not_final_success_and_wrong_operation_cannot_confirm(router):
    issue(router, "teleop_start")
    assert router.events[-1]["phase"] == "submitted"
    router.futures[-1].set_result(SimpleNamespace(success=True, message="accepted operation_id=abc123"))
    assert router.events[-1]["phase"] == "submitted"
    router._on_control(String(data=json.dumps(dict(operation_id="other", phase="completed"))))
    assert router._pending
    router._on_control(String(data=json.dumps(dict(operation_id="abc123", phase="completed"))))
    assert router.events[-1]["phase"] == "confirmed" and not router._pending


def test_control_failure_and_timeout_are_not_announced_as_success(router):
    issue(router, "teleop_start")
    router.futures[-1].set_result(SimpleNamespace(success=False, message="feedback missing"))
    assert router.events[-1]["phase"] == "failed"
    issue(router, "teleop_start")
    router._pending["deadline"] -= 11
    router._tick()
    assert router.events[-1]["phase"] == "failed"


def test_teleop_already_armed_is_idempotent_not_reanchor(router):
    router._control["teleop_armed"] = True
    issue(router, "teleop_start")
    assert not router.futures and router.events[-1]["phase"] == "confirmed"


def test_record_start_requires_teleop_and_matching_profile_and_fresh_state(router):
    issue(router, "record_start")
    assert router.events[-1]["phase"] == "rejected" and not router.collect_commands
    router._control["teleop_armed"] = True
    router._collect["profile_id"] = "other"
    issue(router, "record_start")
    assert not router.collect_commands
    router._collect["profile_id"] = "nero_dual_xhand"
    router._collect_at -= 3
    issue(router, "record_start")
    assert not router.collect_commands


def test_record_command_uses_same_collect_api_and_correlated_actual_ack(router):
    router._control["teleop_armed"] = True
    row = issue(router, "record_start", "voice_host")
    sent = router.collect_commands[-1]
    assert sent["command"] == "start" and sent["request_id"] == row["request_id"]
    router._on_collect_ack(String(data=json.dumps(dict(request_id="wrong", success=True))))
    assert router._pending
    router._on_collect_ack(String(data=json.dumps(dict(request_id=row["request_id"], success=False, message="disk error"))))
    assert router.events[-1]["phase"] == "failed"


def test_voice_discard_needs_confirmation_and_cannot_target_changed_episode(router):
    router._collect.update(state="RECORDING", episode_index=2)
    issue(router, "record_discard", "voice_host")
    assert not router.collect_commands and router.events[-1]["phase"] == "confirmation_required"
    router._collect["episode_index"] = 3
    issue(router, "confirm_discard", "voice_host")
    assert not router.collect_commands


def test_voice_confirmation_and_pedal_long_press_share_collect_discard(router):
    router._collect.update(state="RECORDING", episode_index=2)
    issue(router, "record_discard", "voice_host")
    issue(router, "confirm_discard", "voice_host")
    assert router.collect_commands[-1]["command"] == "discard"
    router._pending = None
    issue(router, "record_discard", "pedal", long_press=True)
    assert router.collect_commands[-1]["command"] == "discard"


def test_pause_can_preempt_start_without_late_start_ack_restoring_old_request(router):
    issue(router, "teleop_start")
    old = router.futures[-1]
    issue(router, "teleop_pause")
    assert router._pending["operation"] == "pause"
    old.set_result(SimpleNamespace(success=True, message="accepted operation_id=old"))
    assert router._pending["operation"] == "pause"


def test_saving_does_not_block_pause_and_save_ack_is_still_reported(router):
    router._collect.update(state="RECORDING", episode_index=2)
    row = issue(router, "record_stop")
    issue(router, "teleop_pause")
    assert router._saving and router._pending["operation"] == "pause"
    router._on_collect_ack(String(data=json.dumps(dict(request_id=row["request_id"], success=True, message="saved"))))
    assert not router._saving and router._pending["operation"] == "pause"
    assert router.events[-1]["phase"] == "confirmed"
