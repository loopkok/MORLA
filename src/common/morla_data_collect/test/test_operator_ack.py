"""Structured commands retain legacy controls and report actual save failures."""
import json
import time
import uuid
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from morla_data_collect.data_collect_node import DataCollectNode


@pytest.fixture
def collector(tmp_path):
    rclpy.init()
    node = DataCollectNode(parameter_overrides=[Parameter("save_root", value=str(tmp_path)), Parameter("session", value="operator")])
    node.acks = []
    node._control_ack_pub = SimpleNamespace(publish=lambda msg: node.acks.append(json.loads(msg.data)))
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command(node, cmd, **extra):
    return dict(command=cmd, request_id=uuid.uuid4().hex, source="voice_host", sent_at=time.time(),
                expected_state=node._state, expected_episode_index=node._episode_index, profile_id=node._profile_id, **extra)


def send(node, row):
    node._on_control(String(data=json.dumps(row)))
    return node.acks[-1]


def test_ack_confirmed_only_after_start_save_and_duplicate_cannot_create_second_episode(collector):
    start = command(collector, "start")
    assert send(collector, start)["success"]
    index = collector._episode_index
    assert send(collector, start)["success"]
    assert collector._episode_index == index
    stop = command(collector, "stop")
    assert send(collector, stop)["success"]
    assert collector._state == "IDLE"
    assert collector.acks[-1]["message"] == "本段已保存"


def test_changed_episode_or_expired_request_cannot_discard_new_episode(collector):
    send(collector, command(collector, "start"))
    old = command(collector, "discard")
    old["expected_episode_index"] += 1
    assert not send(collector, old)["success"]
    assert collector._state == "RECORDING"
    expired = command(collector, "discard")
    expired["sent_at"] -= 3
    assert not send(collector, expired)["success"]
    assert collector._state == "RECORDING"


def test_save_exception_is_failure_not_false_success(collector):
    send(collector, command(collector, "start"))
    original = collector._apply
    collector._apply = lambda cmd: (_ for _ in ()).throw(OSError("disk failed"))
    try:
        result = send(collector, command(collector, "stop"))
        assert not result["success"] and "disk failed" in result["message"]
    finally:
        collector._apply = original


def test_legacy_string_commands_still_work(collector):
    collector._on_control(String(data="start"))
    assert collector._state == "RECORDING"
    collector._on_control(String(data="stop"))
    assert collector._state == "IDLE"
