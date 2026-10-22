import json
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
from std_msgs.msg import String


def test_existing_teleop_controller_uses_normalized_bus_when_router_is_present():
    from morla_bringup.controller_start_gate import ControllerStartGate
    rclpy.init(args=["--ros-args", "-p", "use_control_coordinator:=true"])
    node = ControllerStartGate()
    rows = []
    node._operator_pub = SimpleNamespace(publish=lambda msg: rows.append(json.loads(msg.data)))
    node.count_publishers = lambda topic: 1
    try:
        assert node._request_control("start")
        assert rows[-1]["command"] == "teleop_start" and rows[-1]["source"] == "quest_controller"
        assert node._request_control("release")
        assert rows[-1]["command"] == "release"
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_existing_collect_controller_uses_same_bus_without_duplicate_legacy_publish():
    from morla_data_collect.vr_collect_control import VrCollectControl
    rclpy.init()
    node = VrCollectControl()
    rows, legacy = [], []
    node._operator_pub = SimpleNamespace(publish=lambda msg: rows.append(json.loads(msg.data)))
    node._ctrl_pub = SimpleNamespace(publish=legacy.append)
    node.count_publishers = lambda topic: 1
    try:
        node._send("start")
        assert rows[-1]["command"] == "record_start" and not legacy
        node._send("stop")
        assert rows[-1]["command"] == "record_stop" and not legacy
    finally:
        node.destroy_node(); rclpy.shutdown()
