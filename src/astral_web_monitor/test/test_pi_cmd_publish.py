"""Policy command publisher discovery guard (ROS object mocked, no graph needed)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from astral_web_monitor.monitor_node import MonitorNode  # noqa: E402


class _FakePublisher:
    def __init__(self, counts: list[int]) -> None:
        self._counts = iter(counts)
        self._last_count = 0
        self.messages = []

    def get_subscription_count(self) -> int:
        self._last_count = next(self._counts, self._last_count)
        return self._last_count

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _node_with_publisher(pub: _FakePublisher) -> MonitorNode:
    node = object.__new__(MonitorNode)
    node._pub_pi_cmd = pub
    return node


def test_policy_command_waits_for_subscriber_then_publishes_once() -> None:
    pub = _FakePublisher([0, 0, 1])
    node = _node_with_publisher(pub)

    assert node.publish_pi_cmd("policy", timeout_s=0.1, poll_s=0.001)
    assert [msg.data for msg in pub.messages] == ["policy"]


def test_policy_command_is_not_falsely_reported_sent_without_subscriber() -> None:
    pub = _FakePublisher([0])
    node = _node_with_publisher(pub)

    assert not node.publish_pi_cmd("policy", timeout_s=0.0)
    assert not pub.messages
