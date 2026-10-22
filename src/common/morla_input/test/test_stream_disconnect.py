"""Exercise the receiver's real disconnect/timer wiring without network sockets."""

import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip("rclpy")
from morla_input.quest3_udp_mocap import Quest3UDPMocap as Receiver
from morla_input.stream_health import StreamHealth


def receiver():
    node = SimpleNamespace(
        _input_lock=threading.RLock(), _iobt_lock=threading.Lock(),
        _stream_health=StreamHealth(), is_running=True,
        landmark_cache_left=np.ones((21, 3)), landmark_cache_right=np.ones((21, 3)),
        _packet_guard=SimpleNamespace(make_sync_challenge=lambda *a, **kw: "@q3sync test\n"),
        get_logger=lambda: Mock(), epochs=[], disarms=[],
    )
    node.input_epoch_pub = SimpleNamespace(publish=node.epochs.append)
    node._input_disarm_pub = SimpleNamespace(publish=node.disarms.append)
    node._signal_input_epoch = lambda *a, **kw: Receiver._signal_input_epoch(node, *a, **kw)
    node._on_peer_disconnected = lambda peer: Receiver._on_peer_disconnected(node, peer)
    return node


class ClosedConnection:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def sendall(self, data):
        pass

    def recv(self, size):
        return b""


def test_tcp_disconnect_immediately_disarms_and_clears_caches(monkeypatch):
    monkeypatch.setattr("morla_input.quest3_udp_mocap.rclpy.ok", lambda: True)
    node = receiver()
    node._stream_health.observe("Left wrist:,0", "tcp:127.0.0.1:1000", time.monotonic())
    Receiver._tcp_connection_handler(node, ClosedConnection(), ("127.0.0.1", 1000))
    assert len(node.epochs) == 1 and node.disarms[0].data is True
    assert not node.landmark_cache_left.any() and not node.landmark_cache_right.any()


def test_timeout_is_coalesced_until_new_input_arrives():
    node = receiver()
    node._stream_health.observe("Left wrist:,0", "hands", time.monotonic() - 0.6)
    Receiver._check_input_timeout(node)
    Receiver._check_input_timeout(node)
    assert len(node.epochs) == 1 and len(node.disarms) == 1


def test_obsolete_socket_close_does_not_stop_replacement():
    node = receiver()
    now = time.monotonic()
    node._stream_health.observe("Left wrist:,0", "old", now)
    node._stream_health.observe("Left wrist:,0", "new", now)
    node._on_peer_disconnected("old")
    assert not node.epochs and not node.disarms


def test_replacement_invalidates_anchor_before_publishing_fresh_payload():
    node = receiver()
    now = time.monotonic()
    payload = "Left wrist:,0"
    node._stream_health.observe(payload, "old", now)
    node._packet_guard.accept = lambda *a, **kw: (payload, time.time())
    node._packet_guard.last_session_change = None
    order = []
    signal = node._signal_input_epoch
    node._signal_input_epoch = lambda *a, **kw: (order.append("epoch"), signal(*a, **kw))
    node.process_line = lambda *a: order.append("payload")
    Receiver._process_network_line_locked(node, "packet", time.time(), now, "new")
    assert order == ["epoch", "payload"]
