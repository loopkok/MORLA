"""The Web manager uses the active profile coordinator and reports acceptance."""

from __future__ import annotations

import asyncio

import pytest
from fastapi import HTTPException

from morla_web_manager import web_server


class _Coordinator:
    def __init__(self, available=True, response=(True, "accepted operation_id=abc"), seen=True):
        self.available = available
        self.response = response
        self.seen = seen
        self.calls = []

    def control_available(self):
        return self.available

    def control_state_seen(self):
        return self.seen

    def call_control_service(self, operation):
        self.calls.append(operation)
        return self.response


def test_control_endpoint_routes_to_profile_coordinator(monkeypatch):
    node = _Coordinator(response=(True, "accepted operation_id=abc"))
    monkeypatch.setattr(web_server, "get_node", lambda: node)

    result = asyncio.run(web_server.control_operation("workpos"))

    assert result.ok is True
    assert result.message.endswith("operation_id=abc")
    assert node.calls == ["workpos"]


def test_control_endpoint_does_not_report_success_without_coordinator(monkeypatch):
    monkeypatch.setattr(web_server, "get_node", lambda: _Coordinator(available=False))

    with pytest.raises(HTTPException) as exc:
        asyncio.run(web_server.control_operation("estop"))
    assert exc.value.status_code == 503


def test_legacy_endpoint_bridge_falls_back_only_without_fresh_coordinator(monkeypatch):
    monkeypatch.setattr(web_server, "get_node", lambda: _Coordinator(available=False, seen=False))
    result = asyncio.run(web_server._try_coordinated_control("position"))
    assert result is None


def test_stale_profile_coordinator_never_falls_back_to_legacy_control(monkeypatch):
    monkeypatch.setattr(web_server, "get_node", lambda: _Coordinator(available=False, seen=True))
    with pytest.raises(HTTPException) as exc:
        asyncio.run(web_server._try_coordinated_control("position"))
    assert exc.value.status_code == 503
    assert "refusing legacy control fallback" in exc.value.detail
