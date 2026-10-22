"""Nero HOME must route to the profile coordinator, never legacy Astral HOME."""
import asyncio

from morla_web_manager import web_server


def test_home_uses_unified_coordinator_without_legacy_enable_or_publish(monkeypatch):
    calls = []

    class Coordinator:
        def control_available(self):
            return True

        def call_control_service(self, operation):
            calls.append(operation)
            return True, 'accepted operation_id=home_test'

    monkeypatch.setattr(web_server, 'get_node', lambda: Coordinator())
    result = asyncio.run(web_server.teleop_home())
    assert result.ok
    assert result.message == 'accepted operation_id=home_test'
    assert calls == ['home']
