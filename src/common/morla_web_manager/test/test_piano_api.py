"""HTTP-level mutual exclusion and malformed-input regressions."""
from __future__ import annotations

import asyncio
from pathlib import Path
from types import SimpleNamespace

import httpx
import pytest

from morla_web_manager import web_server as web
from morla_web_manager.piano_manager import PianoManager
from morla_web_manager.piano_store import PianoStore


@pytest.fixture
def api(tmp_path, monkeypatch):
    project = Path(__import__('os').environ.get('MORLA_PIANO_PROJECT', str(Path.home() / 'Desktop/MORLA-piano')))
    store = PianoStore(project, tmp_path)
    manager = PianoManager(store)
    monkeypatch.setattr(web, '_piano_store', store)
    monkeypatch.setattr(web, '_piano_mgr', manager)
    monkeypatch.setattr(web, 'get_node', lambda: None)
    monkeypatch.setattr(web, '_motion_mutation_lock', asyncio.Lock())
    for name in ('_launch_mgr', '_collect_mgr', '_policy_mgr'):
        monkeypatch.setattr(web, name, SimpleNamespace(state='stopped'))
    yield store, manager
    manager.close()


async def request(method, path, **kwargs):
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=web.app), base_url='http://testserver') as client:
        return await client.request(method, path, **kwargs)


def test_readonly_catalog_and_plan_do_not_acquire_robot(api):
    store, manager = api
    scores = asyncio.run(request('GET', '/api/v1/piano/scores'))
    assert scores.status_code == 200
    score_id = scores.json()['data'][0]['id']
    result = asyncio.run(request('POST', '/api/v1/piano/plan', json={'score_id': score_id}))
    assert result.status_code == 200
    assert result.json()['data']['notes'] > 0
    assert not manager.active


@pytest.mark.parametrize('path', [
    '/api/v1/start', '/api/v1/restart', '/api/v1/resume', '/api/v1/teleop/start',
    '/api/v1/teleop/home', '/api/v1/robot/position', '/api/v1/control/enable',
    '/api/v1/profile/launch', '/api/v1/profile/select', '/api/v1/collect/launch/start',
    '/api/v1/collect/control', '/api/v1/infer/launch/start', '/api/v1/infer/cmd',
])
def test_original_mutation_endpoints_are_blocked_while_piano_owns_robot(api, path):
    _, manager = api
    manager._active = True
    try:
        response = asyncio.run(request('POST', path, json={}))
        assert response.status_code == 409
        assert '钢琴会话' in response.json()['detail']
    finally:
        manager._active = False


def test_unrelated_data_routes_are_not_blocked(api):
    _, manager = api
    manager._active = True
    try:
        response = asyncio.run(request('POST', '/api/v1/data/jobs', json={}))
        assert response.status_code == 422  # Existing schema error, not piano arbitration.
    finally:
        manager._active = False


def test_top_level_estop_reaches_piano_even_without_old_coordinator(api, monkeypatch):
    _, manager = api
    called = []
    manager._active = True
    monkeypatch.setattr(manager, 'stop', lambda **kwargs: called.append(kwargs))
    try:
        response = asyncio.run(request('POST', '/api/v1/robot/estop'))
        assert response.status_code == 200
        assert called == [{'emergency': True}]
    finally:
        manager._active = False


def test_direct_config_save_cannot_bypass_active_session(api):
    store, manager = api
    manager._active = True
    try:
        response = asyncio.run(request('POST', '/api/v1/piano/config', json=store.config()))
        assert response.status_code == 409
    finally:
        manager._active = False


def test_cross_origin_control_is_denied(api):
    response = asyncio.run(request('POST', '/api/v1/piano/stop', json={}, headers={'Origin': 'http://evil.example'}))
    assert response.status_code == 403


def test_config_version_and_malformed_shape_return_client_errors(api):
    store, _ = api
    response = asyncio.run(request('POST', '/api/v1/piano/config', json={'revision': 'old', 'config': {}}))
    assert response.status_code == 409
    response = asyncio.run(request('POST', '/api/v1/piano/config/validate', json={'config': {'version': 1}}))
    assert response.status_code == 422


def test_cli_coordinator_and_bare_driver_block_new_hardware_session(api, monkeypatch):
    class Node:
        def snapshot(self):
            return {'control': {'stale': False, 'node_count': 1}}
        def robot_command_publishers(self):
            return {}
        def count_publishers(self, topic):
            return 1 if topic.endswith('/nero_robot_driver/status') else 0
    monkeypatch.setattr(web, 'get_node', lambda: Node())
    response = asyncio.run(request('POST', '/api/v1/piano/session', json={'mode': 'hardware'}))
    assert response.status_code == 409
    assert '现有驱动' in response.json()['detail']


def test_ros_session_fails_closed_without_graph_monitor(api):
    response = asyncio.run(request('POST', '/api/v1/piano/session', json={'mode': 'ros_fake'}))
    assert response.status_code == 409
    assert '核验' in response.json()['detail']


def test_departed_piano_lease_cache_does_not_block_reconnect(api, monkeypatch):
    class Node:
        def snapshot(self):
            return {'infer': {'stale': False, 'node_count': 1, 'activity': None}}
        def robot_command_publishers(self):
            return {}
        def count_publishers(self, topic):
            return 0
    monkeypatch.setattr(web, 'get_node', lambda: Node())
    assert web._piano_blockers('ros_fake') == []
