"""Real MORLA HTTP/session/BindingEngine/recorder; synthetic device and input."""
from importlib import metadata
import json
from pathlib import Path
import time

import numpy as np
import pytest
from fastapi.testclient import TestClient
from morla_core.plugins.registry import default_catalog
from morla_input.adapters.normalized import InputStream
from morla_web_manager.composable_api import create_app

ROOT = Path(__file__).parents[1]


@pytest.fixture
def client(tmp_path, monkeypatch):
    # Source-only tests also exercise the standard installed entry-point ABI.
    original = list(metadata.entry_points(group='morla.adapters')) + list(metadata.entry_points(group='morla.kinematics')) + list(metadata.entry_points(group='morla.controllers'))
    entries = metadata.EntryPoints([
        *(entry for entry in original if 'morla.fairino.fr3' not in entry.name and not entry.name.startswith('morla.fr3.')),
        metadata.EntryPoint(name='morla.fairino.fr3:fake', value='morla_fr3_driver.adapter:create_adapter', group='morla.adapters'),
        metadata.EntryPoint(name='morla.fairino.fr3', value='morla_fr3_driver.kinematics:create_kinematics', group='morla.kinematics'),
        metadata.EntryPoint(name='morla.fr3.cartesian_delta',value='morla_fr3_driver.vr_controller:create_controller',group='morla.controllers'),
        metadata.EntryPoint(name='morla.fr3.cartesian_jog',value='morla_fr3_driver.vr_controller:create_jog_controller',group='morla.controllers'),
    ])
    monkeypatch.setattr(metadata, 'entry_points', lambda **kwargs: entries.select(**kwargs))
    app = create_app(tmp_path / 'store', resource_root=tmp_path / 'locks', diagnostic_root=tmp_path / 'diagnostics')
    catalog = default_catalog()
    if not any(d['plugin_id'] == 'morla.fairino.fr3' for d in catalog.list()):
        catalog.register(json.loads((ROOT / 'descriptor.json').read_text()))
    for name in ('controller.json','jog_controller.json'):
        d=json.loads((ROOT/name).read_text())
        if not any(x['plugin_id']==d['plugin_id'] for x in catalog.list()):catalog.register(d)
    app.state.composable.catalog = catalog
    with TestClient(app) as value:
        yield value


def test_web_fr3_catalog_template_start_quest_ik_record_stop(client):
    components = client.get('/api/v2/components').json()['components']
    descriptor = next(d for d in components if d['plugin_id'] == 'morla.fairino.fr3')
    assert not descriptor['backend_variants']['sim']['available']
    response = client.get('/api/v2/templates/fr3-fake')
    assert response.status_code == 200, response.text
    assembly = response.json()
    saved = client.post('/api/v2/assemblies', json={'assembly': assembly, 'expected_revision': 0})
    assert saved.status_code == 200, saved.text
    resolved = saved.json()['resolved']
    assert resolved['feature_contract']['action_dim'] == 6
    started = client.post('/api/v2/sessions', json={
        'assembly_id': assembly['assembly_id'], 'revision': 1,
        'assembly_hash': resolved['assembly_hash'], 'request_id': 'start'})
    assert started.status_code == 200, started.text
    sid = started.json()['session_id']
    svc = client.app.state.composable
    adapter = svc.session.adapters['arm']
    assert type(adapter).__name__ == 'FR3FakeAdapter'
    assert started.json()['status'] == 'READY_DISARMED' and not adapter.active
    source = InputStream('quest')
    source.reconnect()
    sequence = 0

    def send(x=0., hold=True):
        nonlocal sequence
        sequence += 1
        frame = source.accept({'right_pose': {'position': [x, 0, 0], 'quaternion': [0, 0, 0, 1]},
                               'right_hold': hold}, sequence, epoch=source.epoch,
                              input_basis_id='unity_left_handed_v1')
        return svc.ingest_frame(sid, frame)

    assert not send()['accepted']
    for operation in ['prepare', 'activate']:
        reply = client.post(f'/api/v2/sessions/{sid}/operations',
                            json={'operation': operation, 'request_id': operation})
        assert reply.status_code == 200, reply.text
        assert reply.json()['status'] == 'SUCCEEDED', reply.text
    send()
    # Async IK returns latest completed results; advance the real input clock.
    before = adapter.values[:]
    recording = client.post(f'/api/v2/sessions/{sid}/recordings', json={'task': 'fr3_offline_acceptance'})
    assert recording.status_code == 200, recording.text
    for k in range(1, 8):
        time.sleep(.025)
        result = send(k * .0005)
        if not result['accepted']:
            assert result['reason']=='release_or_reanchor_required',result
        svc.tick()
    deadline=time.monotonic()+1
    while np.linalg.norm(np.asarray(adapter.values)-before)<1e-4 and time.monotonic()<deadline:
        time.sleep(.02);send(.006);svc.tick()
    assert np.linalg.norm(np.asarray(adapter.values) - before) > 1e-4
    held = adapter.values[:]
    assert not send(.02, hold=False)['accepted']
    assert adapter.values == held
    stop_record = client.post(f'/api/v2/sessions/{sid}/recordings/stop', json={})
    assert stop_record.status_code == 200, stop_record.text
    episode = Path(recording.json()['episode_dir'])
    meta = json.loads((episode / 'meta.json').read_text())
    assert meta['schema']['schema_version'] == 3
    import h5py
    with h5py.File(episode / 'robot_data.h5') as h5:
        datasets = []
        h5.visititems(lambda name, obj: datasets.append((name, obj.shape)) if isinstance(obj, h5py.Dataset) else None)
        assert any(len(shape) == 2 and shape[0] > 0 and shape[1] == 6 for _, shape in datasets), datasets
    response = client.post(f'/api/v2/sessions/{sid}/operations', json={'operation': 'stop', 'request_id': 'stop'})
    assert response.status_code == 200, response.text
    assert not adapter.active and not adapter.configured


def test_real_placeholder_rejected_without_opening_sdk(client):
    # The example can compile, but the real adapter must reject unknown IPs
    # before starting a process. Exercise the actual factory directly.
    from morla_core.composition import compile_system, load_document
    from morla_fr3_driver.adapter import create_adapter
    resolved = compile_system(load_document(ROOT / 'config/assembly.fake.yaml'),
                              load_document(ROOT / 'config/deployment.real.example.yaml'),
                              registry=client.app.state.composable.catalog)
    i = next(i for i in resolved['instances'] if i['instance_id'] == 'arm')
    adapter = create_adapter(i, {'clock': time.monotonic_ns})
    with pytest.raises(ValueError, match='IPv4'):
        adapter.configure({})
    assert adapter.transport is None and adapter.thread is None
