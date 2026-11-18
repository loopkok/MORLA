from pathlib import Path
from copy import deepcopy
import shutil
import subprocess
import sys

import h5py
import numpy as np
from fastapi.testclient import TestClient

from morla_input.adapters.normalized import InputFrame
from morla_teleop.engine import BindingEngine
from morla_teleop.kinematics import solver_for
from morla_web_manager.composable_api import create_app


def test_wheel_install_to_web_recording(tmp_path, monkeypatch):
    repository = Path(__file__).resolve().parents[4]
    source = tmp_path / 'source'
    shutil.copytree(repository / 'examples/external_components', source,
                    ignore=shutil.ignore_patterns('build', 'dist', '*.egg-info', '__pycache__'))
    subprocess.run([sys.executable, '-m', 'pip', 'wheel', '--no-build-isolation',
                    '--no-deps', '--no-index', '-w', str(tmp_path), str(source)], check=True)
    wheel = next(tmp_path.glob('morla_example_components-*.whl'))
    installed = tmp_path / 'installed'
    subprocess.run([sys.executable, '-m', 'pip', 'install', '--no-deps', '--no-index',
                    '--target', str(installed), str(wheel)], check=True)
    monkeypatch.syspath_prepend(str(installed))
    assert 'morla_example_components' not in sys.modules

    marker = installed / 'morla_example_components' / 'morla_plugin.json'
    malformed = tmp_path / 'broken'
    pkg = malformed / 'broken_package'
    pkg.mkdir(parents=True)
    (pkg / marker.name).write_text('{"version":9}')
    metadata = malformed / 'broken-1.0.dist-info'
    metadata.mkdir()
    (metadata / 'METADATA').write_text('Name: broken\nVersion: 1.0\n')
    (metadata / 'RECORD').write_text('broken_package/morla_plugin.json,,\n')
    monkeypatch.syspath_prepend(str(malformed))

    app = create_app(tmp_path / 'store', resource_root=tmp_path / 'locks', mount_web=False,
                     diagnostic_root=tmp_path / 'logs')
    with TestClient(app) as client:
        catalog = client.get('/api/v2/components').json()
        assert sum(d['plugin_id'].startswith('example.external.') for d in catalog['components']) == 4
        assert any(item['origin'] == 'wheel:broken' for item in catalog['diagnostics'])
        assert any(row['id'] == 'external-components' for row in client.get('/api/v2/templates').json())
        template = client.get('/api/v2/templates/external-components').json()
        invalid = deepcopy(template)
        invalid['instances']['arm']['parameters'] = {'rate_hz': 0}
        rejected = client.post('/api/v2/assemblies/compile', json={'assembly': invalid})
        assert rejected.status_code == 422
        assert '/instances/arm/parameters/rate_hz' in rejected.text
        template['instances']['input'] = {'plugin': 'morla.input.quest'}
        template['bindings'] = [{'id': 'pinch', 'source': 'input.scalar', 'target': 'gripper.width_goal',
                                 'controller': 'morla.scalar_map',
                                 'parameters': {'input_range': [0, 1], 'output_range': [.05, 0]}}]
        preflight = client.post('/api/v2/deployments/preflight', json={'assembly': template})
        assert preflight.status_code == 200, preflight.text
        assert preflight.json()['preflight']['ready']
        assert 'morla_example_components' not in sys.modules

        saved = client.post('/api/v2/assemblies', json={'assembly': template})
        assert saved.status_code == 200, saved.text
        resolved = saved.json()['resolved']
        assert {item['block_id'] for item in resolved['capture_schema']['state_blocks']} == {'arm.angle', 'gripper.width'}
        assert resolved['capture_schema']['action_blocks'] == []
        started = client.post('/api/v2/sessions', json={
            'assembly_id': template['assembly_id'], 'revision': 1, 'request_id': 'start',
            'assembly_hash': resolved['assembly_hash']})
        assert started.status_code == 200, started.text
        assert 'morla_example_components' in sys.modules
        service = app.state.composable
        session = service.session
        sid = started.json()['session_id']
        base = f'/api/v2/sessions/{sid}'
        activation = client.post(base+'/operations', json={'operation':'activate', 'request_id':'activate'})
        assert activation.json()['status'] == 'SUCCEEDED', activation.text
        engine = BindingEngine(session)
        try:
            frame = InputFrame('input', 'fake-epoch', 1, session.clock(), 'controller', {'scalar': .5})
            targets = engine.targets(frame)
            assert targets[0]['values'] == [.025]
            assert targets[0]['units'] == ['m']
            permit = session.acquire('human_mobile')
            assert session.submit({**permit, 'commands':targets})['accepted']
            assert session.adapters['gripper'].read(session.clock())['ports']['width']['values'] == [.025]
        finally:
            engine.close()
        arm = session.instances['arm']
        solver = solver_for(arm)
        solver.sync_state([.25])
        np.testing.assert_array_equal(solver.solve(np.eye(4)), [.25])
        from morla_core.resource_locator import resolve_plugin_resource
        assert resolve_plugin_resource(arm['geometry']['simulation']['package'], 'arm.xml').is_file()

        for port in ('wide', 'detail'):
            response = client.get(base+'/cameras/camera/frame', params={'port': port})
            assert response.headers['content-type'] == 'image/jpeg'
            assert response.content.startswith(b'\xff\xd8')
        assert client.get(base+'/cameras/camera/frame').status_code == 200
        assert client.get(base+'/cameras/camera/frame', params={'port': 'unavailable'}).status_code == 404

        recorded = client.post(base+'/recordings', json={'task': 'external sensors'})
        assert recorded.status_code == 200, recorded.text
        result = client.post(base+'/recordings/stop', json={})
        assert result.json()['status'] == 'complete', result.text
        episode = Path(recorded.json()['episode_dir'])
        with h5py.File(episode/'robot_data.h5') as recording:
            group = recording['sensors/sensor__telemetry']
            assert group.attrs['codec'] == 'named_fields_v1'
            assert group['pressure'].attrs['unit'] == 'Pa'
            assert group['pressure'].dtype == np.float32
            assert group['temperature'].dtype == np.int16
            np.testing.assert_array_equal(group['pressure'][0], [[1, 2], [3, 4]])
            np.testing.assert_array_equal(group['temperature'][0], [21, 22])
            assert group['timestamps'][0] > 0
            assert bool(group['valid'][0])
        with h5py.File(episode/'camera_data.h5') as recording:
            for label in ('camera__wide', 'camera__detail'):
                assert len(recording[label]['images']) > 0
    sys.modules.pop('morla_example_components', None)
