import time
import pytest
mujoco = pytest.importorskip('mujoco')
from morla_core.composition import compile_system
from morla_runtime import RuntimeSession


def resolved():
    return compile_system({'assembly_version': 1, 'assembly_id': 'runtime_scene', 'revision': 1,
                           'instances': {'a': {'plugin': 'morla.nero.arm', 'backend': 'sim'},
                                         'b': {'plugin': 'morla.astral.arm', 'backend': 'sim', 'parameters': {'side': 'right'}}},
                           'mounts': [{'child': 'a.base', 'parent': 'world', 'transform': [0, .6, 0, 0, 0, 0, 1]},
                                      {'child': 'b.base', 'parent': 'world', 'transform': [0, -.6, 0, 0, 0, 0, 1]}]})


def wait_operation(runtime, request):
    for _ in range(200):
        result = runtime.operation(request)
        if result['status'] != 'RUNNING': return result
        time.sleep(.005)
    raise AssertionError('operation timeout')


def test_single_physics_scene_native_joint_maps_and_runtime_commands(tmp_path):
    system = resolved()
    runtime = RuntimeSession(system, resource_root=tmp_path)
    try:
        state = runtime.start()
        assert state['status'] == 'READY_DISARMED'
        a, b = runtime.adapters['a'], runtime.adapters['b']
        assert a.session is b.session
        scene = a.session
        assert scene.model.nq == 14 and scene.model.nu == 14
        assert len(scene.indices['a']['ctrl']) == len(scene.indices['b']['ctrl']) == 7
        runtime.activate('activate')
        assert wait_operation(runtime, 'activate')['status'] == 'SUCCEEDED'
        permit = runtime.acquire('replay', ['a', 'b'])
        items = []
        for row in system['instances']:
            values = [max(j['lower'] + .01, min(.05, j['upper'] - .01)) for j in row['joints']]
            items.append({'component_id': row['instance_id'], 'names': [j['name'] for j in row['joints']],
                          'values': values, 'units': [j['unit'] for j in row['joints']], 'mode': 'position'})
        receipt = runtime.submit({**permit, 'sequence': 1, 'issued_ns': runtime.clock(), 'ttl_ns': 100_000_000, 'commands': items})
        assert receipt['accepted'], receipt
        time.sleep(.03)
        for item in items:
            assert scene.data.ctrl[scene.indices[item['component_id']]['ctrl']].tolist() == pytest.approx(item['values'])
        current = runtime.state()
        assert current['components']['a']['source'] == 'simulation'
        assert current['components']['a']['synthetic'] and not current['components']['a']['measured']
        assert current['components']['a']['simulation_time_s'] > 0
        runtime.revoke('replay')
        assert not scene.active
        assert runtime.status == 'READY_DISARMED'
    finally:
        runtime.stop()
    assert not scene.thread.is_alive()


def test_missing_resolved_mount_transform_fails_without_starting_sim(tmp_path):
    system = resolved()
    source = system['source_assembly']
    source['mounts'][0].pop('transform')
    source['mounts'][0]['calibration'] = 'unresolved'
    system = compile_system(source)
    runtime = RuntimeSession(system, resource_root=tmp_path)
    with pytest.raises(ValueError, match='resolved transform'):
        runtime.start()
    assert runtime.status == 'FAULTED'


def test_sim_camera_renderer_failure_is_explicit_and_never_fake(tmp_path, monkeypatch):
    monkeypatch.setattr(mujoco, 'Renderer', lambda *a, **kw: (_ for _ in ()).throw(RuntimeError('no graphics context')))
    system = compile_system({'assembly_version': 1, 'assembly_id': 'sim_camera', 'revision': 1,
                             'instances': {'cam': {'plugin': 'morla.camera', 'backend': 'sim'}},
                             'mounts': [{'child': 'cam.optical', 'parent': 'world', 'transform': [0, 0, 2, 0, 0, 0, 1]}]})
    runtime = RuntimeSession(system, resource_root=tmp_path)
    runtime.start()
    adapter = runtime.adapters['cam']
    for _ in range(200):
        if adapter.session.render_error: break
        time.sleep(.005)
    assert 'no graphics context' in adapter.session.render_error
    assert not adapter.health(runtime.clock())['ready']
    assert adapter.read(runtime.clock()).get('ports') == {}
    assert runtime.tick()['status'] == 'READY_DISARMED'
    runtime.stop()
