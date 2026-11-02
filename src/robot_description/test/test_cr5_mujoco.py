"""CR5 sim model must agree with the exact URDF used by local IK."""
import importlib.util
from pathlib import Path
import time
import json
from morla_core.plugins.registry import default_catalog
import numpy as np
import pytest
mujoco = pytest.importorskip('mujoco')
from morla_core.composition import compile_system, load_document
from morla_runtime import RuntimeSession

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('cr5_scene', ROOT/'scripts/composable_scene.py')
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)


def catalog():
    result = default_catalog()
    installed = {(d['plugin_id'], d['plugin_version']) for d in result.list()}
    for path in [ROOT.parents[1]/"src/robots/dobot/morla_cr5_driver/descriptor.json", ROOT.parents[1]/"src/robots/dobot/morla_cr5_driver/controller.json"]:
        descriptor = json.loads(path.read_text())
        # A built workspace already discovers these manifests.
        if (descriptor['plugin_id'], descriptor['plugin_version']) not in installed:
            result.register(descriptor)
    return result

def system():
    return compile_system({'assembly_version': 1, 'assembly_id': 'cr5_sim_test', 'revision': 1,
        'instances': {'arm': {'plugin': 'morla.dobot.cr5', 'backend': 'sim',
                              'parameters': {'initial_positions': [0,-.5,1,-.5,1,0]}}}}, registry=catalog())


def test_generated_model_is_reproducible():
    spec = importlib.util.spec_from_file_location('cr5_generator', ROOT/'scripts/make_cr5_mjcf.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    assert module.generate() == (ROOT/'mjcf/cr5/cr5.xml').read_text()


def test_cr5_mujoco_fk_matches_ik_urdf_across_workspace():
    pytest.importorskip('pinocchio')
    from morla_cr5_driver.urdf_ik import IKSolver
    solver = IKSolver()
    artifact = scene.build_scene(system())
    model = mujoco.MjModel.from_xml_string(artifact['xml']); data = mujoco.MjData(model)
    assert model.nq == model.nu == 6
    assert artifact['joint_map']['arm'] == {f'joint{i}':f'arm__joint{i}' for i in range(1,7)}
    rng = np.random.default_rng(5)
    for q in [np.zeros(6), *rng.uniform(-1.5,1.5,(30,6))]:
        data.qpos[:] = q; mujoco.mj_forward(model,data)
        tip = model.body('arm__Link6').id
        expected = solver.fk(q)
        np.testing.assert_allclose(data.xpos[tip], expected[:3,3], atol=1e-10)
        np.testing.assert_allclose(data.xmat[tip].reshape(3,3), expected[:3,:3], atol=1e-10)


def test_sim_feedback_tracks_command_and_holds_without_hardware(tmp_path):
    runtime = RuntimeSession(system(), resource_root=tmp_path, registry=catalog())
    try:
        runtime.start()
        adapter = runtime.adapters['arm']; session = adapter.session
        assert adapter.read(runtime.clock())['values'] == pytest.approx([0,-.5,1,-.5,1,0], abs=.01)
        adapter.activate()
        target = [0.1,-.45,.95,-.45,1.05,.05]
        adapter.submit({'values':target})
        deadline = time.monotonic() + 5
        while adapter.read(runtime.clock())['simulation_time_s'] < 1.5:
            assert time.monotonic() < deadline
            time.sleep(.01)
        values = adapter.read(runtime.clock())
        assert values['source'] == 'simulation' and not values['measured']
        assert values['values'] == pytest.approx(target, abs=.06)
        adapter.stop()
        assert 'arm' not in session.active
    finally:
        runtime.stop()
    assert not session.thread.is_alive()


def test_cr5_sim_web_template_compiles():
    path = ROOT.parents[1]/'src/robots/dobot/morla_cr5_driver/config/quest.sim.yaml'
    resolved = compile_system(load_document(path), registry=catalog())
    artifact = scene.build_scene(resolved)
    model = mujoco.MjModel.from_xml_string(artifact['xml'])
    assert model.nq == 6 and model.ncam == 1


def test_sim_uses_existing_local_ik_controller():
    pytest.importorskip('pinocchio')
    from morla_cr5_driver.vr_controller import create_controller
    row = system()['instances'][0]
    controller = create_controller({'instance':row})
    try:
        q = np.array([0,-.5,1,-.5,1,0])
        solver = controller.controller.solver
        solver.sync_state(q)
        target = solver.fk(q)
        target[0,3] += .001
        result = solver.solve(target)
        np.testing.assert_allclose(solver.fk(result)[:3,3],target[:3,3],atol=.0002)
    finally:
        controller.close()
