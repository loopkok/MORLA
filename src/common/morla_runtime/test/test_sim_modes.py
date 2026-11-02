"""Shared scene modes apply equally to CR5, Nero and Astral."""
import json
from pathlib import Path
import time
import pytest
import numpy as np
pytest.importorskip('mujoco')
from morla_core.composition import compile_system, load_document
from morla_core.plugins.registry import default_catalog
from morla_runtime import RuntimeSession

ROOT = Path(__file__).resolve().parents[4]


def catalog():
    result = default_catalog()
    for filename in ('descriptor.json','controller.json'):
        descriptor = json.loads((ROOT/'src/robots/dobot/morla_cr5_driver'/filename).read_text())
        if not any(d['plugin_id'] == descriptor['plugin_id'] for d in result.list()):
            result.register(descriptor)
    return result


def wait_for(predicate):
    deadline = time.monotonic()+3
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.005)


@pytest.mark.parametrize('plugin,dim', [('morla.dobot.cr5',6),('morla.nero.arm',7),('morla.astral.arm',7)])
def test_kinematic_exact_target_and_live_mode_change(plugin, dim, tmp_path):
    registry = catalog()
    resolved = compile_system({'assembly_version':1,'assembly_id':'modes','revision':1,
        'instances':{'arm':{'plugin':plugin,'backend':'sim'}},
        'metadata':{'simulation':{'mode':'kinematic','display':'web'}}},registry=registry)
    runtime = RuntimeSession(resolved,resource_root=tmp_path,registry=registry)
    try:
        runtime.start(); adapter=runtime.adapters['arm']; scene=adapter.session
        adapter.activate(); target=[.05]*dim; adapter.submit({'values':target})
        wait_for(lambda: np.allclose(adapter.read(runtime.clock())['values'], target, atol=1e-12))
        assert adapter.read(runtime.clock())['ports']['velocity']['values'] == [0]*dim
        with pytest.raises(RuntimeError,match='DISARM'): scene.set_options(mode='physics')
        adapter.stop(); before=scene.data.qpos.copy()
        scene.set_options(mode='physics')
        assert scene.settings()['mode']=='physics'
        assert scene.data.ctrl[scene.indices['arm']['ctrl']].tolist() == pytest.approx(before)
        wait_for(lambda: np.linalg.norm(scene.data.qvel)>1e-5)
        scene.set_options(mode='kinematic')
        assert adapter.read(runtime.clock())['simulation_mode']=='kinematic'
        with pytest.raises(ValueError): scene.set_options(mode='fake')
    finally:
        runtime.stop()


@pytest.mark.parametrize('path', ['src/robots/dobot/cr5_mujoco_sim/config/composable.sim.yaml',
    'src/robots/nero/xnero_mujoco_sim/config/composable.sim.yaml',
    'src/robots/astral/astral_mujoco_sim/config/composable.sim.yaml'])
def test_robot_templates_compile_and_share_backend(path, tmp_path):
    registry=catalog(); resolved=compile_system(load_document(ROOT/path),registry=registry)
    from morla_runtime.sim_mujoco import _builder
    import mujoco
    model=mujoco.MjModel.from_xml_string(_builder()(resolved)['xml'])
    assert model.nq in (6,14) and model.ncam==1
    assert resolved['source_assembly']['metadata']['simulation']['mode']=='kinematic'


def test_display_selection_uses_single_scene_and_closes_viewer(tmp_path, monkeypatch):
    from morla_runtime import sim_viewer
    class Viewer:
        def __init__(self, scene): self.scene=scene; self.running=False
        def open(self): self.running=True
        def close(self): self.running=False
        def status(self): return {'status':'running' if self.running else 'closed'}
    monkeypatch.setattr(sim_viewer,'LocalViewer',Viewer)
    resolved=compile_system({'assembly_version':1,'assembly_id':'view','revision':1,
        'instances':{'arm':{'plugin':'morla.nero.arm','backend':'sim'}}})
    runtime=RuntimeSession(resolved,resource_root=tmp_path)
    try:
        runtime.start(); scene=runtime.adapters['arm'].session; original=scene.model
        assert scene.set_options(display='local')['viewer']['status']=='running'
        viewer=scene.viewer
        scene.set_options(display='both'); assert scene.viewer is viewer and scene.model is original
        scene.set_options(display='web'); assert not viewer.running and scene.viewer is None
    finally: runtime.stop()
