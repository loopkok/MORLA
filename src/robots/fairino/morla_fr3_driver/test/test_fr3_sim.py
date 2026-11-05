"""Real MuJoCo model/physics through the same backend as the standalone entry."""
import importlib.util
import json
from pathlib import Path
import time
import numpy as np
import pytest
mujoco=pytest.importorskip('mujoco')
from morla_core.composition import compile_system, load_document
from morla_core.plugins.registry import default_catalog
from morla_runtime import RuntimeSession
from morla_fr3_driver.contract import INITIAL
from morla_fr3_driver.urdf_ik import IKSolver

ROOT=Path(__file__).resolve().parents[5]
DRIVER=Path(__file__).resolve().parents[1]
DESCRIPTION=DRIVER.parent/'fr3_description'


def module(path):
    spec=importlib.util.spec_from_file_location(path.stem,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


def catalog():
    result=default_catalog()
    for name in ('descriptor.json','controller.json','jog_controller.json'):
        descriptor=json.loads((DRIVER/name).read_text())
        if not any(d['plugin_id']==descriptor['plugin_id'] for d in result.list()):result.register(descriptor)
    return result


def system(mode='kinematic',parameters=None):
    return compile_system({'assembly_version':1,'assembly_id':'fr3_sim_test','revision':1,
        'instances':{'arm':{'plugin':'morla.fairino.fr3','backend':'sim','parameters':parameters or {}}},
        'metadata':{'simulation':{'mode':mode,'display':'web'}}},registry=catalog())


def builder():return module(ROOT/'src/robot_description/scripts/composable_scene.py').build_scene


def wait_for(predicate,seconds=4):
    until=time.monotonic()+seconds
    while not predicate():
        assert time.monotonic()<until
        time.sleep(.005)


def test_reproducible_native_assets_and_fk_including_flange_and_mount():
    import hashlib
    generator=module(DESCRIPTION/'scripts/make_fr3_mjcf.py')
    assert generator.generate()==(DESCRIPTION/'mjcf/fr3.xml').read_text()
    provenance=json.loads((DESCRIPTION/'SIMULATION.json').read_text())
    assert hashlib.sha256((DESCRIPTION/'mjcf/fr3.xml').read_bytes()).hexdigest()==provenance['mjcf_sha256']
    assert hashlib.sha256((DESCRIPTION/'scripts/make_fr3_mjcf.py').read_bytes()).hexdigest()==provenance['generator_sha256']
    resolved=system();resolved['mounts']=[{'child':'arm.base','parent':'world','transform':[.4,.2,.1,0,0,0,1]}]
    artifact=builder()(resolved);m=mujoco.MjModel.from_xml_string(artifact['xml']);d=mujoco.MjData(m)
    assert m.nq==m.nu==6
    assert artifact['joint_map']['arm']=={f'joint{i}':f'arm__j{i}' for i in range(1,7)}
    assert np.all(m.geom_contype[1:]==0) # Floor only; never advertise collision validation.
    solver=IKSolver();rng=np.random.default_rng(85)
    for q in [np.zeros(6),np.array(INITIAL),*rng.uniform(-1.4,1.4,(60,6))]:
        d.qpos[:]=q;mujoco.mj_forward(m,d);tip=m.body('arm__flange').id
        target=solver.fk(q)
        np.testing.assert_allclose(d.xpos[tip],target[:3,3]+[.4,.2,.1],atol=1e-10)
        np.testing.assert_allclose(d.xmat[tip].reshape(3,3),target[:3,:3],atol=1e-10)
    np.testing.assert_allclose(m.jnt_range[:,0],solver.lower_limits)
    np.testing.assert_allclose(m.jnt_range[:,1],solver.upper_limits)


def test_independent_and_public_templates_share_model_camera_and_options():
    standalone=load_document(DRIVER.parent/'fr3_mujoco_sim/config/composable.sim.yaml')
    public=load_document(DRIVER/'config/quest.sim.yaml');assert standalone==public
    resolved=compile_system(public,registry=catalog());a=builder()(resolved)
    m=mujoco.MjModel.from_xml_string(a['xml'])
    assert m.ncam==1 and m.nq==6
    assert public['metadata']['simulation']=={'mode':'kinematic','display':'web'}
    assert public['bindings'][0]['controller']=='morla.fr3.cartesian_delta'
    assert public['bindings'][0]['deadman']=='quest.right_hold'


@pytest.mark.parametrize('mode',['kinematic','physics'])
def test_runtime_tracks_independent_feedback_holds_and_cleans_up(mode,tmp_path):
    runtime=RuntimeSession(system(mode),registry=catalog(),resource_root=tmp_path)
    try:
        assert runtime.start()['status']=='READY_DISARMED'
        adapter=runtime.adapters['arm'];scene=adapter.session
        before=adapter.read(runtime.clock());adapter.activate()
        target=np.array(INITIAL)+[.03,.02,-.02,.02,.02,.01]
        adapter.submit({'values':target.tolist()})
        if mode=='kinematic':
            wait_for(lambda: np.allclose(adapter.read(runtime.clock())['values'],target,atol=1e-12))
            assert adapter.read(runtime.clock())['ports']['velocity']['values']==[0]*6
        else:
            wait_for(lambda: adapter.read(runtime.clock())['simulation_time_s']>before['simulation_time_s']+1.2)
            np.testing.assert_allclose(adapter.read(runtime.clock())['values'],target,atol=.04)
            assert not np.array_equal(adapter.read(runtime.clock())['values'],target)
        health=adapter.read(runtime.clock())
        assert health['source']=='simulation' and health['synthetic'] and not health['measured']
        assert health['sample_ns']>before['sample_ns'] and scene.data.time>0
        with pytest.raises(RuntimeError,match='DISARM'):scene.set_options(mode='physics' if mode=='kinematic' else 'kinematic')
        ctrl=scene.data.ctrl.copy();adapter.stop()
        np.testing.assert_array_equal(scene.data.ctrl,ctrl)
        with pytest.raises(RuntimeError,match='DISARMED'):adapter.submit({'values':target.tolist()})
        scene.set_options(mode='physics' if mode=='kinematic' else 'kinematic')
        assert scene.pending=={} and not scene.active
    finally:runtime.stop()
    assert not scene.thread.is_alive()


def test_soft_limits_invalid_targets_and_mapping_fail_explicitly(tmp_path):
    lo=(np.array(INITIAL)-.05).tolist();hi=(np.array(INITIAL)+.05).tolist()
    runtime=RuntimeSession(system(parameters={'joint_lower':lo,'joint_upper':hi}),registry=catalog(),resource_root=tmp_path)
    try:
        runtime.start();adapter=runtime.adapters['arm'];adapter.activate()
        np.testing.assert_allclose(adapter.session.model.jnt_range[:,0],lo)
        for values in ([0]*5,[float('nan')]*6,(np.array(INITIAL)+.1).tolist()):
            with pytest.raises(ValueError,match='CONTRACT'):adapter.submit({'values':values})
        assert not adapter.session.pending
    finally:runtime.stop()
    with pytest.raises(ValueError,match='native URDF mapping'):
        builder()(system(parameters={'urdf_joint_signs':[-1,1,1,1,1,1]}))


def test_two_fr3_instances_share_one_scene_without_joint_or_body_collisions():
    resolved=compile_system({'assembly_version':1,'assembly_id':'two_fr3','revision':1,
        'instances':{key:{'plugin':'morla.fairino.fr3','backend':'sim'} for key in ('left','right')},
        'mounts':[{'child':key+'.base','parent':'world','transform':[0,y,0,0,0,0,1]} for key,y in [('left',.5),('right',-.5)]]},registry=catalog())
    artifact=builder()(resolved);model=mujoco.MjModel.from_xml_string(artifact['xml'])
    assert model.nq==model.nu==12
    assert set(artifact['joint_map']['left'].values()).isdisjoint(artifact['joint_map']['right'].values())


@pytest.mark.parametrize('mode',['strict','differential'])
def test_sim_uses_fr3_urdf_ik_without_sdk(mode):
    from morla_fr3_driver.vr_controller import create_controller
    controller=create_controller({'instance':system()['instances'][0],'binding':{'parameters':{'ik_mode':mode}}})
    try:
        solver=controller.controller.solver;solver.sync_state(INITIAL)
        target=solver.fk(INITIAL);target[0,3]+=.001
        result=solver.solve(target)
        assert result is not None and np.isfinite(result).all()
        assert solver.method_name.startswith('fr3_urdf_pinocchio')
    finally:controller.close()


def test_ros_probe_opt_in_accepts_only_kinematic_sim_never_physics_or_real(tmp_path):
    probe=module(ROOT/'docs/testing/fake_ros_gateway_smoke.py')
    path=tmp_path/'resolved.json';resolved=system();path.write_text(json.dumps(resolved))
    with pytest.raises(ValueError):probe.fake_snapshot(path)
    assert probe.fake_snapshot(path,True)['instances'][0]['backend']=='sim'
    path.write_text(json.dumps(system('physics')))
    with pytest.raises(ValueError):probe.fake_snapshot(path,True)
    resolved['instances'][0]['backend']='real';path.write_text(json.dumps(resolved))
    with pytest.raises(ValueError):probe.fake_snapshot(path,True)
