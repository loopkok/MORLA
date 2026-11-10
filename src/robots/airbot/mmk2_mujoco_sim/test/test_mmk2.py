import json
import time
from pathlib import Path
import numpy as np
import pytest
from morla_core.composition import compile_system
from morla_core.plugins.registry import default_catalog
from morla_runtime.session import RuntimeSession
from mmk2_mujoco_sim.scene import MMK2Session, create_adapters, model_xml

PACKAGE = Path(__file__).resolve().parents[1]


def resolved(cameras=False,physics=False):
    catalog=default_catalog()
    for d in json.loads((PACKAGE/'descriptors.json').read_text()):
        if not any(x['plugin_id']==d['plugin_id'] for x in catalog.list()):catalog.register(d)
    a=json.loads((PACKAGE/'config/composable.sim.yaml').read_text())
    if not cameras:
        a['instances']={k:v for k,v in a['instances'].items() if not k.endswith('_camera')}
        a['capture']['images']={}
    if physics:a['metadata']['simulation']['mode']='physics'
    return compile_system(a,registry=catalog)


def wait_for(predicate,timeout=5):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('condition timed out')


@pytest.fixture
def running(tmp_path):
    runtime=RuntimeSession(resolved(),resource_root=tmp_path/'locks',diagnostic_root=tmp_path/'logs')
    runtime.adapters=create_adapters(runtime)
    runtime.start()
    try:yield runtime
    finally:runtime.stop('cleanup')


def command(runtime,key,values):
    row=runtime.instances[key];mode=row['command_modes'][0]
    return dict(component_id=key,names=[j['name'] for j in row['joints']],values=values,mode=mode,units=row['ports'][mode+'_target']['unit'])


def activate(runtime):
    runtime.prepare('prepare')
    wait_for(lambda:runtime.operations['prepare']['status']!='RUNNING')
    assert runtime.operations['prepare']['status']=='SUCCEEDED'
    runtime.activate('activate')
    wait_for(lambda:runtime.operations['activate']['status']!='RUNNING')
    assert runtime.operations['activate']['status']=='SUCCEEDED'


def test_complete_model_and_schema():
    import mujoco, runpy, hashlib
    description=PACKAGE.parent/'mmk2_description'
    generated=runpy.run_path(str(description/'scripts/make_mmk2_mjcf.py'))['generate']()
    assert generated==(description/'mjcf/mmk2.xml').read_text()
    for relative,expected in json.loads((description/'PROVENANCE.json').read_text())['files_sha256'].items():
        assert hashlib.sha256((description/relative).read_bytes()).hexdigest()==expected,relative
    m=mujoco.MjModel.from_xml_string(model_xml())
    assert (m.nq,m.nu,m.ncam)==(24,22,4)
    r=resolved(True)
    assert len(r['instances'])==12
    base=next(b for b in r['capture_schema']['action_blocks'] if b['writer_id']=='base')
    assert base['semantics']=='velocity' and not base['source_state_indices']
    # All physical components share the same declared scene session.
    assert len(r['device_sessions'])==1
    assert sum(len(x['joints']) for x in r['instances'] if x['kind']=='actuator')==19


def test_lifecycle_motion_stop_and_sensors(running):
    r=running;s=r.adapters['base'].session
    assert r.status=='READY_DISARMED'
    assert not r.submit({'commands':[command(r,'left_arm',[.1,0,0,0,0,0])]})['accepted']
    r.reset('reset_after_rejected_dispatch')
    activate(r)
    assert r.submit({'commands':[command(r,'left_arm',[.1,0,0,0,0,0]),command(r,'lift',[.1]),command(r,'left_gripper',[.02]),command(r,'base',[1,1])]})['accepted']
    wait_for(lambda:s.read('left_arm')['values'][0]>.09 and s.read('sensors')['ports']['odometry']['values'][0]>.002)
    assert s.read('lift')['ports']['position']['unit']=='m'
    assert s.read('sensors')['ports']['lidar']['values'][90]<3
    data=s.read('sensors');assert data['synthetic'] and not data['measured']
    assert all(len(data['ports'][k]['values'])==n for k,n in [('gyro',3),('acceleration',3),('odometry',7),('lidar',180)])
    r.disarm()
    x=s.read('sensors')['ports']['odometry']['values'][0]
    time.sleep(.04)
    assert s.read('sensors')['ports']['odometry']['values'][0]==x
    assert s.read('left_arm')['values'][0]==pytest.approx(.1)
    s.set_options(mode='physics');assert np.all(s.targets['base']==0)
    s.set_options(mode='kinematic');assert np.all(s.targets['base']==0)


@pytest.mark.parametrize('change',[{'names':['bad']},{'units':'degree'},{'mode':'velocity'},{'values':[float('nan')]*6},{'values':[100]*6},{'values':[True]*6},{'values':['0']*6}])
def test_reject_invalid_commands(running,change):
    activate(running)
    item=command(running,'left_arm',[0]*6);item.update(change)
    assert not running.submit({'commands':[item]})['accepted']


def test_mode_change_requires_disarm(running):
    activate(running)
    with pytest.raises(RuntimeError,match='DISARM'):
        running.adapters['base'].session.set_options(mode='physics')


def test_reference_physics_and_mimic(tmp_path):
    r=RuntimeSession(resolved(physics=True),resource_root=tmp_path/'locks')
    r.adapters=create_adapters(r);r.start()
    try:
        activate(r);s=r.adapters['left_gripper'].session
        assert r.submit({'commands':[command(r,'left_gripper',[.02]),command(r,'head',[.1,.1])]})['accepted']
        wait_for(lambda:s.read('left_gripper')['values'][0]>.005)
        time.sleep(.2)
        assert s.error is None
        assert abs(s.data.joint('left_gripper_joint').qpos[0]-s.data.joint('left_gripper_joint1').qpos[0])<.003
    finally:r.stop('cleanup')


@pytest.mark.parametrize('side',['left','right'])
def test_fk_ik_and_urdf_frames(side):
    from mmk2_mujoco_sim.kinematics import create_kinematics
    solver=create_kinematics({'plugin_id':'morla.airbot.mmk2.'+side+'_arm'})
    q=np.array([.2,-.6,.8,.2,.3,-.1])
    target=solver.fk(q)
    solver.sync_state(q+.005)
    answer=solver.solve(target)
    assert answer is not None
    assert np.linalg.norm(solver.fk(answer)-target)<.002
    # Independently accumulate URDF transforms, rather than reusing MJCF FK.
    import xml.etree.ElementTree as E
    from scipy.spatial.transform import Rotation
    u=E.parse(PACKAGE.parent/'mmk2_description/urdf/airbot_mmk2.urdf').getroot()
    T=np.eye(4)
    for i in range(1,7):
        j=next(j for j in u.findall('joint') if j.get('name')==side+'_joint'+str(i))
        o=j.find('origin');A=np.eye(4);A[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
        A[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
        B=np.eye(4);B[:3,:3]=Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*q[i-1]).as_matrix()
        T=T@A@B
    j=next(j for j in u.findall('joint') if j.get('name')==side+'_joint_custom_end')
    o=j.find('origin');A=np.eye(4);A[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');A[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
    assert np.allclose(target,T@A,atol=1e-9)


def test_installed_provider_public_runtime(tmp_path):
    # This test requires the normal package installation, not injected adapters.
    from importlib.metadata import entry_points
    if not any(e.name=='morla.airbot.mmk2' for e in entry_points(group='morla.simulation_scenes')):
        pytest.skip('install mmk2_mujoco_sim to verify actual factory discovery')
    r=RuntimeSession(resolved(),resource_root=tmp_path/'locks')
    r.start()
    try:
        assert r.status=='READY_DISARMED'
        assert len({id(a.session) for a in r.adapters.values()})==1
    finally:r.stop('cleanup')


def test_web_discovery_compile_record_and_stop(tmp_path):
    from fastapi.testclient import TestClient
    from morla_web_manager.composable_api import create_app
    import h5py
    app=create_app(tmp_path/'store',resource_root=tmp_path/'locks')
    catalog=app.state.composable.catalog
    for d in json.loads((PACKAGE/'descriptors.json').read_text()):
        if not any(x['plugin_id']==d['plugin_id'] for x in catalog.list()):catalog.register(d)
    with TestClient(app) as client:
        templates=client.get('/api/v2/templates').json()
        assert any(x['id']=='mmk2-sim' for x in templates)
        assembly=client.get('/api/v2/templates/mmk2-sim').json()
        assembly['instances']={k:v for k,v in assembly['instances'].items() if not k.endswith('_camera')}
        assembly['capture']['images']={}
        saved=client.post('/api/v2/assemblies',json={'assembly':assembly,'expected_revision':0})
        assert saved.status_code==200,saved.text
        r=saved.json()['resolved'];assert r['feature_contract']['state_dim']==19 and r['feature_contract']['action_dim']==19
        started=client.post('/api/v2/sessions',json={'assembly_id':assembly['assembly_id'],'revision':1,'assembly_hash':r['assembly_hash'],'request_id':'start'})
        assert started.status_code==200,started.text
        sid=started.json()['session_id'];svc=app.state.composable
        activate(svc.session)
        svc.operator.device_activated()
        lease=svc.session.acquire('human_mobile',ttl_ns=2_000_000_000)
        recorded=client.post(f'/api/v2/sessions/{sid}/recordings',json={'task':'mmk2_acceptance'})
        assert recorded.status_code==200,recorded.text
        for i in range(8):
            rows=[command(svc.session,key,[.01*i,0,0,0,0,0] if key=='left_arm' else [0]*len(row['joints'])) for key,row in svc.session.instances.items() if row['kind']=='actuator']
            receipt=svc.session.submit({'producer':'human_mobile','token':lease['token'],'epoch':lease['epoch'],'sequence':i,'commands':rows})
            assert receipt['accepted'],receipt
            svc.tick();time.sleep(.02)
        stopped=client.post(f'/api/v2/sessions/{sid}/recordings/stop',json={})
        assert stopped.status_code==200,stopped.text
        episode=Path(recorded.json()['episode_dir'])
        assert json.loads((episode/'meta.json').read_text())['recording_status']=='complete'
        files=list(episode.rglob('*.h5'))+list(episode.rglob('*.hdf5'))
        assert files
        with h5py.File(files[0],'r') as f:
            assert len(f.keys())>0
            lidar=f['streams/sensors__lidar/source_time_ns'][:]
            assert len(lidar)>0
            assert np.all(np.diff(lidar)>0)
            assert f['streams/sensors__lidar/values'].shape[1]==180
            assert all(f['streams/sensors__lidar/synthetic'][:])
        quest=client.get('/api/v2/templates/mmk2-quest-sim')
        assert quest.status_code==200,quest.text
        compile_system(quest.json(),registry=catalog)


def test_rgb_depth_thread_and_sensor_clock(tmp_path):
    r=RuntimeSession(resolved(True),resource_root=tmp_path/'locks')
    r.adapters=create_adapters(r);r.start()
    try:
        s=r.adapters['head_camera'].session
        wait_for(lambda:bool(s.read('head_camera').get('ports',{}).get('rgb',{}).get('data')),timeout=10)
        for key in ('head_camera','left_wrist_camera','right_wrist_camera','scene_camera'):
            wait_for(lambda:bool(s.read(key).get('ports',{}).get('depth',{}).get('data')),timeout=10)
            sample=s.read(key)
            assert sample['scene_id']==s.read('sensors')['scene_id']
            rgb,depth=sample['ports']['rgb'],sample['ports']['depth']
            assert rgb['data'][:2]==b'\xff\xd8'
            values=np.frombuffer(depth['data'],dtype='<f4')
            assert values.size==320*240 and np.isfinite(values).all() and values.min()>0
        assert s.render_error is None
    finally:
        result=r.stop('cleanup')
        assert not result['result']['errors'],result


def test_dual_quest_binding_uses_public_ik_and_units(tmp_path):
    from morla_teleop.engine import BindingEngine
    from morla_input.adapters.normalized import InputStream
    catalog=default_catalog()
    for d in json.loads((PACKAGE/'descriptors.json').read_text()):
        if not any(x['plugin_id']==d['plugin_id'] for x in catalog.list()):catalog.register(d)
    a=json.loads((PACKAGE/'config/quest.sim.yaml').read_text())
    a['instances']={k:v for k,v in a['instances'].items() if not k.endswith('_camera')}
    a['capture']['images']={}
    r=RuntimeSession(compile_system(a,registry=catalog),resource_root=tmp_path/'locks')
    r.start()
    try:
        activate(r);engine=BindingEngine(r);source=InputStream('quest');source.reconnect()
        before=np.asarray(r.adapters['left_arm'].read(r.clock())['values'])
        for i in range(4):
            time.sleep(.025)
            ports={}
            for side in ('left','right'):
                ports[side+'_pose']={'position':[0,0,i*.001],'quaternion':[0,0,0,1]}
                ports[side+'_hold']=True
            frame=source.accept(ports,i+1,epoch=source.epoch,input_basis_id='unity_left_handed_v1')
            rows=engine.targets(frame)
            assert {x['component_id'] for x in rows}=={'left_arm','right_arm'}
            assert r.submit({'sequence':i,'commands':rows})['accepted']
        time.sleep(.02)
        assert np.linalg.norm(np.asarray(r.adapters['left_arm'].read(r.clock())['values'])-before)>1e-5
        assert all(np.isfinite(r.adapters[key].read(r.clock())['values']).all() for key in ('left_arm','right_arm'))
    finally:r.stop('cleanup')


def test_uninstalled_or_conflicting_scene_provider_rejected(tmp_path,monkeypatch):
    from morla_runtime.sim_mujoco import make_sim_adapters
    from importlib import metadata
    r=RuntimeSession(resolved(),resource_root=tmp_path/'locks')
    monkeypatch.setattr(metadata,'entry_points',lambda **_:[])
    with pytest.raises(ValueError,match='UNAVAILABLE_OR_AMBIGUOUS'):make_sim_adapters(r)
    r.instances['head']['geometry']['simulation']['provider']='other.provider'
    with pytest.raises(ValueError,match='CONFLICT'):make_sim_adapters(r)


def test_failed_configure_releases_public_resources(tmp_path):
    system=resolved()
    next(row for row in system['instances'] if row['instance_id']=='head')['parameters']['initial_positions']=[20,20]
    runtime=RuntimeSession(system,resource_root=tmp_path/'locks')
    runtime.adapters=create_adapters(runtime)
    with pytest.raises(Exception,match='INVALID_INITIAL_POSITION'):
        runtime.start()
    another=RuntimeSession(resolved(),resource_root=tmp_path/'locks')
    another.adapters=create_adapters(another)
    another.start()
    try:assert another.status=='READY_DISARMED'
    finally:another.stop('cleanup')


def test_base_binding_velocity_units_and_input_loss_stop(tmp_path):
    from morla_input.adapters.normalized import InputStream
    from morla_teleop.engine import BindingEngine
    a=resolved()['source_assembly']
    a['instances']['quest']={'plugin':'morla.input.quest','version':'1.0.0','backend':'fake'}
    a['instances']['base']['parameters']={'command_timeout_s':.1}
    a['bindings']=[{'id':'base_drive','source':'quest.right_axes','target':'base.velocity_target',
                    'controller':'morla.mmk2.differential_drive','deadman':'quest.right_hold','parameters':{}}]
    r=RuntimeSession(compile_system(a,registry=default_catalog()),resource_root=tmp_path/'locks')
    r.start()
    try:
        activate(r);source=InputStream('quest');source.reconnect();engine=BindingEngine(r)
        frame=source.accept({'right_axes':{'x':0.,'y':1.},'right_hold':True},1,epoch=source.epoch)
        rows=engine.targets(frame)
        assert len(rows)==1 and rows[0]['mode']=='velocity' and rows[0]['units']==['rad/s','rad/s']
        assert r.submit({'commands':rows})['accepted']
        scene=r.adapters['base'].session
        wait_for(lambda:scene.read('base')['values'][0]>.005)
        wait_for(lambda:np.allclose(scene.read('base')['ports']['velocity']['values'],0))
        x=scene.read('sensors')['ports']['odometry']['values'][0]
        assert x>0
        time.sleep(.03)
        assert scene.read('sensors')['ports']['odometry']['values'][0]==x
    finally:r.stop('cleanup')


def test_whole_template_compound_bases_and_head_lens():
    import mujoco
    a=json.loads((PACKAGE/'config/quest.whole.sim.yaml').read_text())
    compiled=compile_system(a,registry=default_catalog())
    assert len(compiled['bindings'])==7
    assert compiled['feature_contract']['state_dim']==19
    assert compiled['feature_contract']['action_dim']==19
    m=mujoco.MjModel.from_xml_string(model_xml());d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    body_basis=np.array([[0,0,1],[-1,0,0],[0,1,0]])
    for side in ('left','right'):
        b=next(b for b in a['bindings'] if b['target']==side+'_arm.position_target')
        U=np.array(b['parameters']['input_rotation'])
        R=d.body(side+'_base_link').xmat.reshape(3,3)
        assert np.allclose(R@U,body_basis,atol=1e-9)
        assert np.linalg.det(U)==pytest.approx(-1)
    # Independently inspect the source OBJ bounds: the head lens must be in
    # front of the shell, with its image upright and looking toward the floor.
    mesh=PACKAGE.parent/'mmk2_description/meshes/head_pitch.obj'
    vertices=np.array([[float(x) for x in line.split()[1:4]] for line in mesh.read_text().splitlines() if line.startswith('v ')])
    camera=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_CAMERA,'head')
    assert m.cam_pos[camera,0]>vertices[:,0].max()
    forward=-d.cam_xmat[camera].reshape(3,3)[:,2]
    assert forward[0]>.8 and forward[2]<-.1
