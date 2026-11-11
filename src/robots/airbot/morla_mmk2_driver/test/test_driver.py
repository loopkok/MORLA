"""Actual local gRPC and public Runtime regression; no hardware claims."""
import copy
import json
import time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import grpc
import pytest
from morla_core.plugins.registry import default_catalog
from morla_core.composition import compile_system
from morla_mmk2_driver.vendor import common_pb2 as c, mmk2_pb2 as m, mmk2_pb2_grpc as g
from morla_mmk2_driver.contract import ROLES, PREFIX, validate_target
from morla_mmk2_driver.session import DeviceSession
from morla_mmk2_driver.fake import FakeTransport
from morla_mmk2_driver.adapter import create_adapter

PKG=Path(__file__).resolve().parents[1]


def catalog():
    cat=default_catalog()
    for d in json.loads((PKG/'descriptors.json').read_text()):
        if not any(x['plugin_id']==d['plugin_id'] for x in cat.list()):cat.register(d)
    return cat


def resolve(real=False,cameras=False):
    a=json.loads((PKG/'config/quest.fake.yaml').read_text());a['bindings']=[];a['instances'].pop('quest');a['capture']['auxiliary']=['base.odometry']
    if not cameras:
        a['instances']={k:r for k,r in a['instances'].items() if 'camera' not in k};a['capture']['images']={}
    r=compile_system(a,registry=catalog())
    rows={row['instance_id']:row for row in r['instances']}
    if real:
        for row in rows.values():row['backend']='real';row['parameters'].update(host='127.0.0.1',rpc_timeout_s=.1,startup_timeout_s=.5)
    return r,rows


def wait(predicate,seconds=2):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('condition timed out')


def command(row,values):
    role=row['plugin_id'][len(PREFIX):]
    return {'names':ROLES[role][1],'values':values,'mode':'velocity' if role=='base' else 'position','units':ROLES[role][2]}


class Service(g.MMK2ServiceServicer):
    def __init__(self):
        self.fake=FakeTransport({'rate_hz':50});self.requests=[];self.reject=False;self.stale=False;self.bad=False;self.delay=0.;self.registration=[]
    def create_robot(self,request,context):
        self.registration.append(request)
        if self.delay:time.sleep(self.delay)
        return c.ParamList(params={'domain_id':'0','version':'local-protocol-fixture'})
    def get_robot_state(self,request,context):
        first=None
        for state in self.fake.states():
            if not context.is_active():return
            if first is None:first=copy.deepcopy(state)
            if self.stale:state=first
            if self.bad:del state.joint_state.name[:1]
            yield state
    def set_goal(self,request,context):
        self.requests.append(copy.deepcopy(request))
        return c.GoalStatus(value=c.GoalStatus.INVALID_TARGET if self.reject else c.GoalStatus.SUCCESS,message='fixture')
    def get_image_once(self,request,context):
        assert len(request.components)==len(request.image_types)
        return self.fake.images(request.components[0])


@pytest.fixture
def endpoint():
    server=grpc.server(ThreadPoolExecutor(max_workers=5));service=Service();g.add_MMK2ServiceServicer_to_server(service,server)
    port=server.add_insecure_port('127.0.0.1:0');server.start()
    try:yield service,port
    finally:service.fake.close();server.stop(0).wait(2)


def session_for(endpoint,motion=False,cameras=False):
    _,rows=resolve(True,cameras)
    for row in rows.values():row['parameters'].update(port=endpoint[1],allow_motion=motion,mapping_verified=motion,hold_stop_accepted=motion)
    return DeviceSession(rows)


def test_descriptors_templates_and_semantics():
    cat=catalog()
    for name in ['quest.fake.yaml','assembly.real.yaml','quest.real.example.yaml']:
        a=json.loads((PKG/'config'/name).read_text())
        dep=None
        if name!='quest.fake.yaml':
            dep=json.loads((PKG/'config/deployment.real.example.yaml').read_text());dep['instances']={k:v for k,v in dep['instances'].items() if k in a['instances']}
        r=compile_system(a,dep,registry=cat)
        if name=='quest.fake.yaml':
            assert r['feature_contract']['state_dim']==r['feature_contract']['action_dim']==19
            block=next(b for b in r['capture_schema']['action_blocks'] if b['writer_id']=='base')
            assert block['semantics']=='velocity' and not block['source_state_indices']
        assert len([s for s in r['device_sessions'] if s['session_family']=='mmk2_sdk'])==1


def test_readonly_actual_grpc(endpoint):
    s=session_for(endpoint)
    try:
        s.start();assert all(s.snapshot(k)['ready'] for k in s.rows)
        assert s.snapshot('base')['ports']['velocity']['unit']=='m_rad/s'
        assert 'position' not in s.snapshot('base')['ports']
        with pytest.raises(RuntimeError,match='READ_ONLY'):s.activate('left_arm')
        with pytest.raises(RuntimeError,match='DISARMED'):s.submit('base',command(s.rows['base'],[.01,0]))
        assert not endpoint[0].requests
    finally:s.close()
    assert not endpoint[0].requests
    assert all(not t.is_alive() for t in s.threads)


def test_sdk_goals_hold_zero_and_watchdog(endpoint):
    service,_=endpoint;s=session_for(endpoint,True)
    try:
        s.start()
        for key in s.rows:s.activate(key)
        s.submit('left_arm',command(s.rows['left_arm'],[.02,-.01,0,0,0,0]))
        s.submit('base',command(s.rows['base'],[.03,-.04]))
        wait(lambda:any('base' in r.goals_map and abs(r.goals_map['base'].goal.twist3d.x)>.02 for r in service.requests))
        wait(lambda:service.requests[-1].goals_map['base'].goal.twist3d.x==0,seconds=1)
        goal=next(r.goals_map['left_arm'] for r in service.requests if 'left_arm' in r.goals_map)
        assert list(goal.goal.joint_state.position)==pytest.approx([.005,-.0025,0,0,0,0])
        assert goal.param.trajectory.max_velocity_scaling_factor==.05
        s.stop();count=len(service.requests);time.sleep(.1)
        assert len(service.requests)==count
        assert not s.active and not s.pending and not s.desired
        assert service.requests[-1].goals_map['base'].goal.twist3d.x==0
    finally:s.close()


@pytest.mark.parametrize('problem',['stale','bad','reject'])
def test_feedback_or_goal_fault_stops_without_replay(endpoint,problem):
    service,_=endpoint;s=session_for(endpoint,True)
    try:
        s.start();s.activate('base');s.submit('base',command(s.rows['base'],[.01,0]));wait(lambda:service.requests)
        setattr(service,problem,True)
        wait(lambda:s.fault)
        assert not s.snapshot('base')['ready'] and not s.active
        with s.io:count=len(service.requests)
        time.sleep(.15);assert len(service.requests)==count
        with pytest.raises(RuntimeError):s.activate('base')
    finally:
        service.reject=False;s.close()


def test_rpc_deadline_and_close_on_start_failure(endpoint):
    service,_=endpoint;service.delay=.4;s=session_for(endpoint)
    started=time.monotonic()
    with pytest.raises(grpc.RpcError):s.start()
    assert time.monotonic()-started<.5 and s.closed


def test_missing_names_duplicate_errors_and_nonfinite():
    from morla_mmk2_driver.feedback import decode
    _,rows=resolve();fake=FakeTransport({'rate_hz':20});state=next(fake.states());fake.close()
    assert decode(state,rows['left_arm'])[1]==[0.]*6
    bad=copy.deepcopy(state);bad.joint_state.name[0]='missing'
    with pytest.raises(ValueError):decode(bad,rows['left_arm'])
    bad=copy.deepcopy(state);bad.joint_state.name[0]=bad.joint_state.name[1]
    with pytest.raises(ValueError):decode(bad,rows['left_arm'])
    bad=copy.deepcopy(state);bad.joint_state.position[0]=float('nan')
    with pytest.raises(ValueError):decode(bad,rows['left_arm'])
    bad=copy.deepcopy(state);bad.joint_state.error_code.extend([1]*len(bad.joint_state.name))
    with pytest.raises(ValueError):decode(bad,rows['left_arm'])


def test_command_boundary_and_mapping():
    _,rows=resolve();row=rows['base'];cmd=command(row,[.01,.02])
    for change in [{'units':'rad/s'},{'mode':'position'},{'names':['left_wheel_joint','right_wheel_joint']},{'values':[True,0]},{'values':[float('nan'),0]},{'values':[.16,0]}]:
        with pytest.raises(ValueError):validate_target(row,dict(cmd,**change))
    row=rows['left_gripper'];row['parameters']['joint_offsets']=[.9]
    with pytest.raises(ValueError,match='MAPPED'):validate_target(row,command(row,[0.]))
    assert validate_target(rows['left_arm'],command(rows['left_arm'],[0]*6))==[0]*6


def test_shared_connection_mismatch_and_duplicate_role():
    _,rows=resolve();rows['head']['parameters']['port']=50056
    with pytest.raises(ValueError,match='MISMATCH'):DeviceSession(rows)
    _,rows=resolve();rows['extra']=copy.deepcopy(rows['head'])
    with pytest.raises(ValueError,match='DUPLICATE'):DeviceSession(rows)


def test_camera_actual_grpc_and_shared_cleanup(endpoint):
    from morla_mmk2_driver.camera import CameraAdapter
    s=session_for(endpoint,cameras=True);captures=[CameraAdapter(row,s) for row in s.rows.values() if row['plugin_id'].endswith('.camera')]
    try:
        for camera in captures:camera.configure({})
        wait(lambda:all(c.health(0)['ready'] for c in captures))
        sample=captures[0].read(0)
        assert sample['ports']['rgb']['encoding']=='jpeg'
        assert sample['ports']['depth']['unit']=='sdk_depth' and sample['ports']['depth']['depth_scale_m'] is None
        assert 'data' not in captures[0].health(0)['ports']['rgb']
    finally:captures[0].close()
    assert s.closed and all(not c.thread.is_alive() for c in captures)


def test_public_runtime_lifecycle_and_whole_device_stop(tmp_path):
    from morla_runtime.session import RuntimeSession
    r,_=resolve(cameras=True);runtime=RuntimeSession(r,resource_root=tmp_path/'locks',diagnostic_root=tmp_path/'logs')
    shared={}
    for key,row in runtime.instances.items():
        runtime.adapters[key]=create_adapter(row,{'instances':runtime.instances,'device_session':r['device_sessions'][0],'shared_device_sessions':shared,'clock':runtime.clock})
    try:
        runtime.start();assert runtime.status=='READY_DISARMED'
        runtime.prepare('p');wait(lambda:runtime.operations['p']['status']!='RUNNING');assert runtime.operations['p']['status']=='SUCCEEDED'
        runtime.activate('a');wait(lambda:runtime.operations['a']['status']!='RUNNING');assert runtime.operations['a']['status']=='SUCCEEDED'
        assert len({id(a.session) for a in runtime.adapters.values()})==1
    finally:runtime.stop('test_cleanup')
    s=next(iter(shared.values()));assert s.closed and all(not c.thread.is_alive() for c in s.captures)


def test_si_ros_feedback_classification():
    from morla_runtime.ros_bridge import has_si_joint_position
    _,rows=resolve()
    assert all(has_si_joint_position(rows[k]) for k in ['left_arm','right_arm','head','lift'])
    assert all(not has_si_joint_position(rows[k]) for k in ['base','left_gripper','right_gripper'])


def test_quest_binding_native_velocity_and_arm_ik(tmp_path):
    from morla_runtime.session import RuntimeSession
    from morla_teleop.engine import BindingEngine
    from morla_input.adapters.normalized import InputStream
    a=json.loads((PKG/'config/quest.fake.yaml').read_text())
    a['instances']={k:v for k,v in a['instances'].items() if 'camera' not in k};a['capture']['images']={}
    r=RuntimeSession(compile_system(a,registry=catalog()),resource_root=tmp_path/'locks')
    try:
        r.start();r.prepare('p');wait(lambda:r.operations['p']['status']!='RUNNING');r.activate('a');wait(lambda:r.operations['a']['status']!='RUNNING')
        assert r.operations['a']['status']=='SUCCEEDED'
        engine=BindingEngine(r);source=InputStream('quest');source.reconnect()
        for binding in r.resolved['bindings']:engine._binding_controller(binding)
        for i in range(4):
            time.sleep(.05)
            ports={}
            for side in ['left','right']:
                ports[side+'_pose']={'position':[0,0,i*.0001],'quaternion':[0,0,0,1]}
                ports[side+'_hold']=True;ports[side+'_scalar']=.01*i;ports[side+'_axes']={'x':.1,'y':.1}
            frame=source.accept(ports,i+1,epoch=source.epoch,input_basis_id='unity_left_handed_v1')
            commands=engine.targets(frame)
            assert {c['component_id'] for c in commands}==set(ROLES)
            base=next(c for c in commands if c['component_id']=='base')
            assert base['mode']=='velocity' and base['units']==['m_rad/s']*2 and base['values']==pytest.approx([.005,-.015])
            receipt=r.submit({'sequence':i,'commands':commands});assert receipt['accepted'],receipt
        wait(lambda:abs(r.adapters['base'].read(0)['values'][0])>.001)
    finally:r.stop('cleanup')


def test_web_installed_discovery_lifecycle_camera_recording(tmp_path):
    from fastapi.testclient import TestClient
    from morla_web_manager.composable_api import create_app
    import h5py
    app=create_app(tmp_path/'store',resource_root=tmp_path/'locks')
    with TestClient(app) as client:
        templates=client.get('/api/v2/templates').json()
        assert {'mmk2-sdk-fake','mmk2-sdk-real'}<={r['id'] for r in templates}
        a=client.get('/api/v2/templates/mmk2-sdk-fake').json();assert a.get('assembly_id')
        a['capture']['auxiliary']=['base.odometry']
        saved=client.post('/api/v2/assemblies',json={'assembly':a,'expected_revision':0});assert saved.status_code==200,saved.text
        resolved=saved.json()['resolved']
        started=client.post('/api/v2/sessions',json={'assembly_id':a['assembly_id'],'revision':1,'assembly_hash':resolved['assembly_hash'],'request_id':'start'})
        assert started.status_code==200,started.text
        sid=started.json()['session_id'];svc=app.state.composable
        svc.session.prepare('p');wait(lambda:svc.session.operations['p']['status']!='RUNNING')
        svc.session.activate('a');wait(lambda:svc.session.operations['a']['status']!='RUNNING');assert svc.session.operations['a']['status']=='SUCCEEDED'
        svc.operator.device_activated()
        lease=svc.session.acquire('human_mobile',ttl_ns=2_000_000_000)
        wait(lambda:svc.session.adapters['head_camera'].read(0)['ready'])
        assert client.get(f'/api/v2/sessions/{sid}/cameras/head_camera/frame').status_code==200
        recorded=client.post(f'/api/v2/sessions/{sid}/recordings',json={'task':'sdk_protocol_acceptance'});assert recorded.status_code==200,recorded.text
        for i in range(10):
            commands=[dict(component_id=key,**command(row,([.01,0] if key=='base' else [.001*i]+[0.]*(len(row['joints'])-1)))) for key,row in svc.session.instances.items() if row['kind']=='actuator']
            result=svc.session.submit({'producer':'human_mobile','token':lease['token'],'epoch':lease['epoch'],'sequence':i,'commands':commands});assert result['accepted'],result
            svc.tick();time.sleep(.05)
        stopped=client.post(f'/api/v2/sessions/{sid}/recordings/stop',json={});assert stopped.status_code==200,stopped.text
        episode=Path(recorded.json()['episode_dir'])
        assert json.loads((episode/'meta.json').read_text())['recording_status']=='complete'
        files=list(episode.rglob('*.h5'))+list(episode.rglob('*.hdf5'));assert files
        with h5py.File(files[0],'r') as f:
            base=f['streams/base__velocity'];assert base['values'].shape[1]==2
            assert all(base['synthetic'][:])
            stamps=base['source_time_ns'][:];assert len(stamps)>2 and all(b>a for a,b in zip(stamps,stamps[1:]))
        outcome=svc.session.stop('cleanup');assert not outcome['result']['errors'],outcome


def test_differential_ik_singularity_and_large_hand_motion_are_bounded():
    import numpy as np
    from morla_mmk2_driver.kinematics import create_kinematics
    _,rows=resolve()
    for side in ['left_arm','right_arm']:
        solver=create_kinematics(rows[side]);q=np.zeros(6);target=solver.fk(q).copy()
        # At the zero pose the old strict solve could flip the wrist by >.3rad.
        for displacement in [.0002,.5,-.5,.001]:
            request=target.copy();request[:3,3]+=[displacement,displacement,displacement]
            solver.sync_state(q);result=np.array(solver.solve(request))
            assert np.isfinite(result).all() and max(abs(result-q))<=solver.max_step+1e-12
            assert np.all(result>=solver.bounds[:,0]) and np.all(result<=solver.bounds[:,1])
            q=result
        assert solver.last_report['method']=='mmk2_urdf_differential'


def test_differential_ik_follows_a_static_hand_to_the_target():
    """A held hand must keep being followed, not frozen one lead away.

    The effective goal was integrated from the previous raw target and the lead
    clamp wrote the clamped pose back, so with a stationary hand the deltas
    telescoped to zero and the arm kept the offset it had when the hand stopped
    (0.59 m in the recorded MMK2 session). Re-deriving the goal from the raw
    target every tick closes it.
    """
    import numpy as np
    from morla_mmk2_driver.kinematics import create_kinematics
    _,rows=resolve()
    solver=create_kinematics(rows['left_arm'])
    q=np.array([0,-.8,.8,1.64,.8,0.]);solver.sync_state(q);start=solver.fk(q)
    target=start.copy();target[0,3]+=.05
    for _ in range(400):
        q=np.array(solver.solve(target));solver.sync_state(q)
    assert solver.last_report['raw_position_error_m']<1e-3
    assert solver.last_report['raw_angle_error_rad']<1e-3


def test_fault_reason_is_logged_and_survives_dispatch_rejection():
    _,rows=resolve();events=[]
    s=DeviceSession(rows,diagnostics=lambda kind,data:events.append((kind,data)))
    try:
        s.start();s.activate('base');s._fault(TimeoutError('MMK2_FEEDBACK_STALE'))
        assert events[-1][0]=='device_fault' and events[-1][1]['reason']=='MMK2_FEEDBACK_STALE'
        with pytest.raises(RuntimeError,match='MMK2_DEVICE_FAULT: MMK2_FEEDBACK_STALE'):
            s.submit('base',command(rows['base'],[0.,0.]))
    finally:s.close()


def test_motion_authorization_is_per_actuator_not_a_camera_connection_flag():
    _,rows=resolve(real=True,cameras=True)
    # Test without connecting: inject the explicitly synthetic protocol fixture.
    for field in ['allow_motion','mapping_verified','hold_stop_accepted']:rows['left_arm']['parameters'][field]=True
    s=DeviceSession(rows,transport_factory=FakeTransport)
    try:
        s.start();s.activate('left_arm')
        assert s.active=={'left_arm'}
        with pytest.raises(RuntimeError,match='READ_ONLY'):s.activate('right_arm')
    finally:s.close()
