import copy
import json
from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np
import pytest
from morla_core.composition import load_document,compile_system
from morla_core.plugins.registry import Catalog,default_catalog
from morla_input.adapters.ingress import InputIngressAdapter
from morla_vader5_input.source import Vader5Source
from morla_vader5_input.reader import ReportStream
from morla_vader5_input.mapping import BUTTONS,configuration

ROOT=Path(__file__).resolve().parents[4]
PACKAGE=Path(__file__).parents[1]


def row(seq=1,buttons=(),sticks=None,**kw):
    return {'version':1,'epoch':'session1','sequence':seq,'sample_ns':1_000_000_000,
        'connected':True,'backend':'fake','control_tick':True,'sticks':sticks or [0.,0.,0.,0.],
        'triggers':[0.,0.],'buttons':list(buttons),'gyro':[0.,0.,0.],'accel':[0.,0.,9.80665],
        'raw_hex':'','unknown_bits':[0,0,0,0],**kw}


def source():return Vader5Source('pad',backend='fake',clock=lambda:1_000_000_000)


def test_public_normalizer_axes_modes_and_imu_not_pose():
    s=source();frame=s.event(row(sticks=[.5,.8,-.3,.4],triggers=[0.,.7]))
    assert frame.ports['axes']==dict(x=.8,y=.5,z=.4,roll=0.,pitch=0.,yaw=0.)
    assert frame.ports['scalar']==.7 and frame.synthetic
    assert 'pose' not in frame.ports
    frame=s.event(row(2,buttons=['M1','M2'],sticks=[.5,.8,-.3,.4],triggers=[1.,0.]))
    np.testing.assert_allclose(list(frame.ports['axes'].values()),[0,0,0,.1,.16,-.06])
    assert frame.ports['hold_to_control'] is True
    a=InputIngressAdapter({'instance_id':'pad'},s.clock);a.configure({});a.feed(frame)
    saved=a.read(s.clock());assert saved['synthetic']
    assert len(saved['ports']['axes']['values'])==6 and len(saved['ports']['buttons']['values'])==27
    assert saved['ports']['hold_to_control']['received_ns']==1_000_000_000


def test_buttons_release_gate_and_no_reuse():
    s=source();s.event(row(buttons=['START']));assert not s.actions
    s.event(row(2));s.event(row(3,buttons=['START']));assert s.actions==['teleop_start']
    s.event(row(4,buttons=['START']));assert not s.actions
    s.event(row(5,buttons=['START','SELECT','M1']))
    assert not s.actions and s.latest.ports['hold_to_control']
    s.event(row(6,buttons=['START']));assert not s.actions
    for config in ({'deadman_button':'START'},{'rotation_button':'M1'},
                   {'action_buttons':{'START':'teleop_start','M1':'record_stop'}},
                   {'action_buttons':{'START':'record_start'}},
                   {'action_buttons':{'START':'teleop_start','TURBO':'record_stop'}}):
        assert configuration(config)


def test_every_button_and_replay_source_age_reconnect():
    calls=[];s=Vader5Source('pad',backend='fake',clock=lambda:1_000_000_000,on_transition=lambda:calls.append('transition'))
    for i,b in enumerate(BUTTONS,1):
        f=s.event(row(i,buttons=[b]));assert sum(f.ports['buttons'])==1
    for bad in (row(len(BUTTONS)),row(30,sample_ns=1),row(31,sample_ns=1_000_000_001),
                row(32,backend='real'),row(33,buttons=['UNKNOWN']),row(34,sticks=[float('nan')]*4)):
        with pytest.raises(ValueError):s.event(bad)
    s.event(row(40,epoch='new'));assert len(calls)==2
    with pytest.raises(ValueError):s.event(row(41))
    assert s.event(row(42,epoch='new',connected=False)) is None and not s.connected
    s.event(row(43,epoch='new',buttons=['START']));assert not s.actions


def test_batch_rate_limit_preserves_short_release_and_actions():
    clock=[1_000_000_000];stream=ReportStream(clock=lambda:clock[0],backend='fake')
    def state(buttons):return SimpleNamespace(received_at=clock[0]/1e9,buttons=buttons,
        left_stick=(0,0),right_stick=(0,0),triggers=(0,0),gyro_rad_s=(0,0,0),accel_m_s2=(0,0,9.8),
        raw=SimpleNamespace(unknown_bits=(),raw=b'\0'*32))
    assert stream.packet(state(set()))['control_tick']
    clock[0]+=1_000_000;assert stream.packet(state(set())) is None
    clock[0]+=1_000_000;r=stream.packet(state({'M1'}));assert r and not r['control_tick']
    clock[0]+=1_000_000;r=stream.packet(state(set()));assert r and not r['control_tick']
    assert stream.packet(connected=False)['connected'] is False
    stream=ReportStream(clock=lambda:clock[0],backend='fake')
    assert not stream.packet(state(set()),allow_tick=False)['control_tick']
    assert stream.packet(state(set()),allow_tick=True)['control_tick']


def catalog():
    arm=ROOT/'src/robots/dobot/morla_cr5_driver'
    epg=ROOT/'src/robots/jodell/morla_epg_driver'
    rows=[json.loads((PACKAGE/'descriptor.json').read_text()),json.loads((epg/'descriptor.json').read_text())]
    rows += [json.loads((arm/name).read_text()) for name in ('descriptor.json','jog_controller.json')]
    return Catalog(default_catalog().list()+rows)


def test_configs_and_input_mapping_hash():
    c=catalog()
    for name in ('cr5.fake','epg.fake','nero.example','astral.example'):
        a=load_document(PACKAGE/f'config/{name}.yaml');r=compile_system(a,registry=c)
        assert r['capture_schema']['metadata']['stream_sources']['pad__axes']['instance_id']=='pad'
        assert r['feature_contract']['action_dim'] in (1,6,7)
    a=load_document(PACKAGE/'config/cr5.fake.yaml');r=compile_system(a,registry=c)
    a['instances']['pad']['parameters']={'deadman_button':'M4'}
    newer=compile_system(a,registry=c)
    assert newer['assembly_hash']!=r['assembly_hash'] and newer['resolved_hash']!=r['resolved_hash']


def test_service_record_buttons_gateway_and_policy_exclusion(tmp_path):
    ComposableService=pytest.importorskip('morla_web_manager.composable_service').ComposableService
    service=ComposableService(tmp_path/'store',resource_root=tmp_path/'locks');service.catalog=catalog()
    clock=[1_000_000_000];service.clock=lambda:clock[0]
    a=load_document(PACKAGE/'config/epg.fake.yaml');saved=service.save({'assembly':a,'expected_revision':0})
    state=service.start({'assembly_id':a['assembly_id'],'revision':1,'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'start'})
    sid=state['session_id'];s=Vader5Source('pad',backend='fake',clock=service.clock,on_transition=lambda:service.disconnect_input('pad'))
    service.inputs['pad']=s;seq=[0]
    def send(buttons=(),trigger=0.):
        seq[0]+=1;clock[0]+=20_000_000
        f=s.event(row(seq[0],buttons=buttons,triggers=[0.,trigger],sample_ns=clock[0]))
        service.input_event(sid,'pad',f,s.actions)
        return service.ingest_frame(sid,f)
    try:
        send();send(['START']);assert service.session.status=='ACTIVE'
        send();assert send(['M1'],.4)['accepted']
        first=service.session.adapters['gripper'].values[0]
        assert 0<first<=.05 # No deadman clutch: the scalar applies immediately.
        send(['M1'],.4)
        assert service.session.adapters['gripper'].values[0]==pytest.approx(first)
        send(['M1','Y'],.4);assert service.recorder is not None
        record_path=service.recorder.episode_dir
        # Duplicate callback cannot create a second recording.
        service.input_event(sid,'pad',s.latest,['record_start'])
        assert not service.input_errors
        send(['M1'],.4);service.capture_tick();send(['M1','X'],.4);assert service.recorder is None
        meta=json.loads((record_path/'meta.json').read_text())
        assert meta['recording_status']=='complete'
        assert any(e.get('name')=='input_buttons' for e in meta['events'])
        assert list(record_path.rglob('*.h5')) or list(record_path.rglob('*.hdf5'))
        send(['B']);assert service.session.status=='READY_DISARMED'
        # Authority is no longer enforced: human input dispatches while a policy lease exists.
        service.operation(sid,{'operation':'activate','request_id':'again'})
        service.session.acquire('policy')
        before=list(service.session.adapters['gripper'].values)
        assert send(['M1'],.9)['accepted']
        assert service.session.lease['producer']=='policy'
        assert service.session.adapters['gripper'].values!=before
        service.operation(sid,{'operation':'estop','request_id':'estop'})
        assert service.session.status=='ESTOP_LATCHED'
    finally:service.close()
    assert not service.session.resources.handles


def test_scalar_map_rejects_multi_joint_target():
    a=load_document(PACKAGE/'config/cr5.fake.yaml')
    a['bindings'][0].update(source='pad.scalar',controller='morla.scalar_map',parameters={})
    with pytest.raises(ValueError,match='one target joint'):compile_system(a,registry=catalog())


def test_policy_and_replay_use_same_output_contract(tmp_path):
    from morla_runtime import RuntimeSession
    from morla_learning.contracts import ModelManifest,digest
    from morla_policy_inference.composable import RuntimePolicy,RuntimeReplay
    value=compile_system(load_document(PACKAGE/'config/epg.fake.yaml'),registry=catalog())
    runtime=RuntimeSession(value,registry=catalog(),resource_root=tmp_path)
    runtime.start();runtime.activate('activate')
    features=value['feature_contract']
    m=ModelManifest(1,'act','revision',digest(features),value['geometry_hash'],'norm','checkpoint',
        'dataset',features,{},32,'inference_verified')
    policy=RuntimePolicy(runtime,lambda observation:[.025],m,normalization_hash='norm')
    try:
        policy.acquire();policy.request();reply=None
        for _ in range(100):
            reply=policy.poll()
            if reply is not None:break
            time.sleep(.002)
        assert reply['accepted'] and runtime.adapters['gripper'].values==[.025]
        policy.close()
        runtime.activate('again')
        replay=RuntimeReplay(runtime,value);replay.acquire()
        assert replay.send([.03],sample_ns=runtime.clock())['accepted']
        assert runtime.adapters['gripper'].values==[.03]
    finally:policy.close();runtime.stop()


def test_cr5_jog_uses_existing_solver_and_small_precision_steps(monkeypatch):
    from morla_cr5_driver.vr_controller import create_jog_controller
    c=catalog();a=load_document(PACKAGE/'config/cr5.fake.yaml');resolved=compile_system(a,registry=c)
    instance=next(i for i in resolved['instances'] if i['instance_id']=='arm')
    controller=create_jog_controller({'instance':instance,'binding':a['bindings'][0]})
    measured=np.array([.3,-.7,.9,-.4,.6,.2])
    try:
        assert controller.controller.jog and controller.controller.solver.method_name=='cr5_urdf_pinocchio_dls'
        # 80 um command (LT precision at full stick) must not disappear into the
        # older 0.2 mm VR acceptance tolerance.
        values=dict(x=.2,y=0.,z=0.,roll=0.,pitch=0.,yaw=0.)
        controller(values,measured,.02,a['bindings'][0]['parameters'])
        end=time.monotonic()+1
        while controller.result is None and controller.error is None and time.monotonic()<end:time.sleep(.002)
        assert controller.error is None
        assert np.max(np.abs(controller.result[0]-measured))>1e-6
        result=controller(values,measured,.02,a['bindings'][0]['parameters'])
        assert np.max(np.abs(result-measured))<=.006+1e-12
    finally:controller.close()


def test_port_source_age_is_not_refreshed_by_ros_arrival():
    clock=[1_200_000_000];s=Vader5Source('pad',backend='fake',clock=lambda:clock[0])
    f=s.event(row());assert s.read() is f
    clock[0]+=60_000_000
    assert s.read() is None # Only 60 ms after ROS arrival, but source sample is 260 ms old.


def test_selected_ros_input_plugin_reuses_common_host(monkeypatch):
    import sys,types
    from morla_input.ros_sources import RosInputManager
    from morla_vader5_input.source import create_ros_source
    callbacks={};observed=[]
    class Node:
        def __init__(self,*args):pass
        def create_subscription(self,kind,topic,callback,qos):callbacks[topic]=callback
        def get_logger(self):return SimpleNamespace(error=lambda text:observed.append(('error',text)))
        def destroy_node(self):pass
    class Executor:
        def add_node(self,node):pass
        def spin(self):pass
        def shutdown(self,**kw):pass
    modules={'rclpy':{'ok':lambda:True},'rclpy.node':{'Node':Node},
        'rclpy.executors':{'SingleThreadedExecutor':Executor},
        'rclpy.qos':{'qos_profile_sensor_data':None,'QoSProfile':lambda **kw:kw,'ReliabilityPolicy':SimpleNamespace(RELIABLE=1)},
        'sensor_msgs.msg':{'Joy':object},'geometry_msgs.msg':{'PoseStamped':object,'PoseArray':object},'std_msgs.msg':{'String':object}}
    for name,attrs in modules.items():
        module=types.ModuleType(name);module.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,module)
    entry=SimpleNamespace(name='morla.input.vader5',load=lambda:create_ros_source)
    monkeypatch.setattr('importlib.metadata.entry_points',lambda **kw:[entry] if kw.get('group')=='morla.ros_inputs' else [])
    d=json.loads((PACKAGE/'descriptor.json').read_text())
    manager=RosInputManager([{'instance_id':'pad','plugin_id':d['plugin_id'],'backend':'fake','parameters':d['defaults']}],
        lambda f:observed.append(('frame',f.ports['hold_to_control'])),lambda iid:observed.append(('disconnect',iid)),
        clock=lambda:1_000_000_000,on_event=lambda iid,f,actions:observed.append(('event',actions)))
    manager.start()
    try:
        callback=callbacks['/vader5/events'];callback(SimpleNamespace(data=json.dumps(row())))
        callback(SimpleNamespace(data=json.dumps(row(2,buttons=['START'],control_tick=False))))
        assert ('event',['teleop_start']) in observed
        callback(SimpleNamespace(data=json.dumps(row(3,buttons=['M1'],control_tick=False))))
        count=len([r for r in observed if r[0]=='frame'])
        callback(SimpleNamespace(data=json.dumps(row(4,buttons=[],control_tick=True))))
        assert len([r for r in observed if r[0]=='frame'])==count+1 # Control tick delivers the frame.
    finally:manager.close()
