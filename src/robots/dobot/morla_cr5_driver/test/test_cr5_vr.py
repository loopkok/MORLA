import copy
import json
import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from morla_cr5_driver.kinematics import ControllerSolver,FakeSolver,pose_from_controller,pose_to_controller
from morla_cr5_driver.transport import CR5Transport
from morla_cr5_driver.vr_controller import AsyncCartesianController,create_controller


def pose(x=0.):return {'position':[x,0,0],'quaternion':[0,0,0,1]}


def wait_result(controller, predicate=lambda:True):
    deadline=time.monotonic()+1
    while time.monotonic()<deadline:
        with controller.condition:
            if (controller.result is not None or controller.error is not None) and predicate():return
        time.sleep(.002)
    raise AssertionError('IK worker result missing')


def test_pose_units_and_noncommuting_rotation_roundtrip():
    original=[123,-234,345,25,-35,65]
    p=pose_from_controller(original)
    np.testing.assert_allclose(p[:3,3],[.123,-.234,.345])
    np.testing.assert_allclose(pose_from_controller(pose_to_controller(p)),p,atol=1e-10)


class Dashboard:
    def __init__(self):self.commands=[]
    def request(self,command):
        self.commands.append(command)
        return '0,90,-90,180,-180,45'


def test_seeded_v3_commands_and_bad_reply():
    t=object.__new__(CR5Transport);t.dashboard=Dashboard()
    q=np.deg2rad([0,90,-90,180,-180,45])
    t.forward_kinematics(q)
    t.inverse_kinematics([100,200,300,10,20,30],q)
    assert t.dashboard.commands[0]=='PositiveSolution(0.000000,90.000000,-90.000000,180.000000,-180.000000,45.000000,0,0)'
    assert t.dashboard.commands[1].endswith(',0,0,1,{0.000000,90.000000,-90.000000,180.000000,-180.000000,45.000000})')
    for raw in ('1,2','0,0,0,0,0,nan'):
        with pytest.raises(ValueError):t._six_reply(raw)


def test_controller_solver_serialized_readonly_and_fk():
    calls=[];model=FakeSolver()
    class Transport:
        def forward_kinematics(self,q):calls.append('fk');return pose_to_controller(model.fk(q))
        def inverse_kinematics(self,p,seed):calls.append('ik');return np.rad2deg(model.solve(pose_from_controller(p)))
    a=SimpleNamespace(parameters={'kinematics_verified':True,'joint_lower':[-2]*6,'joint_upper':[2]*6},
                      io=threading.Lock(),shutdown=threading.Event(),transport=Transport())
    solver=ControllerSolver(a);solver.sync_state([0]*6)
    target=solver.fk([0]*6);target[0,3]=.002
    np.testing.assert_allclose(solver.solve(target),[.02,0,0,0,0,0])
    assert calls==['fk','ik','fk']
    a.parameters['joint_upper']=[.001]*6
    with pytest.raises(ValueError,match='limits'):solver.solve(target)


def test_async_anchor_velocity_and_reset_fences_old_results():
    c=AsyncCartesianController(FakeSolver())
    try:
        assert c(pose(),[0]*6,.02,{})==[0]*6
        wait_result(c)
        c(pose(.001),[0]*6,.02,{})
        wait_result(c,lambda:c.result[0][0]>.005)
        result=c(pose(.001),[0]*6,.02,{'max_joint_velocity':.3})
        assert 0<result[0]<=.006
        c.reset()
        assert c(pose(.4),[.2,0,0,0,0,0],.02,{})==[.2,0,0,0,0,0]
        wait_result(c)
        np.testing.assert_allclose(c.result[0],[.2,0,0,0,0,0])
    finally:c.close()
    assert not c.thread.is_alive()


def test_late_result_after_reset_cannot_reappear():
    entered=threading.Event();finish=threading.Event()
    class Blocking(FakeSolver):
        def fk(self,q):entered.set();finish.wait(.5);return super().fk(q)
    c=AsyncCartesianController(Blocking())
    try:
        c(pose(),[0]*6,.02,{})
        assert entered.wait(.2)
        c.reset();finish.set();time.sleep(.02)
        assert c.result is None
    finally:finish.set();c.close()


def test_quest_right_controller_hand_switch_and_loss_revoke_runtime(tmp_path,monkeypatch):
    from morla_core.plugins.registry import Catalog,default_catalog
    from morla_core.composition import load_document
    from morla_input.quest_right import QuestRightSource
    ComposableService=pytest.importorskip('morla_web_manager.composable_service').ComposableService
    root=Path(__file__).parents[5]
    package=Path(__file__).parents[1]
    descriptors=[json.loads((package/name).read_text()) for name in ('descriptor.json','controller.json')]
    descriptors.append(json.loads((root/'src/common/morla_input/quest_right_descriptor.json').read_text()))
    catalog=Catalog(default_catalog().list()+descriptors)
    class Entries(list):
        def select(self,**kwargs):return Entries(e for e in self if all(getattr(e,k)==v for k,v in kwargs.items()))
    entry=SimpleNamespace(group='morla.controllers',name='morla.cr5.cartesian_delta',load=lambda:create_controller)
    monkeypatch.setattr('importlib.metadata.entry_points',lambda **kw:Entries([entry]).select(**kw))
    svc=ComposableService(tmp_path/'store',resource_root=tmp_path/'locks');svc.catalog=catalog
    a=load_document(package/'config/quest_right.fake.yaml')
    saved=svc.save({'assembly':a,'expected_revision':0})
    assert 'morla.cr5.cartesian_delta@1.0.0' in dict(saved['resolved']['lockfile']['plugins'])
    state=svc.start({'assembly_id':a['assembly_id'],'revision':1,'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'start'})
    sid=state['session_id'];op_count=[0];seq=[0]
    def op(name):
        op_count[0]+=1
        return svc.operation(sid,{'operation':name,'request_id':f'{name}-{op_count[0]}'})
    source=QuestRightSource('quest',on_transition=lambda:svc.disconnect_input('quest'))
    svc.inputs['quest']=source
    def send(kind,mode='controller',value=None):
        seq[0]+=1
        row={'version':1,'side':'right','kind':kind,'source':mode,'epoch':'test-stream',
             'sequence':seq[0],'sent_ns':time.time_ns(),'frame_id':'quest_world',
             'values':value if value is not None else [0,0,0,0,0,0,1]}
        frame=source.event(row)
        return svc.ingest_frame(sid,frame) if source.control_event(row,frame) else None
    def landmarks(closed):
        p=[[0.,0.,0.] for _ in range(21)];p[17]=[.08,0,0]
        p[8]=[.005 if closed else .08,0,0];return p
    worker=None
    try:
        send('pose')  # Disarmed packets cannot pre-arm or anchor.
        op('prepare');op('activate')
        assert send('grip',value=1.) is None
        assert send('pose')['accepted'] # Deadman is not enforced.
        send('grip',value=0.)
        send('grip',value=1.)
        assert send('pose')['accepted']
        worker=svc.bindings['right_vr'].registry.plugins['morla.cr5.cartesian_delta']
        wait_result(worker)
        deadline=time.monotonic()+1
        while svc.session.adapters['arm'].values[0]<=0 and time.monotonic()<deadline:
            time.sleep(.01);send('pose',value=[.002,0,0,0,0,0,1])
        assert svc.session.adapters['arm'].values[0]>0
        before=list(svc.session.adapters['arm'].values)
        send('pose','hand',[.3,0,0,0,0,0,1])
        assert svc.session.lease is None and svc.session.status=='READY_DISARMED'
        assert send('grip',value=1.) is None
        op('activate')
        # Landmarks now reach the common binding host for scalar/hand capabilities.
        # Cached pose samples are deduplicated per binding rather than at transport level.
        assert 'accepted' in send('landmarks','hand',landmarks(True))
        assert send('pose','hand',[.3,0,0,0,0,0,1])['accepted']
        send('landmarks','hand',landmarks(False))
        send('landmarks','hand',landmarks(True))
        assert send('pose','hand',[.3,0,0,0,0,0,1])['accepted']
        wait_result(worker)
        np.testing.assert_allclose(svc.session.adapters['arm'].values,before)
        deadline=time.monotonic()+1
        while np.linalg.norm(np.asarray(svc.session.adapters['arm'].values)-before)<1e-5 and time.monotonic()<deadline:
            time.sleep(.01)
            send('landmarks','hand',landmarks(True))
            send('pose','hand',[.302,0,0,0,0,0,1])
        assert np.linalg.norm(np.asarray(svc.session.adapters['arm'].values)-before)>1e-5
        # Pinch loss no longer revokes control; fresh wrist packets keep dispatching.
        time.sleep(.27)
        assert send('pose','hand',[.301,0,0,0,0,0,1])['accepted']
        assert svc.session.status=='ACTIVE'
    finally:svc.close()
    assert worker is not None and not worker.thread.is_alive()
    assert not svc.session.resources.handles


def test_active_control_page_and_operations(tmp_path,monkeypatch):
    pytest.importorskip('fastapi')
    from fastapi.testclient import TestClient
    from morla_core.plugins.registry import Catalog,default_catalog
    from morla_core.composition import load_document
    create_app=pytest.importorskip('morla_web_manager.composable_api').create_app
    from morla_cr5_driver.vr_session import attach_control_page
    from morla_input.adapters.normalized import InputStream
    def guarded_factory(context):
        controller=create_controller(context)
        controller.requires_fresh_input=True
        return controller
    class Entries(list):
        def select(self,**kwargs):return Entries(e for e in self if all(getattr(e,k)==v for k,v in kwargs.items()))
    entry=SimpleNamespace(group='morla.controllers',name='morla.cr5.cartesian_delta',load=lambda:guarded_factory)
    monkeypatch.setattr('importlib.metadata.entry_points',lambda **kw:Entries([entry]).select(**kw))
    package=Path(__file__).parents[1];root=Path(__file__).parents[5]
    app=create_app(tmp_path/'store',resource_root=tmp_path/'locks',mount_web=False)
    svc=app.state.composable
    clock=[1_000_000_000];svc.clock=lambda:clock[0]
    svc.catalog=Catalog(default_catalog().list()+[
        json.loads((package/'descriptor.json').read_text()),
        json.loads((package/'controller.json').read_text()),
        json.loads((root/'src/common/morla_input/quest_right_descriptor.json').read_text())])
    a=load_document(package/'config/quest_right.fake.yaml')
    saved=svc.save({'assembly':a,'expected_revision':0})
    state=svc.start({'assembly_id':a['assembly_id'],'revision':1,
        'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'start'})
    sid=state['session_id'];attach_control_page(app,svc,sid)
    with TestClient(app) as client:
        assert '右侧 VR 遥操' in client.get('/').text
        assert client.get('/cr5-vr/session').json()['session_id']==sid
        for op in ('prepare','activate','disarm','stop'):
            r=client.post(f'/api/v2/sessions/{sid}/operations',json={'operation':op,'request_id':op})
            assert r.status_code==200,r.text
            if op=='activate':
                controller=svc._binding_controller(svc.session.resolved['bindings'][0]).registry.plugins['morla.cr5.cartesian_delta']
                generation=controller.generation
                assert client.post(f'/api/v2/sessions/{sid}/operations',
                    json={'operation':op,'request_id':op}).status_code==200
                assert controller.generation==generation
                clock[0]+=300_000_000
                svc.tick()
                assert svc.session.status=='ACTIVE'
                assert svc.session.adapters['arm'].active
        assert svc.session.status=='STOPPED'
    assert not svc.session.resources.handles


def test_readonly_probe_and_real_backend_controller():
    from morla_cr5_driver.kinematics_check import probe
    model=FakeSolver()
    class Transport:
        def read(self):return {'values':[.1,.2,.3,.1,-.2,.4]}
        def forward_kinematics(self,q):return pose_to_controller(model.fk(q))
        def inverse_kinematics(self,p,q):return np.rad2deg(model.solve(pose_from_controller(p)))
    result=probe(Transport())
    assert result['passed'] and not result['motion_commands_sent']
    assert not result['physical_base_tcp_verified']
    controller=create_controller({'instance':{'plugin_id':'morla.dobot.cr5','backend':'real'},
                                  'binding':{'parameters':{'calibration_verified':False}}})
    try:assert controller.requires_fresh_input
    finally:controller.close()


@pytest.mark.parametrize('parameters',[
    {'max_joint_velocity':True},
    {'position_scale':float('nan')}, {'smoothing_alpha':0},
    {'input_rotation':[[1,0,0],[0,1,0],[0,0,-1]]}, {'unrecognized':1},
])
def test_controller_parameters_fail_before_worker_or_enable(parameters):
    with pytest.raises(ValueError):
        create_controller({'instance':{'plugin_id':'morla.dobot.cr5','backend':'fake'},
                           'binding':{'parameters':parameters}})


def test_ik_failure_discards_queued_work_and_latches_until_reset():
    entered=threading.Event();finish=threading.Event()
    class Failing(FakeSolver):
        def solve(self,target):
            entered.set();finish.wait(.2)
            raise ValueError('unreachable')
    c=AsyncCartesianController(Failing())
    try:
        c(pose(),[0]*6,.02,{});wait_result(c)
        c(pose(.001),[0]*6,.02,{})
        assert entered.wait(.2)
        c(pose(),[0]*6,.02,{})  # A queued reachable target must not erase a failed solve.
        finish.set();wait_result(c,lambda:c.error is not None)
        assert c.error and c.pending is None and c.result is None
        with pytest.raises(RuntimeError,match='unreachable'):c(pose(),[0]*6,.02,{})
        c.reset();c(pose(),[0]*6,.02,{});wait_result(c)
        assert c.error is None
    finally:finish.set();c.close()
