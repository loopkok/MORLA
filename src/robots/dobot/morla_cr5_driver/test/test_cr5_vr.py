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
from morla_cr5_driver.vr_controller import AsyncCartesianController,create_controller,integration_seed


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


def test_integration_seed_follows_the_command_and_is_lead_bounded():
    measured=np.array([.1,0,0,0,0,0])
    np.testing.assert_allclose(integration_seed(measured,None,.25),measured)
    np.testing.assert_allclose(integration_seed(measured,measured+.1,.25),measured+.1)
    np.testing.assert_allclose(integration_seed(measured,measured+.9,.25),measured+.25)


def test_stale_feedback_does_not_reset_the_integration_base():
    # 30004 feedback arrives at ~25 Hz while the loop runs at ~90 Hz. The seed
    # must keep following the command instead of snapping back to the repeated
    # stale sample, which is what made the commanded joints staircase.
    c=AsyncCartesianController(FakeSolver())
    def wait_report(previous):
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            with c.condition:
                sample=c.report.get('sample_ns')
                if sample is not None and sample!=previous:return sample
            time.sleep(.001)
        raise AssertionError('IK worker report missing')
    try:
        previous=None
        for tick in range(30):
            # The first tick anchors at x=0; the hand then sweeps away from it.
            c(pose(.1*tick),[0]*6,.02,{'max_joint_velocity':1.})
            previous=wait_report(previous)
        np.testing.assert_allclose(c.report['feedback_seed'],[0]*6)
        np.testing.assert_allclose(c.report['command_seed'],[.3,0,0,0,0,0])
        assert c.last_output[0]>.3
    finally:c.close()


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


def test_quest_controller_hand_switch_and_loss_revoke_runtime(tmp_path,monkeypatch):
    from morla_core.plugins.registry import Catalog,default_catalog
    from morla_core.composition import load_document
    from morla_input.quest_events import QuestEventSource
    from morla_input.quest_snapshot import QuestSnapshotPublisher
    ComposableService=pytest.importorskip('morla_web_manager.composable_service').ComposableService
    package=Path(__file__).parents[1]
    descriptors=[json.loads((package/name).read_text()) for name in ('descriptor.json','controller.json')]
    installed={(d['plugin_id'],d['plugin_version']) for d in default_catalog().list()}
    # In a built workspace discover_descriptors() already finds these packages;
    # registering them twice is an ambiguous-plugin error, not a config failure.
    catalog=Catalog(default_catalog().list()+[d for d in descriptors if (d['plugin_id'],d['plugin_version']) not in installed])
    class Entries(list):
        def select(self,**kwargs):return Entries(e for e in self if all(getattr(e,k)==v for k,v in kwargs.items()))
    entry=SimpleNamespace(group='morla.controllers',name='morla.cr5.cartesian_delta',load=lambda:create_controller)
    monkeypatch.setattr('importlib.metadata.entry_points',lambda **kw:Entries([entry]).select(**kw))
    svc=ComposableService(tmp_path/'store',resource_root=tmp_path/'locks');svc.catalog=catalog
    a=load_document(package/'config/quest.fake.yaml')
    saved=svc.save({'assembly':a,'expected_revision':0})
    assert 'morla.cr5.cartesian_delta@1.0.0' in dict(saved['resolved']['lockfile']['plugins'])
    state=svc.start({'assembly_id':a['assembly_id'],'revision':1,'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'start'})
    sid=state['session_id'];op_count=[0];seq=[0]
    def op(name):
        op_count[0]+=1
        return svc.operation(sid,{'operation':name,'request_id':f'{name}-{op_count[0]}'})
    source=QuestEventSource('quest',sides=('right',),on_transition=lambda:svc.disconnect_input('quest'))
    svc.inputs['quest']=source
    publisher=QuestSnapshotPublisher()
    levels={'trigger':0.,'grip':0.,'axes':[0.,0.],'buttons':[0]*6}
    def send(kind,mode='controller',value=None):
        seq[0]+=1
        if kind in ('grip','trigger'):
            levels[kind]=value;kind='controller_state';value=levels
        row=publisher.update(epoch='test-stream',side='right',kind=kind,source=mode,
            values=value if value is not None else [0,0,0,0,0,0,1],
            sent_ns=time.time_ns(),frame_id='hand_right' if kind=='landmarks' else 'quest_world',
            basis='unity_left_handed_v1')
        frame=source.event(row)
        return svc.ingest_frame(sid,frame) if source.control_event(row,frame) else None
    def landmarks(closed):
        p=[[0.,0.,0.] for _ in range(21)];p[17]=[.08,0,0]
        p[8]=[.005 if closed else .08,0,0];return p
    worker=None
    try:
        # The fake backend starts at the all-zero pose, which is a CR5
        # singularity: a sub-millimetre lateral target there is only reachable
        # through a large null-space move, so J1's sign is solver-dependent.
        # Start from a typical folded pose, where the same target is a small
        # well-conditioned move.
        svc.session.adapters['arm'].values[:]=[0.,.6,-.6,.3,.5,0.]
        send('pose')  # Disarmed packets cannot pre-arm or anchor.
        op('prepare');op('activate')
        # Continuous follow: the shipped binding has no deadman, so the first
        # fresh pose after activation anchors at the current arm pose and
        # dispatches. Button snapshots carry the same pose sample, so the
        # binding deduplicates it rather than recomputing IK.
        assert send('pose')['accepted']
        assert send('grip',value=1.)['accepted'] is False
        assert send('grip',value=0.)['accepted'] is False
        assert send('pose')['accepted']
        worker=svc.bindings['right_vr'].registry.plugins['morla.cr5.cartesian_delta']
        wait_result(worker)
        report=svc.state(sid)['controllers']['right_vr']['morla.cr5.cartesian_delta']
        assert report['solver']=='cr5_urdf_pinocchio_differential'
        assert report['age_ms'] is not None
        # Keep advancing the pose: a stream gap larger than max_age re-anchors
        # the binding by design, and a constant pose would then command no
        # delta at all.
        # Unity +X is the operator's right, which is base +Y for this cell; the
        # side the arm first turns to depends on the solver, only the movement
        # itself is asserted here.
        deadline=time.monotonic()+1;offset=.002
        while abs(svc.session.adapters['arm'].values[0])<1e-4 and time.monotonic()<deadline:
            time.sleep(.01);offset+=.0002;send('pose',value=[offset,0,0,0,0,0,1])
        assert abs(svc.session.adapters['arm'].values[0])>1e-4
        before=list(svc.session.adapters['arm'].values)
        send('pose','hand',[.3,0,0,0,0,0,1])
        assert svc.session.lease is None and svc.session.status=='READY_DISARMED'
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
        # A >max_age input gap re-anchors the binding without revoking control:
        # the first fresh pose anchors at the current arm pose, so the gap
        # itself commands no jump and tracking resumes without re-activation.
        time.sleep(.27)
        held=list(svc.session.adapters['arm'].values)
        assert svc.session.status=='ACTIVE'
        assert 'accepted' in send('landmarks','hand',landmarks(True))
        assert send('pose','hand',[.301,0,0,0,0,0,1])['accepted']
        wait_result(worker)
        time.sleep(.1)
        np.testing.assert_allclose(svc.session.adapters['arm'].values,held,atol=1e-3)
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
    package=Path(__file__).parents[1]
    app=create_app(tmp_path/'store',resource_root=tmp_path/'locks',mount_web=False)
    svc=app.state.composable
    clock=[1_000_000_000];svc.clock=lambda:clock[0]
    descriptors=[json.loads((package/name).read_text()) for name in ('descriptor.json','controller.json')]
    installed={(d['plugin_id'],d['plugin_version']) for d in default_catalog().list()}
    # Same dedupe as above: a built workspace already discovers these manifests.
    svc.catalog=Catalog(default_catalog().list()+[d for d in descriptors if (d['plugin_id'],d['plugin_version']) not in installed])
    a=load_document(package/'config/quest.fake.yaml')
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
    # The alignment is now allowed to be a det -1 basis change, so an invalid
    # one has to fail on orthonormality rather than on its determinant sign.
    {'input_rotation':[[1,0,0],[0,1,0],[0,0,-2]]}, {'unrecognized':1},
])
def test_controller_parameters_fail_before_worker_or_enable(parameters):
    with pytest.raises(ValueError):
        create_controller({'instance':{'plugin_id':'morla.dobot.cr5','backend':'fake'},
                           'binding':{'parameters':parameters}})


def test_shipped_input_rotation_is_the_unity_to_base_alignment():
    """Quest sends raw Unity (X right, Y up, Z forward); CR5 solves in base_link.

    Nero declares the same step as ``input_rotation_to_robot_body`` and
    multiplies it by the ``t_bracket_base -> right_base_link`` URDF mount.
    Astral's mount is identity. CR5's ``base_link`` is the URDF root, so the
    mount factor is identity and the binding carries the whole Unity -> base
    step alone. Only a yaw is admissible: any roll/pitch would tip the
    operator's up axis away from base +Z, which no CR5 mounting can justify.
    """
    import yaml
    for name in ('config/quest.fake.yaml','config/quest.sim.yaml'):
        document=yaml.safe_load((Path(__file__).parents[1]/name).read_text())
        binding=document['bindings'][0]
        assert binding['input_basis_id']=='unity_left_handed_v1'
        rotation=np.asarray(binding['parameters']['input_rotation'],dtype=float)
        np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
        # Unity is left-handed, so a Unity -> right-handed-base map is a basis
        # change, never a proper rotation. The controller only applies it to
        # increments, so the reflection is safe.
        assert np.isclose(np.linalg.det(rotation),-1)
        # Operator on the base +X side facing the arm: right is base +Y, up is
        # base +Z, pushing forward moves the tool along base -X.
        np.testing.assert_allclose(rotation@[1,0,0],[0,1,0],atol=1e-12)
        np.testing.assert_allclose(rotation@[0,1,0],[0,0,1],atol=1e-12)
        np.testing.assert_allclose(rotation@[0,0,1],[-1,0,0],atol=1e-12)


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
