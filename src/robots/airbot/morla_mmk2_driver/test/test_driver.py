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
        self.goal_delay = 0.
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
        if self.goal_delay:
            time.sleep(self.goal_delay)
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
        # Rate limiting keeps the direction and never overshoots the target.
        # The magnitude comes from the real send interval, not 1/rate_hz.
        position=list(goal.goal.joint_state.position)
        assert 0 < position[0] <= .02
        assert position[1:]==pytest.approx([-position[0]/2,0,0,0,0])
        assert goal.param.trajectory.max_velocity_scaling_factor==.05
        s.stop();count=len(service.requests);time.sleep(.1)
        assert len(service.requests)==count
        assert not s.active and not s.pending and not s.desired
        assert service.requests[-1].goals_map['base'].goal.twist3d.x==0
    finally:s.close()


def test_releasing_one_sdk_arm_clears_its_target_only(endpoint):
    service, _ = endpoint
    session = session_for(endpoint, True)
    try:
        session.start()
        for key in ('left_arm', 'right_arm'):
            session.activate(key)
            session.submit(key, command(session.rows[key], [.05, 0, 0, 0, 0, 0]))
        wait(lambda: any('left_arm' in r.goals_map and 'right_arm' in r.goals_map
                         for r in service.requests))
        session.release_target('left_arm')
        with session.io:
            count = len(service.requests)
            assert 'left_arm' not in session.pending and 'left_arm' not in session.desired
        wait(lambda: len(service.requests) >= count + 2)
        assert all('left_arm' not in r.goals_map and 'right_arm' in r.goals_map
                   for r in service.requests[count:])
        session.submit('left_arm', command(session.rows['left_arm'], [.02, 0, 0, 0, 0, 0]))
        wait(lambda: any('left_arm' in r.goals_map for r in service.requests[count:]))
    finally:
        session.close()


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


def test_rejected_goal_names_the_sdk_targets_and_dispatch_telemetry(endpoint):
    service,_=endpoint;s=session_for(endpoint,True)
    try:
        s.start();s.activate('left_arm')
        assert s.snapshot('left_arm')['dispatch']=={'writes':0,'rate_hz':None,'goal_rtt_s':None}
        service.reject=True
        s.submit('left_arm',command(s.rows['left_arm'],[.02,0,0,0,0,0]))
        wait(lambda:s.fault)
        reason=s.snapshot('left_arm')['reason']
        # A rejected request names neither the component nor the target on its
        # own; the message must carry the exact SDK positions it sent.
        assert 'MMK2_GOAL_REJECTED:1:fixture' in reason and 'sdk_targets=left_arm=[' in reason
        dispatch=s.snapshot('left_arm')['dispatch']
        assert dispatch['writes']>=1 and dispatch['goal_rtt_s'] is not None
    finally:
        service.reject=False;s.close()


def test_start_pose_tolerates_small_excursion_but_not_a_wrong_mapping():
    # A joint resting a few degrees past a soft limit must not lock every command
    # path (teleop arms through the same check); a gross excursion still fails.
    _,rows=resolve(True);s=DeviceSession(rows);row=s.rows['left_arm']
    pose=[0.]*6
    s._check_start_pose('left_arm',row,pose)
    assert 'left_arm' not in s.start_outside
    pose[4]=1.83                                    # joint5 declared +-1.74
    s._check_start_pose('left_arm',row,pose)
    assert s.start_outside['left_arm']==[('left_arm_joint5',1.83,-1.74,1.74)]
    s._check_start_pose('left_arm',row,[0.]*6)      # back inside clears it
    assert 'left_arm' not in s.start_outside
    pose[4]=2.2                                     # >10% of the 3.48 rad range
    with pytest.raises(RuntimeError,match='MMK2_START_POSE_OUTSIDE_LIMITS'):
        s._check_start_pose('left_arm',row,pose)


def test_command_may_only_move_toward_the_range_when_measured_is_outside():
    # Recovery path: a joint resting past its limit must accept commands that
    # bring it back, and nothing else.
    _,rows=resolve(True);row=rows['left_arm'];names=[j['name'] for j in row['joints']]
    outside=[0.]*6;outside[4]=1.83                      # joint5 declared +-1.74
    def command(q):return {'names':names,'mode':'position','units':'rad','values':q}
    assert validate_target(row,command([0,0,0,0,1.78,0]),outside)[4]==1.78
    assert validate_target(row,command(outside),outside)[4]==1.83
    with pytest.raises(ValueError,match='MMK2_SDK_TARGET_OUTSIDE_LIMITS'):
        validate_target(row,command([0,0,0,0,1.90,0]),outside)   # further out
    with pytest.raises(ValueError,match='MMK2_SDK_TARGET_OUTSIDE_LIMITS'):
        validate_target(row,command([0,0,0,0,1.78,0]))           # no measurement
    with pytest.raises(ValueError,match='MMK2_SDK_TARGET_OUTSIDE_LIMITS'):
        validate_target(row,command([0,0,0,0,1.78,0]),[0.]*6)    # measured inside


def test_rpc_deadline_and_close_on_start_failure(endpoint):
    service,_=endpoint;service.delay=.4;s=session_for(endpoint)
    started=time.monotonic()
    with pytest.raises(grpc.RpcError):s.start()
    assert time.monotonic()-started<.5 and s.closed


def test_goal_rpc_timeout_reports_call_and_targets(endpoint):
    from morla_mmk2_driver.transport import Transport
    _, rows = resolve()
    service, port = endpoint
    service.goal_delay = .15
    transport = Transport({**rows['left_arm']['parameters'], 'host': '127.0.0.1',
                           'port': port, 'command_rpc_timeout_s': .03})
    try:
        with pytest.raises(RuntimeError, match=r'MMK2_SET_GOAL_RPC: DEADLINE_EXCEEDED; components=left_arm,right_arm; interface=trajectory; elapsed_ms=.*; deadline_s=0.03'):
            transport.write({'left_arm': [0.]*6, 'right_arm': [0.]*6}, rows)
    finally:
        transport.close()


def test_slow_goal_does_not_reuse_read_rpc_deadline(endpoint):
    from morla_mmk2_driver.transport import Transport
    _, rows = resolve()
    service, port = endpoint
    service.goal_delay = .15
    # rpc_timeout_s only bounds open/state/image calls; a set_goal that takes
    # longer than it must still succeed under the command deadline.
    transport = Transport({**rows['left_arm']['parameters'], 'host': '127.0.0.1',
                           'port': port, 'rpc_timeout_s': .03})
    try:
        assert transport.command_rpc_timeout_s == 3.
        transport.write({'left_arm': [0.]*6}, rows)
        assert transport.last_goal_rtt_s >= .14
    finally:
        transport.close()


def test_session_tolerates_controller_goal_latency(endpoint):
    service, _ = endpoint
    service.goal_delay = .9
    s = session_for(endpoint, True)
    try:
        s.start(); s.activate('left_arm')
        s.submit('left_arm', command(s.rows['left_arm'], [.01, 0, 0, 0, 0, 0]))
        wait(lambda: service.requests, seconds=3)
        time.sleep(1.2)
        assert not s.fault and s.snapshot('left_arm')['ready']
    finally:
        service.goal_delay = 0.
        s.close()


def test_idle_base_does_not_ride_along_every_request(endpoint):
    """A resting base must not cost ~100 ms on every request.

    Measured on the controller (2026-11-13, zero-displacement probe): one
    request with 6 components takes ~56 ms (17.7 Hz), the same request plus a
    zero base takes ~161 ms (6.2 Hz). The controller zeroes a base that hears
    nothing for 250 ms, so only the transition to zero needs sending.
    """
    service,_=endpoint;s=session_for(endpoint,True)
    try:
        s.start();s.activate('left_arm');s.activate('base')
        for _ in range(20):
            s.submit('base',command(s.rows['base'],[0.,0.]))
            s.submit('left_arm',command(s.rows['left_arm'],[0.,0.,0.,0.,0.,0.]))
            time.sleep(.02)
        wait(lambda:len(service.requests)>=6,seconds=3)
        calls=list(service.requests)
        assert len([r for r in calls if 'base' in r.goals_map])<=1,[list(r.goals_map) for r in calls]
        assert len([r for r in calls if 'left_arm' in r.goals_map])>=6
    finally:s.close()


def test_dispatch_probe_records_write_rate_and_step(endpoint,monkeypatch):
    """What the arm receives each write is the real teleop smoothness limit.

    The ik.* probe samples feedback at 10 Hz, so it cannot show either the
    delivered rate or the per-write step; the driver records both itself.
    """
    from morla_mmk2_driver import session as session_module
    records=[]
    monkeypatch.setattr(session_module,'_cadence',lambda prefix,**kw:records.append((prefix,kw)))
    s=session_for(endpoint,True)
    try:
        s.start();s.activate('left_arm');s.activate('right_arm')
        for i in range(10):
            s.submit('left_arm',command(s.rows['left_arm'],[.01*i,0,0,0,0,0]))
            s.submit('right_arm',command(s.rows['right_arm'],[.01*i,0,0,0,0,0]))
            time.sleep(.03)
        wait(lambda:len([r for r in records if r[1].get('event')])>=3,seconds=3)
        assert all(prefix=='dispatch' for prefix,_ in records)
        assert any('write' in kw.get('counts',()) for _,kw in records)
        event=next(kw['event'] for _,kw in records if kw.get('event'))
        assert set(event['arms'])<= {'left_arm','right_arm'} and event['arms']
        assert all(set(a)=={'q','step','clip'} for a in event['arms'].values())
        assert all(len(a['q'])==6 for a in event['arms'].values())
        rtt=[dict(kw['timings'])['goal_rtt'] for _,kw in records if kw.get('timings')]
        assert rtt and all(value>0 for value in rtt)
    finally:s.close()


def test_worker_rate_limits_by_real_elapsed_not_nominal_period(endpoint):
    """set_goal takes ~1 s on the real controller, so the worker sends ~20x
    slower than rate_hz.  Stepping by max_velocity * nominal period then
    commanded ~0.015 rad/s for arms configured at 0.35 rad/s (measured in the
    2026-11-13 work-position run: 0.54 rad in 35 s).  The step must use the
    real time between sends.
    """
    service, _ = endpoint
    service.goal_delay = .4
    s = session_for(endpoint, True)
    try:
        s.start(); s.activate('left_arm')
        s.submit('left_arm', command(s.rows['left_arm'], [.1, 0, 0, 0, 0, 0]))
        wait(lambda: len(service.requests) >= 2, seconds=3)
        goals = [r.goals_map['left_arm'].goal.joint_state.position[0] for r in service.requests]
        # The old code advanced max_velocity * 0.05 (the nominal period) per
        # send; one real 0.4 s interval must advance at least most of that.
        speed = s.rows['left_arm']['parameters']['max_velocity']
        assert goals[1] > speed * .4 * .8, goals
    finally:
        service.goal_delay = 0.
        s.close()


@pytest.mark.parametrize('value',[0,.01,11,float('nan'),True,'3'])
def test_invalid_command_rpc_timeout_is_rejected(value):
    _,rows=resolve();rows['left_arm']['parameters']['command_rpc_timeout_s']=value
    with pytest.raises(ValueError,match='command_rpc_timeout_s'):DeviceSession(rows)


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
    bad=copy.deepcopy(state);bad.joint_state.error_code.extend([2]*len(bad.joint_state.name))
    with pytest.raises(ValueError):decode(bad,rows['left_arm'])


def test_compact_firmware_errors_skip_slide_without_shifting_joint_codes():
    from morla_mmk2_driver.feedback import decode
    _, rows = resolve()
    fake = FakeTransport({'rate_hz': 20})
    try:
        state = next(fake.states())
    finally:
        fake.close()
    joint = state.joint_state
    joint.name.remove('slide_joint')
    joint.name.insert(5, 'slide_joint')
    reported = {'left_arm_joint1': 1, 'left_arm_joint2': -1, 'right_arm_joint6': -2}
    joint.error_code.extend([reported.get(name, 0) for name in joint.name if name != 'slide_joint'])

    _, values, ports = decode(state, rows['left_arm'])
    assert values == [0.] * 6
    assert ports['position']['joint_error_codes']['left_arm_joint1'] == 1
    assert ports['position']['joint_error_codes']['left_arm_joint2'] == -1
    with pytest.raises(ValueError, match='right_arm: right_arm_joint6=-2'):
        decode(state, rows['right_arm'])
    assert decode(state, rows['lift'])[2]['position']['joint_error_codes'] == {'slide_joint': None}

    joint.name[5] = 'unexpected_joint'
    with pytest.raises(ValueError, match='MMK2_JOINT_ERROR_LAYOUT: name=17 error_code=16'):
        decode(state, rows['left_arm'])

    joint.name[5] = 'slide_joint'
    joint.error_code.insert(5, 7)
    with pytest.raises(ValueError, match='lift: slide_joint=7'):
        decode(state, rows['lift'])


def test_command_boundary_and_mapping():
    _,rows=resolve();row=rows['base'];cmd=command(row,[.01,.02])
    for change in [{'units':'rad/s'},{'mode':'position'},{'names':['left_wheel_joint','right_wheel_joint']},{'values':[True,0]},{'values':[float('nan'),0]},{'values':[.16,0]}]:
        with pytest.raises(ValueError):validate_target(row,dict(cmd,**change))
    row=rows['left_gripper'];row['parameters']['joint_offsets']=[.9]
    with pytest.raises(ValueError,match='MAPPED'):validate_target(row,command(row,[0.]))
    assert validate_target(rows['left_arm'],command(rows['left_arm'],[0]*6))==[0]*6


def test_lift_ab_jog_uses_live_levels_and_holds_on_neutral():
    from morla_mmk2_driver.controllers import create_lift_jog
    _, rows = resolve()
    jog = create_lift_jog({'instance': rows['lift']})
    q = [.2]
    parameters = {'speed': .01}
    assert jog.reanchor([1, 0], q, parameters)['acknowledged']
    assert jog([1, 0], q, .05, parameters) is None
    assert jog([0, 0], q, .05, parameters) is None
    assert jog([1, 0], q, .05, parameters) == pytest.approx([.1995])
    assert jog([1, 0], q, .05, parameters) == pytest.approx([.199])
    assert jog([0, 0], q, .05, parameters) == q
    assert jog([0, 1], q, .05, parameters) == pytest.approx([.2005])
    assert jog([1, 1], q, .05, parameters) == q
    assert jog([0, 1], [.87], .05, parameters) == [.87]
    assert jog([1, 0], [-.04], .05, parameters) == [-.04]


def test_lift_button_binding_releases_pending_target_when_buttons_go_stale(monkeypatch):
    from types import SimpleNamespace
    from morla_input.adapters.normalized import InputFrame
    from morla_mmk2_driver.controllers import create_lift_jog
    from morla_teleop import retargeting
    from morla_teleop.engine import BindingEngine

    _, rows = resolve()
    binding = {'id': 'lift_ab', 'source': 'quest.right_ab', 'target': 'lift.position_target',
               'controller': 'morla.mmk2_sdk.lift_button_jog', 'max_age_ms': 100}
    monkeypatch.setattr(retargeting, 'install_controller_plugin',
                        lambda registry, name, context: registry.register(name, create_lift_jog(context)))
    released = []
    now = 1_000_000_000
    rt = SimpleNamespace(
        clock=lambda: now, instances={'lift': rows['lift']}, resolved={'bindings': [binding]},
        adapters={'lift': SimpleNamespace(health=lambda _: {'values': [.2]},
                                         release_target=lambda: released.append(True))},
    )
    engine = BindingEngine(rt)

    def frame(sequence, buttons, stamp):
        return InputFrame('quest', 'epoch', sequence, now, 'right', {'right_ab': buttons},
                          port_metadata={'right_ab': {'received_ns': stamp}})

    assert engine.targets(frame(1, [0, 0], now)) == []
    now += 20_000_000
    assert engine.targets(frame(2, [1, 0], now))[0]['values'] == pytest.approx([.1998])
    stale = now
    now += 110_000_000
    assert engine.targets(frame(3, [1, 0], stale)) == []
    assert released == [True]
    assert engine.targets(frame(4, [1, 0], now)) == []


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


def test_web_work_position_sends_only_selected_sdk_arm(tmp_path):
    from fastapi.testclient import TestClient
    from morla_web_manager.composable_api import create_app

    with TestClient(create_app(tmp_path/'store', resource_root=tmp_path/'locks')) as client:
        assembly = client.get('/api/v2/templates/mmk2-sdk-fake').json()
        assembly.setdefault('metadata', {})['operator_control'] = {
            'work_positions': {'left_arm': [.04, 0., 0., 0., 0., 0.]}, 'work_speed': .4}
        saved = client.post('/api/v2/assemblies', json={'assembly': assembly, 'expected_revision': 0})
        assert saved.status_code == 200, saved.text
        started = client.post('/api/v2/sessions', json={
            'assembly_id': assembly['assembly_id'], 'revision': 1,
            'assembly_hash': saved.json()['resolved']['assembly_hash'], 'request_id': 'start'})
        assert started.status_code == 200, started.text
        service = client.app.state.composable
        runtime = service.session
        sid = started.json()['session_id']
        response = client.post(f'/api/v2/sessions/{sid}/operator/action',
                               json={'action': 'workpos', 'request_id': 'work'})
        assert response.status_code == 200, response.text
        assert service.operator.work['duration_ns'] == pytest.approx(400_000_000, abs=1)
        def finished():
            service.tick()
            return service.operator.work['status'] != 'RUNNING'
        wait(finished)
        assert service.operator.work['status'] == 'SUCCEEDED', service.operator.work
        calls = runtime.adapters['left_arm'].session.transport.calls
        assert calls and all(set(call) == {'left_arm'} for call in calls if call)


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
