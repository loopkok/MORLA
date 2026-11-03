import json
from pathlib import Path
from threading import Event
from types import SimpleNamespace
import time

import pytest
from fastapi.testclient import TestClient
from morla_web_manager.composable_api import create_app
from morla_input.quest_events import QuestEventSource
from morla_input.quest_snapshot import QuestSnapshotPublisher
from morla_input.ros_sources import RosInputManager


@pytest.fixture
def cell(tmp_path,request):
    clock=[1_000_000_000]
    with TestClient(create_app(tmp_path/'store',resource_root=tmp_path/'locks')) as client:
        svc=client.app.state.composable;svc.clock=lambda:clock[0]
        assembly={'assembly_version':1,'assembly_id':'operators','revision':1,
            'metadata':{'operator_control':{'work_positions':{'grip':[.4]},'work_speed':.2}},
            'instances':{'grip':{'plugin':'morla.astral.gripper','version':'1.0.0'},
                         'quest':{'plugin':'morla.input.quest','version':'1.0.0'}},
            'bindings':[{'id':'grip_control','source':'quest.scalar','target':'grip.closure',
                         'deadman':'quest.right_hold','controller':'morla.scalar_map','parameters':{}}],
            'capture':{'state':['grip.position'],'action':['grip.closure'],'images':{}}}
        if getattr(request,'param',None)=='dual':
            import copy
            assembly['instances']['grip2']=copy.deepcopy(assembly['instances']['grip'])
            assembly['bindings'].append({**copy.deepcopy(assembly['bindings'][0]),'id':'second','target':'grip2.closure'})
            assembly['capture']['state'].append('grip2.position')
            assembly['capture']['action'].append('grip2.closure')
        if getattr(request,'param',None)=='continuous_empty':
            assembly['bindings'][0]['deadman']=''
        if getattr(request,'param',None)=='continuous_missing':
            assembly['bindings'][0].pop('deadman')
        saved=svc.save({'assembly':assembly,'expected_revision':0})
        state=svc.start({'assembly_id':'operators','revision':1,'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'launch'})
        sid=state['session_id']
        source=QuestEventSource('quest',clock=svc.clock,wall_clock=svc.clock,sides=('left','right'))
        manager=RosInputManager([],lambda frame:svc.ingest_frame(sid,frame),svc.disconnect_input,
            clock=svc.clock,on_event=lambda iid,frame,actions:svc.input_event(sid,iid,frame,actions))
        manager._guarded=lambda iid,fn:fn()
        manager.sources['quest']=source;svc.inputs['quest']=source
        seq=[0];request=[0];publisher=QuestSnapshotPublisher()
        def wait():
            for _ in range(200):
                if svc.operator.handoff.pending is None:return
                svc.tick();time.sleep(.001)
            raise AssertionError('handoff did not settle')
        def send(kind='controller_state',side='right',buttons=(),grip=0.,trigger=0.,settle=True):
            clock[0]+=10_000_000;seq[0]+=1
            values=([0,0,0,0,0,0,1] if kind=='pose' else
                    {'trigger':trigger,'grip':grip,'axes':[0.,0.],'buttons':[int(i in buttons) for i in range(6)]})
            row=publisher.update(epoch='first',side=side,source='controller',kind=kind,
                values=values,frame_id='quest',basis='unity_left_handed_v1',sent_ns=clock[0])
            manager._quest_event('quest',SimpleNamespace(data=json.dumps(row)))
            if settle:wait()
        def action(name):
            request[0]+=1
            response=client.post(f'/api/v2/sessions/{sid}/operator/action',json={'action':name,'request_id':f'key-{request[0]}'})
            wait()
            return response
        send('pose');send();send('pose',side='left');send(side='left')
        yield SimpleNamespace(client=client,svc=svc,rt=svc.session,sid=sid,clock=clock,send=send,
                              action=action,wait=wait,manager=manager,publisher=publisher,seq=seq)


def test_quest_full_record_save_discard_and_work_position(cell):
    c=cell
    c.send(grip=1.)
    assert c.rt.status=='ACTIVE' and c.svc.operator.activity=='human'
    c.send(grip=1.,trigger=.8)
    c.send(grip=1.,buttons=[0],trigger=.8)
    assert c.svc.recorder is not None
    saved=c.svc.recorder.episode_dir
    c.send(grip=1.,trigger=.8)
    c.send(grip=1.,buttons=[1],trigger=.8)
    assert c.svc.recorder is None and (saved/'meta.json').is_file()
    c.send(grip=1.)
    c.send(grip=1.,buttons=[0])
    discarded=c.svc.recorder.episode_dir
    c.send(grip=1.)
    c.send(grip=1.,buttons=[2])
    assert c.svc.recorder is None and not discarded.exists() and saved.exists()
    c.send(side='left',buttons=[0])
    assert c.svc.operator.activity=='workpos'
    for _ in range(60):
        c.clock[0]+=100_000_000;c.svc.tick()
    assert c.svc.operator.work['status']=='SUCCEEDED'
    assert c.rt.status=='READY_DISARMED'
    assert c.rt.adapters['grip'].values==pytest.approx([.4])


def test_keyboard_hold_expiry_and_registered_actions(cell):
    c=cell
    assert c.action('teleop_start').status_code==200
    # Keyboard S can open an existing hold binding, without holding physical grip.
    c.send(trigger=.7)
    assert c.rt.adapters['grip'].values==[0.]
    url=f'/api/v2/sessions/{c.sid}/operator/hold'
    assert c.client.post(url,json={'client_id':'keyboard-1','held':True,'sequence':1}).status_code==200
    c.send(trigger=.7)
    assert c.rt.adapters['grip'].values==pytest.approx([.7])
    c.clock[0]+=400_000_000
    c.send(trigger=.2)
    assert c.rt.adapters['grip'].values==pytest.approx([.7])
    assert c.action('record_or_release').status_code==200
    directory=c.svc.recorder.episode_dir
    assert c.action('workpos').status_code==422
    assert c.svc.recorder is not None and c.svc.operator.activity=='human'
    assert c.action('record_discard').json()['status']=='discarded'
    assert not directory.exists()
    assert c.action('old_keyboard_shortcut').status_code==422
    assert c.client.post(url,json={'client_id':'keyboard-1','held':False,'sequence':3}).json()['accepted']
    assert c.client.post(url,json={'client_id':'keyboard-1','held':True,'sequence':2}).json()['reason']=='stale_hold'
    assert not c.svc.operator.held()


def test_playback_takeover_and_release_preserve_cursor_and_exclude_manual_writes(cell):
    c=cell
    response=c.client.post(f'/api/v2/sessions/{c.sid}/replay/start',json={
        'frozen_resolved':c.rt.resolved,'actions':[[.1],[.2],[.3]],'hz':10})
    assert response.status_code==200,response.text
    c.svc.tick()
    assert c.rt.adapters['grip'].values==[.1]
    c.send(trigger=.9)
    assert c.rt.adapters['grip'].values==[.1]
    c.send(grip=1.,trigger=.8)
    assert c.svc.operator.return_to=='playback'
    assert c.svc.operator.playback['index']==1
    c.send(grip=1.,trigger=.8)
    assert c.rt.adapters['grip'].values==pytest.approx([.8])
    c.clock[0]+=500_000_000;c.svc.tick()
    assert c.rt.adapters['grip'].values==pytest.approx([.8])
    c.send(grip=1.,trigger=.8)  # Fresh neutral buttons; keep the current manual target.
    c.send(grip=1.,buttons=[0])
    assert c.svc.recorder is None and c.svc.operator.activity=='playback'
    c.svc.tick()
    assert c.rt.adapters['grip'].values==pytest.approx([.8])
    c.clock[0]+=100_000_000;c.svc.tick()
    assert c.rt.adapters['grip'].values==pytest.approx([.9])
    assert c.rt.status=='READY_DISARMED'


def test_policy_takeover_drops_inflight_prediction_and_release_reobserves(cell):
    from morla_policy_inference.composable import RuntimePolicy
    from morla_learning.contracts import ModelManifest,digest
    c=cell
    c.action('teleop_start')
    features=c.rt.resolved['feature_contract']
    manifest=ModelManifest(1,'act','revision',digest(features),c.rt.resolved['geometry_hash'],
        'norm','checkpoint','dataset',features,{},32,'inference_verified')
    unblock=Event();started=Event();calls=[]
    def predict(obs):
        calls.append(obs['state'].copy())
        if len(calls)==1:started.set();unblock.wait(3)
        return [.9 if len(calls)==1 else .25]
    policy=RuntimePolicy(c.rt,predict,manifest,normalization_hash='norm')
    c.svc.policy=policy;c.svc.operator.activity='policy';c.svc.operator.gate='policy';c.svc.operator.paused=False
    policy.acquire();policy.request();assert started.wait(1)
    try:
        c.send(grip=1.,trigger=.6)
        assert policy.suspended and policy.pending is None and not policy.actions
        assert c.svc.operator.return_to=='policy'
        c.send(grip=1.,trigger=.6)
        unblock.set();time.sleep(.01)
        c.svc.tick()
        assert c.rt.adapters['grip'].values==pytest.approx([.6])
        c.send(grip=1.,buttons=[0])
        assert not policy.suspended and c.svc.operator.activity=='policy' and c.svc.recorder is None
        for _ in range(50):
            c.clock[0]+=10_000_000;c.svc.tick();time.sleep(.001)
            if len(calls)>1 and c.rt.adapters['grip'].values==[.25]:break
        assert len(calls)>1 and calls[1][0]==pytest.approx(.6)
        assert c.rt.adapters['grip'].values==pytest.approx([.25])
    finally:unblock.set()


def test_operator_action_idempotence_and_invalid_work_positions(cell):
    c=cell
    body={'action':'record_or_release','request_id':'same-key'}
    url=f'/api/v2/sessions/{c.sid}/operator/action'
    first=c.client.post(url,json=body)
    assert first.status_code==200
    assert c.client.post(url,json=body).json()==first.json()
    assert c.client.post(url,json={**body,'action':'record_discard'}).status_code==422
    c.action('record_discard')
    config=c.rt.resolved['source_assembly']['metadata']['operator_control']
    config['work_positions']['grip']=[99.]
    calls=list(c.rt.adapters['grip'].calls)
    assert c.action('workpos').status_code==422
    assert c.rt.adapters['grip'].calls==calls
    config['work_positions']={}
    assert c.action('workpos').status_code==422


def test_work_position_feedback_loss_stops_and_scheduler_stall_does_not_jump(cell):
    c=cell
    assert c.action('workpos').status_code==200
    c.svc.tick()
    c.clock[0]+=2_000_000_000;c.svc.tick()
    assert c.rt.adapters['grip'].values[0] <= .02+1e-9  # dt capped to 100 ms at .2 units/s.
    adapter=c.rt.adapters['grip'];health=adapter.health
    adapter.health=lambda now:{**health(now),'sample_ns':now-1_000_000_000}
    c.clock[0]+=100_000_000;c.svc.tick()
    assert c.svc.operator.work['status']=='FAILED'
    assert c.rt.status=='READY_DISARMED' and c.svc.operator.error
    assert not adapter.active


def test_pause_cancels_work_position_and_log_has_operator_records(cell):
    c=cell
    c.action('workpos');c.svc.tick()
    assert c.action('teleop_pause').status_code==200
    values=list(c.rt.adapters['grip'].values)
    for _ in range(3):c.clock[0]+=100_000_000;c.svc.tick()
    assert c.svc.operator.work['status']=='CANCELLED' and c.rt.adapters['grip'].values==values
    c.svc.operation(c.sid,{'operation':'stop','request_id':'stop-test'})
    records=[]
    for path in c.rt.diagnostics.directory.glob('trace-*.jsonl'):
        records += [json.loads(line) for line in path.read_text().splitlines()]
    assert any(r['kind']=='operator_request' for r in records)
    assert any(r['kind']=='work_position' for r in records)


def test_legacy_vader_defaults_upgrade_without_rewriting_custom_maps():
    from morla_web_manager.operator_defaults import upgrade,VADER_ACTIONS
    a={'instances':{'pad':{'plugin':'morla.input.vader5','parameters':{'deadman_button':'M1',
        'action_buttons':{'Z':'teleop_start','C':'teleop_pause','Y':'record_start','X':'record_stop'}}}}}
    migrated=upgrade(a)
    assert migrated['instances']['pad']['parameters']['action_buttons']==VADER_ACTIONS
    assert migrated['instances']['pad']['parameters']['deadman_button']=='M3'
    assert a['instances']['pad']['parameters']['deadman_button']=='M1'
    a['instances']['pad']['parameters']['action_buttons']={'M4':'teleop_start'}
    assert upgrade(a)==a


def playback(c, actions=None):
    response=c.client.post(f'/api/v2/sessions/{c.sid}/replay/start',json={
        'frozen_resolved':c.rt.resolved,'actions':actions or [[.1],[.2],[.3]],'hz':10})
    assert response.status_code==200,response.text
    c.svc.tick()


def policy_for(c, predictor):
    from morla_policy_inference.composable import RuntimePolicy
    from morla_learning.contracts import ModelManifest,digest
    c.action('teleop_start')
    features=c.rt.resolved['feature_contract']
    manifest=ModelManifest(1,'act','revision',digest(features),c.rt.resolved['geometry_hash'],
        'norm','checkpoint','dataset',features,{},32,'inference_verified')
    runner=RuntimePolicy(c.rt,predictor,manifest,normalization_hash='norm')
    c.svc.policy=runner;c.svc.operator.activity='policy';c.svc.operator.paused=False;c.svc.operator.gate='policy'
    runner.acquire()
    return runner


def test_partial_anchor_failure_rolls_back_without_committing_human(cell,monkeypatch):
    import copy
    from morla_runtime.adapters import FakeAdapter
    from morla_teleop.controllers import BindingController
    c=cell
    instance=copy.deepcopy(c.rt.instances['grip']);instance['instance_id']='other'
    c.rt.instances['other']=instance
    adapter=FakeAdapter(instance);adapter.configure({'session_id':c.sid});c.rt.adapters['other']=adapter
    binding=copy.deepcopy(c.rt.resolved['bindings'][0]);binding.update(id='other_binding',target='other.closure')
    c.rt.resolved['bindings'].append(binding)
    playback(c)
    previous=copy.deepcopy(c.rt.lease);original=BindingController.reanchor
    waiting=Event();unblock=Event()
    def reanchor(self,*args):
        if self.binding['id']=='other_binding':
            waiting.set();unblock.wait(2);raise RuntimeError('second controller refused anchor')
        return original(self,*args)
    monkeypatch.setattr(BindingController,'reanchor',reanchor)
    try:
        c.send(grip=1.,trigger=.7,settle=False)
        assert waiting.wait(1)
        for _ in range(30):
            c.svc.tick()
            if c.svc.operator.handoff.state()['acks']:break
            time.sleep(.001)
        assert c.svc.operator.activity=='playback' and c.svc.operator.return_to is None
        assert c.svc.operator.handoff.pending is not None
        assert c.rt.adapters['grip'].values==[.1]
        command={'producer':'playback','token':previous['token'],'epoch':previous['epoch'],'sequence':999,
                 'commands':[{'component_id':'grip','values':[.9],'names':['joint1'],'mode':'position'}]}
        assert not c.rt.submit(command)['accepted']
        unblock.set();c.wait()
        assert c.svc.operator.activity=='playback' and not c.svc.operator.paused
        assert c.svc.operator.handoff.state()['status']=='FAILED'
        assert c.rt.lease['epoch']!=previous['epoch']
        resumed=list(c.rt.adapters['grip'].values)
        assert not c.rt.submit(command)['accepted']  # Old commands stay fenced after rollback.
        assert c.rt.adapters['grip'].values==resumed
    finally:unblock.set()


def test_anchor_timeout_and_late_success_cannot_regain_authority(cell,monkeypatch):
    from morla_teleop.controllers import BindingController
    c=cell;playback(c)
    original=BindingController.reanchor;entered=Event();unblock=Event()
    def delayed(self,*args):
        entered.set();unblock.wait(2);return original(self,*args)
    monkeypatch.setattr(BindingController,'reanchor',delayed)
    try:
        c.send(grip=1.,settle=False);assert entered.wait(1)
        c.clock[0]+=3_100_000_000;c.svc.operator.tick()
        assert c.svc.operator.handoff.pending is None
        assert c.svc.operator.handoff.state()['status']=='FAILED'
        assert c.svc.operator.activity=='playback' and c.svc.operator.gate=='playback'
        bindings=c.svc.bindings
        unblock.set();time.sleep(.02)
        c.svc.operator.tick()
        assert c.svc.operator.activity!='human' and c.svc.bindings is bindings
    finally:unblock.set()


def test_reanchor_anchors_on_input_current_after_controller_build(cell,monkeypatch):
    """CR5 controller construction costs ~1 s; the anchor must not go stale behind it."""
    from morla_teleop.engine import BindingEngine
    c=cell
    original=BindingEngine._binding_controller
    def slow(self,binding):
        c.clock[0]+=400_000_000
        c.send('controller_state',settle=False)
        return original(self,binding)
    monkeypatch.setattr(BindingEngine,'_binding_controller',slow)
    c.send(grip=1.)
    assert c.svc.operator.handoff.state()['status']=='SUCCEEDED'
    assert c.svc.operator.activity=='human'
    stuck=c.svc.operator.handoff.state()['acks']['grip_control']
    assert stuck['acknowledged'] is True


@pytest.mark.parametrize('cell',['continuous_empty','continuous_missing'],indirect=True)
def test_continuous_follow_releases_grip_without_gating_and_pause_still_stops(cell):
    c=cell
    assert c.rt.resolved['bindings'][0].get('deadman') in (None,'')
    c.send(grip=1.);assert c.svc.operator.activity=='human'
    c.send(grip=0.,trigger=.7)
    assert c.rt.adapters['grip'].values==pytest.approx([.7])
    c.action('teleop_pause');c.send(grip=0.,trigger=.9)
    assert c.rt.adapters['grip'].values==pytest.approx([.7])


def test_explicit_hold_binding_stops_on_release(cell):
    c=cell;c.send(grip=1.);c.send(grip=1.,trigger=.6)
    assert c.rt.adapters['grip'].values==pytest.approx([.6])
    c.send(grip=0.,trigger=.9)
    assert c.rt.adapters['grip'].values==pytest.approx([.6])


@pytest.mark.parametrize('cell',['dual'],indirect=True)
def test_all_controllers_are_built_before_any_mapping_is_anchored(cell,monkeypatch):
    from morla_teleop.engine import BindingEngine
    from morla_teleop.controllers import BindingController
    c=cell;built=Event();anchors=[]
    original_build=BindingEngine._binding_controller;original_anchor=BindingController.reanchor
    def build(self,binding):
        result=original_build(self,binding)
        if binding['id']=='second':
            assert built.wait(1)
            c.clock[0]+=400_000_000
            c.send('pose',settle=False);c.send(grip=1.,settle=False)
        else:built.set()
        return result
    def anchor(self,frame,measured):
        anchors.append((self.binding['id'],c.clock[0]))
        assert c.clock[0]>=1_400_000_000
        return original_anchor(self,frame,measured)
    monkeypatch.setattr(BindingEngine,'_binding_controller',build)
    monkeypatch.setattr(BindingController,'reanchor',anchor)
    c.send(grip=1.)
    assert c.svc.operator.handoff.state()['status']=='SUCCEEDED'
    assert len(anchors)==2


@pytest.mark.parametrize('refresh_pose',[False,True])
def test_bound_pose_after_build_requires_its_own_fresh_sample(cell,monkeypatch,refresh_pose):
    from morla_teleop.engine import BindingEngine
    from morla_teleop.controllers import BindingController
    c=cell;original_build=BindingEngine._binding_controller
    # This regression uses the real bound-port freshness check on right_pose,
    # independently of the scalar mapping used to keep the fixture hardware-free.
    original_anchor=BindingController.reanchor
    def build(self,binding):
        c.clock[0]+=400_000_000;c.send(settle=False)
        if refresh_pose:c.send('pose',settle=False)
        return original_build(self,binding)
    def anchor(self,frame,measured):
        probe=BindingController({**self.binding,'source':'quest.right_pose'},self.names,clock=c.svc.clock)
        if not probe.input_fresh(frame):raise ValueError('bound pose expired')
        return original_anchor(self,frame,measured)
    monkeypatch.setattr(BindingEngine,'_binding_controller',build)
    monkeypatch.setattr(BindingController,'reanchor',anchor)
    c.send(grip=1.)
    if refresh_pose:
        assert c.svc.operator.handoff.state()['status']=='SUCCEEDED'
    else:
        assert c.svc.operator.handoff.state()['status']=='FAILED'
        assert 'bound pose expired' in c.svc.operator.handoff.state()['error']


def test_pause_during_takeover_cancels_late_anchor(cell,monkeypatch):
    from morla_teleop.controllers import BindingController
    c=cell;playback(c)
    original=BindingController.reanchor;entered=Event();unblock=Event()
    def delayed(self,*args):entered.set();unblock.wait(2);return original(self,*args)
    monkeypatch.setattr(BindingController,'reanchor',delayed)
    try:
        c.send(grip=1.,settle=False);assert entered.wait(1)
        assert c.action('teleop_pause').status_code==200
        assert c.svc.operator.activity=='playback' and c.svc.operator.paused
        assert c.svc.operator.handoff.state()['status']=='CANCELLED'
        unblock.set();time.sleep(.02);c.svc.tick()
        assert c.rt.status=='READY_DISARMED' and c.svc.operator.gate is None
    finally:unblock.set()


def test_paused_policy_takeover_release_stays_paused_until_explicit_resume(cell):
    c=cell;observations=[]
    runner=policy_for(c,lambda obs:(observations.append(obs['state'].copy()),[.25])[1])
    c.action('teleop_pause')
    assert not runner.closed and runner.suspended and c.svc.operator.activity=='policy'
    c.send(grip=1.)
    assert c.svc.operator.activity=='human' and c.svc.operator.interrupted['paused']
    c.send(grip=1.,trigger=.6)
    # Re-centering in HUMAN must not overwrite the interrupted pause state.
    c.send(grip=0.);c.send(grip=1.,trigger=.6)
    assert c.svc.operator.interrupted['paused']
    c.send(grip=1.,buttons=[0],trigger=.6)
    assert c.svc.operator.activity=='policy' and c.svc.operator.paused
    assert c.rt.status=='READY_DISARMED' and observations==[]
    before=list(c.rt.adapters['grip'].values)
    c.clock[0]+=100_000_000;c.svc.tick()
    assert c.rt.adapters['grip'].values==before
    assert c.action('resume').status_code==200
    assert c.svc.operator.activity=='policy' and not c.svc.operator.paused
    assert observations and observations[0][0]==pytest.approx(.6)
    c.svc.tick()
    assert c.rt.adapters['grip'].values==pytest.approx([.25])


def test_release_prediction_failure_keeps_human_and_return_context(cell):
    c=cell
    def broken(obs):raise RuntimeError('model resume refused')
    runner=policy_for(c,broken)
    c.send(grip=1.);c.send(grip=1.,trigger=.6)
    bindings=c.svc.bindings
    c.send(grip=1.,buttons=[0],trigger=.6)
    assert c.svc.operator.handoff.state()['status']=='FAILED'
    assert c.svc.operator.activity=='human' and c.svc.operator.return_to=='policy'
    assert c.svc.operator.gate=='human_mobile' and runner.suspended and not runner.closed
    assert c.svc.bindings is bindings
    c.send(grip=1.,trigger=.65)
    assert c.rt.adapters['grip'].values==pytest.approx([.65])


def test_offset_limit_failure_preserves_human_and_original_playback(cell):
    c=cell;playback(c,[[.1],[.2],[.9]])
    c.send(grip=1.);c.send(grip=1.,trigger=.8)
    c.send(grip=1.,buttons=[0],trigger=.8)
    assert c.svc.operator.handoff.state()['status']=='FAILED'
    assert c.svc.operator.activity=='human' and c.svc.operator.return_to=='playback'
    assert c.svc.operator.playback['offset']==[0.] and c.svc.operator.playback['index']==1
    assert c.rt.adapters['grip'].values==pytest.approx([.8])


def test_guard_rejects_cross_owner_or_retired_epoch_without_stopping_current_owner(cell):
    c=cell;c.action('teleop_start');c.send(grip=1.);c.send(grip=1.,trigger=.4)
    lease=dict(c.rt.lease)
    command={'producer':'human_mobile','token':lease['token'],'epoch':lease['epoch'],'sequence':800,
             'commands':[{'component_id':'grip','names':['joint1'],'values':[.9],'mode':'position'}]}
    assert not c.rt.submit({**command,'producer':'policy'})['accepted']
    assert not c.rt.submit({**command,'epoch':lease['epoch']-1})['accepted']
    assert c.rt.status=='ACTIVE' and c.rt.adapters['grip'].values==pytest.approx([.4])
    assert c.rt.submit(command)['accepted']


def test_paused_playback_release_restores_pause_and_reanchors_on_resume(cell):
    c=cell;playback(c)
    c.action('teleop_pause')
    assert c.svc.operator.playback['index']==1 and c.svc.operator.paused
    c.send(grip=1.);c.send(grip=1.,trigger=.7)
    c.send(grip=1.,buttons=[0],trigger=.7)
    assert c.svc.operator.activity=='playback' and c.svc.operator.paused
    assert c.svc.operator.playback['index']==1
    c.clock[0]+=200_000_000;c.svc.tick()
    assert c.rt.adapters['grip'].values==pytest.approx([.7])
    c.action('resume');c.svc.tick()
    assert c.rt.adapters['grip'].values==pytest.approx([.7])
    c.clock[0]+=100_000_000;c.svc.tick()
    assert c.rt.adapters['grip'].values==pytest.approx([.8])


def test_dispatch_only_device_enable_is_not_a_human_commit(cell,monkeypatch):
    c=cell
    adapter=c.rt.adapters['grip']
    adapter.backend='real'
    def pending_enable():
        adapter.active=True
        return {'accepted':True,'completed':False}
    monkeypatch.setattr(adapter,'activate',pending_enable)
    response=c.action('teleop_start')
    assert response.status_code==200
    assert c.svc.operator.handoff.state()['status']=='FAILED'
    assert c.svc.operator.activity!='human' and c.svc.operator.gate is None
    assert c.rt.status=='READY_DISARMED' and not adapter.active


def test_input_epoch_change_during_anchor_cannot_commit(cell,monkeypatch):
    from morla_teleop.controllers import BindingController
    c=cell;playback(c)
    original=BindingController.reanchor;entered=Event();unblock=Event()
    def delayed(self,*args):entered.set();unblock.wait(2);return original(self,*args)
    monkeypatch.setattr(BindingController,'reanchor',delayed)
    try:
        c.send(grip=1.,settle=False);assert entered.wait(1)
        c.svc.inputs['quest'].reconnect()
        unblock.set();c.wait()
        assert c.svc.operator.handoff.state()['status']=='FAILED'
        assert c.svc.operator.activity=='playback' and c.svc.operator.return_to is None
    finally:unblock.set()


def test_expired_permit_and_previous_runtime_cannot_bypass_workstation_fence(cell):
    c=cell;c.action('teleop_start');c.send(grip=1.);c.send(grip=1.,trigger=.4)
    lease=dict(c.rt.lease)
    def command(permit):
        return {'producer':'human_mobile','token':permit['token'],'epoch':permit['epoch'],'sequence':808,
                'commands':[{'component_id':'grip','names':['joint1'],'values':[.9],'mode':'position'}]}
    c.clock[0]+=600_000_000
    assert not c.rt.submit(command(lease))['accepted']
    assert c.rt.adapters['grip'].values==pytest.approx([.4])
    c.svc.operation(c.sid,{'operation':'stop','request_id':'stop-old'})
    saved=c.svc.store.get('operators',1)
    new=c.svc.start({'assembly_id':'operators','revision':1,
                     'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'new-run'})
    c.svc.operation(new['session_id'],{'operation':'activate','request_id':'new-enable'})
    permit=c.svc.session.acquire('human_mobile')
    assert not c.rt.submit(command(permit))['accepted']
    assert c.svc.session.adapters['grip'].values==[0.]
