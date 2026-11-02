"""Overwrite/drop tests use only the last ROS-sized sample, never a FIFO."""
import json
from types import SimpleNamespace
import pytest
from morla_input.quest_events import QuestEventSource
from morla_input.quest_snapshot import QuestSnapshotPublisher, MAX_COMMANDS
from morla_input.adapters.normalized import InputError
from morla_input.ros_sources import RosInputManager

POSE = [0.,0.,0.,0.,0.,0.,1.]


def hand_points(closed=False):
    points = [[0.,0.,0.] for _ in range(21)]
    points[17] = [.08,0.,0.]; points[8] = [.005 if closed else .08,0.,0.]
    return points


class Wire:
    def __init__(self, sides=('left','right')):
        self.now = 1_000_000_000; self.epoch = 'first'; self.publisher = QuestSnapshotPublisher()
        self.source = QuestEventSource('quest',clock=lambda:self.now,wall_clock=lambda:self.now,sides=sides)
    def update(self, kind='pose', values=None, side='right', source='controller', advance=10_000_000):
        self.now += advance
        return self.publisher.update(epoch=self.epoch,side=side,source=source,kind=kind,
            values=POSE if values is None else values,frame_id='hand_'+side if kind=='landmarks' else 'world',
            sent_ns=self.now,basis='unity_left_handed_v1')
    def state(self, grip=0., buttons=(), side='right', **kwargs):
        return self.update('controller_state',{'trigger':.4,'grip':grip,'axes':[.2,-.3],
            'buttons':[int(i in buttons) for i in range(6)]},side=side,**kwargs)
    def receive(self, row):return self.source.event(row)
    def start(self):
        self.receive(self.update()); self.receive(self.state())


def test_depth_one_keeps_pose_even_when_every_pose_only_packet_is_overwritten():
    w=Wire();frames=[]
    for i in range(200):
        w.update(values=[i/1000,0.,0.,0.,0.,0.,1.])  # dropped by depth=1
        last=w.state();frames.append(w.receive(last))
    assert len(frames)==200 and all('right_pose' in f.ports for f in frames)
    assert frames[-1].ports['right_pose']['position'][0]==.199
    assert frames[-1].ports['axes']=={'x':.2,'y':-.3}
    assert w.now-frames[-1].port_metadata['right_pose']['received_ns']==10_000_000


def test_first_snapshot_with_state_adopts_mode_without_needing_pose_packet():
    w=Wire();w.update();frame=w.receive(w.state(grip=1.,buttons=(0,)))
    assert frame.ports['right_hold'] and frame.ports['right_pose']
    assert not w.source.actions  # no startup operations from preexisting held buttons


def test_carry_forward_never_renews_field_age_or_repeats_its_sample_identity():
    w=Wire();w.start();stamp=w.source.latest.port_metadata['right_pose']['received_ns']
    sample=w.source.latest.port_metadata['right_pose']['sample_sequence']
    for _ in range(10):
        f=w.receive(w.state());assert f.port_metadata['right_pose']['received_ns']==stamp
        assert f.port_metadata['right_pose']['sample_sequence']==sample
    w.now+=300_000_000;f=w.receive(w.state())
    assert 'right_pose' not in f.ports and f.ports['right_hold'] is False


def test_one_clock_pair_is_used_for_all_fields_in_a_snapshot():
    w=Wire();w.update();row=w.state();calls=[]
    def wall():calls.append(1);return w.now
    w.source.wall_clock=wall
    f=w.receive(row)
    assert len(calls)==1 and 'right_pose' in f.ports and 'right_hold' in f.ports


def test_short_press_release_survives_overwrite_and_is_dispatched_once():
    w=Wire();w.start();w.state(buttons=(0,));released=w.state()
    frame=w.receive(released)
    assert frame.ports['right_buttons'][0]==0 and w.source.actions==['record_or_release']
    ids=list(w.source.action_ids);assert ids
    w.receive(w.state());assert not w.source.actions
    ack=w.source.acknowledgement();ack['consumer']='test'
    assert w.publisher.acknowledge(ack,w.now)
    # ACK is per-consumer diagnostic, not global destructive removal.
    assert w.publisher.states['right']['commands']


def test_two_equal_operations_have_distinct_ids_and_order_is_preserved():
    w=Wire();w.start()
    for buttons in [(0,),(),(1,),(),(0,),()]:last=w.state(buttons=buttons)
    w.receive(last)
    assert w.source.actions==['record_or_release','record_stop','record_or_release']
    assert len(set(w.source.action_ids))==3


def test_old_operations_expire_and_receipt_loss_does_not_reexecute_or_block_pose():
    w=Wire();w.start();pressed=w.state(grip=1.);w.receive(pressed)
    assert w.source.actions==['teleop_start']
    # No ACK is sent. Movement still advances; repeated journal is deduplicated.
    f=w.receive(w.update(values=[.1,0.,0.,0.,0.,0.,1.]))
    assert not w.source.actions and f.ports['right_pose']['position'][0]==.1
    w.state();w.state(buttons=(1,));w.now+=300_000_000
    w.receive(w.update());assert not w.source.actions


def test_expired_command_in_fresh_envelope_is_consumed_without_execution():
    w=Wire();w.start();command=w.state(buttons=(0,))['commands'][0]
    w.now+=300_000_000;row=w.update();row['commands']=[command]
    w.receive(row);assert not w.source.actions
    assert w.source.acknowledgement()['command_sequence']==command['id']


def test_reconnect_and_new_epoch_do_not_replay_old_operations_or_held_grip():
    w=Wire();w.start();pressed=w.state(grip=1.)
    w.source.disconnect();w.receive(pressed);assert not w.source.actions
    w.receive(w.state(grip=1.));assert not w.source.actions
    w.receive(w.state());w.receive(w.state(grip=1.));assert w.source.actions==['teleop_start']
    w.epoch='second';w.receive(w.update());w.receive(w.state(grip=1.));assert not w.source.actions
    with pytest.raises(InputError):w.receive(pressed)


def test_reconnection_baselines_both_sides_even_when_other_hand_arrives_later():
    w=Wire();w.start();w.receive(w.update(side='left'));w.receive(w.state(side='left'))
    left_press=w.state(side='left',buttons=(0,))
    w.source.disconnect();w.receive(w.update());w.receive(left_press)
    assert not w.source.actions
    w.receive(w.state(side='left'));w.receive(w.state(side='left',buttons=(0,)))
    assert w.source.actions==['workpos']


def test_hand_mode_clears_controller_state_and_old_generation_cannot_return():
    w=Wire();w.start();old=w.state(grip=1.)
    w.receive(w.update(source='hand'))
    assert not any(k in w.source.latest.ports for k in ('right_hold','right_buttons','scalar'))
    f=w.receive(w.update('landmarks',hand_points(True),source='hand'))
    assert f.ports['right_hold'] and len(f.ports['right_landmarks'])==21 and not w.source.actions
    assert w.state(grip=1.) is None  # late controller data is not a mode switch
    with pytest.raises(InputError):w.receive(old)
    w.receive(w.update(source='controller'));w.receive(w.state(grip=1.));assert not w.source.actions


def test_landmarks_only_packets_carry_wrist_and_do_not_renew_old_wrist():
    w=Wire();w.update(source='hand')
    f=w.receive(w.update('landmarks',hand_points(),source='hand'))
    stamp=f.port_metadata['right_pose']['received_ns'];assert f.ports['right_pose']
    f=w.receive(w.update('landmarks',hand_points(True),source='hand'))
    assert f.port_metadata['right_pose']['received_ns']==stamp
    w.now+=300_000_000
    f=w.receive(w.update('landmarks',hand_points(),source='hand'))
    assert 'right_pose' not in f.ports and 'right_landmarks' in f.ports


def test_other_controller_operations_available_when_motion_selects_one_hand():
    w=Wire(sides=('left',));w.receive(w.update(side='left'))
    w.receive(w.update());w.receive(w.state());f=w.receive(w.state(grip=1.))
    assert 'left_pose' in f.ports and 'right_pose' not in f.ports
    assert w.source.actions==['teleop_start'] and f.ports['right_hold']
    assert 'right_axes' not in f.ports


def test_each_hand_has_own_pose_clock_and_generation():
    w=Wire();w.start();w.receive(w.update(side='left'))
    left_stamp=w.source.latest.port_metadata['left_pose']['received_ns']
    f=w.receive(w.state());assert f.port_metadata['left_pose']['received_ns']==left_stamp
    w.receive(w.update(source='hand'))
    f=w.receive(w.state(side='left'))
    assert 'left_pose' in f.ports and w.source.mode['left']=='controller' and w.source.mode['right']=='hand'


def test_envelope_and_field_validation_fences_old_versions_replays_and_partial_corruption():
    w=Wire();row=w.update();w.receive(row)
    with pytest.raises(InputError):w.receive(row)
    for mutate in [lambda r:r.update(version=2),lambda r:r.update(sent_ns=w.now+1),
                   lambda r:r['fields']['pose'].update(sent_ns=w.now+1),
                   lambda r:r['fields']['pose'].update(values=[0]*7),
                   lambda r:r.update(commands=[{'id':1,'sent_ns':w.now,'action':'estop'}])]:
        row=w.update();mutate(row)
        with pytest.raises(InputError):w.receive(row)


def test_journal_is_bounded_under_burst_and_acknowledgements_are_epoch_scoped():
    w=Wire();w.start()
    for _ in range(50):w.state(buttons=(0,),advance=1);row=w.state(advance=1)
    assert len(row['commands'])==MAX_COMMANDS and row['commands_dropped']==50-MAX_COMMANDS
    w.receive(row);ack=w.source.acknowledgement();ack['consumer']='one'
    assert w.publisher.acknowledge(ack,w.now)
    assert not w.publisher.acknowledge({**ack,'epoch':'old'},w.now)
    w.update(source='hand');assert not w.publisher.acknowledge(ack,w.now)


def test_real_input_manager_receives_snapshot_and_dispatches_numbered_short_press():
    w=Wire();frames=[];operations=[]
    manager=RosInputManager([],frames.append,lambda _:None,clock=lambda:w.now,
        on_event=lambda iid,frame,actions:operations.extend(actions))
    manager.sources['quest']=w.source;manager._guarded=lambda iid,fn:fn()
    def receive(row):manager._quest_event('quest',SimpleNamespace(data=json.dumps(row)))
    w.update();receive(w.state());w.state(buttons=(0,));receive(w.state());receive(w.update())
    assert operations==['record_or_release'] and all('right_pose' in f.ports for f in frames)


def test_receipt_publish_failure_does_not_disconnect_or_block_frame(monkeypatch):
    import sys
    from types import ModuleType
    msg=ModuleType('std_msgs.msg');msg.String=lambda **kw:SimpleNamespace(**kw)
    monkeypatch.setitem(sys.modules,'std_msgs.msg',msg)
    w=Wire();frames=[];disconnected=[]
    manager=RosInputManager([],frames.append,disconnected.append,clock=lambda:w.now)
    manager.sources['quest']=w.source
    manager.quest_ack_publishers[('quest','right')]=SimpleNamespace(publish=lambda _:(_ for _ in ()).throw(OSError('DDS unavailable')))
    manager._quest_event('quest',SimpleNamespace(data=json.dumps(w.update())))
    assert len(frames)==1 and not disconnected and manager.quest_ack_errors==1


def test_builtin_subscription_uses_only_depth_one_snapshots_and_ack_topics(monkeypatch):
    import sys
    from types import ModuleType
    for name,classes in [('sensor_msgs.msg',['Joy']),('geometry_msgs.msg',['PoseArray']),('std_msgs.msg',['String'])]:
        module=ModuleType(name)
        for cls in classes:setattr(module,cls,type(cls,(),{}))
        monkeypatch.setitem(sys.modules,name,module)
    qos=ModuleType('rclpy.qos')
    qos.QoSProfile=lambda **kw:SimpleNamespace(**kw)
    qos.ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort')
    qos.HistoryPolicy=SimpleNamespace(KEEP_LAST='keep_last');qos.qos_profile_sensor_data=object()
    monkeypatch.setitem(sys.modules,'rclpy.qos',qos)
    subscriptions=[];publishers=[]
    manager=RosInputManager([],lambda _:None,lambda _:None)
    manager.node=SimpleNamespace(create_subscription=lambda cls,topic,cb,q:subscriptions.append((topic,q)),
        create_publisher=lambda cls,topic,q:publishers.append((topic,q)))
    from morla_input.builtin_sources import subscribe
    subscribe(manager,{'instance_id':'q','plugin_id':'morla.input.quest','parameters':{'side':'right'}})
    assert [topic for topic,q in subscriptions]==['/quest3/left_teleop_events','/quest3/right_teleop_events']
    assert [topic for topic,q in publishers]==[t+'/ack' for t,q in subscriptions]
    assert all(q.depth==1 and q.reliability=='best_effort' for t,q in subscriptions+publishers)
