"""The work-pose selector controls command scope as well as arrival checks."""
import copy
import math
import time
from types import SimpleNamespace

import pytest

from test_piano_calibration import motion
from test_piano_lane import store


@pytest.mark.parametrize('side',['left','right'])
def test_selected_work_pose_never_commands_peer_or_hands(motion,side):
    import numpy as np
    from morla_web_manager.piano_calibration import WORK_POSE_DEG
    peer='right' if side=='left' else 'left'
    # Peer stays about 150 degrees away from a waypoint, reproducing the report.
    motion.backend.measured[peer+'_arm']=[-.02,1.57,-.51,.08,2.62,-.07,-.04]
    motion.backend.measured['left_hand'][0]=-.01
    motion.backend.measured['right_hand'][4]=-.02
    initial=motion.backend.snapshot()
    sent=[];original=motion.backend.send
    def send(targets):
        sent.append(copy.deepcopy(targets));original(targets)
    motion.backend.send=send
    motion.execute('work_pose',dict(side=side,calibration_confirmed=True))
    assert sent and all(set(q)=={side+'_arm'} for q in sent)
    assert motion.backend.measured[peer+'_arm']==initial[peer+'_arm']
    for hand in ('left_hand','right_hand'):assert motion.backend.measured[hand]==initial[hand]
    assert np.allclose(motion.backend.measured[side+'_arm'],np.deg2rad(WORK_POSE_DEG[side]))
    events=[e for e in motion.engine.events if e['event']=='work_pose_stage']
    assert [e['stage'] for e in events]==[1,2,3,4,5]
    assert all(e['side']==side for e in events)
    motion.hold_measured()
    assert set(sent[-1])=={side+'_arm'}


def test_switching_work_pose_side_then_prepare_restores_bilateral_scope(motion):
    for side in ('left','right'):
        motion.execute('work_pose',dict(side=side,calibration_confirmed=True))
        assert motion.backend.motion_keys=={side+'_arm'}
    motion.engine.prepare()
    assert motion.engine.state=='READY'
    assert motion.backend.motion_keys=={'left_arm','right_arm','left_hand','right_hand'}


def test_engine_blocks_targets_outside_selected_scope(motion):
    motion.backend.claim(keys=['left_arm'])
    motion.engine.targets={'right_arm':[0.]*7}
    with pytest.raises(RuntimeError,match='scope'):motion.engine._send()
    assert not motion.backend.history


@pytest.mark.parametrize('side',[None,'invalid',True])
def test_work_pose_requires_explicit_valid_side(motion,side):
    with pytest.raises(ValueError,match='请选择'):
        motion.execute('work_pose',dict(side=side,calibration_confirmed=True))
    assert not motion.backend.claimed and not motion.backend.history


def test_ros_claim_only_enables_and_publishes_selected_arm(store,monkeypatch):
    monkeypatch.syspath_prepend(str(store.project))
    from morla_piano.ros_backend import RosBackend
    b=RosBackend.__new__(RosBackend)
    b.c=store.config()['config'];b.c['calibrated']=True
    b.names={s+'_'+p:() for s in ('left','right') for p in ('arm','hand')}
    b.measured={s+'_arm':list(b.c['hands'][s]['initial_arm_rad']) for s in ('left','right')}
    b.claimed=False;b.motion_keys=None;b.publishers={};b.lease_pub=None;b.sequence=0;b.epoch='test'
    b.acks=set();b.baseline_rejections={};b.expected_backend='hardware'
    b.statuses={s+'_arm':dict(enabled=False,streaming_enabled=False) for s in ('left','right')}
    b.String=SimpleNamespace;b.JointState=SimpleNamespace;b.lease_qos=object()
    topics=[];calls=[];published=[]
    b.node=SimpleNamespace(create_publisher=lambda _type,topic,_qos:(topics.append(topic) or SimpleNamespace(publish=lambda m:published.append((topic,m)))),
                           destroy_publisher=lambda p:None)
    b._graph_check=lambda:None;b.health=lambda **kw:None;b.pump=lambda:None
    def call(side,verb):
        calls.append((side,verb));b.statuses[side+'_arm'].update(enabled=True,streaming_enabled=True,enable_status='enabled',enable_started_monotonic_s=time.monotonic())
    b._call=call
    b.claim(keys=['left_arm'])
    assert calls==[('left','enable')]
    assert set(b.publishers)=={'left_arm'}
    assert topics==['/policy_inference/state','/left_arm/joint_commands']
    with pytest.raises(RuntimeError,match='scope'):b.send({'right_arm':[0.]*7})
    assert all(topic=='/policy_inference/state' for topic,_ in published)
    # A cached true value must never skip the new enable request.
    b.statuses['right_arm'].update(enabled=True,streaming_enabled=True,enable_status='enabled',enable_started_monotonic_s=time.monotonic()-10)
    b.claim(keys=['right_arm'])
    assert calls==[('left','enable'),('right','enable')]
    assert set(b.publishers)=={'right_arm'}


def test_ros_health_ignores_unselected_position_and_enable_state(store,monkeypatch):
    monkeypatch.syspath_prepend(str(store.project))
    from morla_piano.ros_backend import RosBackend
    b=RosBackend.__new__(RosBackend)
    b.c=store.config()['config'];b.fault=None;b.motion_keys={'right_arm'};b.claimed=True
    b.names={'left_arm':(),'right_arm':(),'left_hand':(),'right_hand':()}
    b.input_errors={'left_arm/feedback':'peer bad feedback'}
    b.received={'right_arm':time.monotonic()};b.status_at=dict(b.received);b.expected_backend='hardware'
    b.statuses={'right_arm':dict(feedback_fresh=True,backend='hardware',connected=True,enabled=True,
        motors_enabled=True,motor_status_fresh=True,position_active=False),
        'left_arm':dict(position_active=True,enabled=False)}
    b.health()
