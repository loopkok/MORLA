"""Memory-only protocol tests: no robot, CAN socket, mode or enable calls."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nero_robot_control import sdk_adapter
from nero_robot_control.sdk_adapter import PyAgxArmAdapter


class LatchingController:
    """Model the observed stale-J7 outcome when J1..6 commit before J7."""
    def __init__(self):
        self.pending=[0]*7;self.actual=[0]*7;self.sent=[];self.calls=0
    def _send_msgs(self,messages):
        for message in messages:
            fields=vars(message)
            self.sent.append(tuple(fields))
            for field,value in fields.items():self.pending[int(field[-1])-1]=value
            if 'joint_6' in fields:self.actual=list(self.pending)
    def move_j(self,q):
        self.calls+=1
        mdeg=[round(math.degrees(v)*1000) for v in q]
        self._send_msgs([SimpleNamespace(joint_1=mdeg[0],joint_2=mdeg[1]),
                         SimpleNamespace(joint_3=mdeg[2],joint_4=mdeg[3]),
                         SimpleNamespace(joint_5=mdeg[4],joint_6=mdeg[5]),SimpleNamespace(joint_7=mdeg[6])])
    move_js=move_j


def adapter(first=False):
    a=PyAgxArmAdapter.__new__(PyAgxArmAdapter)
    a._robot=LatchingController();a.native_j7_first=first
    return a


@pytest.mark.parametrize('first',[False,True])
def test_four_stage_targets_and_stale_j7_controller_model(first,monkeypatch):
    delays=[];monkeypatch.setattr(sdk_adapter.time,'sleep',delays.append)
    a=adapter(first);old7=0
    for target7 in (-60.,0.,-70.,-10.):
        q=[.1,.2,.3,.4,.5,-.2,math.radians(target7)]
        expected=[round(math.degrees(v)*1000) for v in q]
        a.move_j(q)
        assert a._robot.actual[:6]==expected[:6]
        assert a._robot.actual[6]==(expected[6] if first else old7)
        assert '_send_msgs' not in vars(a._robot)
        old7=expected[6]
    assert a._robot.calls==4
    assert delays==([.005]*4 if first else [])
    expected_order=[('joint_7',),('joint_1','joint_2'),('joint_3','joint_4'),('joint_5','joint_6')]
    if not first:expected_order=expected_order[1:]+expected_order[:1]
    assert a._robot.sent==expected_order*4


def test_js_keeps_vendor_order_after_native_j(monkeypatch):
    monkeypatch.setattr(sdk_adapter.time,'sleep',lambda s:None)
    a=adapter(True);a.move_j([0.]*7);a._robot.sent.clear()
    a.move_js([0.]*7)
    assert a._robot.sent==[('joint_1','joint_2'),('joint_3','joint_4'),('joint_5','joint_6'),('joint_7',)]


@pytest.mark.parametrize('bad',['short','wrong_fields','clamped_target'])
def test_unknown_sdk_layout_or_modified_target_sends_no_joint_frame(bad):
    a=adapter(True)
    messages=[SimpleNamespace(joint_1=0,joint_2=0),SimpleNamespace(joint_3=0,joint_4=0),
              SimpleNamespace(joint_5=0,joint_6=0),SimpleNamespace(joint_7=0)]
    if bad=='short':messages.pop()
    elif bad=='wrong_fields':messages[-1]=SimpleNamespace(joint7=0)
    else:messages[-1].joint_7=1
    a._robot.move_j=lambda q:a._robot._send_msgs(messages)
    with pytest.raises(RuntimeError,match='no joint target sent'):a.move_j([0.]*7)
    assert not a._robot.sent and '_send_msgs' not in vars(a._robot)


def test_transport_exception_restores_sender_and_never_replays():
    a=adapter(True);attempts=[]
    def fail(messages):
        attempts.append(messages)
        raise RuntimeError('transport failed')
    a._robot._send_msgs=fail
    with pytest.raises(RuntimeError,match='transport failed'):a.move_j([0.]*7)
    assert a._robot._send_msgs is fail
    assert len(attempts)==1 and list(vars(attempts[0][0]))==['joint_7']


@pytest.mark.parametrize('first',[False,True])
def test_socketcan_echo_only_enabled_for_explicit_compatibility_flag(first,monkeypatch):
    seen=[]
    module=SimpleNamespace(AgxArmFactory=SimpleNamespace(create_arm=lambda cfg:SimpleNamespace()),
        ArmModel=SimpleNamespace(NERO='nero'),NeroFW=SimpleNamespace(DEFAULT='default',V111='v111'),
        create_agx_arm_config=lambda **kw:(seen.append(kw) or kw))
    monkeypatch.setitem(sys.modules,'pyAgxArm',module)
    a=PyAgxArmAdapter('test-no-device',native_j7_first=first)
    assert a.native_j7_first==first
    assert seen[0].get('local_loopback',False)==first
    assert seen[0].get('receive_own_messages',False) is False


def test_position_stream_uses_j_and_deduplicates_only_identical_encoded_targets(monkeypatch):
    monkeypatch.setattr(sdk_adapter.time,'sleep',lambda s:None)
    a=adapter(True);a.stream_position_mode='j'
    first=[0.]*7;first[3]=math.radians(72)
    a.move_js(first)
    assert a._robot.actual[3]==72000 and a._robot.calls==1
    for _ in range(100):a.move_js(first)
    assert a._robot.calls==1
    second=list(first);second[3]=math.radians(73)
    a.move_js(second)
    assert a._robot.actual[3]==73000 and a._robot.calls==2
    a._stream_goal_mdeg=None  # A new enable/mode epoch must send its first goal.
    a.move_js(second)
    assert a._robot.calls==3


def test_position_stream_failed_send_does_not_cache_success():
    a=adapter(True);a.stream_position_mode='j'
    calls=[]
    def fail(q):calls.append(q);raise RuntimeError('send failure')
    a._robot.move_j=fail
    for _ in range(2):
        with pytest.raises(RuntimeError,match='send failure'):a.move_js([0.]*7)
    assert len(calls)==2 and getattr(a,'_stream_goal_mdeg',None) is None


@pytest.mark.parametrize('mode',["js","j"])
def test_stream_configuration_never_enables_or_sends_a_target(mode):
    a=adapter();a.stream_position_mode=mode;a.stream_j_speed_percent=5
    calls=[]
    a._robot=SimpleNamespace(set_speed_percent=lambda v:calls.append(('speed',v)),
        set_motion_mode=lambda v:calls.append(('mode',v)),
        set_auto_set_motion_mode_enabled=lambda v:calls.append(('auto',v)))
    a._stream_goal_mdeg=(0,)*7
    a.configure_streaming()
    expected=[('speed',5),('mode','j'),('auto',False)] if mode=='j' else [('mode','js'),('auto',False)]
    assert calls==expected and a._stream_goal_mdeg is None
