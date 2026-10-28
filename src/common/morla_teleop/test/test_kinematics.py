import numpy as np
import pytest
from dataclasses import replace
from morla_input.adapters import MobileAdapter
from morla_teleop.controllers import ControllerRegistry,BindingController,ControllerError
from morla_teleop.kinematics import CartesianController


class Solver:
    def sync_state(self,seed): self.seed=np.asarray(seed)
    def fk(self,seed):
        pose=np.eye(4);pose[:3,3]=seed[:3];return pose
    def solve(self,target):
        result=self.seed.copy();result[:3]=target[:3,3];return result


def test_nonidentity_tool_does_not_move_neutral_jog():
    tool=np.eye(4);tool[2,3]=.12
    controller=CartesianController(Solver(),jog=True)
    seed=[.1,.2,.3,0,0,0,0]
    target=controller({'x':0,'y':0},seed,.02,
        {'axes_map':{'x':'x','y':'y'},'solver_tip_from_tool':tool.tolist()})
    np.testing.assert_allclose(target,seed)


def test_reanchor_and_source_identity_and_input_gap():
    clock=[100];source=MobileAdapter('phone',clock=lambda:clock[0]);source.reconnect()
    binding={'source':'phone.scalar','target':'tool.closure','deadman':'phone.hold_to_control','controller':'morla.scalar_map'}
    controller=BindingController(binding,['g'],clock=lambda:clock[0])
    def frame(index,down):return source.packet({'epoch':source.epoch,'sequence':index,'hold_to_control':down,'scalar':.4})
    assert controller.step(frame(0,False),[0]) is None
    assert controller.step(frame(1,True),[0])==[.4]
    clock[0]+=300_000_000
    assert controller.step(frame(2,True),[0]) is None
    with pytest.raises(ControllerError):controller.step(replace(frame(3,False),source_id='other'),[0])


def test_per_port_frame_does_not_reanchor_on_unrelated_landmarks():
    clock=[100];source=MobileAdapter('phone',clock=lambda:clock[0]);source.reconnect()
    binding={'source':'phone.scalar','target':'tool.closure','deadman':'phone.hold_to_control','controller':'morla.scalar_map'}
    controller=BindingController(binding,['g'],clock=lambda:clock[0])
    def f(index,down,frame):
        x=source.packet({'epoch':source.epoch,'sequence':index,'hold_to_control':down,'scalar':.4})
        return replace(x,frame_id=frame,port_metadata={'scalar':{'frame_id':'control','received_ns':clock[0]}})
    assert controller.step(f(0,False,'hand_local'),[0]) is None
    assert controller.step(f(1,True,'body'),[0])==[.4]


def test_sdl_button_positions_are_not_legacy_quest_positions():
    from morla_input.adapters import GamepadAdapter
    pad=GamepadAdapter('pad');pad.reconnect();buttons=[0]*21;buttons[10]=1
    frame=pad.joy([0]*6,buttons)
    assert frame.ports['right_shoulder'] is True
    assert frame.ports['guide'] is False
