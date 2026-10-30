import pytest
from morla_input.adapters import MobileAdapter, GamepadAdapter, InputError
from morla_teleop.controllers import BindingController, ControllerError


def test_mobile_replay_old_epoch_and_invalid_values():
    clock = [10]
    source = MobileAdapter("phone", clock=lambda: clock[0]); epoch = source.reconnect()
    frame = source.packet(dict(epoch=epoch, sequence=0, scalar=.5, hold_to_control=False))
    assert source.read() == frame
    with pytest.raises(InputError): source.packet(dict(epoch=epoch, sequence=0, hold_to_control=True))
    source.reconnect()
    with pytest.raises(InputError): source.packet(dict(epoch=epoch, sequence=1, hold_to_control=True))
    with pytest.raises(InputError): source.packet(dict(epoch=source.epoch, sequence=1, scalar=float('nan'), hold_to_control=True))
    clock[0] += 300_000_000
    assert source.read() is None


def test_binding_dispatches_without_deadman_and_expires():
    clock=[10]; source=MobileAdapter("phone", clock=lambda:clock[0]); source.reconnect()
    binding=BindingController(dict(source="phone.scalar", target="grip.closure", deadman="phone.hold_to_control", controller="morla.scalar_map"), ["g"], clock=lambda:clock[0])
    def frame(seq, down): return source.packet(dict(epoch=source.epoch, sequence=seq, scalar=.7, hold_to_control=down))
    assert binding.step(frame(0,True),[.2]) == [.7]
    assert binding.step(frame(1,False),[.2]) == [.7]
    assert binding.step(frame(2,True),[.2]) == [.7]
    clock[0]+=300_000_000
    assert binding.step(source.latest,[.2]) is None
    source.reconnect()
    assert binding.step(frame(0,True),[.2]) == [.7]


def test_gamepad_layout_deadzone_and_disconnect():
    pad=GamepadAdapter("pad"); pad.reconnect()
    f=pad.joy([.01,.5,0,0,0,0], [0]*15)
    assert f.ports["axes"]["left_x"] == 0
    assert f.ports["axes"]["left_y"] == .5
    with pytest.raises(InputError): pad.joy([0]*8,[0]*15)
    pad.disconnect(); assert pad.read() is None
