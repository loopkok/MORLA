"""Explicit hand reference recording must not alter raw data or widen targets."""
import copy
import math

import pytest

from test_piano_lane import store


@pytest.fixture
def setup(store, monkeypatch):
    monkeypatch.syspath_prepend(str(store.project))
    from morla_piano.model import hand_command_seed
    cfg = store.config()['config']
    samples = {
        'left': [-.5, 59.0816, 30.7491, -1.4909, .0833, .5833, 2.0833, .25, 1.1666, -1.1666, 1., 1.1666],
        'right': [.1666, 59.3316, 28.9158, .0764, -1., .4166, 1.3333, -.75, 1.8333, -.75, 1.8333, -1.5],
    }
    for side, deg in samples.items():
        h=cfg['hands'][side]
        h['start_reference_rad']=[math.radians(v) for v in deg]
        h['hover_rad']=hand_command_seed(h['start_reference_rad'],h['hand_limits_rad'],h['start_reference_rad'])
    return cfg


def test_recorded_reference_roundtrips_without_certifying_or_widening(store, setup):
    before=store.config()
    saved=store.save_config(setup,before['revision'])['config']
    assert saved['calibrated'] is False
    for side in ('left','right'):
        assert saved['hands'][side]['start_reference_rad']==setup['hands'][side]['start_reference_rad']
        assert saved['hands'][side]['hand_limits_rad']==before['config']['hands'][side]['hand_limits_rad']
        assert saved['hands'][side]['initial_arm_rad']==before['config']['hands'][side]['initial_arm_rad']


def test_command_seed_is_separate_from_measurements_and_only_changes_boundary_excursions(setup):
    from morla_piano.ros_backend import RosBackend
    b=RosBackend.__new__(RosBackend)
    b.c=setup;b.names={s+'_'+p:() for s in ('left','right') for p in ('arm','hand')}
    measured={s+'_'+p:list(setup['hands'][s]['start_reference_rad' if p=='hand' else 'initial_arm_rad'])
              for s in ('left','right') for p in ('arm','hand')}
    original=copy.deepcopy(measured)
    targets=b._command_seed(measured)
    assert measured==original
    for key,q in targets.items():
        side,part=key.split('_')
        for raw,target,(lo,hi) in zip(original[key],q,setup['hands'][side][part+'_limits_rad']):
            assert lo<=target<=hi
            assert target==raw if lo<=raw<=hi else target==max(lo,min(hi,raw))
    del setup['hands']['left']['start_reference_rad']
    with pytest.raises(RuntimeError,match='未发送目标'):b._command_seed(measured)


@pytest.mark.parametrize('case', ['too_far','wrong_reference','nan','short_reference'])
def test_reference_never_permits_large_or_unmatched_excursions(setup,case):
    from morla_piano.model import hand_command_seed
    h=setup['hands']['left'];q=list(h['start_reference_rad']);ref=list(q)
    if case=='too_far':q[0]=-.05001
    elif case=='wrong_reference':ref[0]=1.
    elif case=='nan':q[0]=float('nan')
    else:ref.pop()
    with pytest.raises(ValueError):hand_command_seed(q,h['hand_limits_rad'],ref)


def test_unapproved_upper_boundary_stays_blocked(setup):
    from morla_piano.model import hand_command_seed
    h=setup['hands']['left'];q=list(h['hover_rad']);q[0]=h['hand_limits_rad'][0][1]+.01
    with pytest.raises(ValueError):hand_command_seed(q,h['hand_limits_rad'],h['start_reference_rad'])


def test_prepare_manual_motion_and_stop_only_send_strictly_legal_targets(setup):
    from morla_piano.engine import Engine,FakeBackend,VirtualClock
    from morla_piano.model import check_limits
    from morla_piano.ros_backend import RosBackend
    from morla_web_manager.piano_calibration import CalibrationMotion
    class Plant(FakeBackend):
        def __init__(self,c,clock):
            super().__init__(c,clock)
            self.names={key:() for key in self.measured}
            for side in ('left','right'):self.measured[side+'_hand']=list(c['hands'][side]['start_reference_rad'])
        def command_snapshot(self):return RosBackend._command_seed(self,self.snapshot())
        def claim(self, keys=None):
            self.motion_keys=frozenset(self.measured if keys is None else keys)
            RosBackend._command_seed(self,self.measured)
            super().claim(keys=keys)
        def send(self,targets):
            for key,q in targets.items():
                side,part=key.split('_');check_limits(q,self.c['hands'][side][part+'_limits_rad'],key)
            super().send(targets)
    clock=VirtualClock();b=Plant(setup,clock);e=Engine(setup,b,clock)
    # Recording/constructing a reference never produces a command.
    assert not b.history and b.measured['left_hand'][0]<0
    e.prepare()
    assert e.state=='READY'
    assert any(row['event']=='hand_boundary_seed' for row in e.events)
    # Repeat with original raw boundary drift for manual takeover and stopping.
    for side in ('left','right'):b.measured[side+'_hand']=list(setup['hands'][side]['start_reference_rad'])
    arms={s:list(b.measured[s+'_arm']) for s in ('left','right')}
    motion=CalibrationMotion(e,b,setup,software=True)
    motion.execute('joint',dict(side='left',part='hand',joint_index=4,target_deg=2.,calibration_confirmed=True))
    assert e.state=='CALIBRATION_IDLE'
    for side in ('left','right'):assert b.measured[side+'_arm']==arms[side]
    b.measured['left_hand'][0]=setup['hands']['left']['start_reference_rad'][0]
    motion.hold_measured()
    e._normal_stop()
    assert e.state=='STOPPED' and b.history
