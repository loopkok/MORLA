"""HOME coordinator contract, using a software plant and virtual time only."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from morla_piano.engine import Engine, FakeBackend, VirtualClock
from morla_piano.model import load_config
from morla_piano import model
from morla_web_manager import piano_calibration as module
from morla_web_manager.piano_calibration import CalibrationMotion
from nero_robot_control.home_trajectory import HOME_FINAL_POSE_DEG, home_targets
from nero_robot_control.waypoint_trajectory import TRAJECTORY_REVISION, default_waypoints_deg


@pytest.fixture
def home_motion(monkeypatch):
    project = Path(model.__file__).resolve().parents[1]
    config = load_config(project / 'config/piano.placeholder.json')
    clock = VirtualClock()
    backend = FakeBackend(config, clock)
    engine = Engine(config, backend, clock)
    motion = CalibrationMotion(engine, backend, config, software=False)
    backend.clients = {}
    backend.node = SimpleNamespace(create_client=lambda *args: object())
    backend.Trigger = object
    backend.statuses = {s+'_arm': dict(trajectory_revision=TRAJECTORY_REVISION,
        workpos_waypoints_deg=default_waypoints_deg(s).tolist(),
        home_final_pose_deg=list(HOME_FINAL_POSE_DEG), position_seq=10,
        streaming_enabled=True) for s in ('left', 'right')}
    monkeypatch.setattr(module.time, 'monotonic', clock.now)
    monkeypatch.setattr(module.time, 'sleep', clock.sleep)
    return motion


def test_home_uses_native_five_services_and_bilateral_stage_barriers(home_motion):
    motion = home_motion
    backend, engine = motion.backend, home_motion.engine
    calls = []
    verbs = ('home', 'home_point3', 'home_point2', 'home_point1', 'home_final_pose')
    def call(side, verb):
        if verb == 'enable':
            calls.append((side, verb))
            return
        stage = verbs.index(verb) + 1
        if stage > 1:
            # Both peers must have completed the preceding stage first.
            assert all(backend.statuses[s+'_arm']['position_stage'] >= stage-1 for s in ('left', 'right'))
        target = home_targets(side)[stage-1].tolist()
        backend.measured[side+'_arm'] = target
        backend.statuses[side+'_arm'].update(position_seq=10+stage, position_kind='home',
            position_stage=stage, position_stage_count=5, position_target=target,
            position_status='arrived' if stage == 5 else 'stage_arrived')
        calls.append((side, verb))
    backend._call = call
    motion.native_work_path(home=True)
    assert calls == [(s, v) for v in (*verbs, 'enable') for s in ('left', 'right')]
    for side in ('left', 'right'):
        assert engine.targets[side+'_arm'] == pytest.approx([0, math.radians(85), 0, 0, 0, 0, 0])
        assert backend.measured[side+'_hand'] == motion.config['hands'][side]['hover_rad']
    assert not backend.stopped


@pytest.mark.parametrize('fault', ['wrong_kind', 'wrong_target', 'driver_failed'])
def test_home_does_not_advance_after_native_stage_fault(home_motion, fault):
    motion = home_motion
    backend = motion.backend
    calls = []
    def call(side, verb):
        calls.append((side, verb))
        assert verb == 'home'
        target = home_targets(side)[0].tolist()
        backend.statuses[side+'_arm'].update(position_seq=11, position_kind='home',
            position_stage=1, position_stage_count=5, position_target=target,
            position_status='stage_arrived')
        if side == 'right':
            if fault == 'wrong_kind': backend.statuses[side+'_arm']['position_kind'] = 'workpos'
            elif fault == 'wrong_target': backend.statuses[side+'_arm']['position_target'] = [0]*7
            else: backend.statuses[side+'_arm']['position_status'] = 'failed'
    backend._call = call
    with pytest.raises(RuntimeError):
        motion.native_work_path(home=True)
    assert calls == [('left', 'home'), ('right', 'home')]
    assert backend.stopped and motion.engine.state == 'FAULT'


def test_home_validates_final_pose_contract_before_claim(home_motion):
    home_motion.backend.statuses['right_arm']['home_final_pose_deg'] = [0]*7
    with pytest.raises(ValueError, match='HOME'):
        home_motion.native_work_path(home=True)
    assert not home_motion.backend.claimed


def test_piano_return_order_final_pose_and_early_j4_stay_velocity_bounded(home_motion):
    motion = home_motion
    e, b = motion.engine, motion.backend
    motion.return_piano_home()
    stages = [x for x in e.events if x['event'] == 'home_stage']
    assert [x['stage'] for x in stages] == [1, 2, 3, 4, 5]
    moves = [x for x in e.events if x['event'] == 'manual_target']
    assert len(moves) == 5
    for side in ('left', 'right'):
        key = side+'_arm'
        from morla_piano.model import arm_target
        assert moves[0]['targets_rad'][key] == arm_target(motion.config, side, 0)
        expected = list(moves[0]['targets_rad'][key])
        for stage, index, value in ((1, 3, 95), (2, 0, 0), (3, 1, 60)):
            expected[index] = math.radians(value)
            assert moves[stage]['targets_rad'][key] == expected
        assert e.targets[key] == pytest.approx([0, math.radians(85), 0, 0, 0, 0, 0])
        final = moves[-1]
        samples = [(stamp, pose[key]) for stamp, pose in b.history
                   if final['monotonic_s']+.8*final['duration_s'] <= stamp
                   < final['monotonic_s']+.85*final['duration_s']]
        assert samples
        assert all(abs(q[3]) < 1e-10 and q[1] < math.radians(85) for _, q in samples)
        maximum = motion.config['timing']['max_arm_velocity_rad_s']
        for (t0, a), (t1, z) in zip(b.history, b.history[1:]):
            if t1 > t0:
                assert max(abs(x-y)/(t1-t0) for x,y in zip(a[key], z[key])) <= maximum+1e-8
    assert e.state == 'STOPPED' and not b.stopped


def test_piano_return_validates_entire_route_before_claim(home_motion):
    for side in ('left', 'right'):
        home_motion.config['hands'][side]['arm_limits_rad'][3][1] = math.radians(94)
    with pytest.raises(ValueError):
        home_motion.return_piano_home()
    assert not home_motion.backend.claimed


@pytest.mark.parametrize('cancelled', [True, False])
def test_piano_return_does_not_continue_after_stop_or_fault(home_motion, cancelled):
    from morla_piano.engine import Stopped
    calls = []
    original = home_motion.move_targets
    def move(targets, **kwargs):
        calls.append(targets)
        if len(calls) == 2:
            raise Stopped('cancelled') if cancelled else RuntimeError('driver fault')
        return original(targets, **kwargs)
    home_motion.move_targets = move
    with pytest.raises(RuntimeError):
        home_motion.return_piano_home()
    assert len(calls) == 2
    assert home_motion.engine.state == ('STOPPED' if cancelled else 'FAULT')
    assert home_motion.backend.stopped is (not cancelled)
