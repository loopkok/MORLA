"""Calibration frame, isolation, validation and motion lifecycle regressions."""
import math
import sys
import time
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from morla_web_manager.piano_calibration import CalibrationMotion, WORK_POSE_DEG, validate_command, validate_work_poses
from test_piano_lane import store, manager, open_software, wait_job, use_virtual_clock
from test_piano_lane import wait_until


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('joint,bounds', list(enumerate([
    (0, 90), (-60, 90), (0, 90), (-5, 17), *[(0, 110)] * 8,
])))
def test_manual_hand_joint_and_pose_accept_exact_manual_endpoints_and_reject_outside(store, side, joint, bounds):
    config = store.config()['config']
    for angle in bounds:
        arguments = dict(side=side, part='hand', joint_index=joint,
                         target_deg=angle, calibration_confirmed=True)
        assert validate_command('joint', arguments, config)['target_deg'] == angle
        pose = [0.] * 12
        pose[joint] = angle
        validate_command('hand_pose', dict(side=side, target_deg=pose, calibration_confirmed=True), config)
    for angle in (bounds[0] - 1e-6, bounds[1] + 1e-6):
        with pytest.raises(ValueError):
            validate_command('joint', dict(side=side, part='hand', joint_index=joint,
                             target_deg=angle, calibration_confirmed=True), config)
        pose = [0.] * 12
        pose[joint] = angle
        with pytest.raises(ValueError):
            validate_command('hand_pose', dict(side=side, target_deg=pose, calibration_confirmed=True), config)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_saved_config_cannot_restore_old_wider_hand_limits(store, side):
    config = store.config()['config']
    config['hands'][side]['hand_limits_rad'][0][1] = 1.832
    with pytest.raises(ValueError, match='放宽'):
        store.validate(config)


@pytest.mark.parametrize('error_deg,expected',[(1.5,True),(2.9,True),(3.1,False),(150.,False)])
def test_work_waypoint_uses_three_degrees_and_still_blocks_large_errors(error_deg,expected):
    from morla_web_manager.piano_calibration import work_waypoint_arrived
    q=[0.]*7;q[0]=math.radians(error_deg)
    assert work_waypoint_arrived(q,[0.]*7) is expected


def test_work_waypoint_requires_complete_finite_feedback():
    from morla_web_manager.piano_calibration import work_waypoint_arrived
    assert not work_waypoint_arrived([0.]*6,[0.]*7)
    assert not work_waypoint_arrived([float('nan')]+[0.]*6,[0.]*7)


@pytest.fixture
def motion(store):
    sys.path.insert(0, str(store.project))
    from morla_piano.engine import Engine, FakeBackend, VirtualClock
    cfg = store.config()["config"]
    clock = VirtualClock()
    backend = FakeBackend(cfg, clock)
    engine = Engine(cfg, backend, clock)
    return CalibrationMotion(engine, backend, cfg, software=True)


def test_requested_work_pose_has_only_authorized_limit_adjustments(store):
    assert validate_work_poses(store.config()["config"]) == WORK_POSE_DEG
    assert WORK_POSE_DEG["left"] == [-25.915, 90.973, -37.488, 72.655, 155, -40.201, 5.965]
    assert WORK_POSE_DEG["right"] == [28.249, 98, 38.828, 58.867, -155, 46.206, -3.757]
    cfg = store.config()["config"]
    cfg["calibration"]["work_pose_deg"]["right"][1] = 104.079
    with pytest.raises(ValueError, match="right 工作位 J2"):
        store.validate(cfg)


@pytest.mark.parametrize("parameters", [
    dict(side="left", part="arm", joint_index=True, target_deg=0),
    dict(side="left", part="hand", joint_index=12, target_deg=0),
    dict(side="right", part="arm", joint_index=4, target_deg=-160.44),
    dict(side="right", part="arm", joint_index=1, target_deg=float("nan")),
])
def test_manual_joint_rejects_invalid_target_before_motion(store, parameters):
    with pytest.raises(ValueError):
        validate_command("joint", dict(calibration_confirmed=True, **parameters), store.config()["config"])


@pytest.mark.parametrize("parameters", [dict(axis="q", frame="base", step=1), dict(axis="x", frame="world", step=1),
                                           dict(axis="x", frame="base", step=6), dict(axis="rx", frame="tool", step=4),
                                           dict(axis="z", frame="tool", step=0)])
def test_manual_cartesian_rejects_unbounded_steps(store, parameters):
    with pytest.raises(ValueError):
        validate_command("cartesian", dict(side="left", calibration_confirmed=True, **parameters), store.config()["config"])


def test_all_manual_motion_requires_explicit_confirmation(store):
    for op in ("joint", "hand_pose", "cartesian", "work_pose"):
        with pytest.raises(ValueError, match="明确"):
            validate_command(op, {}, store.config()["config"])


def test_legacy_config_is_normalized_and_malformed_calibration_rejected(store):
    cfg = store.config()["config"]
    del cfg["calibration"]
    assert store.validate(cfg)["calibration"]["work_pose_deg"] == WORK_POSE_DEG
    cfg["calibration"] = {}
    assert store.validate(cfg)["calibration"]["work_pose_deg"] == WORK_POSE_DEG
    for bad in (None, [], "bad", {"work_pose_deg": []}):
        cfg["calibration"] = bad
        with pytest.raises(ValueError):
            store.validate(cfg)


def test_manual_joint_holds_other_37_joints_and_does_not_prepare(motion):
    initial = motion.backend.snapshot()
    target = math.degrees(initial["left_hand"][4]) + 1
    motion.execute("joint", dict(side="left", part="hand", joint_index=4, target_deg=target, calibration_confirmed=True))
    final = motion.backend.snapshot()
    assert motion.engine.state == "CALIBRATION_IDLE"
    for key, values in initial.items():
        for i, value in enumerate(values):
            assert final[key][i] == pytest.approx(math.radians(target) if (key, i) == ("left_hand", 4) else value)
    assert not any(event["event"] == "prepare_begin" for event in motion.engine.events)
    assert motion.config["calibrated"] is False


def test_large_absolute_joint_jump_does_not_claim_or_send(motion):
    with pytest.raises(ValueError, match="分步"):
        motion.execute("joint", dict(side="left", part="arm", joint_index=0, target_deg=30, calibration_confirmed=True))
    assert not motion.backend.claimed
    assert motion.backend.history == []


@pytest.mark.parametrize("bad", ["feedback", "motors", "controller", "hand_lease"])
def test_native_J_health_keeps_feedback_motor_controller_and_ownership_gates(motion, bad):
    now = time.monotonic()
    statuses = {}
    for side in ("left", "right"):
        statuses[side+"_arm"] = dict(feedback_fresh=True, backend="hardware", connected=True, motors_enabled=True,
                                     motor_status_fresh=True, controller_status_fresh=True, controller_arm_status=0,
                                     enabled=False, position_active=True)
        statuses[side+"_hand"] = dict(feedback_fresh=True, lease_fresh=True, lease_activity="playback", rejections=0)
    b = SimpleNamespace(fault=None, statuses=statuses, received={k: now for k in statuses},
                        status_at={k: now for k in statuses}, expected_backend="hardware", baseline_rejections={})
    motion.backend, motion.software = b, False
    motion.path_health()  # J is allowed to revoke JS streaming, but no other guard.
    if bad == "feedback":
        b.received["left_arm"] = now - 5
    elif bad == "motors":
        statuses["right_arm"]["motors_enabled"] = False
    elif bad == "controller":
        statuses["left_arm"]["controller_arm_status"] = 1
    else:
        statuses["right_hand"]["lease_fresh"] = False
    with pytest.raises(RuntimeError):
        motion.path_health()


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("frame", ["base", "tool"])
@pytest.mark.parametrize("axis,step", [("x", 1), ("y", -1), ("z", 1), ("rx", .5), ("ry", -.5), ("rz", .5)])
def test_cartesian_path_matches_native_fk_axes_and_keeps_branch(motion, side, frame, axis, step):
    positions = motion.backend.snapshot()
    start = motion.ik().fk(positions[side+"_arm"])
    path = motion.cartesian_path(positions, side, axis, frame, step)
    final = motion.ik().fk(path[-1])
    increment = np.zeros(3)
    increment["xyz".index(axis[-1])] = step
    if len(axis) == 1:
        expected = start[:3, 3] + (start[:3, :3] @ increment if frame == "tool" else increment) / 1000
        assert np.linalg.norm(final[:3, 3] - expected) < .0001
        assert Rotation.from_matrix(final[:3, :3] @ start[:3, :3].T).magnitude() < .001
    else:
        rotation = Rotation.from_rotvec(np.deg2rad(increment)).as_matrix()
        expected = start[:3, :3] @ rotation if frame == "tool" else rotation @ start[:3, :3]
        assert Rotation.from_matrix(expected @ final[:3, :3].T).magnitude() < .001
        assert np.linalg.norm(final[:3, 3] - start[:3, 3]) < .0001
    assert max(abs(np.array(path[-1]) - positions[side+"_arm"])) < math.radians(3)
    assert not motion.backend.claimed


def test_native_work_path_preserves_original_four_points_then_new_work_pose(motion):
    from nero_robot_control.waypoint_trajectory import waypoint_targets
    initial_hands = {key: q for key, q in motion.backend.snapshot().items() if key.endswith("hand")}
    motion.execute("work_pose", dict(side="both",calibration_confirmed=True))
    for side in ("left", "right"):
        for point in waypoint_targets(side):
            assert any(np.allclose(values[side+"_arm"], point) for _, values in motion.backend.history)
        assert np.allclose(motion.backend.measured[side+"_arm"], np.deg2rad(WORK_POSE_DEG[side]))
        assert motion.backend.measured[side+"_hand"] == initial_hands[side+"_hand"]
    assert motion.engine.state == "CALIBRATION_IDLE"


def test_software_worker_streams_measured_positions_and_invalidates_ready(manager, tmp_path):
    use_virtual_clock(manager, tmp_path)
    token = open_software(manager)
    assert wait_job(manager, manager.command(token, "prepare"), token)["ok"]
    initial = manager.snapshot()["positions"]
    result = wait_job(manager, manager.command(token, "joint", side="left", part="arm", joint_index=0,
                      target_deg=math.degrees(initial["left_arm"][0])+.5, calibration_confirmed=True), token)
    assert result["ok"], result
    # The result and periodic telemetry are separate IPC messages.
    wait_until(lambda: manager.snapshot()["state"] == "CALIBRATION_IDLE"
               and manager.snapshot()["positions"]["left_arm"][0] == pytest.approx(initial["left_arm"][0] + math.radians(.5)))
    manager.store.seed()
    with pytest.raises(ValueError, match="初始位"):
        manager.command(token, "play", score_id=manager.store.list_scores()[0]["id"])
    for op, params in (("cartesian", dict(side="right", axis="z", frame="base", step=1)),
                       ("hand_pose", dict(side="left", target_deg=[math.degrees(q) for q in initial["left_hand"]]))):
        result = wait_job(manager, manager.command(token, op, calibration_confirmed=True, **params), token)
        assert result["ok"], result
    manager.stop()


def test_manual_stop_holds_measured_hand_without_visiting_old_hover(manager):
    token = open_software(manager)
    target = 15.
    result = wait_job(manager, manager.command(token, "joint", side="left", part="hand", joint_index=4,
                                             target_deg=target, calibration_confirmed=True), token)
    assert result["ok"]
    manager.stop()
    wait_until(lambda: manager.snapshot()["state"] == "STOPPED")
    captured = wait_job(manager, manager.command(token, "capture"), token)
    assert captured["result"]["positions"]["left_hand"][4] == pytest.approx(math.radians(target))


def test_stop_interrupts_manual_interpolation_and_does_not_resume(manager):
    token = open_software(manager)
    job = manager.command(token, "joint", side="left", part="arm", joint_index=0, target_deg=14., calibration_confirmed=True)
    wait_until(lambda: manager.snapshot()["state"] == "CALIBRATING")
    manager.stop()
    wait_until(lambda: manager.snapshot()["state"] == "STOPPED" and not manager.snapshot()["busy"])
    first = manager.snapshot()["positions"]["left_arm"][0]
    time.sleep(.35)
    assert manager.snapshot()["positions"]["left_arm"][0] == pytest.approx(first)
    assert first < math.radians(14)


@pytest.mark.parametrize('side,part', [('left','arm'),('right','arm'),('left','hand'),('right','hand')])
def test_manual_joint_claims_and_commands_only_selected_part(motion,side,part):
    key=side+'_'+part
    target=math.degrees(motion.backend.snapshot()[key][0])+.5
    sent=[]
    original_send=motion.backend.send
    def send(targets):
        sent.append(set(targets))
        original_send(targets)
    motion.backend.send=send
    motion.execute('joint',dict(side=side,part=part,joint_index=0,target_deg=target,calibration_confirmed=True))
    assert motion.backend.motion_keys=={key}
    assert set(motion.engine.targets)=={key}
    assert sent and all(keys=={key} for keys in sent)
    assert any(row['event']=='manual_request' for row in motion.engine.events)
    assert any(row['event']=='manual_target' for row in motion.engine.events)


@pytest.mark.parametrize('offender', [0,1])
def test_arrival_timeout_identifies_moving_or_holding_joint_before_normal_stop(motion,offender):
    import copy
    b,e=motion.backend,motion.engine
    initial=b.snapshot()
    def send(targets):
        # Either J1 fails to reach its target, or the unchanged J2 drifts.
        b.measured.update(copy.deepcopy(targets))
        if offender==0:b.measured['left_arm'][0]=initial['left_arm'][0]
        else:b.measured['left_arm'][1]=initial['left_arm'][1]+math.radians(4)
    b.send=send
    target=math.degrees(initial['left_arm'][0])+6
    with pytest.raises(RuntimeError,match=f'left_arm J{offender+1}.*目标=.*实测=.*误差=.*容差='):
        motion.execute('joint',dict(side='left',part='arm',joint_index=0,target_deg=target,calibration_confirmed=True))
    event=next(row for row in e.events if row['event']=='arrival_timeout')
    assert set(event['joints'])=={'left_arm'}
    assert math.degrees(event['joints']['left_arm']['error_rad'][offender])==pytest.approx(6 if offender==0 else 4)
    events=[row['event'] for row in e.events]
    # Unconfirmed arrival keeps power and releases fingers: normal stop plus an
    # explicit motion_unconfirmed event, not FAULT/emergency stop.
    assert events.index('arrival_timeout')<events.index('motion_unconfirmed')
    assert 'fault' not in events
    assert not b.stopped and e.state=='STOPPED' and e.pressed is None


@pytest.mark.parametrize('error_deg,passed', [(2.62,True),(2.99,True),(3.01,False)])
def test_arm_joint_manual_arrival_three_degrees_with_dwell(motion,error_deg,passed):
    import copy
    b,e=motion.backend,motion.engine
    original=b.snapshot()
    def send(targets):
        b.measured.update(copy.deepcopy(targets))
        b.measured['left_arm'][4]=original['left_arm'][4]-math.radians(error_deg)
    b.send=send
    request=dict(side='left',part='arm',joint_index=0,target_deg=math.degrees(original['left_arm'][0])+1,calibration_confirmed=True)
    timing=copy.deepcopy(e.c['timing'])
    if passed:
        motion.execute('joint',request)
        assert e.state=='CALIBRATION_IDLE' and not b.stopped
        assert e.clock.now()>=.5
    else:
        with pytest.raises(RuntimeError,match='J5.*容差=3.000'):
            motion.execute('joint',request)
        assert not b.stopped and e.state=='STOPPED'
        assert any(row['event']=='motion_unconfirmed' for row in e.events)
    assert e.c['timing']==timing


def test_hand_manual_keeps_original_arrival_tolerance(motion):
    import copy
    b,e=motion.backend,motion.engine
    def send(targets):
        b.measured.update(copy.deepcopy(targets))
        b.measured['left_hand'][4]-=math.radians(2.62)
    b.send=send
    target=math.degrees(b.snapshot()['left_hand'][4])+3
    with pytest.raises(RuntimeError,match='left_hand.*容差=1.432'):
        motion.execute('joint',dict(side='left',part='hand',joint_index=4,target_deg=target,calibration_confirmed=True))


def test_work_pose_final_stage_uses_arm_tolerance_and_cartesian_default_stays_strict(motion):
    calls=[]
    motion.native_work_path=lambda sides: motion.claim_at_measured(keys=[s+'_arm' for s in sides])
    motion.engine._await_arrival=lambda targets,**kw:calls.append(kw)
    motion.execute('work_pose',dict(side='left',calibration_confirmed=True))
    assert calls[-1]==dict(tolerance_rad=math.radians(3),dwell_s=.2)
    motion.move_targets({'left_arm':motion.backend.snapshot()['left_arm']})
    assert calls[-1]=={}


@pytest.mark.parametrize('side,part,index,bias_deg', [('left','arm',3,2.62),('right','arm',3,2.62),('left','hand',4,.8),('right','hand',4,.8)])
def test_consecutive_manual_clicks_do_not_rebase_tracking_error_or_drop_holding_targets(motion,side,part,index,bias_deg):
    import copy
    b,e=motion.backend,motion.engine
    key=side+'_'+part
    b.claim(keys=[key])
    commanded=b.snapshot()[key]
    # Physical feedback differs from the held command, as under load.
    bias=math.radians(bias_deg)
    b.measured[key]=[v-bias for v in commanded]
    if part=='hand':
        # Keep unused thumb joints at their valid zero boundary.
        b.measured[key][:4]=commanded[:4]
    e.targets={key:list(commanded)}
    e.state='CALIBRATION_IDLE'
    sent=[]
    def send(targets):
        sent.append(copy.deepcopy(targets))
        b.measured.update({k:[v-bias if part=='arm' or i>=4 else v for i,v in enumerate(q)] for k,q in targets.items()})
    b.send=send
    original=copy.deepcopy(commanded)
    for turn,joint in enumerate((index,index+1,index),1):
        before=copy.deepcopy(e.targets[key])
        start=len(sent)
        target=math.degrees(before[joint])+1
        motion.execute('joint',dict(side=side,part=part,joint_index=joint,target_deg=target,calibration_confirmed=True))
        assert sent[start][key]==pytest.approx(before)
        for row in sent[start:]:
            assert all(row[key][i]==before[i] for i in range(len(before)) if i!=joint)
        expected=list(before);expected[joint]=math.radians(target)
        assert e.targets[key]==pytest.approx(expected)
    assert e.targets[key][0]==original[0]
    seeds=[row for row in e.events if row['event']=='manual_control_seed']
    assert [row['source'] for row in seeds]==['retained_command']*3
    assert not b.stopped


def test_first_takeover_and_scope_change_ignore_stale_engine_targets(motion):
    b,e=motion.backend,motion.engine
    measured=b.snapshot()
    e.targets={'left_arm':[.2]*7}
    motion.claim_at_measured(keys=['left_arm'])
    assert e.targets=={'left_arm':measured['left_arm']}
    e.targets['left_arm'][0]+=.01
    motion.claim_at_measured(keys=['right_arm'])
    assert e.targets=={'right_arm':measured['right_arm']}
    assert [row['source'] for row in e.events if row['event']=='manual_control_seed']==['measured_takeover']*2


def test_telemetry_reports_command_targets_separately_from_raw_feedback(motion):
    import copy
    b,e=motion.backend,motion.engine
    b.claim(keys=['left_arm'])
    e.targets={'left_arm':list(b.measured['left_arm'])}
    e.targets['left_arm'][0]+=.02
    raw=copy.deepcopy(b.measured)
    telemetry=motion.telemetry()
    assert telemetry['positions']==raw
    assert telemetry['command_targets']==e.targets
    telemetry['command_targets']['left_arm'][0]=99
    assert e.targets['left_arm'][0]!=99 and b.measured==raw
    e.state='FAULT'
    assert motion.telemetry()['command_targets']=={}


def test_retaining_command_does_not_bypass_tracking_error_stop(motion):
    b,e=motion.backend,motion.engine
    b.claim(keys=['left_arm'])
    e.targets={'left_arm':list(b.measured['left_arm'])}
    e.state='CALIBRATION_IDLE'
    b.measured['left_arm'][4]-=math.radians(10)
    with pytest.raises(RuntimeError,match='跟踪误差超限'):
        motion.execute('joint',dict(side='left',part='arm',joint_index=0,target_deg=.5,calibration_confirmed=True))
    assert b.stopped and not b.history
