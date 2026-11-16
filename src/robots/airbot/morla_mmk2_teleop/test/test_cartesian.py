import os
import threading

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from morla_mmk2_teleop.analytic_dh import AnalyticDhKinematics
from morla_mmk2_teleop.urdf_ik import ArmKinematics, error_and_jacobian, log6
from morla_mmk2_teleop.vr_controller import AsyncCartesianController, create_controller


@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
def test_urdf_se3_derivative_matches_joint_perturbations(side, sign):
    solver = ArmKinematics({'plugin_id': f'morla.airbot.mmk2.{side}_arm'})
    q = np.array([.1, -.7, .9, sign * 1.6, sign * .4, .2])
    current, geometric = solver._placement(q)
    target = solver.fk(q + np.array([.02, .01, -.03, .05, .08, -.09]))
    body = np.vstack([current[:3, :3].T @ part for part in (geometric[:3], geometric[3:])])
    _, derivative = error_and_jacobian(current, target, body)
    offsets = np.eye(6) * 1e-5
    numeric = np.column_stack([
        (log6(np.linalg.inv(solver.fk(q + dq)) @ target)
         - log6(np.linalg.inv(solver.fk(q - dq)) @ target)) / 2e-5
        for dq in offsets
    ])
    np.testing.assert_allclose(derivative, numeric, atol=1e-8)


def test_recorded_wrist_branch_output_respects_step_lead_and_limits():
    # MORLA-run-2026-11-11_0049, record 129906. The output stage now follows
    # FR3/CR5: a plain lead clip plus a per-tick step clip, with no second FK
    # gate (the gate used to hold a fifth of the ticks). It must still bound the
    # step, stay inside the measured lead window and inside the joint limits.
    solver = ArmKinematics({'plugin_id': 'morla.airbot.mmk2.left_arm'}, clock=lambda: 0.)
    q = np.array([-.3188876369003639, -.2016800071874097, 1.1421473981007884,
                  1.9361305770410056, .27674488438272365, -.13017635151801338])
    target = np.array([
        [.8025738999625113, -.5928651597857423, -.06622716520575557, .32829888725831174],
        [-.18609287716379194, -.1433363266403577, -.9720206471748164, -.07555381376405107],
        [.5667844177179411, .7924428053667183, -.22536597803275255, .5697959590362862],
        [0., 0., 0., 1.],
    ])
    candidate = np.asarray(solver.solve(target, seed=q, max_joint_step=.036574204))
    goal = solver.goal.copy()
    before = solver.command_metrics(q, goal, target)
    controller = AsyncCartesianController(solver)
    try:
        # Feedback trailing at the edge of the lead window: the lead clip alone
        # shapes the step, and the contract is the bound, not cost descent.
        feedback = q - .078 * np.sign(candidate - q)
        output, report = controller._output(q, candidate, feedback, .004, .08)
        assert 0 < report['output_step_rad'] <= .004 + 1e-12
        assert np.max(np.abs(output - feedback)) <= .08 + 1e-12
        assert np.all(output >= solver.bounds[:, 0] - 1e-12)
        assert np.all(output <= solver.bounds[:, 1] + 1e-12)
        # Feedback on the command: the lead clip is inert, so the single
        # direction-preserving speed clip must keep the solver's descent
        # direction (the 20 mm -> 52 mm regression was a component-wise clip).
        output, _ = controller._output(q, candidate, q, .004, .08)
        after = solver.command_metrics(output, goal, target)
        assert after['cost'] < before['cost']
        np.testing.assert_allclose(output - q, (candidate - q) * (.004 / np.max(np.abs(candidate - q))), atol=1e-12)
    finally:
        controller.close()


def test_reset_discards_worker_result_from_previous_anchor():
    q = [0., -.7, .9, 1.6, .4, 0.]
    pose = {'position': [0., 0., 0.], 'quaternion': [0., 0., 0., 1.]}
    turned = {**pose, 'quaternion': Rotation.from_rotvec([.1, 0., 0.]).as_quat().tolist()}
    solver = ArmKinematics({'plugin_id': 'morla.airbot.mmk2_sdk.left_arm'})
    controller = AsyncCartesianController(solver)
    entered, release, retired = threading.Event(), threading.Event(), threading.Event()
    solve = controller.solver.solve

    def blocked(target, seed=None, max_joint_step=None):
        entered.set()
        assert release.wait(2.)
        return solve(target, seed=seed, max_joint_step=max_joint_step)

    controller.solver.solve = blocked
    controller.set_diagnostic_sink(lambda report: retired.set() if report.get('discarded') else None)
    try:
        controller.reanchor(pose, q, {})
        assert controller(turned, q, .02, {}) == q
        assert entered.wait(2.)
        controller.reset()
        release.set()
        assert retired.wait(2.)
        assert controller.result is None
        controller.reanchor(turned, q, {})
        assert controller(turned, q, .02, {}) == q
    finally:
        release.set()
        controller.close()


def test_factory_runs_each_arm_in_its_own_process():
    left = create_controller({'instance': {'plugin_id': 'morla.airbot.mmk2_sdk.left_arm', 'backend': 'real'}})
    try:
        right = create_controller({'instance': {'plugin_id': 'morla.airbot.mmk2_sdk.right_arm', 'backend': 'real'}})
        try:
            pose = {'position': [0., 0., 0.], 'quaternion': [0., 0., 0., 1.]}
            assert left.requires_fresh_input and right.requires_fresh_input
            left.reanchor(pose, [0., -.7, .9, 1.6, .4, 0.], {})
            right.reanchor(pose, [0., -.7, .9, -1.6, -.4, 0.], {})
            reports = [left.diagnostics(), right.diagnostics()]
            assert len({os.getpid(), *(report['worker_pid'] for report in reports)}) == 3
            for report in reports:
                assert (report['worker_started_ns'] <= report['worker_reply_ns']
                        <= report['parent_received_ns'] <= report['completed_ns'])
            right.reset()
            assert right.result is None
        finally:
            right.close()
        assert not right.worker.process.is_alive()
    finally:
        left.close()
    assert not left.worker.process.is_alive()


def test_recorded_wrist_hold_has_a_position_preserving_rotation_step():
    solver = ArmKinematics({'plugin_id': 'morla.airbot.mmk2.left_arm'},
                           max_step_rad=.04, clock=lambda: 0.)
    seed = np.array([-.024109490995, -.713435968143, 1.017589062555,
                     2.956110973717, .049032498536, -1.209934232937])
    target = np.array([
        [.920899276705, -.380286611513, .085595649821, .427443973659],
        [.031253867295, -.146846593892, -.988665400245, -.048962755488],
        [.388545644607, .913136447069, -.123345494811, .465515410535],
        [0., 0., 0., 1.],
    ])
    result = np.asarray(solver.solve(target, seed=seed))
    before = solver.command_metrics(seed, solver.goal, target)
    after = solver.last_report
    assert after['reason'] == 'tracking_rotation_fallback'
    assert after['raw_angle_error_rad'] < before['raw_angle_error_rad']
    assert after['raw_position_error_m'] <= max(before['raw_position_error_m'], solver.position_lead_m) + 1e-10
    assert after['cost'] < before['cost']
    assert np.max(np.abs(result - seed)) <= .04 + 1e-10
    assert np.all(result >= solver.bounds[:, 0]) and np.all(result <= solver.bounds[:, 1])


def test_reused_ik_target_is_reached_without_overshoot():
    solver = ArmKinematics({'plugin_id': 'morla.airbot.mmk2.left_arm'})
    controller = AsyncCartesianController(solver)
    pose = {'position': [0., 0., 0.], 'quaternion': [0., 0., 0., 1.]}
    initial = np.array([0., -.7, .9, 1.6, .4, 0.])
    target = initial.copy()
    target[0] = .025
    try:
        # Freeze the result while feeding multiple control frames.
        with controller.condition:
            controller.result = {'values': target, 'requested_ns': controller.clock(),
                                 'has_goal': True, 'hold': None}
            measured = initial
            for expected in (.016, .025, .025, .025):
                measured = controller(pose, measured, .01, {'max_joint_velocity': 1.6})
                assert measured[0] == pytest.approx(expected)
            for reason in ('holding', 'deadline_hold'):
                controller.result = {'values': initial, 'requested_ns': controller.clock(),
                                     'has_goal': True, 'hold': reason}
                assert controller(pose, measured, .01, {'max_joint_velocity': 1.6}) == measured
                assert controller.output_report['output_reason'] == reason
    finally:
        controller.close()


def test_position_guard_can_be_disabled_per_binding():
    instance = {'plugin_id': 'morla.airbot.mmk2.left_arm', 'backend': 'sim'}
    seed = np.array([-.024109490995, -.713435968143, 1.017589062555,
                     2.956110973717, .049032498536, -1.209934232937])
    target = np.array([
        [.920899276705, -.380286611513, .085595649821, .427443973659],
        [.031253867295, -.146846593892, -.988665400245, -.048962755488],
        [.388545644607, .913136447069, -.123345494811, .465515410535],
        [0., 0., 0., 1.],
    ])
    default = ArmKinematics(instance, max_step_rad=.04, clock=lambda: 0.)
    default.solve(target, seed=seed)
    assert default.last_report['position_guard_enabled'] is True
    assert default.last_report['raw_position_error_m'] <= default.last_report['position_guard_m'] + 1e-10

    controller = create_controller({'instance': instance, 'binding': {
        'parameters': {'position_guard_enabled': False}}})
    try:
        pose = {'position': [0., 0., 0.], 'quaternion': [0., 0., 0., 1.]}
        controller.worker.solve(0, pose, seed, 0., {}, True, None)
        turned = {**pose, 'quaternion': Rotation.from_rotvec([.1, 0., 0.]).as_quat().tolist()}
        _, _, _, process_report = controller.worker.solve(0, turned, seed, .01, {}, False, seed)
        assert process_report['position_guard_enabled'] is False
        solver = controller.solver
        solver.clock = lambda: 0.
        result = np.asarray(solver.solve(target, seed=seed))
        report = solver.last_report
        assert report['position_guard_enabled'] is False
        assert report['position_guard_m'] is None
        assert report['reason'] == 'tracking'
        assert report['raw_position_error_m'] > default.last_report['position_guard_m']
        assert report['cost_after'] < report['cost_before']
        assert np.max(np.abs(result - seed)) <= .04 + 1e-10
        assert np.all(result >= solver.bounds[:, 0]) and np.all(result <= solver.bounds[:, 1])
    finally:
        controller.close()


def test_iterative_ik_keeps_the_same_bounded_goal():
    instance = {'plugin_id': 'morla.airbot.mmk2.left_arm'}
    seed = np.array([0., -.7, .9, 1.6, .4, 0.])
    single = ArmKinematics(instance, max_step_rad=.01, clock=lambda: 0.)
    iterative = ArmKinematics(instance, max_step_rad=.01, clock=lambda: 0., ik_iterations=3)
    target = single.fk(seed)
    target[:3, 3] += [.06, .01, 0.]
    target[:3, :3] = Rotation.from_rotvec([.2, 0., 0.]).as_matrix() @ target[:3, :3]
    single.solve(target, seed=seed)
    result = iterative.solve(target, seed=seed)
    np.testing.assert_allclose(iterative.goal, single.goal)
    assert iterative.last_report['iterations'] == 3
    assert iterative.last_report['cost_after'] < single.last_report['cost_after']
    assert np.max(np.abs(result - seed)) <= .03 + 1e-10


@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
@pytest.mark.parametrize('mode,method', [
    ('urdf_numerical', 'mmk2_urdf_least_squares'),
    ('analytic_dh', 'mmk2_analytic_dh'),
])
def test_absolute_ik_mode_solves_pose_in_arm_process(side, sign, mode, method):
    controller = create_controller({
        'instance': {'plugin_id': f'morla.airbot.mmk2.{side}_arm', 'backend': 'sim'},
        'binding': {'parameters': {'ik_mode': mode, 'position_guard_enabled': False}},
    })
    try:
        q = np.array([0., -.7, .9, sign * 1.6, sign * .4, 0.])
        pose = {'position': [0., 0., 0.], 'quaternion': [0., 0., 0., 1.]}
        controller.reanchor(pose, q, {})
        moved = {'position': [.008, 0., 0.], 'quaternion': Rotation.from_rotvec([.08, 0., 0.]).as_quat().tolist()}
        result, _, _, report = controller.worker.solve(controller.generation, moved, q, .01, {}, False, q)
        assert report['method'] == method
        assert report['worker_pid'] != os.getpid()
        assert report['cost_after'] < report['cost_before']
        assert np.all(result >= controller.solver.bounds[:, 0])
        assert np.all(result <= controller.solver.bounds[:, 1])
    finally:
        controller.close()


def test_analytic_dh_preserves_wrist_branch_at_singularity_and_holds_when_unreachable():
    solver = AnalyticDhKinematics({'plugin_id': 'morla.airbot.mmk2.left_arm'})
    seed = np.array([.2, -.8, 1.1, 1.5, 0., -.4])
    target = solver.fk(seed + [0., 0., 0., .06, 0., .06])
    solution = solver.solve(target, seed=seed)
    assert solver.last_report['converged']
    assert solver.last_report['raw_position_error_m'] < 5e-5
    assert solver.last_report['raw_angle_error_rad'] < 1e-4
    np.testing.assert_allclose(np.asarray(solution) - seed, [0., 0., 0., .06, 0., .06], atol=2e-5)

    target[0, 3] += 2.
    np.testing.assert_array_equal(solver.solve(target, seed=seed), seed)
    assert solver.last_report['reason'] == 'holding'
    assert not solver.last_report['converged']
