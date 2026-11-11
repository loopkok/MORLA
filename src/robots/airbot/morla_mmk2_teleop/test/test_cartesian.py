import threading

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

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


def test_recorded_wrist_branch_output_improves_after_limiting():
    # MORLA-run-2026-11-11_0049, record 129906. The old output increased
    # the bounded position error from 20 mm to 52 mm after component clipping.
    solver = ArmKinematics({'plugin_id': 'morla.airbot.mmk2.left_arm'}, clock=lambda: 0.)
    q = np.array([-.3188876369003639, -.2016800071874097, 1.1421473981007884,
                  1.9361305770410056, .27674488438272365, -.13017635151801338])
    target = np.array([
        [.8025738999625113, -.5928651597857423, -.06622716520575557, .32829888725831174],
        [-.18609287716379194, -.1433363266403577, -.9720206471748164, -.07555381376405107],
        [.5667844177179411, .7924428053667183, -.22536597803275255, .5697959590362862],
        [0., 0., 0., 1.],
    ])
    candidate = solver.solve(target, seed=q, max_joint_step=.036574204)
    controller = AsyncCartesianController(solver)
    try:
        feedback = q - .078 * np.sign(candidate - q)
        output, report = controller._output(q, candidate, feedback, .004, .08, solver.goal, target)
    finally:
        controller.close()
    assert 0 < report['output_step_rad'] <= .004 + 1e-12
    assert np.max(np.abs(output - feedback)) <= .08 + 1e-12
    assert report['output_cost_after'] < report['output_cost_before']
    assert report['output_position_error_m'] <= solver.last_report['position_guard_m'] + 1e-10


def test_reset_discards_worker_result_from_previous_anchor():
    q = [0., -.7, .9, 1.6, .4, 0.]
    pose = {'position': [0., 0., 0.], 'quaternion': [0., 0., 0., 1.]}
    turned = {**pose, 'quaternion': Rotation.from_rotvec([.1, 0., 0.]).as_quat().tolist()}
    controller = create_controller({'instance': {'plugin_id': 'morla.airbot.mmk2_sdk.left_arm', 'backend': 'real'}})
    entered, release, retired = threading.Event(), threading.Event(), threading.Event()
    solve = controller.solver.solve

    def blocked(target, seed=None, max_joint_step=None):
        entered.set()
        assert release.wait(2.)
        return solve(target, seed=seed, max_joint_step=max_joint_step)

    controller.solver.solve = blocked
    controller.set_diagnostic_sink(lambda report: retired.set() if report.get('discarded') else None)
    try:
        assert controller.requires_fresh_input
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
