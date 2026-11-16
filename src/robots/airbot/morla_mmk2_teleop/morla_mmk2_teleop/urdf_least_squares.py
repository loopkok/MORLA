"""Bounded absolute-target IK using Astral's trust-region objective."""
import numpy as np
from scipy.optimize import least_squares

from morla_core.coordinates import validate_transform
from .urdf_ik import ArmKinematics, error_and_jacobian, vector6


class _BudgetExhausted(Exception):
    pass


class LeastSquaresKinematics(ArmKinematics):
    method_name = 'mmk2_urdf_least_squares'
    method_label = 'URDF 有界数值求解'

    def solve(self, target, seed=None, max_joint_step=None):
        raw = validate_transform(target)
        q0 = vector6(self.q if seed is None else seed)
        lower, upper = self.bounds.T
        if np.any(q0 < lower - 1e-8) or np.any(q0 > upper + 1e-8):
            raise ValueError('MMK2 IK seed exceeds deployment joint limits')
        q0 = np.clip(q0, lower, upper)
        if max_joint_step is not None:
            if not np.isfinite(max_joint_step) or max_joint_step < 0:
                raise ValueError('Invalid MMK2 per-tick joint step')
            lower = np.maximum(lower, q0 - max_joint_step)
            upper = np.minimum(upper, q0 + max_joint_step)

        began = self.clock()
        current = self.fk(q0)
        # This mode solves the absolute pose, as Astral does. The controller
        # applies the feedback lead window and rate limit when issuing commands.
        goal = self.goal = raw.copy()
        self.raw_target = raw.copy()
        before = self.command_metrics(q0, goal, raw)
        position_bound = max(before['raw_position_error_m'], self.position_lead_m)
        weights = np.array([1., 1., 1., .3, .3, .3])
        regularization = .01
        limit_weight = .12
        zero = np.clip(np.zeros(6), lower, upper)
        margin_lo = np.minimum(.18, .30 * np.maximum(zero - lower, .001))
        margin_hi = np.minimum(.18, .30 * np.maximum(upper - zero, .001))
        best_q, best_metrics = q0, before
        best_cost = float('inf')
        cached_q = None
        evaluations = 0
        budget_exhausted = False

        def evaluate(q):
            nonlocal cached_q, residual, jacobian, best_q, best_metrics, best_cost, evaluations
            if cached_q is not None and np.array_equal(cached_q, q):
                return
            if self.clock() - began >= .02:
                raise _BudgetExhausted
            pose, geometric = self._placement(q)
            basis = pose[:3, :3].T
            body = np.vstack((basis @ geometric[:3], basis @ geometric[3:]))
            error, derivative = error_and_jacobian(pose, goal, body)
            lo = margin_lo - (q - lower)
            hi = margin_hi - (upper - q)
            residual = np.concatenate((weights * error, regularization * (q - q0),
                                       limit_weight * (np.maximum(lo, 0) + np.maximum(hi, 0))))
            jacobian = np.vstack((weights[:, None] * derivative, regularization * np.eye(6),
                                  limit_weight * np.diag((hi > 0).astype(float) - (lo > 0))))
            evaluations += 1
            cached_q = q.copy()
            cost = float(residual @ residual)
            if cost < best_cost:
                position_error = float(np.linalg.norm(pose[:3, 3] - raw[:3, 3]))
                if not self.position_guard_enabled or position_error <= position_bound + 1e-10:
                    metrics = self.command_metrics(q, goal, raw)
                    if self.clock() - began < .02 and metrics['cost'] <= before['cost']:
                        best_q, best_metrics, best_cost = q.copy(), metrics, cost

        residual = jacobian = None

        def fun(q):
            evaluate(q)
            return residual

        def jac(q):
            evaluate(q)
            return jacobian

        if np.all(lower < upper):
            try:
                least_squares(fun, q0, jac=jac, bounds=(lower, upper), method='trf',
                              max_nfev=12, ftol=1e-7, xtol=1e-7, gtol=1e-7)
            except _BudgetExhausted:
                budget_exhausted = True

        self.q = best_q.copy()
        step = float(np.max(np.abs(best_q - q0)))
        self.last_report = {
            'method': self.method_name, 'method_label': self.method_label,
            'reason': 'tracking' if step > 1e-8 else 'holding',
            'evaluations': evaluations, 'budget_exhausted': budget_exhausted,
            'elapsed_s': self.clock() - began, 'joint_step_rad': step,
            'position_guard_enabled': self.position_guard_enabled,
            'position_guard_m': position_bound if self.position_guard_enabled else None,
            'cost_before': before['cost'], 'cost_after': best_metrics['cost'],
            **best_metrics,
        }
        return best_q.tolist()
