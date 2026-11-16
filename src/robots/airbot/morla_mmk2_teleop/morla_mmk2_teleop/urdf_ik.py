"""MMK2 full-chain URDF FK/Jacobian and bounded SE(3) differential IK.

Each arm is solved in its own base frame. The gripper-centre fixed transform
is included in both FK and the Jacobian. Simulation and SDK adapters use this
same implementation and intersect their deployment limits with the URDF.
"""
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from morla_core.coordinates import validate_transform
from morla_core.resource_locator import resolve_plugin_resource


PARAMETERS = {
    'position_lead_m': (.02, .25),
    'rotation_lead_rad': (.35, 1.),
    'damping_min': (.005, 1.),
    'damping_max': (.145, 1.),
    'singularity_threshold': (.05, 1.),
}


def vector6(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.isfinite(result).all():
        raise ValueError('MMK2 requires six finite joints')
    return result.copy()


def differential_parameters(instance):
    given = instance.get('parameters', {})
    result = {}
    for name, (default, maximum) in PARAMETERS.items():
        value = given.get(name, default)
        if type(value) not in (int, float) or not np.isfinite(value) or not 0 < value <= maximum:
            raise ValueError('Invalid MMK2 differential IK ' + name)
        result[name] = float(value)
    if result['damping_min'] > result['damping_max']:
        raise ValueError('MMK2 damping_min exceeds damping_max')
    return result


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def log6(transform):
    """SE(3) logarithm in linear/angular order, as used by FR3/CR5."""
    rotation = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    angle = float(np.linalg.norm(rotation))
    cross = skew(rotation)
    coefficient = 1. / 12. + angle * angle / 720. if angle < 1e-4 else (1. - .5 * angle / np.tan(.5 * angle)) / (angle * angle)
    inverse_left_jacobian = np.eye(3) - .5 * cross + coefficient * cross @ cross
    return np.r_[inverse_left_jacobian @ transform[:3, 3], rotation]


def error_and_jacobian(current, target, body_jacobian):
    """Differentiate Log(current^-1 target) against a body-frame twist.

    Central differences of the six-dimensional Lie perturbation implement the
    Jlog6 correction without requiring a second robot model or a ROS library.
    The URDF joint Jacobian itself is analytic.
    """
    delta = np.linalg.inv(current) @ target
    derivative = np.empty((6, 6))
    epsilon = 1e-6
    for column in range(6):
        positive = np.eye(4)
        negative = np.eye(4)
        if column < 3:
            positive[column, 3] = epsilon
            negative[column, 3] = -epsilon
        else:
            axis = np.eye(3)[column - 3] * epsilon
            positive[:3, :3] = Rotation.from_rotvec(axis).as_matrix()
            negative[:3, :3] = Rotation.from_rotvec(-axis).as_matrix()
        derivative[:, column] = (log6(negative @ delta) - log6(positive @ delta)) / (2. * epsilon)
    return log6(delta), derivative @ body_jacobian


class ArmKinematics:
    method_name = 'mmk2_urdf_differential'
    method_label = '连续微分跟随'

    def __init__(self, instance, *, max_step_rad=.02, clock=time.monotonic, position_guard_enabled=True, ik_iterations=1):
        side = instance['plugin_id'].split('.')[-1].removesuffix('_arm')
        if side not in ('left', 'right'):
            raise ValueError('MMK2 IK requires a left or right arm')
        self.clock = clock
        if type(position_guard_enabled) is not bool:
            raise ValueError('MMK2 position_guard_enabled must be a boolean')
        self.position_guard_enabled = position_guard_enabled
        if type(ik_iterations) is not int or not 1 <= ik_iterations <= 3:
            raise ValueError('MMK2 ik_iterations must be an integer between 1 and 3')
        self.ik_iterations = ik_iterations
        self.max_step = float(max_step_rad)
        if not np.isfinite(self.max_step) or not 0 < self.max_step <= .3:
            raise ValueError('Invalid MMK2 differential step')
        self.parameters = differential_parameters(instance)
        for name, value in self.parameters.items():
            setattr(self, name, value)
        self.base_frame = side + '_base_link'
        self.tip_frame = side + '_gripper_center'
        self.joint_names = [f'{side}_joint{number}' for number in range(1, 7)]
        path = resolve_plugin_resource('mmk2_description', 'urdf/airbot_mmk2.urdf')
        root = ET.parse(path).getroot()
        parents = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
        chain = []
        link = self.tip_frame
        while link != self.base_frame:
            if link not in parents:
                raise ValueError('MMK2 URDF arm chain is incomplete')
            joint = parents[link]
            chain.append(joint)
            link = joint.find('parent').get('link')
        self.chain = []
        bounds = []
        names = []
        for joint in reversed(chain):
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            moving = joint.get('type') != 'fixed'
            axis = np.zeros(3)
            if moving:
                if joint.get('type') != 'revolute':
                    raise ValueError('Unexpected MMK2 arm joint type')
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
                limit = joint.find('limit')
                bounds.append([float(limit.get('lower')), float(limit.get('upper'))])
                names.append(joint.get('name'))
            self.chain.append((transform, axis, moving))
        if names != self.joint_names:
            raise ValueError('MMK2 URDF arm joint order differs from the control contract')
        self.bounds = np.asarray(bounds)
        given = instance.get('parameters', {})
        declared = instance.get('joints', [])
        lower = given.get('joint_lower', [joint['lower'] for joint in declared] if declared else self.bounds[:, 0])
        upper = given.get('joint_upper', [joint['upper'] for joint in declared] if declared else self.bounds[:, 1])
        self.bounds[:, 0] = np.maximum(self.bounds[:, 0], vector6(lower))
        self.bounds[:, 1] = np.minimum(self.bounds[:, 1], vector6(upper))
        if np.any(self.bounds[:, 0] >= self.bounds[:, 1]):
            raise ValueError('MMK2 deployment limits do not intersect the URDF')
        self.q = np.zeros(6)
        self.last_report = {}
        self.reset_teleop()

    def reset_teleop(self):
        self.last_report = {}
        self.goal = None
        self.raw_target = None

    def sync_state(self, q):
        self.q = vector6(q)

    def _placement(self, q, *, with_jacobian=True):
        q = vector6(q)
        transform = np.eye(4)
        origins = []
        axes = []
        index = 0
        for origin, axis, moving in self.chain:
            transform = transform @ origin
            if moving:
                if with_jacobian:
                    origins.append(transform[:3, 3].copy())
                    axes.append(transform[:3, :3] @ axis)
                displacement = np.eye(4)
                displacement[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                transform = transform @ displacement
                index += 1
        if not with_jacobian:
            return transform, None
        jacobian = np.column_stack([
            np.r_[np.cross(axis, transform[:3, 3] - origin), axis]
            for origin, axis in zip(origins, axes)
        ])
        return transform, jacobian

    def fk(self, q):
        return self._placement(q, with_jacobian=False)[0]

    def jacobian(self, q):
        return self._placement(q)[1]

    def command_metrics(self, q, goal, raw_target):
        transform = self.fk(q)
        error = log6(np.linalg.inv(transform) @ goal)
        weighted = error * np.array([1., 1., 1., .3, .3, .3])
        return {
            'cost': float(weighted @ weighted),
            'position_error_m': float(np.linalg.norm(transform[:3, 3] - goal[:3, 3])),
            'angle_error_rad': float(Rotation.from_matrix(transform[:3, :3].T @ goal[:3, :3]).magnitude()),
            'raw_position_error_m': float(np.linalg.norm(transform[:3, 3] - raw_target[:3, 3])),
            'raw_angle_error_rad': float(Rotation.from_matrix(transform[:3, :3].T @ raw_target[:3, :3]).magnitude()),
        }

    def _bounded_goal(self, raw, current):
        goal = raw.copy()
        translation = goal[:3, 3] - current[:3, 3]
        distance = float(np.linalg.norm(translation))
        if distance > self.position_lead_m:
            goal[:3, 3] = current[:3, 3] + translation * self.position_lead_m / distance
        rotation = Rotation.from_matrix(goal[:3, :3] @ current[:3, :3].T).as_rotvec()
        angle = float(np.linalg.norm(rotation))
        if angle > self.rotation_lead_rad:
            goal[:3, :3] = Rotation.from_rotvec(rotation * self.rotation_lead_rad / angle).as_matrix() @ current[:3, :3]
        self.goal = goal.copy()
        self.raw_target = raw.copy()
        return goal

    def _rotation_at_fixed_position(self, q, jacobian, derivative, error, damping):
        """Find a damped angular step in the translational Jacobian nullspace."""
        free = np.ones(6, dtype=bool)
        step = np.zeros(6)
        at_lower = q <= self.bounds[:, 0] + 1e-9
        at_upper = q >= self.bounds[:, 1] - 1e-9
        for _ in range(7):
            if np.count_nonzero(free) <= 1:
                return step
            _, singular_values, right_vectors = np.linalg.svd(jacobian[:3, free], full_matrices=True)
            rank = np.count_nonzero(singular_values > 1e-8)
            nullspace = right_vectors[rank:].T
            if not nullspace.shape[1]:
                return step
            angular = derivative[3:, free] @ nullspace
            regularization = damping * damping * np.eye(nullspace.shape[1])
            step[free] = -nullspace @ np.linalg.solve(
                angular.T @ angular + regularization, angular.T @ error[3:])
            outward = (at_lower & (step < 0)) | (at_upper & (step > 0))
            if not outward.any():
                return step
            free[outward] = False
            step.fill(0.)
        return step

    def solve(self, target, seed=None, max_joint_step=None):
        raw = validate_transform(target)
        q = self.q.copy() if seed is None else vector6(seed)
        if np.any(q < self.bounds[:, 0] - 1e-8) or np.any(q > self.bounds[:, 1] + 1e-8):
            raise ValueError('MMK2 IK seed exceeds deployment joint limits')
        limit = self.max_step
        if max_joint_step is not None:
            if isinstance(max_joint_step, bool) or not np.isfinite(max_joint_step) or max_joint_step < 0:
                raise ValueError('Invalid MMK2 per-tick joint step')
            limit = min(limit, max_joint_step)
        began = self.clock()
        result = self._solve_step(raw, q, limit, began)
        if self.ik_iterations == 1 or self.last_report['reason'] == 'deadline_hold':
            return result

        first = self.last_report
        last = first
        goal = self.goal
        position = np.asarray(result)
        iterations = 1
        duration = self.clock() - began
        while (iterations < self.ik_iterations
               and last['reason'] in ('tracking', 'tracking_rotation_fallback')
               and self.clock() - began + duration + .001 < .02):
            started = self.clock()
            trial = self._solve_step(raw, position, limit, began, goal)
            duration = max(duration, self.clock() - started)
            iterations += 1
            if self.last_report['reason'] not in ('tracking', 'tracking_rotation_fallback'):
                break
            last = self.last_report
            position = np.asarray(trial)

        elapsed = self.clock() - began
        if elapsed > .02:
            position = q.copy()
            last = {**first, **self.command_metrics(q, goal, raw),
                    'cost_after': first['cost_before'], 'reason': 'deadline_hold',
                    'converged': False, 'bounded_target_converged': False}
        self.q = position.copy()
        self.last_report = {**last, 'cost_before': first['cost_before'], 'iterations': iterations,
                            'joint_step_rad': float(np.max(np.abs(position - q))), 'elapsed_s': elapsed}
        return position.tolist()

    def _solve_step(self, raw, q, limit, began, goal=None):
        current, jacobian = self._placement(q)
        if goal is None:
            goal = self._bounded_goal(raw, current)
        body_jacobian = jacobian.copy()
        body_jacobian[:3] = current[:3, :3].T @ jacobian[:3]
        body_jacobian[3:] = current[:3, :3].T @ jacobian[3:]
        error, derivative = error_and_jacobian(current, goal, body_jacobian)
        weights = np.array([1., 1., 1., .3, .3, .3])
        weighted = weights * error
        matrix = weights[:, None] * derivative
        sigma = float(np.linalg.svd(matrix, compute_uv=False)[-1])
        proximity = max(0., 1. - sigma / self.singularity_threshold)
        damping = self.damping_min + (self.damping_max - self.damping_min) * proximity * proximity
        regularization = damping * damping * np.eye(6)
        resting_low = q <= self.bounds[:, 0] + 1e-9
        resting_high = q >= self.bounds[:, 1] - 1e-9
        free = np.ones(6, dtype=bool)
        delta = np.zeros(6)
        for iteration in range(7):
            active_matrix = matrix[:, free]
            delta[free] = -active_matrix.T @ np.linalg.solve(
                active_matrix @ active_matrix.T + regularization, weighted)
            if iteration == 0:
                unconstrained = delta.copy()
            outward = (resting_low & (delta < 0)) | (resting_high & (delta > 0))
            if not outward.any():
                break
            free[outward] = False
            delta[outward] = 0.
        pressure = float(np.linalg.norm(unconstrained[~free]))
        if not np.isfinite(delta).all():
            raise ValueError('MMK2 IK produced a non-finite update')
        delta *= min(1., limit / max(float(np.max(np.abs(delta))), 1e-12))
        before = self.command_metrics(q, goal, raw)
        result = q.copy()
        metrics = before
        reason = 'holding'
        # Keep the raw hand-position error bounded during orientation tracking.
        position_bound = max(before['raw_position_error_m'], self.position_lead_m)
        scales = (1., .5, .25, .125, .0625, .03125, .015625)
        position_rejected = False
        rotation_rejections = set()
        for scale in scales:
            if self.clock() - began > .02:
                reason = 'deadline_hold'
                break
            unconstrained = q + scale * delta
            trial = np.clip(unconstrained, self.bounds[:, 0], self.bounds[:, 1])
            pressure = max(pressure, float(np.linalg.norm(unconstrained - trial)))
            candidate = self.command_metrics(trial, goal, raw)
            if candidate['cost'] < before['cost'] - 1e-16:
                if (not self.position_guard_enabled
                        or candidate['raw_position_error_m'] <= position_bound + 1e-10):
                    result, metrics, reason = trial, candidate, 'tracking'
                    break
                position_rejected = True

        if (self.position_guard_enabled and reason == 'holding' and position_rejected and limit > 0
                and before['raw_angle_error_rad'] > 1e-6 and self.clock() - began <= .02):
            rotation_step = self._rotation_at_fixed_position(q, jacobian, derivative, error, damping)
            maximum = float(np.max(np.abs(rotation_step)))
            if np.isfinite(maximum) and maximum > 1e-12:
                rotation_step *= min(1., limit / maximum)
                for scale in scales:
                    if self.clock() - began > .02:
                        break
                    trial = np.clip(q + scale * rotation_step, self.bounds[:, 0], self.bounds[:, 1])
                    candidate = self.command_metrics(trial, goal, raw)
                    if candidate['raw_position_error_m'] > position_bound + 1e-10:
                        rotation_rejections.add('position')
                    elif candidate['raw_angle_error_rad'] >= before['raw_angle_error_rad'] - 1e-6:
                        rotation_rejections.add('angle')
                    elif candidate['cost'] >= before['cost'] - 1e-16:
                        rotation_rejections.add('cost')
                    else:
                        result, metrics, reason = trial, candidate, 'tracking_rotation_fallback'
                        break
            else:
                rotation_rejections.add('no_direction')
        if self.clock() - began > .02:
            result, metrics, reason = q.copy(), before, 'deadline_hold'
        self.q = result.copy()
        self.last_report = {
            'method': self.method_name, 'method_label': self.method_label,
            'reason': reason, 'exact_convergence': False,
            'holding_cause': ('position_guard' if position_rejected else 'no_cost_descent')
            if reason == 'holding' else None,
            'rotation_rejections': sorted(rotation_rejections) if reason == 'holding' else [],
            'active_joint_limits': np.flatnonzero(~free).tolist(),
            'max_step_rad': limit, 'iterations': 1, 'damping': damping, 'sigma_min': sigma,
            'position_lead_m': self.position_lead_m, 'rotation_lead_rad': self.rotation_lead_rad,
            'limit_pressure_rad': pressure, 'joint_step_rad': float(np.max(np.abs(result - q))),
            'cost_before': before['cost'], 'cost_after': metrics['cost'],
            'final_fk_verified': True, 'position_guard_enabled': self.position_guard_enabled,
            'position_guard_m': position_bound if self.position_guard_enabled else None,
            **metrics,
            'converged': metrics['raw_position_error_m'] <= 1e-4 and metrics['raw_angle_error_rad'] <= 1e-3,
            'bounded_target_converged': metrics['position_error_m'] <= 1e-4 and metrics['angle_error_rad'] <= 1e-3,
            'elapsed_s': self.clock() - began,
        }
        return result.tolist()


def create_kinematics(instance):
    parameters = instance.get('parameters', {})
    maximum = .02
    if instance['plugin_id'].startswith('morla.airbot.mmk2_sdk.'):
        maximum = min(maximum, parameters.get('max_velocity', .1) / parameters.get('rate_hz', 20.), parameters.get('max_command_lead', .15) / 2.)
    return ArmKinematics(instance, max_step_rad=maximum)
