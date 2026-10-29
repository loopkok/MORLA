"""Pose conversion and legacy diagnostic/test helpers.

VR uses urdf_ik.IKSolver exclusively. ControllerSolver and FakeSolver remain
for the earlier protocol regression; neither is selected by the VR factory.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from morla_core.coordinates import validate_transform


def vector6(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('Expected six finite values')
    return values


def pose_from_controller(values):
    values = vector6(values)
    pose = np.eye(4)
    pose[:3, 3] = values[:3] / 1000.0
    pose[:3, :3] = Rotation.from_euler('xyz', values[3:], degrees=True).as_matrix()
    return pose


def pose_to_controller(pose):
    pose = validate_transform(pose)
    return [* (pose[:3, 3] * 1000.0),
            * Rotation.from_matrix(pose[:3, :3]).as_euler('xyz', degrees=True)]


class ControllerSolver:
    """Uses the adapter's owned dashboard, serialized with enable/stop, never writes targets."""
    def __init__(self, adapter):
        if adapter.parameters.get('kinematics_verified') is not True:
            raise ValueError('CR5 kinematics/base/tool calibration must be verified')
        self.adapter = adapter
        self.seed = None

    def sync_state(self, q):
        self.seed = vector6(q).copy()

    def _request(self, operation, *args):
        if not self.adapter.io.acquire(timeout=.05):
            raise TimeoutError('CR5 dashboard busy')
        try:
            if self.adapter.shutdown.is_set() or self.adapter.transport is None:
                raise RuntimeError('CR5 adapter closed')
            return getattr(self.adapter.transport, operation)(*args)
        finally:
            self.adapter.io.release()

    def fk(self, q):
        return pose_from_controller(self._request('forward_kinematics', vector6(q)))

    def solve(self, target):
        if self.seed is None:
            raise RuntimeError('Measured seed required')
        result = np.deg2rad(vector6(self._request('inverse_kinematics',
                                                pose_to_controller(target), self.seed)))
        lo = np.asarray(self.adapter.parameters['joint_lower'])
        hi = np.asarray(self.adapter.parameters['joint_upper'])
        if np.any(result < lo) or np.any(result > hi):
            raise ValueError('CR5 IK outside deployment limits')
        # Detect incompatible controller coordinate conventions instead of moving on a bad solution.
        actual = self.fk(result)
        position_error = np.linalg.norm(actual[:3, 3] - target[:3, 3])
        angle_error = Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).magnitude()
        if position_error > .002 or angle_error > .02:
            raise ValueError('CR5 IK/FK verification failed')
        return result


class FakeSolver:
    """Protocol fixture only: q[:3]*0.1 metres and XYZ Euler q[3:]. Not CR5 geometry."""
    def sync_state(self, q):
        self.seed = vector6(q)

    def fk(self, q):
        q = vector6(q)
        result = np.eye(4)
        result[:3, 3] = q[:3] * .1
        result[:3, :3] = Rotation.from_euler('xyz', q[3:]).as_matrix()
        return result

    def solve(self, target):
        target = validate_transform(target)
        return np.r_[target[:3, 3] / .1, Rotation.from_matrix(target[:3, :3]).as_euler('xyz')]
