"""Standard-DH FR3 flange solver, independent of CR5/Nero and the network SDK.

Manual 4.0.1, table 2.4-1. SI metres/radians; no tool/TCP baked into FK.
Numerical pose IK stays near its measured seed and returns None on failure.
"""
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from morla_core.coordinates import validate_transform
from .contract import limits, positions

A = (0., -.280, -.240, 0., 0., 0.)
D = (.140, 0., 0., .102, .102, .100)
ALPHA = (math.pi / 2, 0., 0., math.pi / 2, -math.pi / 2, 0.)


class FR3Kinematics:
    def __init__(self, parameters=None):
        self.parameters = parameters or {}
        self.lower, self.upper = map(np.asarray, limits(self.parameters))
        self.seed = None
        self.last_report = {}

    def sync_state(self, q):
        array = np.asarray(q, dtype=float)
        if array.shape != (6,):
            raise ValueError('FR3 IK requires six joints')
        self.seed = np.asarray(positions(array.tolist(), self.parameters))

    def chain(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('FR3 FK requires six finite radians')
        transform = np.eye(4)
        origins, axes = [], []
        for theta, a, d, alpha in zip(q, A, D, ALPHA):
            origins.append(transform[:3, 3].copy())
            axes.append(transform[:3, 2].copy())
            c, s, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
            transform = transform @ np.array([
                [c, -s * ca, s * sa, a * c], [s, c * ca, -c * sa, a * s],
                [0., sa, ca, d], [0., 0., 0., 1.],
            ])
        return transform, origins, axes

    def fk(self, q):
        return self.chain(q)[0]

    def jacobian(self, q):
        transform, origins, axes = self.chain(q)
        return np.array([np.r_[np.cross(z, transform[:3, 3] - p), z]
                         for p, z in zip(origins, axes)]).T

    def solve(self, target):
        target = validate_transform(target)
        if self.seed is None:
            raise ValueError('FR3 IK requires measured joint seed')
        q = self.seed.copy()
        lo = np.maximum(self.lower, self.seed - .3)
        hi = np.minimum(self.upper, self.seed + .3)
        started = time.monotonic()
        for iteration in range(50):
            current = self.fk(q)
            error = np.r_[target[:3, 3] - current[:3, 3],
                          Rotation.from_matrix(target[:3, :3] @ current[:3, :3].T).as_rotvec()]
            pos, angle = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
            self.last_report = {'position_error_m': float(pos), 'rotation_error_rad': float(angle),
                                'iterations': iteration, 'solve_ms': (time.monotonic() - started) * 1000}
            if pos < .0002 and angle < .002:
                return q
            if time.monotonic() - started > .03:
                break
            jac = self.jacobian(q)
            delta = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 1e-5, error)
            norm = np.max(np.abs(delta))
            if norm > .05:
                delta *= .05 / norm
            candidate = np.clip(q + delta, lo, hi)
            if np.max(np.abs(candidate - q)) < 1e-9:
                break
            q = candidate
        self.last_report['held'] = True
        return None


def create_kinematics(instance):
    return FR3Kinematics(instance.get('parameters'))
