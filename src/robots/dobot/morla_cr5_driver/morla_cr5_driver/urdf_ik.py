"""CR5 fixed-base URDF IK: local, measured-seeded, bounded SE(3) DLS.

No network calls or controller IK fallback. API matches the public Cartesian
controller (sync_state / fk / solve); returned joints stay in hardware order.
"""
import hashlib
import time
import numpy as np
from morla_core.coordinates import validate_transform
from morla_core.resource_locator import resolve_robot_asset, morla_source_root, ResourceLookupError
from .kinematics import vector6

MODEL_ID = 'cr5_arm'
MODEL_SHA256 = 'bf3df73a243f8dff6b1f4810b340a661780e3dd27f0bfbbfb3ec8d99ac7d5a73'
JOINT_NAMES = tuple('joint'+str(i) for i in range(1, 7))


def default_urdf_path():
    try:
        return resolve_robot_asset(MODEL_ID)
    except ResourceLookupError:
        # Source-only tests/tools; never fall back to an external workspace.
        from morla_core.resource_locator import package_share
        try:
            package_share('robot_description')
        except ResourceLookupError:
            return morla_source_root()/'src/robot_description/urdf/cr5/cr5.urdf'
        raise  # An installed but incomplete package must not silently use source.


class IKSolver:
    """One solver per controller worker; Pinocchio Data is deliberately not shared."""
    method_name = 'cr5_urdf_pinocchio_dls'
    base_frame = 'base_link'
    tip_frame = 'Link6'

    def __init__(self, *, joint_lower=None, joint_upper=None, joint_signs=None,
                 joint_offsets=None, urdf_path=None, max_iterations=100,
                 max_solve_s=.05, clock=time.monotonic):
        import pinocchio as pin
        self.pin = pin
        self.path = default_urdf_path() if urdf_path is None else urdf_path
        from pathlib import Path
        self.path = Path(self.path)
        self.model_sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        if self.model_sha256 != MODEL_SHA256:
            raise ValueError('CR5 URDF hash differs from the versioned model contract')
        self.model = pin.buildModelFromUrdf(str(self.path))
        if self.model.nq != 6 or self.model.nv != 6 or tuple(self.model.names)[1:] != JOINT_NAMES:
            raise ValueError('CR5 URDF must contain exactly joint1..joint6 in canonical order')
        self.data = self.model.createData()
        self.tip_id = self.model.getFrameId(self.tip_frame)
        if self.tip_id >= self.model.nframes:
            raise ValueError('CR5 URDF tip frame missing')
        self.signs = vector6([1]*6 if joint_signs is None else joint_signs)
        self.offsets = vector6([0]*6 if joint_offsets is None else joint_offsets)
        if not np.all(np.isin(self.signs, [-1, 1])):
            raise ValueError('CR5 joint signs must be +1/-1')
        # q_model = signs * q_hardware + offsets. Intersect both limit sets.
        a = (self.model.lowerPositionLimit-self.offsets)/self.signs
        b = (self.model.upperPositionLimit-self.offsets)/self.signs
        self.lower_limits, self.upper_limits = np.minimum(a,b), np.maximum(a,b)
        if joint_lower is not None:self.lower_limits=np.maximum(self.lower_limits,vector6(joint_lower))
        if joint_upper is not None:self.upper_limits=np.minimum(self.upper_limits,vector6(joint_upper))
        if np.any(self.lower_limits >= self.upper_limits):
            raise ValueError('CR5 deployment and URDF limits have no intersection')
        if type(max_iterations) is not int or not 1<=max_iterations<=500:
            raise ValueError('Invalid CR5 IK iteration bound')
        if not np.isfinite(max_solve_s) or not 0<max_solve_s<=.25:
            raise ValueError('Invalid CR5 IK deadline')
        self.max_iterations, self.max_solve_s, self.clock = max_iterations, max_solve_s, clock
        self.seed = None
        self.last_report = {}

    def sync_state(self, q):
        q=vector6(q)
        if np.any(q<self.lower_limits) or np.any(q>self.upper_limits):
            raise ValueError('Measured CR5 joints outside URDF/deployment limits')
        self.seed=q.copy()

    def _placement(self, q):
        self.pin.forwardKinematics(self.model,self.data,self.signs*q+self.offsets)
        self.pin.updateFramePlacements(self.model,self.data)
        return self.data.oMf[self.tip_id].copy()

    def fk(self, q):
        return self._placement(vector6(q)).homogeneous.copy()

    def solve(self, target):
        if self.seed is None:raise RuntimeError('Measured CR5 seed required')
        target=validate_transform(target)
        desired=self.pin.SE3(target[:3,:3],target[:3,3])
        q=self.seed.copy();started=self.clock()
        weights=np.diag([1.,1.,1.,.3,.3,.3])
        # A local trust region prevents selection of a distant equivalent branch.
        lo=np.maximum(self.lower_limits,self.seed-.3)
        hi=np.minimum(self.upper_limits,self.seed+.3)
        reason='iteration_limit';pos=angle=float('inf')
        for iteration in range(self.max_iterations):
            if self.clock()-started>self.max_solve_s:
                reason='deadline';break
            current=self._placement(q)
            delta=current.actInv(desired)
            error=self.pin.log6(delta).vector
            pos=float(np.linalg.norm(current.translation-desired.translation))
            angle=float(np.linalg.norm(self.pin.log3(current.rotation.T@desired.rotation)))
            if pos<=.0002 and angle<=.002:
                # Independent FK acceptance on the exact returned hardware vector.
                actual=self.fk(q)
                accepted=(np.isfinite(q).all() and np.all(q>=lo) and np.all(q<=hi)
                          and np.linalg.norm(actual[:3,3]-target[:3,3])<=.0002
                          and np.linalg.norm(self.pin.log3(actual[:3,:3].T@target[:3,:3]))<=.002
                          and self.clock()-started<=self.max_solve_s)
                if accepted:
                    self.last_report={'success':True,'iterations':iteration,'position_error_m':pos,
                                      'angle_error_rad':angle,'elapsed_s':self.clock()-started}
                    return q.copy()
                reason='fk_verification';break
            J=self.pin.computeFrameJacobian(self.model,self.data,self.signs*q+self.offsets,
                                            self.tip_id,self.pin.ReferenceFrame.LOCAL)
            J=-self.pin.Jlog6(delta.inverse())@J@np.diag(self.signs)
            A=weights@J;e=weights@error
            # Damped least squares remains finite near singularities.
            dq=-A.T@np.linalg.solve(A@A.T+1e-5*np.eye(6),e)
            dq*=min(1.,.06/max(float(np.max(np.abs(dq))),1e-12))
            cost=float(e@e);improved=False
            for alpha in (1.,.5,.25,.125):
                trial=np.clip(q+alpha*dq,lo,hi)
                trial_error=weights@self.pin.log6(self._placement(trial).actInv(desired)).vector
                if float(trial_error@trial_error)<cost-1e-16:
                    q=trial;improved=True;break
            if not improved:
                reason='stalled_or_unreachable';break
        self.last_report={'success':False,'reason':reason,'position_error_m':pos,
                          'angle_error_rad':angle,'elapsed_s':self.clock()-started}
        return None
