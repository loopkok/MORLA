# Adapted from MORLA CR5 implementation at 063168d. Algorithm unchanged;
# FR3 model, joint names, limits, frames and factories are device-specific.
"""Bounded full-Jacobian FR3 teleop IK, with absorbing target lead limits.

Uses the versioned FR3 Pinocchio model and hardware joint mapping. It does
not split position and wrist into separate joint groups. One DLS update per
sample returns a feasible improving step or a hold; it does not claim exact
pose convergence. IKSolver remains the separate strict pose solver.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from morla_core.coordinates import validate_transform
from .math_utils import vector6
from .urdf_ik import IKSolver


class DifferentialIKSolver(IKSolver):
    method_name = 'fr3_urdf_pinocchio_differential'

    def __init__(self, *, position_lead_m=.05, rotation_lead_rad=.35,
                 damping_min=.005, damping_max=.15, singularity_threshold=.05,
                 max_step_rad=.04, **kwargs):
        super().__init__(**kwargs)
        for name, value, maximum in [('position_lead_m',position_lead_m,.25),
                                    ('rotation_lead_rad',rotation_lead_rad,1.),
                                    ('damping_min',damping_min,1.),('damping_max',damping_max,1.),
                                    ('singularity_threshold',singularity_threshold,1.),
                                    ('max_step_rad',max_step_rad,.15)]:
            if type(value) not in (float,int) or not np.isfinite(value) or not 0<value<=maximum:
                raise ValueError('Invalid differential IK '+name)
        if damping_min>damping_max:
            raise ValueError('Differential IK damping_min exceeds damping_max')
        self.position_lead_m=position_lead_m;self.rotation_lead_rad=rotation_lead_rad
        self.damping_min=damping_min;self.damping_max=damping_max
        self.singularity_threshold=singularity_threshold;self.max_step_rad=max_step_rad
        self.reset_teleop()

    def reset_teleop(self):
        self.previous_raw=None
        self.goal=None
        self.last_report={}

    def _target(self, raw, current):
        if self.previous_raw is None:
            goal=raw.copy()
        else:
            goal=self.goal.copy()
            goal[:3,3]+=raw[:3,3]-self.previous_raw[:3,3]
            goal[:3,:3]=(raw[:3,:3]@self.previous_raw[:3,:3].T)@goal[:3,:3]
        # Store raw and effective targets separately: excess demand is absorbed,
        # so reversing the hand at a boundary acts immediately, without unwind.
        self.previous_raw=raw.copy()
        translation=goal[:3,3]-current[:3,3]
        distance=float(np.linalg.norm(translation))
        if distance>self.position_lead_m:
            goal[:3,3]=current[:3,3]+translation*(self.position_lead_m/distance)
        rotation=Rotation.from_matrix(goal[:3,:3]@current[:3,:3].T).as_rotvec()
        angle=float(np.linalg.norm(rotation))
        if angle>self.rotation_lead_rad:
            goal[:3,:3]=Rotation.from_rotvec(rotation*(self.rotation_lead_rad/angle)).as_matrix()@current[:3,:3]
        self.goal=goal.copy()
        return goal, distance>self.position_lead_m, angle>self.rotation_lead_rad

    def solve(self, target, seed=None):
        """Same bounded absorbing DLS policy as CR5, using the FR3 model.

        Explicit seed integrates from the last command; measured feedback still
        bounds command lead in the asynchronous controller. No feedback rate
        or physical servo equivalence is assumed for FR3.
        """
        if self.seed is None:
            raise RuntimeError('Measured FR3 seed required')
        raw=validate_transform(target)
        q0=vector6(self.seed if seed is None else seed)
        started=self.clock();q=q0.copy();current=self._placement(q)
        goal, pos_limited, rot_limited=self._target(raw,current.homogeneous)
        desired=self.pin.SE3(goal[:3,:3],goal[:3,3])
        delta=current.actInv(desired)
        weights=np.diag([1.,1.,1.,.3,.3,.3])
        error=weights@self.pin.log6(delta).vector
        J=self.pin.computeFrameJacobian(self.model,self.data,self.signs*q+self.offsets,
                                        self.tip_id,self.pin.ReferenceFrame.LOCAL)
        J=-self.pin.Jlog6(delta.inverse())@J@np.diag(self.signs)
        A=weights@J
        sigma=float(np.linalg.svd(A,compute_uv=False)[-1])
        proximity=float(np.clip(1-sigma/self.singularity_threshold,0,1))
        damping=self.damping_min+(self.damping_max-self.damping_min)*proximity**2
        dq=-A.T@np.linalg.solve(A@A.T+damping**2*np.eye(6),error)
        if not np.isfinite(dq).all():
            raise ValueError('FR3 differential IK nonfinite update')
        dq*=min(1.,self.max_step_rad/max(float(np.max(np.abs(dq))),1e-12))
        cost=float(error@error);result=q.copy();reason='holding';limit_pressure=0.
        for alpha in (1.,.5,.25,.125):
            if self.clock()-started>self.max_solve_s:
                reason='deadline_hold';break
            trial_unclipped=q+alpha*dq
            trial=np.clip(trial_unclipped,self.lower_limits,self.upper_limits)
            limit_pressure=max(limit_pressure,float(np.linalg.norm(trial_unclipped-trial)))
            residual=weights@self.pin.log6(self._placement(trial).actInv(desired)).vector
            if float(residual@residual)<cost-1e-16:
                result=trial;reason='tracking';break
        # A late update must not become a motion command. A hold is still a
        # valid teleop response; bad input/model/numerics remain exceptions.
        if self.clock()-started>self.max_solve_s:
            result=q.copy();reason='deadline_hold'
        actual=self.fk(result)
        pos=float(np.linalg.norm(actual[:3,3]-goal[:3,3]))
        angle=float(Rotation.from_matrix(actual[:3,:3].T@goal[:3,:3]).magnitude())
        bounded_converged=pos<=self.position_tolerance_m and angle<=self.angle_tolerance_rad
        raw_pos=float(np.linalg.norm(actual[:3,3]-raw[:3,3]))
        raw_angle=float(Rotation.from_matrix(actual[:3,:3].T@raw[:3,:3]).magnitude())
        converged=raw_pos<=self.position_tolerance_m and raw_angle<=self.angle_tolerance_rad
        self.last_report={'mode':'differential','accepted':True,'converged':bool(converged),
            'bounded_target_converged':bool(bounded_converged),
            'reason':'converged' if converged else ('absorbed_target_hold' if bounded_converged else reason),'position_error_m':pos,'angle_error_rad':angle,
            'raw_position_error_m':raw_pos,'raw_angle_error_rad':raw_angle,
            'absorbed_position_m':float(np.linalg.norm(raw[:3,3]-goal[:3,3])),
            'absorbed_angle_rad':float(Rotation.from_matrix(raw[:3,:3]@goal[:3,:3].T).magnitude()),
            'position_lead_limited':bool(pos_limited),'rotation_lead_limited':bool(rot_limited),
            'sigma_min':sigma,'singularity_proximity':proximity,'damping':damping,
            'limit_pressure_rad':limit_pressure,'joint_step_rad':float(np.max(np.abs(result-q))),
            'elapsed_s':float(self.clock()-started)}
        return result
