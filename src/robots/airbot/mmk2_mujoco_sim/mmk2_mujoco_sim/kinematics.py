"""Per-arm bounded differential IK using the exact URDF-derived scene.

Targets and FK are expressed in that arm's URDF base frame, so chassis/lift
motion does not change the meaning of a hand-relative teleoperation target.

The policy is the same one morla_mmk2_driver and FR3/CR5 use: absorb excess
Cartesian demand into a bounded effective goal, take one improving DLS step per
tick, and always return a feasible joint command. A converged solve is not
claimed, and a tick that cannot improve holds the current joints. The earlier
sim-only variant iterated to a pose tolerance, hard-clipped to the joint range
each iteration, and returned None when the 35 ms budget ran out; at a wrist stop
the residual then stopped falling, the loop spun until the deadline, and the
controller discarded the tick. The arm froze for up to 1.9 s while the operator
kept turning. Holding is a valid teleoperation answer, and the absorbing goal
means the excess demand is dropped instead of accumulating into a long freeze.
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .scene import model_xml


# Optional instance parameters. Defaults and bounds match morla_mmk2_driver and
# the CR5/FR3 differential solvers; the same defaults keep sim and real equal.
_PARAMETERS = (('position_lead_m', .02, .25),
               ('rotation_lead_rad', .35, 1.),
               ('damping_min', .005, 1.),
               ('damping_max', .145, 1.),
               ('singularity_threshold', .05, 1.))


def differential_parameters(instance):
    given = (instance or {}).get('parameters') or {}
    values = {}
    for name, default, maximum in _PARAMETERS:
        value = given.get(name, default)
        if type(value) not in (int, float) or not np.isfinite(value) or not 0 < value <= maximum:
            raise ValueError('Invalid MMK2 differential IK ' + name)
        values[name] = float(value)
    if values['damping_min'] > values['damping_max']:
        raise ValueError('MMK2 differential IK damping_min exceeds damping_max')
    return values


class ArmKinematics:
    def __init__(self, instance):
        import mujoco
        self.mj = mujoco
        side = instance['plugin_id'].split('.')[-1].removesuffix('_arm')
        self.model = mujoco.MjModel.from_xml_string(model_xml())
        self.data = mujoco.MjData(self.model)
        ids = [mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,side+'_joint'+str(i)) for i in range(1,7)]
        self.qadr = self.model.jnt_qposadr[ids]; self.dadr = self.model.jnt_dofadr[ids]
        self.bounds = self.model.jnt_range[ids].copy()
        self.base = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,side+'_base_link')
        self.tip = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,side+'_gripper_center')
        self.q = np.zeros(6); self.last_report = {}
        parameters = differential_parameters(instance)
        self.position_lead_m = parameters['position_lead_m']
        self.rotation_lead_rad = parameters['rotation_lead_rad']
        self.damping_min = parameters['damping_min']
        self.damping_max = parameters['damping_max']
        self.singularity_threshold = parameters['singularity_threshold']
        self.reset_teleop()

    def reset_teleop(self):
        # Only the last effective goal is kept, for diagnostics. Nothing is
        # carried into the next tick: the goal is derived from the raw target.
        self.goal = None

    def sync_state(self, q):
        q = np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():
            raise ValueError('MMK2_IK_INVALID_SEED')
        self.q = q.copy()

    def fk(self, q):
        self.sync_state(q)
        self.data.qpos[self.qadr] = q
        self.mj.mj_forward(self.model,self.data)
        R = self.data.xmat[self.base].reshape(3,3)
        T = np.eye(4)
        T[:3,:3] = R.T @ self.data.xmat[self.tip].reshape(3,3)
        T[:3,3] = R.T @ (self.data.xpos[self.tip]-self.data.xpos[self.base])
        return T

    def solve(self, target, seed=None):
        """One improving DLS step toward an absorbing, lead-bounded goal.

        The effective goal is re-derived from `target` every tick rather than
        integrated from the previous one. Integrating looked equivalent (the
        increments telescope) but the lead clamp writes the clamped pose back,
        so every tick the arm spent saturated or rate-limited *dropped* that
        demand permanently: after one wrist saturation the arm kept a standing
        offset of ~0.2 m / 1.1 rad from the hand pose for the rest of the
        session, and the measured `raw_position_error_m` never returned to
        zero even with the hand still. Re-deriving keeps the same per-tick
        absorbing behaviour (excess demand beyond the lead is not commanded)
        while making the map absolute again: the arm converges to the hand
        whenever the pose is reachable, and a reversal still acts immediately
        because the goal direction comes from the current raw error.
        """
        target = np.asarray(target,dtype=float)
        if target.shape!=(4,4) or not np.isfinite(target).all():
            raise ValueError('MMK2_IK_INVALID_TARGET')
        q0 = self.q.copy() if seed is None else np.asarray(seed,dtype=float).copy()
        if q0.shape!=(6,) or not np.isfinite(q0).all():
            raise ValueError('MMK2_IK_INVALID_SEED')
        began = time.monotonic()
        current = self.fk(q0)
        goal = target.copy()
        translation = goal[:3,3]-current[:3,3]
        distance = float(np.linalg.norm(translation))
        if distance>self.position_lead_m:
            goal[:3,3]=current[:3,3]+translation*(self.position_lead_m/distance)
        rotation = Rotation.from_matrix(goal[:3,:3]@current[:3,:3].T).as_rotvec()
        angle = float(np.linalg.norm(rotation))
        if angle>self.rotation_lead_rad:
            goal[:3,:3]=Rotation.from_rotvec(rotation*(self.rotation_lead_rad/angle)).as_matrix()@current[:3,:3]
        self.goal = goal.copy()
        def error(T):
            return np.r_[goal[:3,3]-T[:3,3],.3*Rotation.from_matrix(goal[:3,:3]@T[:3,:3].T).as_rotvec()]
        residual = error(current)
        jp,jr = np.zeros((3,self.model.nv)),np.zeros((3,self.model.nv))
        self.mj.mj_jacBody(self.model,self.data,jp,jr,self.tip)
        R = self.data.xmat[self.base].reshape(3,3)
        J = np.vstack((R.T@jp[:,self.dadr],.3*R.T@jr[:,self.dadr]))
        sigma = float(np.linalg.svd(J,compute_uv=False)[-1])
        damping = self.damping_min+(self.damping_max-self.damping_min)*max(0.,1-sigma/self.singularity_threshold)**2
        # A joint resting on its stop cannot move outward, but the damped
        # least-squares direction still spends part of the step on it. Sending
        # that step means the clip cancels part of it, the trial no longer
        # reduces the error and every backtracking alpha fails, so the arm
        # wedged at the stop even when the hand came back. Solve again with the
        # outward joints held (active set) so the remaining joints carry the
        # step, and report the refused demand as limit pressure.
        resting_low = q0 <= self.bounds[:,0]+1e-9
        resting_high = q0 >= self.bounds[:,1]-1e-9
        free = np.ones(6,bool)
        dq = np.zeros(6)
        unconstrained = None
        blocked = np.zeros(6,bool)
        for _ in range(6):
            Jf = J[:,free]
            dqf = Jf.T@np.linalg.solve(Jf@Jf.T+damping**2*np.eye(6),residual)
            if unconstrained is None:
                unconstrained = dqf.copy()
            dq[:] = 0.
            dq[free] = dqf
            outward = (resting_low&(dq<0))|(resting_high&(dq>0))
            if not outward.any():
                break
            blocked |= outward
            free[outward] = False
        if not np.isfinite(dq).all():
            raise ValueError('MMK2_IK_NONFINITE_UPDATE')
        limit_pressure = float(np.linalg.norm(unconstrained[blocked])) if blocked.any() else 0.
        result = q0.copy(); reason = 'holding'
        for alpha in (1.,.5,.25,.125):
            if time.monotonic()-began>.02:
                reason = 'deadline_hold'; break
            trial_unclipped = q0+alpha*dq
            trial = np.clip(trial_unclipped,self.bounds[:,0],self.bounds[:,1])
            limit_pressure = max(limit_pressure,float(np.linalg.norm(trial_unclipped-trial)))
            candidate = error(self.fk(trial))
            if float(candidate@candidate)<float(residual@residual)-1e-18:
                result = trial; reason = 'tracking'; break
        if time.monotonic()-began>.02:
            result = q0.copy(); reason = 'deadline_hold'
        self.q = result.copy()
        actual = self.fk(result)
        pos = float(np.linalg.norm(actual[:3,3]-goal[:3,3]))
        ang = float(Rotation.from_matrix(actual[:3,:3].T@goal[:3,:3]).magnitude())
        raw_pos = float(np.linalg.norm(actual[:3,3]-target[:3,3]))
        raw_ang = float(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude())
        self.last_report = dict(method='mmk2_urdf_differential',reason=reason,
            exact_convergence=False,damping=damping,sigma_min=sigma,
            position_lead_m=self.position_lead_m,rotation_lead_rad=self.rotation_lead_rad,
            limit_pressure_rad=limit_pressure,joint_step_rad=float(np.max(np.abs(result-q0))),
            position_error_m=pos,angle_error_rad=ang,
            raw_position_error_m=raw_pos,raw_angle_error_rad=raw_ang,
            converged=raw_pos<=1e-4 and raw_ang<=1e-3,
            bounded_target_converged=pos<=1e-4 and ang<=1e-3,
            elapsed_s=float(time.monotonic()-began))
        return result.tolist()


def create_kinematics(instance):
    return ArmKinematics(instance)
