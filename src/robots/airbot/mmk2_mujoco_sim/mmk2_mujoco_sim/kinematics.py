"""Independent per-arm bounded DLS IK using the exact URDF-derived scene.

Targets and FK are expressed in that arm's URDF base frame, so chassis/lift
motion does not change the meaning of a hand-relative teleoperation target.
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .scene import model_xml


class ArmKinematics:
    def __init__(self,instance):
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

    def sync_state(self,q):
        q = np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():
            raise ValueError('MMK2_IK_INVALID_SEED')
        self.q = q.copy()

    def fk(self,q):
        self.sync_state(q)
        self.data.qpos[self.qadr] = q
        self.mj.mj_forward(self.model,self.data)
        R = self.data.xmat[self.base].reshape(3,3)
        T = np.eye(4)
        T[:3,:3] = R.T @ self.data.xmat[self.tip].reshape(3,3)
        T[:3,3] = R.T @ (self.data.xpos[self.tip]-self.data.xpos[self.base])
        return T

    def solve(self,target,seed=None):
        target = np.asarray(target,dtype=float)
        if target.shape!=(4,4) or not np.isfinite(target).all():
            raise ValueError('MMK2_IK_INVALID_TARGET')
        q = self.q.copy() if seed is None else np.asarray(seed,dtype=float).copy()
        began = time.monotonic()
        for iteration in range(60):
            T = self.fk(q)
            error = np.r_[target[:3,3]-T[:3,3],Rotation.from_matrix(target[:3,:3]@T[:3,:3].T).as_rotvec()]
            if np.linalg.norm(error[:3])<1e-4 and np.linalg.norm(error[3:])<1e-3:
                self.q=q.copy(); self.last_report=dict(converged=True,iterations=iteration)
                return q.tolist()
            if time.monotonic()-began>.035:
                break
            jp,jr = np.zeros((3,self.model.nv)),np.zeros((3,self.model.nv))
            self.mj.mj_jacBody(self.model,self.data,jp,jr,self.tip)
            R = self.data.xmat[self.base].reshape(3,3)
            J = np.vstack((R.T@jp[:,self.dadr],R.T@jr[:,self.dadr]))
            delta = J.T@np.linalg.solve(J@J.T+.002**2*np.eye(6),error)
            q = np.clip(q+np.clip(delta,-.1,.1),self.bounds[:,0],self.bounds[:,1])
        self.last_report=dict(converged=False,iterations=iteration)
        return None


def create_kinematics(instance):
    return ArmKinematics(instance)
