"""Explicit IK/controller adapters around the established MORLA solvers."""
from __future__ import annotations
import time
import numpy as np
from .controllers import ControllerError


def solver_for(instance):
    """Trusted built-in registration; assembly cannot supply Python import paths."""
    plugin=instance['plugin_id']; parameters=instance.get('parameters',{})
    from importlib import metadata
    providers=[entry for entry in metadata.entry_points(group='morla.kinematics') if entry.name==plugin]
    if len(providers)>1: raise ControllerError('ambiguous kinematics provider: '+plugin)
    if providers: return providers[0].load()(instance)
    raise ControllerError('no installed IK adapter for '+plugin)


class CartesianController:
    def __init__(self,solver,*,jog=False,clock=time.monotonic_ns,max_solve_ns=50_000_000):
        self.solver,self.jog,self.clock,self.max_solve_ns=solver,jog,clock,max_solve_ns
        self.reset()

    def reset(self):
        self.anchor_input=None; self.anchor_robot=None

    def __call__(self,value,measured,dt,parameters):
        from scipy.spatial.transform import Rotation
        rotation=np.asarray(parameters.get('input_rotation',np.eye(3)),dtype=float).reshape(3,3)
        if not np.isfinite(rotation).all() or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-8) or not np.isclose(np.linalg.det(rotation),1):
            raise ControllerError('input rotation must be a proper calibrated rotation')
        seed=np.asarray(measured,dtype=float)
        self.solver.sync_state(seed)
        tool=np.asarray(parameters.get('solver_tip_from_tool',np.eye(4)),dtype=float).reshape(4,4)
        from morla_core.coordinates import validate_transform
        validate_transform(tool)
        current=np.asarray(self.solver.fk(seed),dtype=float)@tool
        if self.jog:
            if not isinstance(value,dict): raise ControllerError('Cartesian jog requires named axes')
            mapping=parameters.get('axes_map',{'x':'left_x','y':'left_y','z':None,'roll':None,'pitch':None,'yaw':None})
            if not isinstance(mapping,dict) or set(mapping)-{'x','y','z','roll','pitch','yaw'}:
                raise ControllerError('invalid Cartesian axes map')
            axes=[]
            for axis in ('x','y','z','roll','pitch','yaw'):
                key=mapping.get(axis)
                if key is not None and key not in value: raise ControllerError('missing Cartesian axis '+key)
                axes.append(value[key] if key is not None else 0.0)
            axes=np.asarray(axes,dtype=float)
            if not np.isfinite(axes).all() or np.any(np.abs(axes)>1): raise ControllerError('invalid jog axis')
            speed=float(parameters.get('linear_speed',0.08)); angular=float(parameters.get('angular_speed',0.3))
            target=current.copy(); target[:3,3]+=rotation@axes[:3]*speed*dt
            target[:3,:3]=Rotation.from_rotvec(rotation@axes[3:]*angular*dt).as_matrix()@current[:3,:3]
        else:
            if not isinstance(value,dict): raise ControllerError('pose requires position/quaternion')
            position=np.asarray(value['position'],dtype=float); quat=np.asarray(value['quaternion'],dtype=float)
            if position.shape!=(3,) or quat.shape!=(4,) or not np.isfinite(position).all() or not np.isfinite(quat).all() or not np.isclose(np.linalg.norm(quat),1,atol=1e-3):
                raise ControllerError('invalid pose')
            input_pose=np.eye(4); input_pose[:3,3]=position; input_pose[:3,:3]=Rotation.from_quat(quat).as_matrix()
            if self.anchor_input is None:
                self.anchor_input=input_pose; self.anchor_robot=current.copy()
                return seed.tolist()
            scale=float(parameters.get('position_scale',1))
            target=self.anchor_robot.copy()
            target[:3,3]+=rotation@(position-self.anchor_input[:3,3])*scale
            delta=input_pose[:3,:3]@self.anchor_input[:3,:3].T
            target[:3,:3]=rotation@delta@rotation.T@self.anchor_robot[:3,:3]
        # Tool offsets must already be resolved and versioned by the compiler.
        target_tip=target@np.linalg.inv(tool)
        result=self.solver.solve(target_tip)
        if result is None: raise ControllerError('IK unreachable or limit failure')
        result=np.asarray(result,dtype=float)
        if result.shape!=seed.shape or not np.isfinite(result).all(): raise ControllerError('invalid IK result')
        return result.tolist()


def install_kinematics(registry,instance,solver=None):
    solver=solver or solver_for(instance)
    registry.register('morla.cartesian_delta_ik',CartesianController(solver))
    jog=CartesianController(solver,jog=True)
    jog.requires_fresh_input=instance.get('backend')=='real'
    registry.register('morla.gamepad_cartesian_jog',jog)
    return registry
