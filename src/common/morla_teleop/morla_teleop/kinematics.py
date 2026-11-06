"""Explicit IK/controller adapters around the established MORLA solvers."""
from __future__ import annotations
import inspect
import copy
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
        # Nero/Astral analytic solvers keep a continuity branch across solves.
        # Resetting it every tick turns each frame into a full cold search
        # (~100 ms) and makes the 50 Hz input queue go stale. Only reset the
        # branch on the first solve after an anchor/gap; later ticks warm-start.
        try:
            self._warm_sync='reset_branch' in inspect.signature(solver.sync_state).parameters
        except (TypeError,ValueError):
            self._warm_sync=False
        # A solver that accepts an explicit integration base lets the caller
        # (CR5 differential IK) step from the last command instead of being
        # re-seeded from a slow measured sample every tick.
        try:
            self._seed_aware='seed' in inspect.signature(solver.solve).parameters
        except (TypeError,ValueError):
            self._seed_aware=False
        self.reset()

    def reset(self):
        self.anchor_input=None; self.anchor_robot=None
        # Jog integrates on a commanded pose kept across ticks, never on the
        # measured pose: anchoring on the measurement would fold the servo's
        # gravity droop into the next command, so the arm would sink while a
        # stick is held and stay sunk after release.
        self.jog_target=None
        # Jog directions that hit an unreachable target. Retrying them costs a
        # ~120 ms failed solve per frame, so they stay masked until released.
        self.jog_blocked=None
        self._synced=False
        self.last_report={}

    def diagnostics(self):
        return copy.deepcopy(self.last_report)

    def reanchor(self, value, measured, parameters):
        """Acknowledge an FK/input anchor without generating a motion target."""
        from scipy.spatial.transform import Rotation
        from morla_core.coordinates import validate_transform, validate_basis_change
        seed=np.asarray(measured,dtype=float)
        if seed.ndim!=1 or not np.isfinite(seed).all():raise ControllerError('invalid anchor feedback')
        validate_basis_change(parameters.get('input_rotation',np.eye(3)))
        tool=np.asarray(parameters.get('solver_tip_from_tool',np.eye(4)),dtype=float)
        validate_transform(tool)
        self.reset()
        reset_solver=getattr(self.solver,'reset_teleop',None)
        if reset_solver is not None:reset_solver()
        self.solver.sync_state(seed)
        current=np.asarray(self.solver.fk(seed),dtype=float)@tool
        validate_transform(current)
        if self.jog:
            if not isinstance(value,dict) or not all(type(v) in (int,float) and np.isfinite(v) and abs(v)<=1 for v in value.values()):
                raise ControllerError('invalid anchor axes')
            mapping=parameters.get('axes_map',{'x':'left_x','y':'left_y'})
            if not isinstance(mapping,dict) or any(key is not None and key not in value for key in mapping.values()):
                raise ControllerError('invalid anchor axes map')
            self.jog_target=current.copy()
        else:
            position=np.asarray(value['position'],dtype=float);quat=np.asarray(value['quaternion'],dtype=float)
            if position.shape!=(3,) or quat.shape!=(4,) or not np.isfinite(position).all() or not np.isfinite(quat).all() or not np.isclose(np.linalg.norm(quat),1,atol=1e-3):
                raise ControllerError('invalid anchor pose')
            self.anchor_input=np.eye(4)
            self.anchor_input[:3,3]=position
            self.anchor_input[:3,:3]=Rotation.from_quat(quat).as_matrix()
            self.anchor_robot=current.copy()
        self._synced=True
        self.last_report={'stage':'anchored','feedback_seed':seed.tolist(),'current_tcp':current.tolist(),
                          'input':copy.deepcopy(value),'reanchor_acknowledged':True}
        return {'acknowledged':True,'kind':'cartesian','current_tcp':current.tolist()}

    def __call__(self,value,measured,dt,parameters,seed=None):
        """Map one input tick to joint commands.

        `seed` is the joint vector the solver integrates from. Callers that
        hold a commanded joint state (the CR5 async controller) pass it so the
        step continues from the last command instead of being re-seeded from a
        slow measured sample; `measured` still provides the true pose for the
        reported `current_tcp` and the feedback diagnostics.
        """
        from scipy.spatial.transform import Rotation
        from morla_core.coordinates import validate_basis_change
        # Raw-Unity input is left-handed, so the calibrated alignment is a basis
        # change with det -1. It only ever maps increments (and their rotation
        # conjugation stays proper), so accepting the reflection is safe here.
        try:
            rotation=validate_basis_change(
                np.asarray(parameters.get('input_rotation',np.eye(3)),dtype=float).reshape(3,3))
        except ValueError as exc:
            raise ControllerError('input rotation must be an orthonormal basis change: '+str(exc)) from exc
        try:
            position_rotation=validate_basis_change(np.asarray(parameters.get('input_position_rotation',rotation),dtype=float).reshape(3,3))
        except ValueError as exc:
            raise ControllerError('invalid input position mapping: '+str(exc)) from exc
        measured=np.asarray(measured,dtype=float)
        start=measured if seed is None else np.asarray(seed,dtype=float)
        if start.shape!=measured.shape or not np.isfinite(start).all():
            raise ControllerError('invalid controller seed')
        if not self._synced:
            self.solver.sync_state(start)
            self._synced=True
        elif self._warm_sync:
            self.solver.sync_state(start,reset_branch=False)
        else:
            self.solver.sync_state(start)
        tool=np.asarray(parameters.get('solver_tip_from_tool',np.eye(4)),dtype=float).reshape(4,4)
        from morla_core.coordinates import validate_transform
        validate_transform(tool)
        current=np.asarray(self.solver.fk(measured),dtype=float)@tool
        self.last_report={'stage':'mapping','input':copy.deepcopy(value),'feedback_seed':measured.tolist(),
                          'integration_seed':start.tolist(),
                          'current_tcp':current.tolist(),'dt_s':dt}
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
            if self.jog_blocked is not None:
                # A rejected analytic solve scans every arm-angle candidate
                # (~120 ms, vs ~5 ms for a warm solve), so re-attempting the
                # same wall at input rate starves the whole chain. Keep the
                # blocked directions out of the target until the operator
                # pulls the stick back, which clears them.
                if np.any(axes*self.jog_blocked<0):
                    self.jog_blocked=None
                else:
                    axes=np.where(axes*self.jog_blocked>0,0.0,axes)
            speed=float(parameters.get('linear_speed',0.08)); angular=float(parameters.get('angular_speed',0.3))
            if self.jog_target is None:
                self.jog_target=current.copy()
            # Integrate into a candidate and commit only after a successful
            # solve: a refused step must not wind the target further out.
            target=self.jog_target.copy()
            if np.any(axes):
                target[:3,3]+=rotation@axes[:3]*speed*dt
                target[:3,:3]=Rotation.from_rotvec(rotation@axes[3:]*angular*dt).as_matrix()@target[:3,:3]
        else:
            if not isinstance(value,dict): raise ControllerError('pose requires position/quaternion')
            position=np.asarray(value['position'],dtype=float); quat=np.asarray(value['quaternion'],dtype=float)
            if position.shape!=(3,) or quat.shape!=(4,) or not np.isfinite(position).all() or not np.isfinite(quat).all() or not np.isclose(np.linalg.norm(quat),1,atol=1e-3):
                raise ControllerError('invalid pose')
            input_pose=np.eye(4); input_pose[:3,3]=position; input_pose[:3,:3]=Rotation.from_quat(quat).as_matrix()
            if self.anchor_input is None:
                self.anchor_input=input_pose; self.anchor_robot=current.copy()
                self.last_report.update(stage='anchored',target_tcp=current.tolist(),target_values=start.tolist())
                return start.tolist()
            scale=float(parameters.get('position_scale',1))
            target=self.anchor_robot.copy()
            target[:3,3]+=position_rotation@(position-self.anchor_input[:3,3])*scale
            delta=input_pose[:3,:3]@self.anchor_input[:3,:3].T
            target[:3,:3]=rotation@delta@rotation.T@self.anchor_robot[:3,:3]
        # Tool offsets must already be resolved and versioned by the compiler.
        target_tip=target@np.linalg.inv(tool)
        self.last_report.update(stage='solving',target_tcp=target.tolist(),target_tip=target_tip.tolist())
        solve_arguments={'seed':start} if (seed is not None and self._seed_aware) else {}
        result=self.solver.solve(target_tip,**solve_arguments)
        self.last_report.update(stage='held' if result is None else 'solved',
                                target_values=None if result is None else np.asarray(result).tolist(),
                                solver_report=copy.deepcopy(getattr(self.solver,'last_report',{})))
        if result is None:
            # Both legacy teleop loops (nero/astral) skip the tick and stay
            # armed on an unreachable pose. Raising here let one reach/limit
            # event revoke the composable session. Dropping the frame keeps
            # the last feasible jog target, so reversing the stick resumes
            # immediately instead of requiring prepare/activate again.
            if self.jog: self.jog_blocked=axes.copy()
            return None
        result=np.asarray(result,dtype=float)
        if result.shape!=start.shape or not np.isfinite(result).all(): raise ControllerError('invalid IK result')
        if self.jog: self.jog_target=target
        return result.tolist()


def install_kinematics(registry,instance,solver=None):
    solver=solver or solver_for(instance)
    registry.register('morla.cartesian_delta_ik',CartesianController(solver))
    jog=CartesianController(solver,jog=True)
    jog.requires_fresh_input=instance.get('backend')=='real'
    registry.register('morla.gamepad_cartesian_jog',jog)
    return registry
