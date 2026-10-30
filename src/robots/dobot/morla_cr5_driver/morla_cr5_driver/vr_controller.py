"""Asynchronous CR5 pose controller; all movement still goes through runtime.submit."""
import copy
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
from morla_teleop.kinematics import CartesianController
from .kinematics import vector6
from .urdf_ik import IKSolver
from morla_core.coordinates import validate_rotation, validate_transform


def validate_parameters(parameters, *, jog=False):
    allowed={'position_scale','input_rotation','solver_tip_from_tool','max_joint_step',
             'max_joint_velocity','calibration_verified','smoothing_alpha'}
    if jog:allowed|={'axes_map','linear_speed','angular_speed'}
    if not isinstance(parameters,dict) or set(parameters)-allowed:
        raise ValueError('Unknown CR5 VR controller parameter')
    for name,maximum in [('position_scale',2),('max_joint_step',.3),
                          ('max_joint_velocity',1),('smoothing_alpha',1)]:
        if name in parameters:
            value=parameters[name]
            if type(value) not in (int,float) or not np.isfinite(value) or not 0<value<=maximum:
                raise ValueError('Invalid CR5 VR '+name)
    if jog:
        mapping=parameters.get('axes_map',{})
        if not isinstance(mapping,dict) or set(mapping)-{'x','y','z','roll','pitch','yaw'}:
            raise ValueError('Invalid Cartesian axes_map')
        for key,maximum in [('linear_speed',.25),('angular_speed',1)]:
            if key in parameters and (type(parameters[key]) not in (int,float) or not np.isfinite(parameters[key]) or not 0<parameters[key]<=maximum):
                raise ValueError('Invalid Cartesian '+key)
    if 'input_rotation' in parameters:validate_rotation(parameters['input_rotation'])
    if 'solver_tip_from_tool' in parameters:validate_transform(parameters['solver_tip_from_tool'])


class AsyncCartesianController:
    def __init__(self, solver, *, clock=time.monotonic_ns, jog=False):
        self.clock = clock
        self.jog=jog
        self.controller = CartesianController(solver, jog=jog, max_solve_ns=250_000_000)
        self.condition = threading.Condition()
        self.generation = 0
        self.worker_generation = -1
        self.filtered_pose = None
        self.pending = None
        self.result = None
        self.error = None
        self.first_request_ns = None
        self.closed = False
        self.thread = threading.Thread(target=self._run, daemon=True, name='cr5-vr-kinematics')
        self.thread.start()

    def reset(self):
        with self.condition:
            self.generation += 1
            self.pending = self.result = self.error = None
            self.first_request_ns = None

    def __call__(self, value, measured, dt, parameters):
        measured = vector6(measured)
        now = self.clock()
        with self.condition:
            if self.closed: raise RuntimeError('CR5 VR controller closed')
            if self.error: raise RuntimeError('CR5 IK failed: ' + self.error)
            if self.first_request_ns is None: self.first_request_ns = now
            if self.result is None and now - self.first_request_ns > 300_000_000:
                raise TimeoutError('CR5 IK initial result expired')
            if self.result is not None and now - self.result[1] > 300_000_000:
                raise TimeoutError('CR5 IK result expired')
            self.pending = (self.generation, copy.deepcopy(value), measured.copy(), dt,
                            copy.deepcopy(parameters), now)
            self.condition.notify()
            result = measured.copy() if self.result is None else self.result[0].copy()
        # Limit actual outgoing joint velocity, independently of controller response rate.
        speed = float(parameters.get('max_joint_velocity', .3))
        if not 0 < speed <= 1: raise ValueError('Invalid CR5 joint velocity')
        step = speed * min(.05, max(0., dt))
        return (measured + np.clip(result - measured, -step, step)).tolist()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed: return
                generation, value, measured, dt, parameters, requested_ns = self.pending
                self.pending = None
            try:
                if generation != self.worker_generation:
                    self.controller.reset()
                    self.filtered_pose = None
                    self.worker_generation = generation
                if not self.jog:
                    alpha=float(parameters.get('smoothing_alpha',1.))
                    if not 0<alpha<=1:raise ValueError('Invalid CR5 pose smoothing')
                    position=np.asarray(value['position'],dtype=float)
                    quat=np.asarray(value['quaternion'],dtype=float)
                    if (position.shape!=(3,) or quat.shape!=(4,) or not np.isfinite(position).all()
                            or not np.isfinite(quat).all() or not np.isclose(np.linalg.norm(quat),1,atol=1e-3)):
                        raise ValueError('Invalid CR5 input pose')
                    rotation=Rotation.from_quat(quat).as_matrix()
                    if self.filtered_pose is not None:
                        previous_p,previous_r=self.filtered_pose
                        position=previous_p+alpha*(position-previous_p)
                        delta=Rotation.from_matrix(rotation@previous_r.T).as_rotvec()
                        rotation=Rotation.from_rotvec(alpha*delta).as_matrix()@previous_r
                    self.filtered_pose=(position.copy(),rotation.copy())
                    value={'position':position.tolist(),
                           'quaternion':Rotation.from_matrix(rotation).as_quat().tolist()}
                result = self.controller(value, measured, dt, parameters)
                if self.clock() - requested_ns > 250_000_000:
                    raise TimeoutError('CR5 full FK/IK deadline exceeded')
                error = None
            except Exception as exc:
                result = None
                error = str(exc)
            with self.condition:
                if generation == self.generation and not self.closed:
                    self.error = error
                    if error:
                        self.pending = self.result = None
                    if result is not None: self.result = (vector6(result), requested_ns)

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = self.result = None
            self.condition.notify_all()
        self.thread.join(timeout=1)
        if self.thread.is_alive(): raise TimeoutError('CR5 IK worker did not close')


def _create(context, *, jog=False):
    instance = context['instance']
    parameters=context.get('binding',{}).get('parameters',{})
    validate_parameters(parameters,jog=jog)
    if instance['plugin_id'] != 'morla.dobot.cr5':
        raise ValueError('CR5 controller requires CR5 target')
    if instance['backend'] not in ('fake','real'):
        raise ValueError('CR5 VR only supports fake or verified real backend')
    hardware=instance.get('parameters',{})
    solver=IKSolver(**({'position_tolerance_m':.00001,'angle_tolerance_rad':.0001} if jog else {}),
                    joint_lower=hardware.get('joint_lower'),joint_upper=hardware.get('joint_upper'),
                    joint_signs=hardware.get('urdf_joint_signs'),joint_offsets=hardware.get('urdf_joint_offsets'))
    controller=AsyncCartesianController(solver, clock=context.get('clock', time.monotonic_ns),jog=jog)
    controller.requires_fresh_input=instance['backend']=='real'
    return controller


def create_controller(context):
    return _create(context)


def create_jog_controller(context):
    return _create(context,jog=True)
