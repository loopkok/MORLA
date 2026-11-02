"""Asynchronous CR5 pose controller; all movement still goes through runtime.submit."""
import copy
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
from morla_teleop.kinematics import CartesianController
from .kinematics import vector6
from .urdf_ik import IKSolver
from morla_core.coordinates import validate_basis_change, validate_transform


def validate_parameters(parameters, *, jog=False):
    allowed={'position_scale','input_rotation','solver_tip_from_tool','max_joint_step',
             'max_joint_velocity','calibration_verified','smoothing_alpha','ik_mode',
             'position_lead_m','rotation_lead_rad','damping_min','damping_max',
             'singularity_threshold','differential_step_rad'}
    if jog:allowed|={'axes_map','linear_speed','angular_speed'}
    if not isinstance(parameters,dict) or set(parameters)-allowed:
        raise ValueError('Unknown CR5 VR controller parameter')
    if parameters.get('ik_mode', 'differential') not in ('differential', 'strict'):
        raise ValueError('Invalid CR5 ik_mode')
    for name,maximum in [('position_lead_m',.25),('rotation_lead_rad',1),
                          ('damping_min',1),('damping_max',1),('singularity_threshold',1),
                          ('differential_step_rad',.15),('position_scale',2),('max_joint_step',.3),
                          ('max_joint_velocity',1),('smoothing_alpha',1)]:
        if name in parameters:
            value=parameters[name]
            if type(value) not in (int,float) or not np.isfinite(value) or not 0<value<=maximum:
                raise ValueError('Invalid CR5 VR '+name)
    if parameters.get('damping_min',.005)>parameters.get('damping_max',.15):
        raise ValueError('CR5 damping_min exceeds damping_max')
    if jog:
        mapping=parameters.get('axes_map',{})
        if not isinstance(mapping,dict) or set(mapping)-{'x','y','z','roll','pitch','yaw'}:
            raise ValueError('Invalid Cartesian axes_map')
        for key,maximum in [('linear_speed',.25),('angular_speed',1)]:
            if key in parameters and (type(parameters[key]) not in (int,float) or not np.isfinite(parameters[key]) or not 0<parameters[key]<=maximum):
                raise ValueError('Invalid Cartesian '+key)
    if 'input_rotation' in parameters:validate_basis_change(parameters['input_rotation'])
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
        self.anchor_waiter = None
        self.result = None
        self.last_output = None
        self.error = None
        self.report = {}
        self.diagnostic_sink = None
        self.trace_context = {}
        self.first_request_ns = None
        self.closed = False
        self.thread = threading.Thread(target=self._run, daemon=True, name='cr5-vr-kinematics')
        self.thread.start()

    def reset(self):
        with self.condition:
            if self.anchor_waiter is not None and not self.anchor_waiter.done():
                self.anchor_waiter.set_exception(RuntimeError('CR5 reanchor cancelled'))
            self.generation += 1
            self.pending = self.result = self.error = None
            self.first_request_ns = None
            self.last_output = None
            self.report = {}

    def reanchor(self, value, measured, parameters):
        from concurrent.futures import Future
        self.reset()
        waiter=Future()
        with self.condition:
            if self.closed:raise RuntimeError('CR5 controller closed')
            self.anchor_waiter=waiter
            self.pending=(self.generation,copy.deepcopy(value),vector6(measured),0.,
                          copy.deepcopy(parameters),self.clock(),dict(self.trace_context),waiter)
            self.condition.notify()
        return waiter.result(timeout=2.5)

    def set_trace_context(self, context):
        with self.condition: self.trace_context = dict(context)

    def set_diagnostic_sink(self, sink):
        with self.condition: self.diagnostic_sink = sink

    def diagnostics(self):
        with self.condition:
            return {'solver': self.controller.solver.method_name if hasattr(self.controller.solver, 'method_name') else type(self.controller.solver).__name__,
                    'error': self.error, **copy.deepcopy(self.report),
                    'age_ms': max(0.,(self.clock()-self.report['sample_ns'])/1e6) if 'sample_ns' in self.report else None}

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
                            copy.deepcopy(parameters), now, dict(self.trace_context), None)
            self.condition.notify()
            result = measured.copy() if self.result is None else self.result[0].copy()
        # Limit actual outgoing joint velocity, independently of controller response rate.
        # The limit is applied to the previous command, not to the measured pose:
        # anchoring on the measurement would let the commanded pose trail the arm
        # wherever gravity takes it, so the servo would never build the error needed
        # to hold position (and would ratchet down until IK loses the seed).
        speed = float(parameters.get('max_joint_velocity', .3))
        if not 0 < speed <= 1: raise ValueError('Invalid CR5 joint velocity')
        step = speed * min(.05, max(0., dt))
        origin = measured if self.last_output is None else self.last_output
        self.last_output = origin + np.clip(result - origin, -step, step)
        return self.last_output.tolist()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed: return
                generation, value, measured, dt, parameters, requested_ns, trace_context, anchor_waiter = self.pending
                self.pending = None
            raw_value = copy.deepcopy(value)
            try:
                if generation != self.worker_generation:
                    self.controller.reset()
                    reset_solver=getattr(self.controller.solver,'reset_teleop',None)
                    if reset_solver is not None: reset_solver()
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
                if anchor_waiter is not None:
                    ack=self.controller.reanchor(value,measured,parameters)
                    result=measured.tolist()
                else:
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
                    self.report = copy.deepcopy(getattr(self.controller.solver, 'last_report', {}))
                    self.report['sample_ns'] = requested_ns
                    self.report['completed_ns'] = self.clock()
                    self.report['input_context'] = trace_context
                    self.report['error'] = error
                    self.report['generation'] = generation
                    self.report['feedback_seed'] = measured.tolist()
                    self.report['candidate_values'] = None if result is None else vector6(result).tolist()
                    self.report['delivered_to_device'] = False
                    self.report['raw_input'] = raw_value
                    self.report['filtered_input'] = copy.deepcopy(value)
                    self.report['cartesian'] = self.controller.diagnostics()
                    if error:
                        self.pending = self.result = None
                    if result is not None:
                        self.result = (vector6(result), requested_ns)
                        if anchor_waiter is not None:self.last_output=vector6(result)
                    elif error is None and self.result is not None:
                        # An unreachable jog tick is a live answer holding the
                        # last feasible target, not a stalled worker: refresh
                        # the timestamp so the 300 ms expiry does not revoke
                        # the session while the stick is held at a limit.
                        self.result = (self.result[0], requested_ns)
                if anchor_waiter is not None and not anchor_waiter.done():
                    if generation!=self.generation or self.closed:
                        anchor_waiter.set_exception(RuntimeError('CR5 reanchor retired'))
                    elif error:anchor_waiter.set_exception(RuntimeError(error))
                    else:anchor_waiter.set_result(ack)

            with self.condition:
                sink=self.diagnostic_sink
                report=copy.deepcopy(self.report) if generation==self.generation else {
                    'discarded':True,'generation':generation,'sample_ns':requested_ns,'input_context':trace_context}
            if sink is not None:
                try: sink(report)
                except Exception: pass  # Logging cannot poison the IK worker.

    def close(self):
        with self.condition:
            if self.anchor_waiter is not None and not self.anchor_waiter.done():
                self.anchor_waiter.set_exception(RuntimeError('CR5 controller closed'))
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
    if instance['backend'] not in ('fake','sim','real'):
        raise ValueError('CR5 VR requires fake, sim or real backend')
    hardware=instance.get('parameters',{})
    mode=parameters.get('ik_mode','differential')
    solver_type=IKSolver
    options={}
    if mode=='differential':
        from .differential_ik import DifferentialIKSolver
        solver_type=DifferentialIKSolver
        options={key:parameters[key] for key in ('position_lead_m','rotation_lead_rad',
                 'damping_min','damping_max','singularity_threshold') if key in parameters}
        options['max_step_rad']=min(parameters.get('differential_step_rad',.04),parameters.get('max_joint_step',.15))
    solver=solver_type(**options, **({'position_tolerance_m':.00001,'angle_tolerance_rad':.0001} if jog else {}),
                    joint_lower=hardware.get('joint_lower'),joint_upper=hardware.get('joint_upper'),
                    joint_signs=hardware.get('urdf_joint_signs'),joint_offsets=hardware.get('urdf_joint_offsets'),
                    # Strict mode retains its existing local-window policy.
                    # Differential mode uses bounded steps and absorbing lead limits.
                    retry_trust=(.3,) if jog else (.3,.6,1.2))
    controller=AsyncCartesianController(solver, clock=context.get('clock', time.monotonic_ns),jog=jog)
    controller.requires_fresh_input=instance['backend']=='real'
    return controller


def create_controller(context):
    return _create(context)


def create_jog_controller(context):
    return _create(context,jog=True)
