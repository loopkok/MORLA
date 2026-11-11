"""MMK2 asynchronous Cartesian controller through the public Runtime gateway.

Uses the FR3/CR5 request/worker/last-command integration pattern. Every emitted
joint vector is direction-preserving, lead-bounded and FK-checked after its
output speed limit. The worker only owns a latest request and cannot issue I/O.
"""
import copy
from concurrent.futures import Future
import threading
import time

import numpy as np
from morla_core.coordinates import validate_basis_change, validate_transform
from morla_teleop.kinematics import CartesianController, limit_joint_step
from .urdf_ik import ArmKinematics, PARAMETERS, vector6


CONTROLLER_ID = 'morla.mmk2.cartesian_delta'
ARM_PLUGINS = tuple(prefix + side + '_arm' for prefix in ('morla.airbot.mmk2.', 'morla.airbot.mmk2_sdk.') for side in ('left', 'right'))


def validate_parameters(parameters):
    allowed = {
        'input_position_rotation', 'input_rotation', 'solver_tip_from_tool',
        'position_scale', 'rotation_scale', 'smoothing_alpha', 'calibration_verified',
        'max_joint_step', 'max_joint_velocity', 'command_lead_rad',
        'differential_step_rad', 'ik_mode', *PARAMETERS,
    }
    if not isinstance(parameters, dict) or set(parameters) - allowed:
        raise ValueError('Unknown MMK2 Cartesian controller parameter')
    if parameters.get('ik_mode', 'differential') != 'differential':
        raise ValueError('MMK2 Cartesian controller requires differential IK')
    bounds = {name: maximum for name, (_, maximum) in PARAMETERS.items()}
    bounds.update(position_scale=2., rotation_scale=2., smoothing_alpha=1.,
                  max_joint_step=.3, max_joint_velocity=3.14,
                  command_lead_rad=1., differential_step_rad=.15)
    for name, maximum in bounds.items():
        value = parameters.get(name)
        if name in parameters and (type(value) not in (int, float) or not np.isfinite(value) or not 0 < value <= maximum):
            raise ValueError('Invalid MMK2 Cartesian ' + name)
    if 'calibration_verified' in parameters and type(parameters['calibration_verified']) is not bool:
        raise ValueError('Invalid MMK2 calibration flag')
    if parameters.get('damping_min', .005) > parameters.get('damping_max', .145):
        raise ValueError('MMK2 damping_min exceeds damping_max')
    for name in ('input_rotation', 'input_position_rotation'):
        if name in parameters:
            validate_basis_change(parameters[name])
    if 'solver_tip_from_tool' in parameters:
        validate_transform(parameters['solver_tip_from_tool'])


class AsyncCartesianController:
    def __init__(self, solver, *, clock=time.monotonic_ns,
                 device_command_lead=.15, device_joint_velocity=3.14):
        self.clock = clock
        self.solver = solver
        self.device_command_lead = device_command_lead
        self.device_joint_velocity = device_joint_velocity
        self.controller = CartesianController(solver, step_cap=False)
        self.condition = threading.Condition()
        self.generation = 0
        self.worker_generation = -1
        self.pending = None
        self.result = None
        self.last_output = None
        self.error = None
        self.report = {}
        self.output_report = {}
        self.trace_context = {}
        self.diagnostic_sink = None
        self.anchor_waiter = None
        self.first_request_ns = None
        self.closed = False
        self.thread = threading.Thread(target=self._run, daemon=True, name='mmk2-cartesian-ik')
        self.thread.start()

    def reset(self):
        with self.condition:
            if self.anchor_waiter is not None and not self.anchor_waiter.done():
                self.anchor_waiter.set_exception(RuntimeError('MMK2 reanchor cancelled'))
            self.generation += 1
            self.pending = self.result = self.error = None
            self.last_output = None
            self.first_request_ns = None
            self.report = {}
            self.output_report = {}

    def reanchor(self, value, measured, parameters):
        validate_parameters(parameters)
        self.reset()
        waiter = Future()
        with self.condition:
            if self.closed:
                raise RuntimeError('MMK2 controller closed')
            self.anchor_waiter = waiter
            self.pending = (self.generation, copy.deepcopy(value), vector6(measured), 0.,
                            copy.deepcopy(parameters), self.clock(), dict(self.trace_context), waiter, None)
            self.condition.notify()
        return waiter.result(timeout=2.5)

    def set_trace_context(self, context):
        with self.condition:
            self.trace_context = dict(context)

    def set_diagnostic_sink(self, sink):
        with self.condition:
            self.diagnostic_sink = sink

    def diagnostics(self):
        with self.condition:
            return {'solver': self.solver.method_name, 'error': self.error,
                    **copy.deepcopy(self.report), **copy.deepcopy(self.output_report),
                    'age_ms': max(0., (self.clock() - self.report['sample_ns']) / 1e6) if 'sample_ns' in self.report else None}

    def _output(self, origin, candidate, measured, step, lead, goal, raw):
        candidate, fraction = limit_joint_step(origin, candidate, step)
        delta = candidate - origin
        # Intersect the speed-bounded segment with the measured lead box.
        lead_scale = 1.
        for current, change, feedback in zip(origin, delta, measured):
            if change > 0:
                lead_scale = min(lead_scale, max(0., (feedback + lead - current) / change))
            elif change < 0:
                lead_scale = min(lead_scale, max(0., (feedback - lead - current) / change))
        before = self.solver.command_metrics(origin, goal, raw)
        bound = max(before['raw_position_error_m'], self.solver.position_lead_m)
        accepted = origin.copy()
        metrics = before
        scale = lead_scale
        for _ in range(10):
            trial = origin + scale * delta
            error = self.solver.command_metrics(trial, goal, raw)
            if error['cost'] <= before['cost'] + 1e-14 and error['raw_position_error_m'] <= bound + 1e-10:
                accepted, metrics = trial, error
                break
            scale *= .5
        return accepted, {
            'output_values': accepted.tolist(), 'output_step_rad': float(np.max(np.abs(accepted - origin))),
            'output_step_limit_rad': step, 'output_cost_before': before['cost'],
            'output_cost_after': metrics['cost'], 'output_position_error_m': metrics['raw_position_error_m'],
            'output_angle_error_rad': metrics['raw_angle_error_rad'], 'output_fk_verified': True,
            'output_step_scale': 0. if np.array_equal(accepted, origin) else fraction * scale,
        }

    def __call__(self, value, measured, dt, parameters):
        validate_parameters(parameters)
        measured = vector6(measured)
        if isinstance(dt, bool) or not np.isfinite(dt) or dt < 0:
            raise ValueError('Invalid MMK2 control dt')
        speed = min(parameters.get('max_joint_velocity', .3), self.device_joint_velocity)
        lead = min(parameters.get('command_lead_rad', .08), self.device_command_lead)
        step = min(parameters.get('max_joint_step', .02), speed * min(.05, dt))
        now = self.clock()
        with self.condition:
            if self.closed:
                raise RuntimeError('MMK2 controller closed')
            if self.error:
                raise RuntimeError('MMK2 IK failed: ' + self.error)
            if self.first_request_ns is None:
                self.first_request_ns = now
            if self.result is None and now - self.first_request_ns > 300_000_000:
                raise TimeoutError('MMK2 IK initial result expired')
            if self.result is not None and now - self.result['requested_ns'] > 300_000_000:
                raise TimeoutError('MMK2 IK result expired')
            origin = measured.copy() if self.last_output is None else self.last_output.copy()
            if np.max(np.abs(origin - measured)) > lead + 1e-8:
                raise ValueError('MMK2 feedback moved outside the command-lead window')
            self.pending = (self.generation, copy.deepcopy(value), measured.copy(), min(.05, dt),
                            copy.deepcopy(parameters), now, dict(self.trace_context), None, origin.copy())
            self.condition.notify()
            completed = copy.deepcopy(self.result)
            if completed is None or completed['goal'] is None:
                self.last_output = origin
                self.output_report = {'output_values': origin.tolist(), 'output_step_rad': 0., 'output_reason': 'waiting_for_ik'}
                return origin.tolist()
            output, diagnostic = self._output(origin, completed['values'], measured, step, lead,
                                              completed['goal'], completed['raw'])
            self.last_output = output.copy()
            self.output_report = {**diagnostic, 'output_candidate_ns': completed['requested_ns'],
                                  'output_generation': self.generation}
            return output.tolist()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                generation, value, measured, dt, parameters, requested, context, waiter, seed = self.pending
                self.pending = None
            try:
                if generation != self.worker_generation:
                    self.controller.reset()
                    self.solver.reset_teleop()
                    self.worker_generation = generation
                if waiter is not None:
                    acknowledgement = self.controller.reanchor(value, measured, parameters)
                    result = measured.copy()
                else:
                    speed = min(parameters.get('max_joint_velocity', .3), self.device_joint_velocity)
                    cap = min(parameters.get('max_joint_step', .02), speed * dt)
                    result = self.controller(value, measured, dt, parameters, seed=seed, max_joint_step=cap)
                if self.clock() - requested > 250_000_000:
                    raise TimeoutError('MMK2 FK/IK deadline exceeded')
                error = None
            except Exception as exc:
                result, error = None, str(exc)
            with self.condition:
                current = generation == self.generation and not self.closed
                if current:
                    self.error = error
                    self.report = {
                        **copy.deepcopy(self.solver.last_report), 'sample_ns': requested,
                        'completed_ns': self.clock(), 'generation': generation, 'input_context': context,
                        'feedback_seed': measured.tolist(), 'command_seed': None if seed is None else seed.tolist(),
                        'candidate_values': None if result is None else list(result),
                        'cartesian': self.controller.diagnostics(), 'delivered_to_device': False,
                    }
                    if result is not None:
                        self.result = {'values': vector6(result), 'requested_ns': requested,
                                       'goal': None if self.solver.goal is None else self.solver.goal.copy(),
                                       'raw': None if self.solver.raw_target is None else self.solver.raw_target.copy()}
                        if waiter is not None:
                            self.last_output = vector6(result)
                    elif error:
                        self.pending = self.result = None
                if waiter is not None and not waiter.done():
                    if not current:
                        waiter.set_exception(RuntimeError('MMK2 reanchor retired'))
                    elif error:
                        waiter.set_exception(RuntimeError(error))
                    else:
                        waiter.set_result(acknowledgement)
                sink = self.diagnostic_sink
                report = copy.deepcopy(self.report) if current else {'discarded': True, 'generation': generation}
            if sink is not None:
                try:
                    sink(report)
                except Exception:
                    pass

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = self.result = None
            if self.anchor_waiter is not None and not self.anchor_waiter.done():
                self.anchor_waiter.set_exception(RuntimeError('MMK2 controller closed'))
            self.condition.notify_all()
        self.thread.join(timeout=1.)
        if self.thread.is_alive():
            raise TimeoutError('MMK2 IK worker did not close')


def create_controller(context):
    instance = context['instance']
    if instance['plugin_id'] not in ARM_PLUGINS:
        raise ValueError('MMK2 Cartesian controller requires an MMK2 arm')
    parameters = context.get('binding', {}).get('parameters', {})
    validate_parameters(parameters)
    hardware = instance.get('parameters', {})
    solver_instance = copy.deepcopy(instance)
    for name in PARAMETERS:
        if name in parameters:
            solver_instance.setdefault('parameters', {})[name] = parameters[name]
    maximum = min(parameters.get('differential_step_rad', .02), parameters.get('max_joint_step', .02))
    real = instance.get('backend') == 'real'
    sdk = instance['plugin_id'].startswith('morla.airbot.mmk2_sdk.')
    velocity = hardware.get('max_velocity', .1) if sdk else 3.14
    lead = hardware.get('max_command_lead', .15) if sdk else .15
    if sdk:
        maximum = min(maximum, velocity / hardware.get('rate_hz', 20.))
    solver = ArmKinematics(solver_instance, max_step_rad=maximum)
    controller = AsyncCartesianController(solver, clock=context.get('clock', time.monotonic_ns),
                                          device_command_lead=lead, device_joint_velocity=velocity)
    controller.requires_fresh_input = real
    return controller
