"""MMK2 asynchronous Cartesian controller through the public Runtime gateway.

The worker solves a Cartesian request from a seed snapshotted with
the request. The caller tracks the resulting absolute joint target, bounded
by the measured lead window and the per-tick joint speed. The solver keeps its
own (larger) internal step; the output stage never gates on a second FK check.
The worker only owns a latest request and cannot issue I/O.
"""
import copy
from concurrent.futures import Future
import multiprocessing
import threading
import time

import numpy as np
from morla_core.cadence_probe import record as cadence_record
from morla_core.coordinates import validate_basis_change, validate_transform
from morla_teleop.kinematics import CartesianController, limit_joint_step
from .analytic_dh import AnalyticDhKinematics
from .urdf_ik import ArmKinematics, PARAMETERS, vector6
from .urdf_least_squares import LeastSquaresKinematics


CONTROLLER_ID = 'morla.mmk2.cartesian_delta'
ARM_PLUGINS = tuple(prefix + side + '_arm' for prefix in ('morla.airbot.mmk2.', 'morla.airbot.mmk2_sdk.') for side in ('left', 'right'))

IK_SOLVERS = {
    'differential': ArmKinematics,
    'urdf_numerical': LeastSquaresKinematics,
    'analytic_dh': AnalyticDhKinematics,
}


def integration_seed(measured, last_output, lead):
    """Integrate from the last delivered command, bounded by current feedback.

    Same rule as the FR3 and CR5 controllers: a feedback excursion beyond the
    lead window re-anchors the seed to its edge instead of aborting teleop.
    """
    if last_output is None:
        return measured.copy()
    return measured + np.clip(last_output - measured, -lead, lead)


def validate_parameters(parameters):
    allowed = {
        'input_position_rotation', 'input_rotation', 'solver_tip_from_tool',
        'position_scale', 'rotation_scale', 'smoothing_alpha', 'calibration_verified',
        'max_joint_step', 'max_joint_velocity', 'command_lead_rad',
        'differential_step_rad', 'ik_mode', 'position_guard_enabled', 'ik_iterations', *PARAMETERS,
    }
    if not isinstance(parameters, dict) or set(parameters) - allowed:
        raise ValueError('Unknown MMK2 Cartesian controller parameter')
    if type(parameters.get('position_guard_enabled', True)) is not bool:
        raise ValueError('MMK2 position_guard_enabled must be a boolean')
    iterations = parameters.get('ik_iterations', 1)
    if type(iterations) is not int or not 1 <= iterations <= 3:
        raise ValueError('MMK2 ik_iterations must be an integer between 1 and 3')
    mode = parameters.get('ik_mode', 'differential')
    if not isinstance(mode, str) or mode not in IK_SOLVERS:
        raise ValueError('Unknown MMK2 IK mode')
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


class _ArmSolver:
    """Owns the Cartesian anchor and URDF state for one arm."""

    def __init__(self, solver):
        self.solver = solver
        self.controller = CartesianController(solver, step_cap=False)
        self.generation = -1

    def solve(self, generation, value, measured, dt, parameters, reanchor, seed):
        started = time.monotonic_ns()
        cpu_started = time.thread_time_ns()
        if generation != self.generation:
            self.controller.reset()
            self.solver.reset_teleop()
            self.generation = generation
        acknowledgement = None
        if reanchor:
            acknowledgement = self.controller.reanchor(value, measured, parameters)
            result = measured.copy()
        else:
            result = self.controller(value, measured, dt, parameters, seed=seed)
        report = {
            **self.solver.last_report,
            'cartesian': self.controller.diagnostics(),
            'worker_pid': multiprocessing.current_process().pid,
            'worker_started_ns': started,
            'worker_wall_ns': time.monotonic_ns() - started,
            'worker_cpu_ns': time.thread_time_ns() - cpu_started,
        }
        return result, acknowledgement, self.solver.goal is not None, report


def _serve_arm_solver(connection, solver):
    with connection:
        worker = _ArmSolver(solver)
        try:
            connection.send(None)
            while True:
                request = connection.recv()
                try:
                    result = worker.solve(*request)
                    error = None
                except Exception as exc:
                    result, error = None, str(exc)
                connection.send((result, error, time.monotonic_ns()))
        except (EOFError, BrokenPipeError, ConnectionResetError):
            return


class _ArmSolverProcess:
    """One request in flight. The parent controller owns the latest-value slot."""

    def __init__(self, solver):
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_serve_arm_solver, args=(child, solver),
                                       name='morla-mmk2-ik-' + solver.base_frame.split('_')[0],
                                       daemon=True)
        try:
            self.process.start()
        except BaseException:
            self.connection.close()
            raise
        finally:
            child.close()
        try:
            if not self.connection.poll(5):
                raise TimeoutError('MMK2 IK process startup timed out')
            self.connection.recv()
        except BaseException:
            self.close()
            raise

    def solve(self, *request):
        if self.connection.closed:
            raise RuntimeError('MMK2 IK process is unavailable')
        try:
            self.connection.send(request)
            if not self.connection.poll(.25):
                raise TimeoutError('MMK2 IK process response timed out')
            result, error, reply_ns = self.connection.recv()
            received_ns = time.monotonic_ns()
        except (EOFError, OSError):
            # Retire the pipe on timeout or exit so a delayed reply can never
            # be mistaken for the next generation's result.
            self.close()
            raise
        if error is not None:
            raise RuntimeError(error)
        result[-1].update(worker_reply_ns=reply_ns, parent_received_ns=received_ns)
        return result

    def close(self):
        self.connection.close()
        self.process.join(timeout=.3)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=.3)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=.3)
        if self.process.is_alive():
            raise TimeoutError('MMK2 IK process did not exit')


class AsyncCartesianController:
    def __init__(self, solver, *, clock=time.monotonic_ns,
                 device_command_lead=.15, device_joint_velocity=3.14, use_process=False):
        self.clock = clock
        self.solver = solver
        self.device_command_lead = device_command_lead
        self.device_joint_velocity = device_joint_velocity
        self.worker = _ArmSolverProcess(solver) if use_process else _ArmSolver(solver)
        self.condition = threading.Condition()
        self._cadence_prefix = 'ik.' + solver.base_frame.split('_')[0]
        self._cadence_last_used = None
        self._rotation_probe_ns = None
        self.generation = 0
        self.pending = None
        self.result = None
        self.last_output = None
        self.error = None
        self.report = {}
        self.output_report = {}
        self.trace_context = {}
        self.diagnostic_sink = None
        self.last_detail_ns = None
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
            self.last_detail_ns = None

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

    def _output(self, origin, candidate, measured, step, lead):
        """Shape the worker candidate in the FR3/CR5 output shape.

        Bound the candidate to the measured lead window, then clip the applied
        joint step against the previous command. The differential solve already
        backtracks on cost and on the position guard, so no second FK gate is
        applied here: the one that used to live here rejected a fifth of the
        ticks outright and halved the delivered speed while it did.
        """
        bounded = np.clip(candidate, measured - lead, measured + lead)
        # ONE direction-preserving speed clip, as the old code did and unlike a
        # raw per-component np.clip: MMK2's wrist branches have very unequal
        # joint components, and clipping them independently turns the solver's
        # descent direction sideways (the 20 mm -> 52 mm regression). Scaling
        # the whole step keeps direction and still saturates at `step`.
        shaped, fraction = limit_joint_step(origin, bounded, step)
        # Never command a joint further outside its limits than the previous
        # command already was; the clip is inert while origin is in bounds.
        lower = np.minimum(self.solver.bounds[:, 0], origin)
        upper = np.maximum(self.solver.bounds[:, 1], origin)
        output = np.clip(shaped, lower, upper)
        return output, {
            'output_values': output.tolist(), 'output_step_rad': float(np.max(np.abs(output - origin))),
            'output_step_limit_rad': step, 'output_step_scale': fraction,
            'output_fk_verified': False,
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
            seed = integration_seed(measured, self.last_output, lead)
            completed = self.result
            if completed is None or not completed['has_goal']:
                output = origin
                self.output_report = {'output_values': origin.tolist(), 'output_step_rad': 0., 'output_reason': 'waiting_for_ik'}
            else:
                # A differential result is an absolute joint target at its
                # solve seed. Rebasing it on a newer command can overshoot,
                # even if each result is used only once.
                candidate = origin if completed['hold'] else completed['values']
                output, diagnostic = self._output(origin, candidate, measured, step, lead)
                self.output_report = {**diagnostic, 'output_candidate_ns': completed['requested_ns'],
                                      'output_generation': self.generation}
                if completed['hold']:
                    self.output_report['output_reason'] = completed['hold']
            self.last_output = output.copy()
            cadence_counts = ['submitted']
            if self.pending is not None:
                cadence_counts.append('overwritten')
            if completed is not None and completed['has_goal']:
                candidate_key = (self.generation, completed['requested_ns'])
                cadence_counts.append('reused' if candidate_key == self._cadence_last_used else 'used_new')
                self._cadence_last_used = candidate_key
            output_step = self.output_report.get('output_step_rad', 0.)
            if output_step <= 1e-8:
                cadence_counts.append('output_zero')
            elif step > 0 and output_step >= .999 * step:
                cadence_counts.append('output_step_cap')
                if speed * min(.05, dt) <= parameters.get('max_joint_step', .02):
                    cadence_counts.append('output_velocity_cap')
            cadence_record(self._cadence_prefix, counts=cadence_counts)
            self.pending = (self.generation, copy.deepcopy(value), measured.copy(), min(.05, dt),
                            copy.deepcopy(parameters), now, dict(self.trace_context), None, seed)
            self.condition.notify()
            return output.tolist()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                (generation, value, measured, dt, parameters, requested, context,
                 waiter, seed) = self.pending
                self.pending = None
            worker_report = {}
            try:
                result, acknowledgement, has_goal, worker_report = self.worker.solve(
                    generation, value, measured, dt, parameters, waiter is not None, seed)
                if self.clock() - requested > 250_000_000:
                    raise TimeoutError('MMK2 FK/IK deadline exceeded')
                error = None
            except Exception as exc:
                result, error = None, str(exc)
            with self.condition:
                current = generation == self.generation and not self.closed
                if current:
                    self.error = error
                    if result is not None:
                        candidate = vector6(result)
                        reason = worker_report.get('reason')
                        self.result = {'values': candidate, 'requested_ns': requested,
                                       'has_goal': has_goal,
                                       'hold': reason if reason in ('holding', 'deadline_hold') else None}
                        if waiter is not None:
                            self.last_output = candidate.copy()
                    elif error:
                        self.pending = self.result = None
                    self.report = {
                        **worker_report, 'sample_ns': requested,
                        'completed_ns': self.clock(), 'generation': generation, 'input_context': context,
                        'feedback_seed': measured.tolist(), 'command_seed': None if seed is None else seed.tolist(),
                        'candidate_values': None if result is None else list(result),
                        'delivered_to_device': False,
                    }
                if waiter is not None and not waiter.done():
                    if not current:
                        waiter.set_exception(RuntimeError('MMK2 reanchor retired'))
                    elif error:
                        waiter.set_exception(RuntimeError(error))
                    else:
                        waiter.set_result(acknowledgement)
                sink = self.diagnostic_sink
                # The control trace still has the latest report on every
                # input. Sample the duplicate worker detail at 20 Hz.
                detail_ns = self.clock()
                emit = sink is not None and (not current or error is not None or waiter is not None
                    or self.last_detail_ns is None or detail_ns - self.last_detail_ns >= 50_000_000)
                if emit and current:
                    self.last_detail_ns = detail_ns
                report = self.report if current else {'discarded': True, 'generation': generation}
            if waiter is None and worker_report:
                reason = worker_report.get('reason', 'unknown')
                timings = [
                    ('queue', worker_report['worker_started_ns'] - requested),
                    ('wall', worker_report['worker_wall_ns']),
                    ('thread_cpu', worker_report['worker_cpu_ns']),
                ]
                if current and 'parent_received_ns' in worker_report:
                    timings.extend((
                        ('reply_to_parent', worker_report['parent_received_ns'] - worker_report['worker_reply_ns']),
                        ('parent_publish', report['completed_ns'] - worker_report['parent_received_ns']),
                    ))
                counts = ['completed', reason]
                if reason == 'holding':
                    cause = worker_report.get('holding_cause', 'unknown')
                    counts.append('holding_' + cause)
                    if worker_report.get('active_joint_limits'):
                        counts.append('holding_active_limit')
                    counts.extend('holding_fallback_' + item
                                  for item in worker_report.get('rotation_rejections', ()))
                event = None
                if current and (self._rotation_probe_ns is None
                                or requested - self._rotation_probe_ns >= 100_000_000):
                    cartesian = worker_report.get('cartesian', {})
                    pose = cartesian.get('input')
                    tcp = cartesian.get('current_tcp')
                    target = cartesian.get('target_tip')
                    if isinstance(pose, dict) and tcp is not None:
                        self._rotation_probe_ns = requested
                        event = {
                            'sample_ns': requested,
                            'generation': generation,
                            'grip_active': True,
                            'hand_quaternion': pose.get('quaternion'),
                            'hand_position': pose.get('position'),
                            'tcp_rotation': [row[:3] for row in tcp[:3]],
                            'target_rotation': None if target is None else [row[:3] for row in target[:3]],
                            'feedback_joints': measured.tolist(),
                            'raw_angle_error_rad': worker_report.get('raw_angle_error_rad'),
                            'raw_position_error_m': worker_report.get('raw_position_error_m'),
                            'reason': reason,
                            'active_joint_limits': worker_report.get('active_joint_limits', []),
                        }
                cadence_record(self._cadence_prefix, counts=counts, timings=timings, event=event)
            if emit:
                try:
                    sink(copy.deepcopy(report))
                except Exception:
                    pass

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = self.result = None
            if self.anchor_waiter is not None and not self.anchor_waiter.done():
                self.anchor_waiter.set_exception(RuntimeError('MMK2 controller closed'))
            self.condition.notify_all()
        self.thread.join(timeout=1.5)
        if self.thread.is_alive():
            raise TimeoutError('MMK2 IK worker did not close')
        if isinstance(self.worker, _ArmSolverProcess):
            self.worker.close()


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
    # Same split as FR3/CR5: the solver's internal step is independent of the
    # output speed cap and must stay comfortably larger than it, so the plain
    # output clip is what bounds the motion.
    maximum = min(parameters.get('differential_step_rad', .04), parameters.get('max_joint_step', .15))
    real = instance.get('backend') == 'real'
    sdk = instance['plugin_id'].startswith('morla.airbot.mmk2_sdk.')
    velocity = hardware.get('max_velocity', .1) if sdk else 3.14
    lead = hardware.get('max_command_lead', .15) if sdk else .15
    if sdk:
        maximum = min(maximum, velocity / hardware.get('rate_hz', 20.))
    solver_type = IK_SOLVERS[parameters.get('ik_mode', 'differential')]
    solver = solver_type(solver_instance, max_step_rad=maximum,
                           position_guard_enabled=parameters.get('position_guard_enabled', True),
                           ik_iterations=parameters.get('ik_iterations', 1))
    controller = AsyncCartesianController(solver, clock=context.get('clock', time.monotonic_ns),
                                          device_command_lead=lead, device_joint_velocity=velocity,
                                          use_process=True)
    controller.requires_fresh_input = real
    return controller
