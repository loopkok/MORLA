"""FR3 lifecycle, latest-target servo scheduling and strict joint boundary."""
import time

from morla_runtime.cached_actuator import CachedActuatorAdapter
from morla_runtime.adapters import FakeAdapter
from .contract import INITIAL, positions, validate_parameters, validate_target, vector6
from .sdk_backend import require_safe


class FR3Adapter(CachedActuatorAdapter):
    # This is a host I/O deadline, not a verified controller watchdog.
    supports_watchdog = False

    def __init__(self, instance, context, transport_factory=None):
        super().__init__(instance, context, transport_factory)
        self.streaming = False
        self.last_sent = None
        self.desired = None

    def validate_configuration(self):
        validate_parameters(self.parameters, real=True)

    def make_transport(self, parameters):
        from .transport import FR3Transport
        return FR3Transport(parameters)

    def _run(self):
        period = 1. / self.parameters['rate_hz']
        deadline = time.monotonic()
        while not self.shutdown.is_set():
            try:
                with self.io:
                    feedback = self.transport.read()
                    values = vector6(feedback['values'])
                    stamp = feedback['sample_ns']
                    age = (self.clock() - stamp) / 1e9
                    if not 0 <= age <= self.parameters['feedback_timeout_s']:
                        raise TimeoutError('FR3 feedback stale')
                    with self.lock:
                        self.sample = {
                            'ready': True, 'sample_ns': stamp, 'source': 'hardware',
                            'measured': True, 'synthetic': False, 'values': values,
                            'ports': {'position': {'values': values, 'valid': True,
                                                 'received_ns': stamp}},
                            'device_status': {k: v for k, v in feedback.items()
                                              if k not in ('values', 'sample_ns')},
                        }
                    require_safe(feedback)
                    with self.lock:
                        if self.streaming and feedback['enabled'] != 1:
                            raise RuntimeError('FR3 lost enable state')
                        if self.pending is not None:
                            self.desired, self.pending = self.pending, None
                        desired = self.desired
                        if desired is not None and self.streaming:
                            start = self.last_sent if self.last_sent is not None else values
                            step = self.parameters['max_velocity_rad_s'] * period
                            q = [a + max(-step, min(step, b - a)) for a, b in zip(start, desired)]
                            q = positions(q, self.parameters)
                            if max(abs(a - b) for a, b in zip(q, values)) > self.parameters['max_command_lead_rad']:
                                raise RuntimeError('FR3 servo target exceeds measured command lead')
                        else:
                            q = None
                    if q is not None:
                        self.transport.write(q)
                        with self.lock:
                            self.last_sent = q
            except Exception as exc:
                with self.lock:
                    was_streaming = self.streaming
                    self.active = self.streaming = False
                    self.pending = self.desired = self.last_sent = None
                    self.sample.update(ready=False, fault=True, reason=str(exc))
                    for port in self.sample.get('ports', {}).values():
                        port['valid'] = False
                if was_streaming:
                    try:
                        with self.io:
                            self.transport.stop()
                    except Exception as stop_error:
                        with self.lock:
                            self.sample['stop_error'] = str(stop_error)
                return  # no automatic re-arm or replay after faults
            deadline += period
            now = time.monotonic()
            if deadline < now:
                deadline = now + period  # never burst queued catch-up writes
            self.shutdown.wait(max(0., deadline - now))

    def health(self, now_ns):
        result = super().health(now_ns)
        result['ready'] = result['ready'] and not result.get('fault') and (
            0 <= now_ns - result['sample_ns'] <= self.parameters['feedback_timeout_s'] * 1e9)
        for port in result.get('ports', {}).values():
            port['valid'] = result['ready']
        return result

    def activate(self):
        with self.io:
            self.prepare()
            try:
                self.transport.activate()
            except Exception as exc:
                with self.lock:
                    self.sample.update(ready=False, fault=True, reason=str(exc))
                raise
            with self.lock:
                self.active = self.streaming = True
                self.pending = None
                # Re-activation after release retains the hold position; after
                # stop there is no old target and the first command must be new.
                self.desired = self.last_sent
        return True

    def submit(self, target):
        q = validate_target(target, self.instance)
        with self.lock:
            if not self.active or self.sample.get('fault'):
                raise RuntimeError('ADAPTER_DISARMED')
            if not 0 <= self.clock() - self.sample['sample_ns'] <= self.parameters['feedback_timeout_s'] * 1e9:
                raise RuntimeError('FR3 feedback stale')
            if max(abs(a - b) for a, b in zip(q, self.sample['values'])) > self.parameters['max_command_lead_rad']:
                raise ValueError('FR3 target too far from measured position; use incremental targets')
            self.pending = q
        return {'accepted': True, 'completed': False, 'phase': 'queued'}

    def release(self):
        # Serialize with an in-flight write; when this returns no late motion
        # target can replace the hold. Hold last SENT, never an unsent target.
        with self.io:
            with self.lock:
                self.active = False
                self.pending = None
                self.desired = self.last_sent
        return True

    def stop(self):
        with self.io:
            with self.lock:
                self.active = self.streaming = False
                self.pending = self.desired = self.last_sent = None
            if self.transport is not None:
                self.transport.stop()
        return True

    def close(self):
        self.shutdown.set()
        if self.thread is not None:
            # Includes an in-progress operation and a possible fault-stop.
            self.thread.join(timeout=2 * self.parameters['operation_timeout_s'] + 2)
            if self.thread.is_alive():
                raise TimeoutError('FR3 device worker did not exit')
        if self.transport is not None:
            try:
                self.stop()  # also cleans up a softly released servo
            finally:
                self.transport.close()


class FR3FakeAdapter(FakeAdapter):
    def configure(self, context):
        validate_parameters(self.instance['parameters'], real=False)
        self.values = positions(self.instance['parameters'].get('initial_positions', INITIAL),
                                self.instance['parameters'])
        super().configure(context)

    def submit(self, target):
        validate_target(target, self.instance)
        return super().submit(target)


def create_adapter(instance, context):
    if instance['backend'] == 'fake':
        return FR3FakeAdapter(instance)
    if instance['backend'] == 'real':
        return FR3Adapter(instance, context)
    raise ValueError('FR3 sim is owned by the shared Runtime; direct factory supports fake and real')
