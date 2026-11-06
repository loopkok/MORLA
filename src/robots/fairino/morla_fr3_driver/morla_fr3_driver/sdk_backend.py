"""Pinned FAIRINO SDK boundary. This module runs only in the owned subprocess.

The pinned vendor ctypes layout is retained. A checked TCP 20004 receiver
copies complete validated frames atomically; reading SDK.robot_state_pkg alone has no freshness
contract and its getters sometimes return cached values with success code 0.
"""
import math
import time

from .contract import vector6


def ack(name, result):
    if type(result) is not int or result != 0:
        raise RuntimeError(f'FAIRINO {name} rejected: {result!r}')


def require_safe(sample):
    for key in ('main_code', 'sub_code', 'emergency_stop', 'safety_stop0', 'safety_stop1', 'collision'):
        if sample[key] != 0:
            raise RuntimeError(f'FR3 controller fault: {key}={sample[key]}')


class SDKBackend:
    def __init__(self, parameters):
        # No assembly-controlled imports or SDK path; exact source ships in this package.
        from .vendor import Robot
        self.parameters = parameters
        self.sdk = None
        self.owned_enable = False
        self.servo_started = False
        from .realtime_state import checked_rpc_class
        rpc = checked_rpc_class(Robot, parameters)
        try:
            self.sdk = rpc(parameters['host'])
            if not self.sdk.is_conect:
                raise ConnectionError('FAIRINO SDK connection failed (SDK V2.1.5: XML-RPC 20003 / realtime 20004)')
            self._wait(lambda sample: True)
        except Exception:
            self.close()
            raise

    def read(self):
        sample = self.sdk.morla_frames.read()
        if sample is None:
            raise RuntimeError('FR3 has no complete TCP 20004 feedback frame')
        age = (time.monotonic_ns() - sample['sample_ns']) / 1e9
        if not 0 <= age <= self.parameters['feedback_timeout_s']:
            raise TimeoutError(f'FR3 feedback stale ({age:.3f}s); no automatic reconnect')
        if not self.sdk.SDK_state or not self.sdk.sock_cli_state_state:
            raise ConnectionError('FR3 SDK reports communication failure')
        return sample

    def _wait(self, predicate, *, after=0):
        deadline = time.monotonic() + self.parameters['operation_timeout_s'] * .8
        last_error = None
        while time.monotonic() < deadline:
            try:
                sample = self.read()
            except (RuntimeError, TimeoutError) as exc:
                last_error = exc
            else:
                if sample['sample_ns'] > after and predicate(sample):
                    return sample
            time.sleep(.008)
        raise TimeoutError(f'FR3 state confirmation timed out: {last_error or "state not reached"}')

    def activate(self):
        sample = self.read()
        require_safe(sample)
        if self.servo_started and sample['enabled'] == 1:
            return  # resume a soft hold without opening a second servo session
        # Ownership is set before sending: an error/late reply may still have enabled the arm.
        self.owned_enable = True
        try:
            stamp = time.monotonic_ns()
            ack('RobotEnable(1)', self.sdk.RobotEnable(1))
            sample = self._wait(lambda s: s['enabled'] == 1, after=stamp)
            require_safe(sample)
            ack('ServoMoveStart', self.sdk.ServoMoveStart())
            self.servo_started = True
        except Exception:
            self.stop()
            raise

    def write(self, q):
        if not self.servo_started:
            raise RuntimeError('FR3 servo is not active')
        sample = self.read()
        require_safe(sample)
        if sample['enabled'] != 1:
            raise RuntimeError('FR3 lost enable state')
        ack('ServoJ', self.sdk.ServoJ(
            [math.degrees(v) for v in vector6(q)], [0., 0., 0., 0.],
            cmdT=1. / self.parameters['rate_hz']))

    def stop(self):
        if not self.owned_enable:
            return  # never-activated read-only sessions send no control opcodes
        errors = []
        # Attempt all three operations even when a controller rejects one of them.
        for name, args in [('StopMotion', ()), ('ServoMoveEnd', ()), ('RobotEnable', (0,))]:
            try:
                ack(name, getattr(self.sdk, name)(*args))
            except Exception as exc:
                errors.append(str(exc))
        stamp = time.monotonic_ns()
        self.servo_started = False
        try:
            self._wait(lambda s: s['enabled'] == 0, after=stamp)
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            raise RuntimeError('FR3 stop unconfirmed: ' + '; '.join(errors))
        self.owned_enable = False

    def forward_kinematics(self, q):
        result = self.sdk.GetForwardKin([math.degrees(v) for v in vector6(q)])
        if not isinstance(result, (tuple, list)) or len(result) != 2:
            raise RuntimeError(f'Invalid GetForwardKin reply: {result!r}')
        ack('GetForwardKin', result[0])
        return vector6(result[1])

    def close(self):
        if self.sdk is not None:
            try:
                if self.owned_enable:
                    self.stop()
            finally:
                self.sdk.CloseRPC()
