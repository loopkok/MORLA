"""Pinned FAIRINO SDK boundary. This module runs only in the owned subprocess.

The vendor decoder is retained. A small subclass observes complete CNDE frames
and copies state atomically; reading SDK.robot_state_pkg alone has no freshness
contract and its getters sometimes return cached values with success code 0.
"""
import copy
import math
import threading
import time

from .contract import vector6


def ack(name, result):
    if type(result) is not int or result != 0:
        raise RuntimeError(f'FAIRINO {name} rejected: {result!r}')


def checked_cnde_class(module):
    class CheckedCNDE(module.FRCNDEClient):
        def __init__(self, *args, **kwargs):
            self.morla_lock = threading.Lock()
            self.morla_sample = None
            self.morla_error = None
            self.morla_sequence = 0
            self.morla_frame = None
            super().__init__(*args, **kwargs)

        def _parse_and_set_field(self, *args, **kwargs):
            try:
                return super()._parse_and_set_field(*args, **kwargs)
            except Exception as exc:
                self.morla_error = f'CNDE field decode failed: {exc}'
                raise

        def _parse_cnde_state_data(self, data):
            with self.morla_lock:
                if self.morla_error:
                    return  # malformed state is latched until a new session
                try:
                    size = sum(module.get_cnde_type_size(self._all_states[s][3])
                               for s in self._config_states)
                    if len(data) != size:
                        raise ValueError(f'CNDE payload length {len(data)}, expected {size}')
                    super()._parse_cnde_state_data(data)
                    if self.morla_error:
                        return
                    state = self._robot_state_pkg
                    if self.morla_frame == state.frame_cnt:
                        return  # repeated frame never refreshes sample_ns
                    values = vector6(list(state.jt_cur_pos))
                    self.morla_sequence += 1
                    self.morla_frame = state.frame_cnt
                    self.morla_sample = {
                        'values': [math.radians(v) for v in values],
                        'sample_ns': time.monotonic_ns(),
                        'frame_count': int(state.frame_cnt), 'sequence': self.morla_sequence,
                        'enabled': int(state.rbtEnableState),
                        'main_code': int(state.main_code), 'sub_code': int(state.sub_code),
                        'emergency_stop': int(state.EmergencyStop),
                        'safety_stop0': int(state.safety_stop0_state),
                        'safety_stop1': int(state.safety_stop1_state),
                        'collision': int(state.collisionState),
                        'robot_mode': int(state.robot_mode),
                    }
                except Exception as exc:
                    self.morla_error = str(exc)
    return CheckedCNDE


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
        Robot.FRCNDEClient = checked_cnde_class(Robot)
        Robot.DEFAULT_CNDE_STATES = [getattr(Robot.RobotState, name) for name in (
            'FrameCnt', 'JointCurPos', 'RbtEnableState', 'MainCode', 'SubCode',
            'EmergencyStop', 'SafetyStop0State', 'SafetyStop1State', 'CollisionState', 'RobotMode')]
        Robot.DEFAULT_CNDE_PERIOD = 8
        Robot.RPC._reconnect_enable = False
        try:
            self.sdk = Robot.RPC(parameters['host'])
            if not self.sdk.is_connect:
                raise ConnectionError('FAIRINO SDK connection failed (XML-RPC 20003 / CNDE 20005)')
            ack('SetReConnectParam', self.sdk.SetReConnectParam(False, 0, 1000))
            self._wait(lambda sample: True)
        except Exception:
            self.close()
            raise

    def read(self):
        client = self.sdk._cnde_client
        if client is None:
            raise ConnectionError('FR3 CNDE is disconnected')
        with client.morla_lock:
            if client.morla_error:
                raise RuntimeError(client.morla_error)
            sample = copy.deepcopy(client.morla_sample)
        if sample is None:
            raise RuntimeError('FR3 has no complete CNDE feedback frame')
        age = (time.monotonic_ns() - sample['sample_ns']) / 1e9
        if not 0 <= age <= self.parameters['feedback_timeout_s']:
            raise TimeoutError(f'FR3 feedback stale ({age:.3f}s); no automatic reconnect')
        if not self.sdk.SDK_state or self.sdk._com_err_flag[0] != 0:
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
