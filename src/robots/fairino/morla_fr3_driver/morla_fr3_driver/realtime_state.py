"""Checked V2.1.5 / Robot V3.8.5 TCP 20004 boundary, not a CNDE parser.

Keep vendor Robot.py unchanged. Its ctypes layout is the versioned wire contract;
replace only its receive/reconnect/close routines to make frame ownership explicit.
"""
import copy
import ctypes
import math
import socket
import threading
import time
import xmlrpc.client

from .contract import vector6

# FR3-V1-001 running V3.8.25-QX sends 953 body bytes, two more than the packed
# RobotStatePkg (951). The extra bytes sit in the trailing all-zero region: every
# field this driver consumes (positions, code, mode, enable, e-stop, safety and
# collision flags) decodes from unchanged offsets, and the frame checksum covers
# the whole 960 bytes. Accept that one extra length instead of failing the
# session; anything else is still a layout mismatch.
EXTRA_FRAME_BYTES = 2


class StateFrames:
    def __init__(self, module):
        self.module = module
        self.size = ctypes.sizeof(module.RobotStatePkg)
        self.lengths = (self.size, self.size + EXTRA_FRAME_BYTES)
        self.buffer = bytearray()
        self.lock = threading.Lock()
        self.sample = None
        self.error = None
        self.sequence = 0
        self.frame_count = None
        self.state = None

    def fail(self, error):
        with self.lock:
            if self.error is None:
                self.error = str(error)

    def feed(self, chunk):
        """Handle arbitrary TCP fragmentation/coalescing; latch malformed frames."""
        with self.lock:
            if self.error is not None:
                return
            self.buffer.extend(chunk)
            try:
                while len(self.buffer) >= 5:
                    if self.buffer[:2] != b'\x5a\x5a':
                        raise ValueError('FR3 20004 frame header mismatch')
                    total = int.from_bytes(self.buffer[3:5], 'little') + 7
                    if total not in self.lengths:
                        raise ValueError(f'FR3 20004 frame length {total}, expected {self.lengths[0]} or {self.lengths[1]}; SDK/firmware layout mismatch')
                    if len(self.buffer) < total:
                        return
                    raw = bytes(self.buffer[:total])
                    del self.buffer[:total]
                    if sum(raw[:-2]) & 0xffff != int.from_bytes(raw[-2:], 'little'):
                        raise ValueError('FR3 20004 frame checksum mismatch')
                    state = self.module.RobotStatePkg.from_buffer_copy(raw[:self.size])
                    values = vector6(list(state.jt_cur_pos))
                    count = int(state.frame_cnt) & 0xff
                    if count == self.frame_count:
                        continue
                    self.state = state
                    self.sequence += 1
                    self.frame_count = count
                    self.sample = {
                        'values': [math.radians(v) for v in values],
                        'sample_ns': time.monotonic_ns(),
                        'frame_count': count, 'sequence': self.sequence,
                        'frame_extra_bytes': total - self.size,
                        'enabled': int(state.rbtEnableState),
                        'main_code': int(state.main_code), 'sub_code': int(state.sub_code),
                        'emergency_stop': int(state.EmergencyStop),
                        'safety_stop0': int(state.safety_stop0_state),
                        'safety_stop1': int(state.safety_stop1_state),
                        'collision': int(state.collisionState),
                        'robot_mode': int(state.robot_mode),
                    }
            except Exception as exc:
                self.error = str(exc)

    def read(self):
        with self.lock:
            if self.error is not None:
                raise ConnectionError(self.error)
            return copy.deepcopy(self.sample)


class _TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection

    def request(self, host, handler, request_body, verbose=False):
        # Transport.request retries a disconnected request once by default.
        # Servo/enable/stop must never be silently replayed.
        return self.single_request(host, handler, request_body, verbose)


class _SingleAttemptProxy:
    def __init__(self, host, parameters):
        url = f'http://{host}:20003'
        self.fast = xmlrpc.client.ServerProxy(url, transport=_TimeoutTransport(parameters['rpc_timeout_s']))
        self.slow = xmlrpc.client.ServerProxy(url, transport=_TimeoutTransport(parameters['operation_timeout_s']*.8))

    def __getattr__(self, name):
        proxy = self.slow if name in {'RobotEnable','ServoMoveStart','ServoMoveEnd','StopMotion'} else self.fast
        def call(*args):
            try:
                return getattr(proxy, name)(*args)
            except OSError as exc:
                # Vendor methods catch socket.error in unbounded retry loops.
                raise RuntimeError(f'FR3 XML-RPC {name} failed; not retried: {exc}') from exc
        return call

    def close(self):
        self.fast('close')()
        self.slow('close')()


def checked_rpc_class(module, parameters):
    class CheckedRPC(module.RPC):
        def __init__(self, host):
            self.morla_frames = StateFrames(module)
            self.morla_thread = None
            self.closeRPC_state = False
            self.reconnect_flag = False
            self.SDK_state = True
            super().__init__(host)
            self.robot('close')()
            self.robot = _SingleAttemptProxy(host, parameters)

        def connect_to_robot(self):
            try:
                self.sock_cli_state = socket.create_connection(
                    (self.ip_address, self.ROBOT_REALTIME_PORT),
                    timeout=parameters['rpc_timeout_s'])
                self.sock_cli_state.settimeout(.05)
                self.sock_cli_state_state = True
                return True
            except OSError as exc:
                self.sock_cli_state_state = False
                self.SDK_state = False
                self.morla_frames.fail(f'FR3 {self.ip_address}:20004 connection failed: {exc}')
                return False

        def reconnect(self):
            # Late commands must never resume merely because a socket reconnects.
            self.SDK_state = False
            self.sock_cli_state_state = False
            self.morla_frames.fail('FR3 20004 disconnected; no automatic reconnect')
            return False

        def robot_state_routine_thread(self):
            self.morla_thread = threading.current_thread()
            try:
                if not self.sock_cli_state_state:
                    return
                last_data = time.monotonic()
                while not self.stop_event.is_set():
                    try:
                        chunk = self.sock_cli_state.recv(65536)
                    except socket.timeout:
                        if time.monotonic() - last_data > parameters['feedback_timeout_s']:
                            raise TimeoutError('FR3 20004 feedback stream timed out')
                        continue
                    if not chunk:
                        raise ConnectionError('FR3 20004 feedback EOF')
                    last_data = time.monotonic()
                    self.morla_frames.feed(chunk)
                    sample = self.morla_frames.read()
                    with self.morla_frames.lock:
                        if self.morla_frames.state is not None:
                            self.robot_state_pkg = self.morla_frames.state
                    if sample is not None and (time.monotonic_ns()-sample['sample_ns'])/1e9 > parameters['feedback_timeout_s']:
                        raise TimeoutError('FR3 repeated 20004 frames; feedback stale')
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.SDK_state = False
                    self.morla_frames.fail(exc)
            finally:
                self.sock_cli_state_state = False

        def CloseRPC(self):
            # Vendor implementation stores the actual receiver only in a local
            # variable and may skip cleanup when class is_conect becomes False.
            self.closeRPC_state = True
            self.stop_event.set()
            sock = self.sock_cli_state
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                sock.close()
                self.sock_cli_state = None
            thread = self.morla_thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=.5)
            if self.robot is not None:
                # ServerProxy.close is a *remote method*. Use the local closer.
                self.robot.close()
                self.robot = None
            self.SDK_state = False
    return CheckedRPC
