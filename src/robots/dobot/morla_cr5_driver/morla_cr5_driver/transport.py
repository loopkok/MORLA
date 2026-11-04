"""Bounded CR5 TCP protocols. Constructors never enable motors."""
import math
import re
import select
import socket
import time
import threading


# Dobot's official Python wrapper (dobot_api.py) declares these ServoJ
# defaults and accepted ranges. Keep the defaults unchanged here: they are the
# vendor's, not values tuned for this cell's firmware, payload or teleop lag.
# A deployment may override them with `servo_t` / `servo_lookahead_time` /
# `servo_gain` once on-robot evidence justifies new values.
SERVO_DEFAULTS = {'t': .1, 'lookahead_time': 50., 'gain': 500.}
SERVO_RANGES = {'t': (.02, 3600.), 'lookahead_time': (20., 100.), 'gain': (200., 1000.)}


def servo_parameters(parameters):
    """Validated ServoJ parameters, defaulting to Dobot's documented values."""
    servo = {}
    for name, default in SERVO_DEFAULTS.items():
        value = parameters.get('servo_' + name, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'CR5 ServoJ {name} must be a finite number')
        low, high = SERVO_RANGES[name]
        if not low <= value <= high:
            raise ValueError(f'CR5 ServoJ {name} outside [{low:g}, {high:g}]')
        servo[name] = float(value)
    return servo


def parse_cr5_feedback(packet):
    import numpy as np
    from .vendor.dobot_feedback import MyType
    if len(packet) != MyType.itemsize:
        raise ValueError('CR5 feedback length mismatch')
    frame = np.frombuffer(packet, dtype=MyType.newbyteorder('<'), count=1)[0]
    if int(frame['len']) != MyType.itemsize or int(frame['test_value']) != 0x123456789abcdef:
        raise ValueError('CR5 feedback layout/magic mismatch')
    values = [math.radians(float(v)) for v in frame['q_actual']]
    if not all(map(math.isfinite, values)):
        raise ValueError('CR5 invalid measured feedback')
    return {'values': values, 'enable_status': int(frame['enable_status']),
            'error_status': int(frame['error_status']), 'device_time_raw': int(frame['time_stamp'])}


class DobotChannel:
    def __init__(self, host, port, timeout):
        self.timeout = timeout
        try:
            self.socket = socket.create_connection((host, port), timeout=timeout)
        except OSError as exc:
            # Bare "timed out" loses the only fact the operator needs.
            raise ConnectionError(f'CR5 {host}:{port} unreachable: {exc}') from exc
        self.socket.settimeout(timeout)
        # The dashboard carries no request id, so a reply that missed its
        # deadline is indistinguishable from the next command's reply. A
        # timeout therefore poisons the channel until the late bytes go away.
        self.dirty = False

    def _resync(self):
        self.dirty = False
        if not hasattr(self.socket, 'fileno'):
            return
        while True:
            ready, _, _ = select.select([self.socket], [], [], 0)
            if not ready:
                return
            if not self.socket.recv(4096):
                raise ConnectionError('CR5 disconnected')

    def request(self, command):
        if getattr(self, 'dirty', False):
            self._resync()
        deadline = time.monotonic() + self.timeout
        self.socket.settimeout(self.timeout)
        self.socket.sendall(command.encode('ascii'))
        data = bytearray()
        try:
            while b';' not in data:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('CR5 response deadline exceeded')
                self.socket.settimeout(remaining)
                part = self.socket.recv(1024)
                if not part:
                    raise ConnectionError('CR5 disconnected')
                data.extend(part)
                if len(data) > 8192:
                    raise ValueError('CR5 response oversized')
        except TimeoutError:
            self.dirty = True
            raise
        response = data.decode('ascii').strip()
        match = re.fullmatch(r'(-?\d+),\s*\{([^}]*)\},\s*(.*);', response, re.S)
        if not match or int(match[1]) != 0 or match[3].strip() != command:
            raise RuntimeError('CR5 rejected or mismatched response: ' + response)
        return match[2]

    def close(self):
        self.socket.close()


class CR5Transport:
    def __init__(self, parameters):
        # Reject bad ServoJ values before any socket is opened.
        self.servo = servo_parameters(parameters)
        host = parameters['host']; timeout = parameters['timeout_s']
        # Enable/disable run the controller's brake and servo sequence before
        # they answer: measured 5.2 s to ack EnableRobot and 4.2 s to show
        # enable_status=1 on feedback, against sub-millisecond read commands.
        # Only those two calls get the wider deadline; FK/IK keep timeout_s.
        self.enable_timeout = float(parameters.get('enable_timeout_s', 10.))
        self.dashboard = DobotChannel(host, 29999, timeout)
        try:
            self.motion = DobotChannel(host, 30003, timeout)
            try:
                self.feedback = socket.create_connection((host, 30004), timeout=timeout)
            except OSError as exc:
                raise ConnectionError(f'CR5 {host}:30004 unreachable: {exc}') from exc
            self.feedback.settimeout(timeout)
        except Exception:
            if hasattr(self, 'motion'): self.motion.close()
            self.dashboard.close()
            raise
        self.sample = None; self.error = None; self.closed = threading.Event()
        self.updated = threading.Condition()
        self.thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self.thread.start()

    def _feedback_loop(self):
        from .vendor.dobot_feedback import MyType
        buffer = bytearray()
        previous = None
        try:
            while not self.closed.is_set():
                part = self.feedback.recv(MyType.itemsize - len(buffer))
                if not part: raise ConnectionError('CR5 feedback disconnected')
                buffer.extend(part)
                if len(buffer) != MyType.itemsize: continue
                sample = parse_cr5_feedback(bytes(buffer)); buffer.clear()
                if previous is not None and sample['device_time_raw'] <= previous:
                    raise ValueError('CR5 feedback device clock stalled/regressed')
                previous = sample['device_time_raw']
                sample['sample_ns'] = time.monotonic_ns()
                with self.updated:
                    self.sample = sample
                    self.updated.notify_all()
        except Exception as exc:
            with self.updated:
                self.error = str(exc); self.updated.notify_all()

    def read(self):
        with self.updated:
            self.updated.wait_for(lambda: self.sample is not None or self.error, timeout=.5)
            if self.error or self.sample is None: raise RuntimeError(self.error or 'CR5 feedback unavailable')
            if self.sample['error_status']: raise RuntimeError('CR5 controller reports robot error')
            return dict(self.sample)

    def _wait_enabled(self, enabled, after_ns):
        with self.updated:
            ok = self.updated.wait_for(lambda: self.error or (
                self.sample is not None and self.sample['sample_ns'] > after_ns and
                self.sample['enable_status'] == enabled), timeout=self.enable_timeout)
            if not ok or self.error: raise RuntimeError('CR5 enable/disable feedback unconfirmed')

    def _state_command(self, command):
        """Run a command whose ack may arrive long after it took effect.

        The controller acts first and answers afterwards, so a missed deadline
        says nothing about whether the command ran. Feedback is the authority;
        the unread ack is discarded before the next command on this channel.
        Only this call gets the wide deadline: FK/IK share the channel and must
        keep the short one.
        """
        channel = self.dashboard
        original, channel.timeout = channel.timeout, max(channel.timeout, self.enable_timeout)
        try:
            return channel.request(command)
        except TimeoutError:
            return None
        finally:
            channel.timeout = original

    def activate(self):
        after = time.monotonic_ns()
        self._state_command('EnableRobot()')
        self._wait_enabled(1, after)

    def write(self, values):
        numbers = ','.join(f'{math.degrees(v):.6f}' for v in values)
        servo = self.servo
        self.motion.request('ServoJ(' + numbers + f",t={servo['t']:.6f},"
                            f"lookahead_time={servo['lookahead_time']:.6f},gain={servo['gain']:.6f})")

    @staticmethod
    def _six_reply(raw):
        values = [float(v) for v in raw.split(',')]
        if len(values) != 6 or not all(map(math.isfinite, values)):
            raise ValueError('CR5 malformed FK/IK result')
        return values

    def forward_kinematics(self, joints):
        numbers = ','.join(f'{math.degrees(v):.6f}' for v in joints)
        return self._six_reply(self.dashboard.request('PositiveSolution(' + numbers + ',0,0)'))

    def inverse_kinematics(self, pose, seed):
        numbers = ','.join(f'{v:.6f}' for v in pose)
        near = ','.join(f'{math.degrees(v):.6f}' for v in seed)
        command = 'InverseSolution(' + numbers + ',0,0,1,{' + near + '})'
        return self._six_reply(self.dashboard.request(command))

    def stop(self):
        # Disable rather than a zero-position command; real stop latency requires acceptance.
        after = time.monotonic_ns()
        self._state_command('DisableRobot()')
        self._wait_enabled(0, after)

    def close(self):
        self.closed.set()
        self.feedback.close(); self.thread.join(timeout=1)
        if self.thread.is_alive(): raise TimeoutError('CR5 feedback worker did not exit')
        self.motion.close(); self.dashboard.close()
