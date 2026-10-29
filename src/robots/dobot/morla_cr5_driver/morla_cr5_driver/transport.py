"""Bounded CR5 TCP protocols. Constructors never enable motors."""
import math
import re
import socket
import time
import threading


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
        self.socket = socket.create_connection((host, port), timeout=timeout)
        self.socket.settimeout(timeout)

    def request(self, command):
        deadline = time.monotonic() + self.timeout
        self.socket.settimeout(self.timeout)
        self.socket.sendall(command.encode('ascii'))
        data = bytearray()
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
        response = data.decode('ascii').strip()
        match = re.fullmatch(r'(-?\d+),\s*\{([^}]*)\},\s*(.*);', response, re.S)
        if not match or int(match[1]) != 0 or match[3].strip() != command:
            raise RuntimeError('CR5 rejected or mismatched response: ' + response)
        return match[2]

    def close(self):
        self.socket.close()


class CR5Transport:
    def __init__(self, parameters):
        host = parameters['host']; timeout = parameters['timeout_s']
        self.dashboard = DobotChannel(host, 29999, timeout)
        try:
            self.motion = DobotChannel(host, 30003, timeout)
            self.feedback = socket.create_connection((host, 30004), timeout=timeout)
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
                self.sample['enable_status'] == enabled), timeout=.5)
            if not ok or self.error: raise RuntimeError('CR5 enable/disable feedback unconfirmed')

    def activate(self):
        self.dashboard.request('EnableRobot()')
        self._wait_enabled(1, time.monotonic_ns())

    def write(self, values):
        numbers = ','.join(f'{math.degrees(v):.6f}' for v in values)
        self.motion.request('ServoJ(' + numbers + ',t=0.100000,lookahead_time=50.000000,gain=500.000000)')

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
        self.dashboard.request('DisableRobot()')
        self._wait_enabled(0, time.monotonic_ns())

    def close(self):
        self.closed.set()
        self.feedback.close(); self.thread.join(timeout=1)
        if self.thread.is_alive(): raise TimeoutError('CR5 feedback worker did not exit')
        self.motion.close(); self.dashboard.close()
