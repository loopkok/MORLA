"""Soft release on a streaming device: hold the measured pose, never power down."""
import time
from morla_runtime.cached_actuator import CachedActuatorAdapter


class Transport:
    def __init__(self):
        self.values = [0.1]
        self.writes = []
        self.stopped = 0
        self.activated = 0

    def activate(self): self.activated += 1

    def write(self, values): self.writes.append(list(values))

    def read(self): return {'values': list(self.values)}

    def stop(self): self.stopped += 1

    def close(self): pass


def wait_for(predicate, timeout=.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate(): return True
        time.sleep(.005)
    return False


def test_release_holds_last_target_without_stopping_transport():
    transport = Transport()
    adapter = CachedActuatorAdapter({'instance_id': 'arm', 'joints': [{'name': 'j1'}],
                                     'parameters': {'rate_hz': 200}},
                                    {'clock': lambda: 0}, transport_factory=lambda parameters: transport)
    adapter.configure({})
    try:
        adapter.activate()
        assert transport.activated == 1
        adapter.submit({'values': [0.5]})
        assert wait_for(lambda: [0.5] in transport.writes)
        # The arm settles below its target under gravity; a hold on the
        # measurement would hand back that offset on every release.
        transport.values = [0.25]
        assert wait_for(lambda: adapter.sample['values'] == [0.25])
        assert adapter.release() is True
        writes = len(transport.writes)
        assert wait_for(lambda: transport.writes[writes:].count([0.5]) >= 5)
        assert transport.stopped == 0 and not adapter.active
        assert adapter.stop() is True
        assert transport.stopped == 1
    finally:
        adapter.close()


def test_release_without_a_command_holds_the_measurement():
    transport = Transport()
    transport.values = [0.4]
    adapter = CachedActuatorAdapter({'instance_id': 'arm', 'joints': [{'name': 'j1'}],
                                     'parameters': {'rate_hz': 200}},
                                    {'clock': lambda: 0}, transport_factory=lambda parameters: transport)
    adapter.configure({})
    try:
        adapter.activate()
        assert wait_for(lambda: adapter.sample['values'] == [0.4])
        assert adapter.release() is True
        writes = len(transport.writes)
        assert wait_for(lambda: transport.writes[writes:].count([0.4]) >= 5)
        assert transport.stopped == 0
    finally:
        adapter.close()


def test_release_without_fresh_feedback_falls_back_to_stop():
    transport = Transport()
    adapter = CachedActuatorAdapter({'instance_id': 'arm', 'joints': [{'name': 'j1'}],
                                     'parameters': {'rate_hz': 200}},
                                    {'clock': lambda: 0}, transport_factory=lambda parameters: transport)
    adapter.configure({})
    try:
        adapter.sample.update(ready=False, fault=True, reason='FAULT')
        assert adapter.release() is True
        assert transport.stopped == 1
        assert transport.writes == []
    finally:
        adapter.close()
