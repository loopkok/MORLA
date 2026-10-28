"""Independent runtime review counterexamples; no ROS or hardware required."""
import copy
import json
import tempfile
import threading
from types import SimpleNamespace as S
from morla_core.composition import compile_system
from morla_runtime import RuntimeSession, FakeAdapter
from morla_runtime.native_ros import NativeDeviceSession, NativeRosAdapter, NativeRosTransport


def run():
    resolved = compile_system({'assembly_version': 1, 'assembly_id': 'review', 'revision': 1,
                               'instances': {'arm': {'plugin': 'morla.fake.arm'}}})
    results = {}
    tampered = copy.deepcopy(resolved)
    tampered.pop('resolved_hash')
    tampered['instances'][0]['joints'][0]['upper'] = 9999
    try:
        RuntimeSession(tampered, resource_root=tempfile.mkdtemp())
        results['missing_hash_rejected'] = False
    except Exception as exc:
        results['missing_hash_rejected'] = str(exc) == 'RESOLVED_HASH_REQUIRED'

    class Transport:
        def __init__(self): self.calls = 0
        def operation(self, key, op): self.calls += 1; return False
    transport = Transport()
    device = NativeDeviceSession({'session_family': 'nero'}, [], S(), None)
    device.transport, device.enabled = transport, True
    adapter = NativeRosAdapter({'instance_id': 'arm', 'deployment_verified': True,
                                'watchdog_verified': True}, device)
    first, second = adapter.stop(), adapter.stop()
    results['failed_stop_ack_cannot_be_erased'] = not first and not second and transport.calls == 2 and device.enabled

    bridge = NativeRosTransport.__new__(NativeRosTransport)
    bridge.plan = {'ports': {'g': {'native_names': ['right_gripper'], 'plugin_id': 'morla.astral.gripper',
                                  'gripper_open_rad': .8, 'gripper_closed_rad': 0.}}}
    bridge.samples, bridge.source_clocks, bridge.source_faults = {}, {}, set()
    bridge.clock, bridge.lock = lambda: 100, threading.Lock()
    message = S(name=['right_gripper'], position=[.8], velocity=[],
                header=S(stamp=S(sec=1, nanosec=0)))
    bridge._sample('g', message)
    sample = bridge.samples['g']
    results['command_echo_not_hardware_measurement'] = (
        not sample['ready'] and not sample['measured'] and sample['source'] == 'command_echo'
        and sample['ports']['position']['unit'] == 'ratio' and sample['values'] == [0.])

    class BrokenStop(FakeAdapter):
        def stop(self): self.calls.append(('failed_stop',)); return False
    runtime = RuntimeSession(resolved, resource_root=tempfile.mkdtemp(),
                             adapters={'arm': BrokenStop(resolved['instances'][0])})
    try:
        runtime.start()
        runtime.activate('activate')
        runtime.estop()
        reset = runtime.reset('reset')
        results['reset_requires_acknowledged_stop'] = reset['status'] == 'FAILED' and runtime.status == 'ESTOP_LATCHED'
    finally:
        runtime.stop()

    future = S(done=lambda: True, result=lambda: S(success=True, message='queued'))
    client = S(wait_for_service=lambda timeout_sec: True, srv_type=S(Request=lambda: S()),
               call_async=lambda request: future)
    stale = NativeRosTransport.__new__(NativeRosTransport)
    stale.plan = {'ports': {'n': {'plugin_id': 'morla.nero.arm'}}}
    stale.flags, stale.clients = {}, {('n', 'stop'): client}
    stale.statuses = {'n': (100, {'estop_status': 'sent', 'estop_seq': 1})}
    stale.clock, stale.lock = lambda: 100, threading.Lock()
    try:
        accepted = stale.operation('n', 'stop', timeout=.01)
        results['previous_nero_stop_ack_rejected'] = accepted is False
    except (TimeoutError, RuntimeError):
        results['previous_nero_stop_ack_rejected'] = True
    correlated_future = S(done=lambda: True, result=lambda: S(success=True, message='queued [morla_seq=2]'))
    stale.clients[('n', 'stop')].call_async = lambda request: correlated_future
    try:
        accepted = stale.operation('n', 'stop', timeout=.01)
        results['previous_nero_generation_rejected'] = accepted is False
    except (TimeoutError, RuntimeError):
        results['previous_nero_generation_rejected'] = True
    stale.statuses['n'] = (100, {'estop_status': 'sent', 'estop_seq': 2})
    results['matching_nero_generation_acknowledged'] = stale.operation('n', 'stop', timeout=.01) is True
    return results


if __name__ == '__main__':
    results = run()
    print(json.dumps(results, indent=2))
    raise SystemExit(0 if all(results.values()) else 1)
