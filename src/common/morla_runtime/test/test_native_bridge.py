import os
import subprocess
import sys
import pytest
from morla_core.device_guard import LegacyDeviceGuard
from morla_runtime import ResourceManager, RuntimeSession
from morla_runtime.native_ros import native_plan, make_native_adapters


def astral_system():
    instances = [{'instance_id': 'right', 'plugin_id': 'morla.astral.arm', 'kind': 'actuator', 'role': 'right',
                  'parameters': {'side': 'right'}, 'backend': 'real', 'deployment_verified': True,
                  'watchdog_verified': True, 'joints': [{'name': 'joint' + str(i), 'unit': 'rad'} for i in range(7)]},
                 {'instance_id': 'tool', 'plugin_id': 'morla.astral.gripper', 'kind': 'actuator', 'role': 'right',
                  'parameters': {'side': 'right'}, 'backend': 'real', 'deployment_verified': True,
                  'watchdog_verified': True, 'joints': [{'name': 'closure', 'unit': 'ratio'}]}]
    device = {'session_id': 'astral:board.01', 'session_family': 'astral_board', 'instances': ['right', 'tool'],
              'backend': 'real', 'resource_key': 'ip:192.168.1.2',
              'parameters': {'board_scope_verified': True, 'unselected_joints_policy': 'held'}}
    return {'instances': instances, 'device_sessions': [device]}


def test_astral_single_connection_scopes_and_no_auto_ready():
    system = astral_system()
    plan = native_plan(system['device_sessions'][0], system['instances'], '123')
    assert len(plan['argv']) == 1
    argv = plan['argv'][0]
    assert 'auto_ready:=false' in argv
    assert 'enable_full_body_cmd:=false' in argv
    assert 'enable_head_cmd:=false' in argv
    assert 'managed_arm_sides:=["right"]' in argv
    assert not any('bash' == value for value in argv)
    assert len(plan['ports']['right']['native_names']) == 7
    for public in ('/left_arm/joint_commands', '/right_arm/joint_commands', '/astral/joint_commands', '/head/joint_commands'):
        assert any(value.startswith(public + ':=/morla_sessions/') for value in argv)


class Transport:
    instances = []
    def __init__(self, plan, fd, root, clock):
        self.plan, self.clock, self.calls = plan, clock, []
        self.__class__.instances.append(self)
    def health(self, key): return {'ready': True, 'sample_ns': self.clock(), 'source': 'hardware', 'measured': True}
    def operation(self, key, op): self.calls.append((key, op)); return True
    def submit(self, key, cmd): self.calls.append((key, 'submit')); return {'accepted': True}
    def close(self): self.calls.append(('close',))


def test_native_shared_session_only_opens_once(tmp_path):
    Transport.instances.clear()
    system = astral_system()
    runtime = RuntimeSession(system, allow_unverified_test_fixture=True, resource_root=tmp_path)
    adapters = make_native_adapters(runtime, Transport)
    runtime.adapters.update(adapters)
    runtime.start()
    assert len(Transport.instances) == 1
    runtime.activate('enable')
    _wait(runtime, 'enable')
    calls = Transport.instances[0].calls
    assert sum(op == ('right', 'enable') or op == ('tool', 'enable') for op in calls) == 1
    runtime.stop()
    assert calls.count(('close',)) == 1


def test_unselected_board_scope_cannot_enable(tmp_path):
    system = astral_system()
    system['device_sessions'][0]['parameters'] = {}
    runtime = RuntimeSession(system, allow_unverified_test_fixture=True, resource_root=tmp_path)
    runtime.adapters.update(make_native_adapters(runtime, Transport))
    runtime.start()
    runtime.activate('enable')
    assert _wait(runtime, 'enable')['status'] == 'FAILED'
    assert runtime.status == 'FAULTED'
    runtime.stop()


def test_legacy_lane_conflicts_both_directions(tmp_path):
    legacy = LegacyDeviceGuard(tmp_path)
    manager = ResourceManager(tmp_path)
    with pytest.raises(RuntimeError): manager.acquire(['morla:active-system'])
    legacy.close()
    manager.acquire(['morla:active-system'])
    with pytest.raises(RuntimeError): LegacyDeviceGuard(tmp_path)
    manager.release()


def test_delegated_fd_owned_child_accepts_but_plain_child_rejects(tmp_path):
    manager = ResourceManager(tmp_path)
    manager.acquire(['morla:active-system'])
    env = os.environ.copy()
    env['MORLA_RESOURCE_ROOT'] = str(tmp_path)
    code = 'from morla_core.device_guard import LegacyDeviceGuard; g=LegacyDeviceGuard()'
    plain = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True)
    assert plain.returncode != 0
    env['MORLA_RUNTIME_LOCK_FD'] = str(manager.lane_fd)
    owned = subprocess.run([sys.executable, '-c', code], env=env, pass_fds=(manager.lane_fd,), capture_output=True)
    assert owned.returncode == 0, owned.stderr.decode()
    manager.release()


def test_failed_native_stop_is_retried_and_never_becomes_success(tmp_path):
    from morla_runtime.native_ros import NativeDeviceSession
    system = astral_system()
    runtime = RuntimeSession(system, resource_root=tmp_path, allow_unverified_test_fixture=True)
    session = NativeDeviceSession(system['device_sessions'][0], system['instances'], runtime, Transport)
    session.configure()
    session.enabled = True
    calls = []
    session.transport.operation = lambda key, op: calls.append(op) or False
    assert session.operation('right', 'stop') is False
    assert session.enabled
    assert session.operation('right', 'stop') is False
    assert calls == ['stop', 'stop']


def test_native_gripper_echo_units_source_and_clock_fencing():
    import threading
    from types import SimpleNamespace as NS
    from morla_runtime.native_ros import NativeRosTransport
    transport = object.__new__(NativeRosTransport)
    system = astral_system()
    transport.plan = native_plan(system['device_sessions'][0], system['instances'], '123')
    transport.lock = threading.Lock()
    transport.samples, transport.source_clocks, transport.source_faults = {}, {}, set()
    transport.clock = lambda: 1_000_000_000
    message = NS(name=['right_gripper'], position=[0.4], velocity=[], header=NS(stamp=NS(sec=10, nanosec=1)))
    transport._sample('tool', message)
    value = transport.health('tool')
    assert value['values'] == pytest.approx([0.5])
    assert value['source'] == 'command_echo'
    assert not value['measured'] and not value['ready']
    assert value['ports']['position']['unit'] == 'ratio'
    message.header.stamp.sec = 9
    transport._sample('tool', message)
    assert transport.health('tool')['reason'] == 'SOURCE_CLOCK_REGRESSION'
    message.header.stamp.sec = 11
    transport._sample('tool', message)
    assert not transport.health('tool')['ready']


def test_native_operations_do_not_block_state_and_cancel_late_enable(tmp_path):
    import threading
    import time
    from morla_runtime import FakeAdapter
    entered, release = threading.Event(), threading.Event()
    class Slow(FakeAdapter):
        backend = 'real'
        deployment_verified = True
        watchdog_verified = True
        def activate(self):
            entered.set()
            release.wait(timeout=2)
            return super().activate()
    system = astral_system()
    system['instances'] = [system['instances'][0]]
    system['device_sessions'][0]['instances'] = ['right']
    adapter = Slow(system['instances'][0])
    runtime = RuntimeSession(system, adapters={'right': adapter}, resource_root=tmp_path,
                             allow_unverified_test_fixture=True)
    runtime.start()
    result = runtime.activate('slow')
    assert result['status'] == 'RUNNING'
    assert entered.wait(timeout=.5)
    began = time.monotonic()
    assert runtime.state()['status'] == 'READY_DISARMED'
    assert time.monotonic() - began < .05
    runtime.estop()
    release.set()
    for _ in range(100):
        if runtime.operation('slow')['status'] != 'RUNNING': break
        time.sleep(.005)
    assert runtime.status == 'ESTOP_LATCHED'
    assert runtime.operation('slow')['status'] == 'CANCELLED'
    assert not adapter.active
    runtime.stop()


def _wait(runtime, request_id):
    import time
    for _ in range(200):
        result = runtime.operation(request_id)
        if result['status'] != 'RUNNING': return result
        time.sleep(.005)
    raise AssertionError('operation did not settle')


@pytest.mark.parametrize('message,status,accepted', [
    ('queued', {'estop_status': 'sent', 'estop_seq': 7}, False),
    ('queued [morla_seq=8]', {'estop_status': 'sent', 'estop_seq': 7}, False),
    ('queued [morla_seq=8]', {'estop_status': 'sent', 'estop_seq': 8}, True),
    ('queued [morla_seq=8]', {'estop_status': 'queued', 'estop_seq': 8}, False),
])
def test_native_nero_ack_requires_exact_request_correlation(message, status, accepted):
    import threading
    from types import SimpleNamespace as NS
    from morla_runtime.native_ros import NativeRosTransport
    transport = object.__new__(NativeRosTransport)
    transport.plan = {'ports': {'n': {'plugin_id': 'morla.nero.arm'}}}
    transport.clock = lambda: 100
    transport.lock, transport.flags = threading.Lock(), {}
    transport.statuses = {'n': (100, status)}
    future = NS(done=lambda: True, result=lambda: NS(success=True, message=message))
    client = NS(wait_for_service=lambda **kw: True, srv_type=NS(Request=lambda: NS()), call_async=lambda req: future)
    transport.clients = {('n', 'stop'): client}
    if accepted:
        assert transport.operation('n', 'stop', timeout=.01)
    else:
        with pytest.raises((RuntimeError, TimeoutError)):
            transport.operation('n', 'stop', timeout=.01)
