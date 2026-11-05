import copy
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
from morla_core.composition import compile_system, load_document, verify_resolved
from morla_core.plugins.registry import Catalog, default_catalog
from morla_runtime import RuntimeSession
from morla_fr3_driver.adapter import FR3Adapter, FR3FakeAdapter
from morla_fr3_driver.contract import INITIAL, LOWER, UPPER, validate_parameters, validate_target
from morla_fr3_driver.kinematics import FR3Kinematics
from morla_fr3_driver.sdk_backend import SDKBackend, ack, checked_cnde_class
from morla_fr3_driver.transport import FR3Transport

ROOT = Path(__file__).parents[1]
DESCRIPTOR = json.loads((ROOT / 'descriptor.json').read_text())


def parameters():
    return {**copy.deepcopy(DESCRIPTOR['defaults']), 'host': '127.0.0.1'}


def system(filename='assembly.fake.yaml'):
    catalog = default_catalog()
    if not any(d['plugin_id'] == DESCRIPTOR['plugin_id'] for d in catalog.list()):
        catalog.register(DESCRIPTOR)
    resolved = compile_system(load_document(ROOT / 'config' / filename), registry=catalog)
    return catalog, resolved


def instance():
    return next(i for i in system()[1]['instances'] if i['instance_id'] == 'arm')


def target(i, values=None):
    return {'component_id': i['instance_id'], 'names': [j['name'] for j in i['joints']],
            'values': list(INITIAL) if values is None else values, 'units': ['rad'] * 6, 'mode': 'position'}


def wait_until(predicate, seconds=1):
    deadline = time.monotonic() + seconds
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('timed out')
        time.sleep(.002)


def feedback(values=None, enabled=0):
    return {'values': list(INITIAL) if values is None else values,
            'sample_ns': time.monotonic_ns(), 'enabled': enabled, 'main_code': 0,
            'sub_code': 0, 'emergency_stop': 0, 'safety_stop0': 0, 'safety_stop1': 0,
            'collision': 0, 'frame_count': 1, 'sequence': 1, 'robot_mode': 0}


class MemoryTransport:
    def __init__(self, params):
        self.values = list(INITIAL)
        self.enabled = 0
        self.writes = []
        self.stops = 0
        self.closed = False
        self.stale = False
        self.error = False

    def read(self):
        sample = feedback(self.values[:], self.enabled)
        if self.stale:
            sample['sample_ns'] = 1
        if self.error:
            sample['emergency_stop'] = 1
        return sample

    def activate(self):
        self.enabled = 1

    def write(self, q):
        self.writes.append(q[:])
        self.values = q[:]

    def stop(self):
        self.enabled = 0
        self.stops += 1

    def close(self):
        self.closed = True


def configured():
    i = instance()
    i['backend'] = 'real'
    i['parameters']['host'] = '127.0.0.1'
    adapter = FR3Adapter(i, {'clock': time.monotonic_ns}, MemoryTransport)
    adapter.configure({})
    wait_until(lambda: adapter.health(time.monotonic_ns())['ready'])
    return adapter, i


def test_catalog_compile_fake_and_independent_runtime(tmp_path):
    catalog, resolved = system()
    assert verify_resolved(resolved, registry=catalog)
    assert resolved['feature_contract']['state_dim'] == resolved['feature_contract']['action_dim'] == 6
    i = instance()
    a = FR3FakeAdapter(i)
    runtime = RuntimeSession(resolved, registry=catalog, resource_root=tmp_path,
                             adapters={'arm': a})
    try:
        assert runtime.start()['status'] == 'READY_DISARMED'
        assert not a.active
        runtime.prepare('prepare')
        runtime.activate('activate')
        permit = runtime.acquire('test')
        values = list(INITIAL)
        values[0] += .02
        receipt = runtime.submit({**permit, 'sequence': 1, 'issued_ns': runtime.clock(),
                                  'ttl_ns': 100_000_000, 'commands': [target(i, values)]})
        assert receipt['accepted']
        assert a.values == values
        sample = a.health(runtime.clock())
        assert sample['source'] == 'fake' and sample['synthetic'] and not sample['measured']
        runtime.revoke('test')
        assert not a.active
    finally:
        runtime.stop()
    assert not runtime.resources.handles


@pytest.mark.parametrize('mutation', [
    {'values': [0] * 5}, {'values': [float('nan')] * 6}, {'values': [float('inf')] * 6},
    {'values': [True] * 6}, {'values': ['0'] * 6}, {'values': [9.] * 6},
    {'mode': 'velocity'}, {'units': ['deg'] * 6}, {'names': ['joint1'] * 6},
    {'names': ['joint6', 'joint5', 'joint4', 'joint3', 'joint2', 'joint1']},
    {'component_id': 'other'},
])
def test_command_contract_rejects_invalid_and_wrong_order(mutation):
    i = instance()
    with pytest.raises(ValueError):
        validate_target({**target(i), **mutation}, i)


@pytest.mark.parametrize('mutation', [
    {'rate_hz': 0}, {'rate_hz': 500}, {'rate_hz': True}, {'rpc_timeout_s': float('nan')},
    {'joint_lower': [-9.] * 6}, {'joint_upper': [9.] * 6}, {'host': 'REPLACE_CONTROLLER_IP'},
])
def test_invalid_deployment_fails_before_connect(mutation):
    with pytest.raises(ValueError):
        validate_parameters({**parameters(), **mutation}, real=True)


def test_real_adapter_no_startup_motion_velocity_hold_and_reactivation():
    a, i = configured()
    try:
        assert not a.transport.writes and a.transport.enabled == 0
        a.prepare()
        assert a.transport.enabled == 0
        with pytest.raises(RuntimeError, match='DISARMED'):
            a.submit(target(i))
        a.activate()
        q = list(INITIAL)
        q[0] += .03
        a.submit(target(i, q))
        wait_until(lambda: len(a.transport.writes) >= 4)
        writes = [list(INITIAL)] + a.transport.writes[:]
        assert all(max(abs(x-y) for x, y in zip(left, right)) <= .3 / 125 + 1e-12
                   for left, right in zip(writes, writes[1:]))
        a.release()
        hold = a.last_sent[:]
        count = len(a.transport.writes)
        wait_until(lambda: len(a.transport.writes) > count + 2)
        assert all(row == hold for row in a.transport.writes[count:])
        with pytest.raises(RuntimeError, match='DISARMED'):
            a.submit(target(i, q))
        a.activate()
        a.stop()
        count = len(a.transport.writes)
        time.sleep(.04)
        assert len(a.transport.writes) == count
        assert a.pending is None and a.desired is None
    finally:
        a.close()
    assert a.transport.closed


@pytest.mark.parametrize('fault', ['stale', 'error'])
def test_fault_stops_even_when_softly_released_and_invalidates_port(fault):
    a, i = configured()
    try:
        a.activate()
        a.submit(target(i))
        wait_until(lambda: a.transport.writes)
        a.release()
        setattr(a.transport, fault, True)
        wait_until(lambda: a.health(time.monotonic_ns()).get('fault'))
        wait_until(lambda: a.transport.stops > 0)
        state = a.health(time.monotonic_ns())
        assert not state['ready'] and not state['ports']['position']['valid']
        assert not a.active and a.pending is None and a.desired is None
        with pytest.raises(RuntimeError):
            a.activate()
    finally:
        a.close()


def test_close_after_release_stops_and_far_target_is_rejected():
    a, i = configured()
    a.activate()
    q = list(INITIAL)
    q[0] = .5
    with pytest.raises(ValueError, match='too far'):
        a.submit(target(i, q))
    a.release()
    a.close()
    assert a.transport.stops and a.transport.closed


def test_fk_matches_manual_zero_pose_and_jacobian_finite_difference():
    solver = FR3Kinematics()
    np.testing.assert_allclose(solver.fk([0.] * 6)[:3, 3], [-.52001, -.20199925066, .037999258013], atol=1e-10)
    q = np.array(INITIAL)
    jac = solver.jacobian(q)
    from scipy.spatial.transform import Rotation
    base = solver.fk(q)
    for i in range(6):
        v = q.copy()
        v[i] += 1e-6
        moved = solver.fk(v)
        finite = np.r_[(moved[:3, 3] - base[:3, 3]) / 1e-6,
                       Rotation.from_matrix(moved[:3, :3] @ base[:3, :3].T).as_rotvec() / 1e-6]
        np.testing.assert_allclose(jac[:, i], finite, atol=1e-6)


def test_ik_small_motion_and_unreachable_hold():
    solver = FR3Kinematics()
    q = np.array(INITIAL)
    solver.sync_state(q)
    goal = solver.fk(q + [.01, -.01, .01, .01, -.01, .01])
    result = solver.solve(goal)
    assert result is not None
    np.testing.assert_allclose(solver.fk(result), goal, atol=.002)
    goal[0, 3] = 10
    assert solver.solve(goal) is None


def blocked_worker(pipe, params):
    pipe.send((True, None))
    pipe.recv()
    time.sleep(60)


def test_sdk_hang_is_bounded_process_terminated_no_late_reuse():
    transport = FR3Transport({**parameters(), 'rpc_timeout_s': .05}, worker=blocked_worker)
    start = time.monotonic()
    with pytest.raises(TimeoutError, match='physical stop UNCONFIRMED'):
        transport.read()
    assert time.monotonic() - start < 2
    assert not transport.process.is_alive()
    with pytest.raises(RuntimeError, match='new session'):
        transport.write([0.] * 6)
    transport.close()


def vendor_module(monkeypatch):
    # Only the optional TLS import is stubbed. Use the actual pinned SDK parser;
    # these tests make no socket connections and do not claim TLS validation.
    if 'morla_fr3_driver.vendor.Robot' not in sys.modules:
        tls = ModuleType('mbedtls.tls')
        for name in ('WantReadError', 'WantWriteError', 'ClientContext', 'DTLSConfiguration', 'TrustStore'):
            setattr(tls, name, type(name, (Exception,), {}))
        monkeypatch.setitem(sys.modules, 'mbedtls', ModuleType('mbedtls'))
        monkeypatch.setitem(sys.modules, 'mbedtls.tls', tls)
    from morla_fr3_driver.vendor import Robot
    return Robot


def test_actual_vendor_cnde_decoder_freshness_units_wrap_and_short_packet(monkeypatch):
    module = vendor_module(monkeypatch)
    client = checked_cnde_class(module)(module.RobotStatePkg(), [0])
    client._config_states = [getattr(module.RobotState, key) for key in (
        'FrameCnt', 'JointCurPos', 'RbtEnableState', 'MainCode', 'SubCode', 'EmergencyStop',
        'SafetyStop0State', 'SafetyStop1State', 'CollisionState', 'RobotMode')]
    def packet(count):
        return struct.pack('<B6diii5B', count, 0, 90, -90, 180, -180, 45, 1, 0, 0, 0, 0, 0, 0, 0)
    client._parse_cnde_state_data(packet(255))
    assert client.morla_error is None
    sample = client.morla_sample
    assert sample['values'] == pytest.approx([0, math.pi/2, -math.pi/2, math.pi, -math.pi, math.pi/4])
    client._parse_cnde_state_data(packet(255))
    assert client.morla_sample['sample_ns'] == sample['sample_ns']
    client._parse_cnde_state_data(packet(0))
    assert client.morla_sample['sequence'] == 2
    client._parse_cnde_state_data(packet(1)[:-1])
    assert 'length' in client.morla_error
    assert client.morla_sample['sequence'] == 2


class SDKDouble:
    def __init__(self):
        self.calls = []
        self.enabled = 0

    def RobotEnable(self, state):
        self.calls.append(('RobotEnable', state))
        self.enabled = state
        return 0

    def ServoMoveStart(self): self.calls.append(('ServoMoveStart',)); return 0
    def ServoMoveEnd(self): self.calls.append(('ServoMoveEnd',)); return 0
    def StopMotion(self): self.calls.append(('StopMotion',)); return 0
    def CloseRPC(self): self.calls.append(('CloseRPC',))
    def ServoJ(self, q, axis, **kwargs): self.calls.append(('ServoJ', q, axis, kwargs)); return 0


def backend_double():
    b = object.__new__(SDKBackend)
    b.parameters = parameters()
    b.sdk = SDKDouble()
    b.owned_enable = b.servo_started = False
    b.read = lambda: feedback(enabled=b.sdk.enabled)
    return b


def test_sdk_activation_ack_state_servoj_units_and_stop_order():
    b = backend_double()
    b.activate()
    b.write([0, math.pi/2, -math.pi/2, 0, 0, 0])
    call = b.sdk.calls[-1]
    assert call[1] == pytest.approx([0, 90, -90, 0, 0, 0])
    assert call[2] == [0.] * 4 and call[3] == {'cmdT': .008}
    b.stop()
    assert b.sdk.calls[-3:] == [('StopMotion',), ('ServoMoveEnd',), ('RobotEnable', 0)]
    assert not b.owned_enable
    b.close()
    assert b.sdk.calls[-1] == ('CloseRPC',)


def test_never_enabled_close_is_read_only_and_bad_ack_fails():
    b = backend_double()
    b.close()
    assert b.sdk.calls == [('CloseRPC',)]
    for value in [-1, None, True, [0], '0']:
        with pytest.raises(RuntimeError):
            ack('ServoJ', value)


def test_stop_attempts_disable_after_rejected_stopmotion():
    b = backend_double()
    b.activate()
    b.sdk.StopMotion = lambda: -1
    with pytest.raises(RuntimeError, match='stop unconfirmed'):
        b.stop()
    assert b.sdk.calls[-1] == ('RobotEnable', 0)


def test_activate_requires_new_enabled_feedback_and_cleans_partial_enable():
    b = backend_double()
    b.parameters['operation_timeout_s'] = .04
    b.read = lambda: feedback(enabled=0)
    with pytest.raises(TimeoutError):
        b.activate()
    assert ('ServoMoveStart',) not in b.sdk.calls
    assert ('RobotEnable', 0) in b.sdk.calls


def test_sdk_source_is_unmodified():
    import hashlib
    provenance = json.loads((ROOT / 'PROVENANCE.json').read_text())
    assert hashlib.sha256((ROOT / 'morla_fr3_driver/vendor/Robot.py').read_bytes()).hexdigest() == provenance['sha256']
