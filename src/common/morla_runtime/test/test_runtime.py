import copy
import time
import pytest
from morla_runtime import RuntimeSession, RuntimeErrorCode, ResourceBusy, FakeAdapter, LatestMailbox


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now


def system(two=False):
    ids = ['arm', 'second'] if two else ['arm']
    return {'instances': [{'instance_id': key, 'plugin_id': 'morla.fake.arm', 'kind': 'actuator',
                          'backend': 'fake', 'joints': [{'name': 'j1', 'unit': 'rad', 'lower': -1, 'upper': 1}]} for key in ids],
            'device_sessions': [{'session_id': 'board', 'resource_key': 'device:board', 'instances': ids}],
            'control_groups': []}


@pytest.fixture
def runtime(tmp_path):
    value = RuntimeSession(system(), allow_unverified_test_fixture=True, resource_root=tmp_path, clock=Clock())
    value.start()
    yield value
    value.stop()


def arm(runtime):
    assert runtime.activate('activate')['status'] == 'SUCCEEDED'
    return runtime.acquire('human')


def command(runtime, permit, seq=1, ids=('arm',)):
    return {**permit, 'issued_ns': runtime.clock(), 'ttl_ns': 100_000_000, 'sequence': seq,
            'commands': [{'component_id': key, 'names': ['j1'], 'values': [0.2], 'units': ['rad'], 'mode': 'position'} for key in ids]}


def test_disarmed_idempotence_and_no_automatic_enable(runtime):
    assert runtime.status == 'READY_DISARMED'
    assert all(call[0] != 'activate' for call in runtime.adapters['arm'].calls)
    with pytest.raises(RuntimeErrorCode): runtime.acquire('human')
    runtime.activate('activate')
    runtime.activate('activate')
    assert sum(call[0] == 'activate' for call in runtime.adapters['arm'].calls) == 1
    with pytest.raises(RuntimeErrorCode): runtime.prepare('activate')


def test_host_lock_and_cleanup(tmp_path):
    first, second = RuntimeSession(system(), allow_unverified_test_fixture=True, resource_root=tmp_path), RuntimeSession(system(), allow_unverified_test_fixture=True, resource_root=tmp_path)
    first.start()
    with pytest.raises(ResourceBusy): second.start()
    first.stop()
    third = RuntimeSession(system(), allow_unverified_test_fixture=True, resource_root=tmp_path)
    third.start()
    third.stop()


@pytest.mark.parametrize('field,value', [('session_id', 'old'), ('epoch', -1), ('token', 'copied'), ('producer', 'other'),
                                        ('issued_ns', 99_000_000_000), ('ttl_ns', -1), ('sequence', True)])
def test_fencing(runtime, field, value):
    permit = arm(runtime)
    payload = command(runtime, permit)
    payload[field] = value
    assert not runtime.submit(payload)['accepted']
    assert not any(call[0] == 'submit' for call in runtime.adapters['arm'].calls)


@pytest.mark.parametrize('change', ['missing', 'duplicate', 'unit', 'nan', 'limit', 'mode'])
def test_target_contract(runtime, change):
    payload = command(runtime, arm(runtime))
    item = payload['commands'][0]
    if change == 'missing': item['names'] = []
    if change == 'duplicate': item['names'] = ['j1', 'j1']; item['values'] = [0, 0]
    if change == 'unit': item['units'] = ['m']
    if change == 'nan': item['values'] = [float('nan')]
    if change == 'limit': item['values'] = [2]
    if change == 'mode': item['mode'] = 'effort'
    assert not runtime.submit(payload)['accepted']


def test_sequence_ttl_and_reconnect(runtime):
    permit = arm(runtime)
    payload = command(runtime, permit)
    assert runtime.submit(payload)['accepted']
    assert not runtime.submit(payload)['accepted']
    runtime.clock.now += 600_000_000
    assert not runtime.submit(command(runtime, permit, 2))['accepted']
    runtime.tick()
    assert runtime.status == 'READY_DISARMED'
    assert not runtime.adapters['arm'].active
    runtime.activate('again')
    new = runtime.acquire('human')
    assert new['epoch'] > permit['epoch']
    assert not runtime.submit(command(runtime, permit, 3))['accepted']


def test_partial_dispatch_stops_all(tmp_path):
    resolved = system(True)
    class Failing(FakeAdapter):
        def submit(self, payload): raise RuntimeError('sdk write failed')
    adapters = {'arm': FakeAdapter(resolved['instances'][0]), 'second': Failing(resolved['instances'][1])}
    runtime = RuntimeSession(resolved, allow_unverified_test_fixture=True, resource_root=tmp_path, adapters=adapters)
    runtime.start()
    permit = arm(runtime)
    result = runtime.submit(command(runtime, permit, ids=('arm', 'second')))
    assert result['dispatched'] == ['arm']
    assert runtime.status == 'FAULTED'
    assert runtime.lease is None
    assert not any(adapter.active for adapter in adapters.values())
    runtime.stop()


def test_switch_missing_ack_does_not_grant(runtime):
    permit = arm(runtime)
    runtime.adapters['arm'].stop = lambda: False
    with pytest.raises(RuntimeErrorCode): runtime.switch('policy', input_received_ns=runtime.clock())
    assert runtime.lease is None
    assert runtime.status == 'FAULTED'
    assert not runtime.submit(command(runtime, permit))['accepted']


def test_stale_feedback_and_clock_regression(runtime):
    arm(runtime)
    runtime.adapters['arm'].health = lambda now: {'ready': True, 'sample_ns': now - 1_000_000_000}
    runtime.tick()
    assert runtime.status == 'FAULTED'
    runtime.clock.now -= 1
    with pytest.raises(RuntimeErrorCode): runtime.tick()
    assert runtime.status == 'FAULTED'


def test_unavailable_real_never_fake(tmp_path):
    resolved = system()
    resolved['instances'][0]['backend'] = 'real'
    runtime = RuntimeSession(resolved, allow_unverified_test_fixture=True, resource_root=tmp_path)
    with pytest.raises(RuntimeErrorCode, match='BACKEND_UNAVAILABLE'): runtime.start()
    assert runtime.adapters == {}


def test_group_grant_atomic(runtime):
    runtime.resolved['control_groups'] = [{'members': ['arm', 'absent']}]
    runtime.activate('activate')
    with pytest.raises(RuntimeErrorCode, match='PARTIAL_GROUP'): runtime.acquire('human', ['arm'])
    assert runtime.lease is None


def test_mailbox_bounded():
    mailbox = LatestMailbox()
    for i in range(10000): mailbox.put(i)
    assert mailbox.take() == 9999
    assert mailbox.dropped == 9999
    assert mailbox.take() is None


def test_callback_capture_and_revoke(runtime):
    captured = []
    runtime.on_command(lambda requested, receipt: captured.append((requested, receipt)))
    permit = arm(runtime)
    assert runtime.submit(command(runtime, permit))['accepted']
    assert len(captured) == 1
    runtime.revoke('human')
    assert runtime.status == 'READY_DISARMED'
    assert runtime.lease is None


def test_fault_cannot_be_cleared_by_switch_or_revoke(runtime):
    arm(runtime)
    runtime.estop()
    assert runtime.status == 'ESTOP_LATCHED'
    with pytest.raises(RuntimeErrorCode): runtime.switch('human', input_received_ns=runtime.clock())
    with pytest.raises(RuntimeErrorCode): runtime.revoke()
    assert runtime.status == 'ESTOP_LATCHED'
    assert runtime.reset('reset')['status'] == 'SUCCEEDED'
    assert runtime.status == 'READY_DISARMED'


def test_expired_lease_requires_rearm_even_without_tick(runtime):
    arm(runtime)
    runtime.clock.now += 1_000_000_000
    with pytest.raises(RuntimeErrorCode): runtime.acquire('policy')
    assert runtime.status == 'READY_DISARMED'
    assert runtime.lease is None


def test_actual_compiler_contract_dispatch_and_tamper(tmp_path):
    from morla_core.composition import compile_system, CompilationError
    resolved = compile_system({'assembly_version': 1, 'assembly_id': 'runtime_test', 'revision': 1,
                               'instances': {'a': {'plugin': 'morla.nero.arm', 'parameters': {'side': 'left'}}}})
    runtime = RuntimeSession(resolved, allow_unverified_test_fixture=True, resource_root=tmp_path)
    runtime.start()
    permit = arm(runtime)
    row = resolved['instances'][0]
    payload = {**permit, 'sequence': 1, 'issued_ns': runtime.clock(), 'ttl_ns': 100_000_000,
               'commands': [{'component_id': 'a', 'names': [j['name'] for j in row['joints']],
                             'values': [max(j['lower'], min(0, j['upper'])) for j in row['joints']],
                             'units': [j['unit'] for j in row['joints']], 'mode': 'position'}]}
    result = runtime.submit(payload)
    assert result['accepted'], result
    runtime.stop()
    resolved['instances'][0]['backend'] = 'real'
    with pytest.raises(CompilationError): RuntimeSession(resolved, allow_unverified_test_fixture=True, resource_root=tmp_path)


def test_fake_samples_dimensions_and_image_provenance(tmp_path):
    from morla_core.composition import compile_system
    resolved = compile_system({'assembly_version': 1, 'assembly_id': 'sample_test', 'revision': 1,
                               'instances': {'sensor': {'plugin': 'morla.fake.sensor'}, 'cam': {'plugin': 'morla.camera'}}})
    runtime = RuntimeSession(resolved, allow_unverified_test_fixture=True, resource_root=tmp_path)
    runtime.start()
    assert len(runtime.adapters['sensor'].read(runtime.clock())['ports']['values']['values']) == 3
    image = runtime.adapters['cam'].read(runtime.clock())['ports']['rgb']
    assert image['encoding'] == 'jpeg'
    assert image['data'].startswith(b'\xff\xd8')
    assert image['synthetic'] and not image['measured']
    runtime.stop()


def test_stripping_compiled_hash_does_not_bypass_validation(tmp_path):
    from morla_core.composition import compile_system
    resolved = compile_system({'assembly_version': 1, 'assembly_id': 'strict', 'revision': 1,
                               'instances': {'a': {'plugin': 'morla.fake.arm'}}})
    del resolved['resolved_hash']
    with pytest.raises(RuntimeErrorCode, match='RESOLVED_HASH_REQUIRED'):
        RuntimeSession(resolved, resource_root=tmp_path)


def test_operation_declarations_and_idempotent_scope(runtime):
    with pytest.raises(RuntimeErrorCode, match='UNSUPPORTED_OPERATION'):
        runtime.invoke('home', 'home')
    runtime.instances['arm']['operations'] = {'home': {'moves': True}, 'disable': {'moves': False}}
    assert runtime.invoke('home', 'home')['status'] == 'SUCCEEDED'
    assert runtime.invoke('home', 'home')['status'] == 'SUCCEEDED'
    assert runtime.adapters['arm'].calls.count(('operation', 'home')) == 1
    with pytest.raises(RuntimeErrorCode, match='REQUEST_ID_REUSED'):
        runtime.invoke('disable', 'home')


def test_reset_requires_confirmed_stop_before_clearing_latch(runtime):
    arm(runtime)
    adapter = runtime.adapters['arm']
    adapter.stop = lambda: False
    runtime.estop()
    assert runtime.reset('reset')['status'] == 'FAILED'
    assert runtime.status == 'ESTOP_LATCHED'
    assert runtime.estop_latched
    adapter.stop = lambda: True
    assert runtime.reset('retry-reset')['status'] == 'SUCCEEDED'
    assert not runtime.estop_latched


def test_sensor_only_session_lifecycle_never_needs_fake_motor(tmp_path):
    from morla_core.composition import compile_system
    system = compile_system({'assembly_version': 1, 'assembly_id': 'sensor_only', 'revision': 1,
                             'instances': {'sensor': {'plugin': 'morla.fake.sensor'}}})
    runtime = RuntimeSession(system, resource_root=tmp_path)
    runtime.start()
    assert runtime.prepare('prepare')['status'] == 'SUCCEEDED'
    assert runtime.activate('activate')['status'] == 'SUCCEEDED'
    assert runtime.status == 'ACTIVE'
    with pytest.raises(RuntimeErrorCode): runtime.acquire('policy')
    runtime.stop()


def test_health_exception_or_bad_timestamp_stops_active_group(runtime):
    arm(runtime)
    runtime.adapters['arm'].health = lambda now: {'ready': True, 'sample_ns': 'invalid'}
    assert runtime.tick()['status'] == 'FAULTED'
    assert runtime.lease is None
    assert not runtime.adapters['arm'].active
