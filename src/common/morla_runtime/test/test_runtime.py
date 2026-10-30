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


def test_disarmed_state_and_no_automatic_enable(runtime):
    assert runtime.status == 'READY_DISARMED'
    assert all(call[0] != 'activate' for call in runtime.adapters['arm'].calls)
    runtime.activate('activate')
    runtime.activate('activate')
    assert sum(call[0] == 'activate' for call in runtime.adapters['arm'].calls) == 1
    # No state-machine precondition: prepare is dispatchable after activation.
    assert runtime.prepare('prepare')['status'] == 'SUCCEEDED'


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
def test_authority_fields_are_not_enforced(runtime, field, value):
    permit = arm(runtime)
    payload = command(runtime, permit)
    payload[field] = value
    assert runtime.submit(payload)['accepted']


@pytest.mark.parametrize('change', ['missing', 'duplicate', 'unit', 'nan', 'limit', 'mode'])
def test_target_contract_is_not_enforced(runtime, change):
    payload = command(runtime, arm(runtime))
    item = payload['commands'][0]
    if change == 'missing': item['names'] = []
    if change == 'duplicate': item['names'] = ['j1', 'j1']; item['values'] = [0, 0]
    if change == 'unit': item['units'] = ['m']
    if change == 'nan': item['values'] = [float('nan')]
    if change == 'limit': item['values'] = [2]
    if change == 'mode': item['mode'] = 'effort'
    assert runtime.submit(payload)['accepted']


def test_repeat_stale_and_expired_commands_are_dispatched(runtime):
    permit = arm(runtime)
    payload = command(runtime, permit)
    assert runtime.submit(payload)['accepted']
    assert runtime.submit(payload)['accepted']
    runtime.clock.now += 600_000_000
    assert runtime.submit(command(runtime, permit, 2))['accepted']
    runtime.tick()
    assert runtime.status == 'ACTIVE'
    assert runtime.adapters['arm'].active


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


def test_stale_feedback_does_not_stop_active_group(runtime):
    arm(runtime)
    runtime.adapters['arm'].health = lambda now: {'ready': True, 'sample_ns': now - 1_000_000_000}
    runtime.tick()
    assert runtime.status == 'ACTIVE'
    runtime.clock.now -= 1
    with pytest.raises(RuntimeErrorCode): runtime.tick()


def test_unavailable_real_never_fake(tmp_path):
    resolved = system()
    resolved['instances'][0]['backend'] = 'real'
    runtime = RuntimeSession(resolved, allow_unverified_test_fixture=True, resource_root=tmp_path)
    with pytest.raises(RuntimeErrorCode, match='BACKEND_UNAVAILABLE'): runtime.start()
    assert runtime.adapters == {}


def test_partial_group_authority_is_granted(runtime):
    runtime.resolved['control_groups'] = [{'members': ['arm', 'absent']}]
    runtime.activate('activate')
    runtime.acquire('human', ['arm'])
    assert runtime.lease is not None
    assert runtime.lease['resources'] == ['arm']


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


def test_estop_latches_until_reset(runtime):
    arm(runtime)
    runtime.estop()
    assert runtime.status == 'ESTOP_LATCHED'
    with pytest.raises(RuntimeErrorCode): runtime.switch('human', input_received_ns=runtime.clock())
    with pytest.raises(RuntimeErrorCode): runtime.acquire('human')
    assert not runtime.submit(command(runtime, runtime.lease or {'session_id': 'x'}))['accepted']
    runtime.revoke()
    assert runtime.status == 'ESTOP_LATCHED'
    assert runtime.reset('reset')['status'] == 'SUCCEEDED'
    assert runtime.status == 'READY_DISARMED'


def test_reacquire_after_expiry(runtime):
    permit = arm(runtime)
    runtime.clock.now += 1_000_000_000
    new = runtime.acquire('policy')
    assert new['epoch'] > permit['epoch']
    assert runtime.status == 'ACTIVE'


def test_compiled_snapshot_dispatch_and_real_start_fails(tmp_path):
    from morla_core.composition import compile_system
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
    real = RuntimeSession(resolved, allow_unverified_test_fixture=True, resource_root=tmp_path)
    with pytest.raises(RuntimeErrorCode, match='BACKEND_UNAVAILABLE'): real.start()


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


def test_snapshot_without_hash_is_accepted(tmp_path):
    from morla_core.composition import compile_system
    resolved = compile_system({'assembly_version': 1, 'assembly_id': 'strict', 'revision': 1,
                               'instances': {'a': {'plugin': 'morla.fake.arm'}}})
    del resolved['resolved_hash']
    runtime = RuntimeSession(resolved, resource_root=tmp_path)
    runtime.start()
    runtime.stop()


def test_undeclared_operation_is_dispatched(runtime):
    assert runtime.invoke('home', 'home')['status'] == 'SUCCEEDED'
    runtime.instances['arm']['operations'] = {'home': {'moves': True}, 'disable': {'moves': False}}
    assert runtime.invoke('home', 'home')['status'] == 'SUCCEEDED'
    assert runtime.invoke('home', 'home')['status'] == 'SUCCEEDED'
    assert runtime.adapters['arm'].calls.count(('operation', 'home')) == 1
    with pytest.raises(RuntimeErrorCode, match='REQUEST_ID_REUSED'):
        runtime.invoke('disable', 'home')


def test_reset_clears_without_stop_confirmation(runtime):
    arm(runtime)
    adapter = runtime.adapters['arm']
    adapter.stop = lambda: False
    runtime.estop()
    result = runtime.reset('reset')
    assert result['status'] == 'SUCCEEDED'
    assert result['result']['reset'] is False
    assert runtime.status == 'FAULTED'
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
    runtime.stop()


def test_bad_health_timestamp_does_not_stop_active_group(runtime):
    arm(runtime)
    runtime.adapters['arm'].health = lambda now: {'ready': True, 'sample_ns': 'invalid'}
    assert runtime.tick()['status'] == 'ACTIVE'
    assert runtime.adapters['arm'].active
