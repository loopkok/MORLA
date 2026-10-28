from types import SimpleNamespace
from morla_runtime import RuntimeSession, FakeAdapter
from morla_runtime.plugins import load_installed_adapter
import pytest


def test_only_selected_installed_adapter_loads_after_resource_lock(monkeypatch, tmp_path):
    imported = []
    class Entries(list):
        def select(self, **kwargs): return self
    def factory(instance, context):
        assert context['lane_fd'] is not None
        assert context['device_session']['session_id'] == 'device'
        assert context['shared_device_sessions'] == {}
        return FakeAdapter(instance)
    chosen = SimpleNamespace(name='thirdparty.three_axis:fake', load=lambda: imported.append('chosen') or factory)
    ignored = SimpleNamespace(name='unselected.dangerous:real', load=lambda: (_ for _ in ()).throw(AssertionError('unexpected import')))
    monkeypatch.setattr('morla_runtime.plugins.metadata.entry_points', lambda: Entries([chosen, ignored]))
    resolved = {'instances': [{'instance_id': 'arm', 'plugin_id': 'thirdparty.three_axis', 'kind': 'actuator',
                              'backend': 'fake', 'joints': [{'name': 'j' + str(i), 'unit': 'rad'} for i in range(3)]}],
                'device_sessions': [{'session_id': 'device', 'resource_key': 'usb:custom', 'instances': ['arm']}]}
    runtime = RuntimeSession(resolved, resource_root=tmp_path, allow_unverified_test_fixture=True)
    assert imported == []
    runtime.start()
    assert imported == ['chosen']
    assert len(runtime.adapters['arm'].values) == 3
    runtime.stop()


def test_ambiguous_selected_adapter_rejected_without_import(monkeypatch):
    class Entries(list):
        def select(self, **kwargs): return self
    entry = SimpleNamespace(name='custom:fake', load=lambda: (_ for _ in ()).throw(AssertionError('must not import')))
    monkeypatch.setattr('morla_runtime.plugins.metadata.entry_points', lambda: Entries([entry, entry]))
    with pytest.raises(ValueError, match='AMBIGUOUS_ADAPTER'):
        load_installed_adapter({'plugin_id': 'custom', 'backend': 'fake'}, {})


def test_external_real_preflight_is_metadata_only(monkeypatch):
    entry = SimpleNamespace(name='external.direct:real',
                            load=lambda: (_ for _ in ()).throw(AssertionError('preflight imported device code')))
    monkeypatch.setattr('importlib.metadata.entry_points', lambda **kwargs: [entry])
    runtime = RuntimeSession({'instances': [{'instance_id': 'arm', 'plugin_id': 'external.direct',
                              'backend': 'real', 'kind': 'actuator'}]},
                              allow_unverified_test_fixture=True)
    result = runtime.preflight()
    assert result['ready']
    assert result['components'][0]['probe']=='installed_adapter_metadata_only_dependencies_unchecked'
