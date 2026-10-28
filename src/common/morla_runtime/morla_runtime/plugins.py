"""Selected installed adapter loading; no assembly-controlled import strings."""
from importlib import metadata


def load_installed_adapter(instance, context):
    entries = metadata.entry_points()
    entries = list(entries.select(group='morla.adapters')) if hasattr(entries, 'select') else list(entries.get('morla.adapters', []))
    identity = instance['plugin_id'] + ':' + instance.get('backend', 'fake')
    matches = [entry for entry in entries if entry.name == identity]
    if len(matches) > 1:
        raise ValueError('AMBIGUOUS_ADAPTER_ENTRY_POINT: ' + identity)
    if not matches:
        return None
    # Only the chosen installed entry point is imported, after the runtime owns
    # the host/device locks. Descriptors and compilation never call this module.
    factory = matches[0].load()
    adapter = factory(instance, context)
    required = ['configure', 'health', 'close']
    if instance.get('kind') == 'actuator':
        required += ['prepare', 'activate', 'submit', 'stop']
    if any(not callable(getattr(adapter, method, None)) for method in required):
        raise TypeError('ADAPTER_PROTOCOL_INCOMPLETE: ' + identity)
    return adapter
