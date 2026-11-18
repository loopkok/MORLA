"""Selected installed adapter loading; no assembly-controlled import strings."""
from importlib import metadata
import re


def adapter_entry(instance):
    entries = metadata.entry_points()
    entries = list(entries.select(group='morla.adapters')) if hasattr(entries, 'select') else list(entries.get('morla.adapters', []))
    identity = instance['plugin_id'] + ':' + instance.get('backend', 'fake')
    owner = instance.get('plugin_distribution')
    if owner:
        def normalized(name):
            return re.sub(r'[-_.]+', '-', name).lower()
        entries = [entry for entry in entries if entry.dist is not None
                   and normalized(entry.dist.name) == normalized(owner)]
    matches = [entry for entry in entries if entry.name == identity]
    if len(matches) > 1:
        raise ValueError('AMBIGUOUS_ADAPTER_ENTRY_POINT: ' + identity)
    return matches[0] if matches else None


def load_installed_adapter(instance, context):
    entry = adapter_entry(instance)
    if entry is None:
        if instance.get('plugin_distribution') and not (
                instance.get('backend') == 'sim' and instance.get('geometry', {}).get('simulation')):
            raise ValueError('MISSING_ADAPTER_ENTRY_POINT: ' + instance['plugin_id'])
        return None
    # Only the chosen installed entry point is imported, after the runtime owns
    # the host/device locks. Descriptors and compilation never call this module.
    factory = entry.load()
    adapter = factory(instance, context)
    required = ['configure', 'health', 'close']
    if instance.get('kind') == 'actuator':
        required += ['prepare', 'activate', 'submit', 'stop']
    elif instance.get('kind') == 'sensor' and instance.get('plugin_distribution'):
        required.append('read')
    if any(not callable(getattr(adapter, method, None)) for method in required):
        raise TypeError('ADAPTER_PROTOCOL_INCOMPLETE: ' + entry.name)
    return adapter
