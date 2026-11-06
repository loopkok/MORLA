"""Static plugin-owned presets; discovery never imports a vendor SDK."""
from morla_core.composition.diagnostics import CompilationError
from morla_core.composition import load_document
from morla_core.resource_locator import resolve_plugin_resource, ResourceLookupError


def template_document(entry):
    return load_document(resolve_plugin_resource(entry['package'],entry['path']))


def template_entries(catalog):
    result = {}
    for descriptor in catalog.list():
        for entry in descriptor.get('web', {}).get('templates', []):
            identity = entry['id']
            if identity in result:
                raise ValueError('ambiguous template registration: '+identity)
            result[identity] = entry
    return result


def list_templates(catalog):
    result=[]
    for identity, entry in sorted(template_entries(catalog).items()):
        try:
            document=template_document(entry)
            for instance in document.get('instances',{}).values():
                catalog.get(instance['plugin'],instance.get('version','1.0.0'))
        except (ResourceLookupError, CompilationError):
            # Optional robot packages may be absent; do not show a broken preset.
            continue
        result.append({'id':identity,'title':entry['title'],'assembly_id':document['assembly_id']})
    return result


def load_template(catalog, identity):
    entries=template_entries(catalog)
    if identity not in entries:
        raise FileNotFoundError('unknown device template')
    return template_document(entries[identity])
