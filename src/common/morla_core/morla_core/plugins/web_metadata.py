"""Validate declarative presentation metadata without executing plugin code."""
from pathlib import PurePosixPath
import re
from morla_core.composition.diagnostics import CompilationError


def validate_web_metadata(value):
    def fail():
        raise CompilationError('invalid web metadata')
    if not isinstance(value, dict): fail()
    for key in ('fields','connection_fields','ik_modes'):
        if key in value and (not isinstance(value[key],list) or not all(isinstance(x,str) for x in value[key])): fail()
    for key in ('field_backends','field_conditions','preferred_controllers'):
        if key in value and not isinstance(value[key],dict): fail()
    if 'templates' in value and not isinstance(value['templates'],list): fail()
    ids=set()
    for entry in value.get('templates',[]):
        if not isinstance(entry,dict) or set(entry)!={'id','title','package','path'}: fail()
        if not all(isinstance(x,str) and x for x in entry.values()): fail()
        if not re.fullmatch('[a-z][a-z0-9_-]*',entry['id']) or not re.fullmatch('[a-z][a-z0-9_]*',entry['package']): fail()
        path=PurePosixPath(entry['path'])
        if path.is_absolute() or '..' in path.parts or '\\' in entry['path'] or path.suffix not in ('.yaml','.yml','.json'): fail()
        if entry['id'] in ids: fail()
        ids.add(entry['id'])
    if 'binding_presets' in value and not isinstance(value['binding_presets'],list): fail()
    for preset in value.get('binding_presets',[]):
        if not isinstance(preset,dict) or not isinstance(preset.get('controller'),str) or not isinstance(preset.get('source_plugin'),str) or not isinstance(preset.get('defaults'),dict) or not isinstance(preset.get('parameters',{}),dict): fail()
