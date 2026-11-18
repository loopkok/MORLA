"""The default bindings between capabilities and named component ports."""
import re

from morla_core.composition.diagnostics import CompilationError
from morla_core.composition.parser import fields

_DTYPES = {'bool', 'uint8', 'uint16', 'uint32', 'uint64', 'int8', 'int16',
           'int32', 'int64', 'float16', 'float32', 'float64'}
_RESERVED_FIELDS = {'timestamps', 'arrival_timestamps', 'source_index',
                    'source_offsets', 'source_timestamps'}


def image_ports(component):
    ports = component.get('ports', {})
    images = [name for name, port in ports.items()
              if port.get('direction', 'output') == 'output' and port.get('semantic') == 'image']
    if 'rgb' in images:
        images.remove('rgb')
        images.insert(0, 'rgb')
    return images


def port_roles(component):
    ports = component.get('ports', {})
    images = image_ports(component)
    roles = {'preview': images, 'images': images, 'sensors': [],
             'feedback': 'position' if 'position' in ports else None,
             'command': next((key for key in ('position_target', 'closure') if key in ports), None)}
    roles.update(component.get('port_roles', {}))
    return roles


def validate_port_roles(component):
    roles = component.get('port_roles', {})
    fields(roles, {'feedback', 'command', 'preview', 'images', 'sensors'})
    ports = component['ports']
    for name, port in ports.items():
        if 'sample_fields' not in port:
            continue
        specs = port['sample_fields']
        path = '/ports/' + name + '/sample_fields'
        if (port['direction'] != 'output' or port['semantic'] == 'image'
                or not isinstance(specs, dict) or not specs):
            raise CompilationError('sample_fields requires a non-image output', path)
        for field, spec in specs.items():
            if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_]*', field) or field in _RESERVED_FIELDS:
                raise CompilationError('invalid or reserved sample field', path + '/' + field)
            fields(spec, {'shape', 'dtype', 'unit'}, ('shape', 'dtype'), path + '/' + field)
            if (not isinstance(spec['shape'], list) or
                    any(type(dim) is not int or dim < 1 for dim in spec['shape']) or
                    not isinstance(spec['dtype'], str) or spec['dtype'] not in _DTYPES):
                raise CompilationError('invalid fixed-shape field', path + '/' + field)
    for role, value in roles.items():
        if value is None and role in {'feedback', 'command'}:
            continue
        names = value if role in {'preview', 'images', 'sensors'} else [value]
        if not isinstance(names, list) or any(not isinstance(name, str) for name in names):
            raise CompilationError('invalid port role', '/port_roles/' + role)
        if len(set(names)) != len(names):
            raise CompilationError('duplicate port in role', '/port_roles/' + role)
        for name in names:
            port = ports.get(name, {})
            direction = 'input' if role == 'command' else 'output'
            image = port.get('semantic') == 'image'
            if port.get('direction') != direction or image != (role in {'preview', 'images'}):
                raise CompilationError('incompatible port role: ' + name, '/port_roles/' + role)
