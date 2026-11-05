"""FR3 standard model, manual 4.0.1 tables 2.1-1 and 2.4-1."""
import math
import ipaddress

LOWER = tuple(map(math.radians, [-175, -265, -150, -265, -175, -175]))
UPPER = tuple(map(math.radians, [175, 85, 150, 85, 175, 175]))
URDF_LOWER = (-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543)
URDF_UPPER = (3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543)
INITIAL = (0., -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.)


def vector6(value):
    if not isinstance(value, (list, tuple)) or len(value) != 6 or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError('FR3 requires six finite numeric joint values')
    return [float(v) for v in value]


def limits(parameters):
    lo = vector6(parameters.get('joint_lower', LOWER))
    hi = vector6(parameters.get('joint_upper', UPPER))
    if any(a < outer_a or b > outer_b or a >= b
           for a, b, outer_a, outer_b in zip(lo, hi, LOWER, UPPER)):
        raise ValueError('FR3 deployment limits must lie within manual soft limits')
    signs = vector6(parameters.get('urdf_joint_signs', [1.] * 6))
    offsets = vector6(parameters.get('urdf_joint_offsets', [0.] * 6))
    if any(s not in (-1., 1.) for s in signs):
        raise ValueError('FR3 URDF joint signs must be +1/-1')
    a = [(v-o)/s for v,o,s in zip(URDF_LOWER,offsets,signs)]
    b = [(v-o)/s for v,o,s in zip(URDF_UPPER,offsets,signs)]
    lo = [max(x,min(u,v)) for x,u,v in zip(lo,a,b)]
    hi = [min(x,max(u,v)) for x,u,v in zip(hi,a,b)]
    if any(x>=y for x,y in zip(lo,hi)):
        raise ValueError('FR3 URDF/deployment limits have no intersection')
    return lo, hi


def positions(value, parameters):
    q = vector6(value)
    lo, hi = limits(parameters)
    if any(v < a or v > b for v, a, b in zip(q, lo, hi)):
        raise ValueError('FR3 target outside deployment joint limits')
    return q


def validate_parameters(parameters, *, real):
    limits(parameters)
    for key, lower, upper in (
        ('rate_hz', 50, 125), ('rpc_timeout_s', .05, 1),
        ('startup_timeout_s', 1, 30), ('operation_timeout_s', 1, 15),
        ('feedback_timeout_s', .02, .5), ('max_velocity_rad_s', .001, 1),
        ('max_command_lead_rad', .001, .3),
    ):
        value = parameters.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f'FR3 {key} must be in [{lower}, {upper}]')
    if real:
        # An explicit IP also prevents a typo/place-holder being resolved on the network.
        try:
            ipaddress.IPv4Address(parameters.get('host', ''))
        except (ValueError, TypeError) as exc:
            raise ValueError('FR3 real deployment requires an explicit controller IPv4 address') from exc


def validate_target(target, instance):
    if target.get('component_id') != instance['instance_id']:
        raise ValueError('FR3 component_id mismatch')
    if target.get('names') != [j['name'] for j in instance['joints']]:
        raise ValueError('FR3 canonical joint names/order mismatch')
    if target.get('mode') != 'position' or target.get('units') != ['rad'] * 6:
        raise ValueError('FR3 only accepts position targets in radians')
    return positions(target.get('values'), instance['parameters'])
