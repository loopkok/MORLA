"""Decode measured named feedback without the vendor SDK's missing-joint zeros."""
import math
from .contract import ROLES, role_of, vector, mapping
from .transport import stamp_ns


_POSITION_JOINTS = frozenset(
    name for role, (_, names, _) in ROLES.items() if role != 'base' for name in names
)
_SLIDE_JOINT = ROLES['lift'][1][0]
_NONBLOCKING_ERROR_CODES = {-1, 0, 1}


def _joint_errors(joint, names):
    codes = joint.error_code
    if not codes:
        return {}
    if len(codes) == len(names):
        return dict(zip(names, codes))
    if len(codes) == len(names) - 1 and set(names) == _POSITION_JOINTS:
        # The MMK2 state bridge packs only reported error_id interfaces.
        # Its full joint state includes slide_joint, whose driver omits error_id.
        reported_names = (name for name in names if name != _SLIDE_JOINT)
        return dict(zip(reported_names, codes))
    raise ValueError(f'MMK2_JOINT_ERROR_LAYOUT: name={len(names)} error_code={len(codes)}')


def decode(state, row):
    role = role_of(row)
    if role == 'base':
        base = state.base_state
        if not state.HasField('base_state') or not base.HasField('velocity') or not base.HasField('pose'):
            raise ValueError('MMK2_BASE_FEEDBACK_MISSING')
        stamp = stamp_ns(base.stamp)
        signs, offsets = mapping(row)
        if any(offsets):
            raise ValueError('MMK2_BASE_VELOCITY_OFFSETS_UNSUPPORTED')
        values = vector([base.velocity.x*signs[0], base.velocity.omega*signs[1]], 2)
        pose = vector([base.pose.x,base.pose.y,0.,0.,0.,math.sin(base.pose.theta/2),math.cos(base.pose.theta/2)],7)
        return stamp, values, {'velocity': {'values': values}, 'odometry': {'values': pose}}
    joint = state.joint_state
    names = list(joint.name)
    if len(set(names)) != len(names) or len(joint.position) != len(names):
        raise ValueError('MMK2_DUPLICATE_OR_INCOMPLETE_JOINT_STATE')
    role_names = ROLES[role][1]
    indices = [names.index(name) for name in role_names]
    error_codes = _joint_errors(joint, names)
    faults = [
        f'{name}={error_codes[name]}' for name in role_names
        if error_codes.get(name, 0) not in _NONBLOCKING_ERROR_CODES
    ]
    if faults:
        raise ValueError(f'MMK2_JOINT_ERROR: {role}: {", ".join(faults)}')
    signs, offsets = mapping(row)
    values = vector([joint.position[i]*s+o for i,s,o in zip(indices,signs,offsets)],len(indices))
    ports = {'position': {'values': values}}
    if error_codes:
        ports['position']['joint_error_codes'] = {name: error_codes.get(name) for name in role_names}
    if joint.velocity:
        if len(joint.velocity) != len(names):
            raise ValueError('MMK2_INCOMPLETE_VELOCITY')
        ports['velocity'] = {'values': vector([joint.velocity[i]*s for i,s in zip(indices,signs)],len(indices))}
    stamp = stamp_ns(joint.header.stamp if joint.HasField('header') and joint.header.HasField('stamp') else state.stamp)
    return stamp, values, ports
