"""Decode measured named feedback without the vendor SDK's missing-joint zeros."""
import math
from .contract import ROLES, role_of, vector, mapping
from .transport import stamp_ns


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
    indices = [names.index(name) for name in ROLES[role][1]]
    if joint.error_code:
        if len(joint.error_code) != len(names) or any(joint.error_code[i] for i in indices):
            raise ValueError('MMK2_JOINT_ERROR')
    signs, offsets = mapping(row)
    values = vector([joint.position[i]*s+o for i,s,o in zip(indices,signs,offsets)],len(indices))
    ports = {'position': {'values': values}}
    if joint.velocity:
        if len(joint.velocity) != len(names):
            raise ValueError('MMK2_INCOMPLETE_VELOCITY')
        ports['velocity'] = {'values': vector([joint.velocity[i]*s for i,s in zip(indices,signs)],len(indices))}
    stamp = stamp_ns(joint.header.stamp if joint.HasField('header') and joint.header.HasField('stamp') else state.stamp)
    return stamp, values, ports
