"""Explicit SDK names, native units and strict per-component command boundary."""
import math

PREFIX = 'morla.airbot.mmk2_sdk.'
# The controller's set_goal service time is ~0.7-1.2 s on real hardware
# (both arms ~1.2 s); the read-RPC deadline must not be reused for it.
COMMAND_RPC_TIMEOUT_DEFAULT_S = 3.0
ROLES = {
    'left_arm': ('left_arm', [f'left_arm_joint{i}' for i in range(1,7)], 'rad'),
    'right_arm': ('right_arm', [f'right_arm_joint{i}' for i in range(1,7)], 'rad'),
    'left_gripper': ('left_arm_eef', ['left_arm_eef_gripper_joint'], 'sdk_gripper'),
    'right_gripper': ('right_arm_eef', ['right_arm_eef_gripper_joint'], 'sdk_gripper'),
    'lift': ('spine', ['slide_joint'], 'm'),
    'head': ('head', ['head_yaw_joint','head_pitch_joint'], 'rad'),
    'base': ('base', ['linear_x','angular_z'], 'm_rad/s'),
}
ARM_LIMITS = [(-3.14,2.06),(-2.8,.17),(-.087,3.14),(-2.96,2.96),(-1.74,1.74),(-3.,3.)]
CONNECTION_FIELDS = ('host','port','robot_name','domain_id','rpc_timeout_s','command_rpc_timeout_s','startup_timeout_s','feedback_timeout_s','rate_hz','command_interface','trajectory_scale')


def role_of(row):
    if not row['plugin_id'].startswith(PREFIX):raise ValueError('MMK2_SDK_PLUGIN_ID')
    role=row['plugin_id'][len(PREFIX):]
    if role not in {*ROLES,'camera'}:raise ValueError('MMK2_SDK_UNKNOWN_ROLE')
    return role


def vector(value,n):
    if not isinstance(value,(list,tuple)) or len(value)!=n or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value):
        raise ValueError('MMK2_SDK_FINITE_NAMED_VECTOR_REQUIRED')
    return list(map(float,value))


def goal_velocity(row):
    """Cruise velocity each forward_position goal asks the controller to use.

    `max_velocity` stays the safety cap for how far a goal may advance per
    write (the worker's step limiter and the arm IK step both use it). The
    controller treats a goal as "move here at this speed, then stop", so a
    cruise far above the teleop speed makes every goal finish early and the
    arm stall until the next write -- a stop/go buzz at the write rate.
    `goal_max_velocity` decouples that smoothness knob from the speed cap; a
    value above `max_velocity` just means "as fast as this component may go".
    """
    p=row['parameters']
    return p.get('goal_max_velocity',p['max_velocity'])


def limits(row):
    p=row['parameters'];n=len(row['joints'])
    lower=vector(p.get('joint_lower',[j['lower'] for j in row['joints']]),n)
    upper=vector(p.get('joint_upper',[j['upper'] for j in row['joints']]),n)
    if any(lo>=hi or lo<j['lower'] or hi>j['upper'] for lo,hi,j in zip(lower,upper,row['joints'])):
        raise ValueError('MMK2_SDK_DEPLOYMENT_LIMITS_MUST_NARROW')
    return lower,upper


def mapping(row):
    n=len(row['joints']);p=row['parameters']
    signs=vector(p.get('joint_signs',[1]*n),n);offsets=vector(p.get('joint_offsets',[0]*n),n)
    if any(x not in (-1.,1.) for x in signs):raise ValueError('MMK2_SDK_INVALID_JOINT_SIGNS')
    return signs,offsets


def validate_configuration(row):
    p=row['parameters'];role=role_of(row)
    for name,lo,hi in [('rpc_timeout_s',.02,2),('startup_timeout_s',.1,30),('feedback_timeout_s',.02,2),('rate_hz',1,100),('trajectory_scale',.001,.2)]:
        value=p.get(name)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError('MMK2_SDK_INVALID_'+name)
    # Optional so assemblies resolved before this parameter existed keep working.
    value=p.get('command_rpc_timeout_s')
    if value is not None and (isinstance(value,bool) or not isinstance(value,(int,float)) or
                              not math.isfinite(value) or not .05<=value<=10):
        raise ValueError('MMK2_SDK_INVALID_command_rpc_timeout_s')
    for name in (() if role=='camera' else ('allow_motion','mapping_verified','hold_stop_accepted')):
        if type(p.get(name)) is not bool:raise ValueError('MMK2_SDK_BOOLEAN_REQUIRED_'+name)
    if p.get('command_interface') not in ('trajectory','forward_position'):raise ValueError('MMK2_SDK_COMMAND_INTERFACE')
    if role!='camera':
        limits(row);mapping(row)
        for name in (() if role=='base' else ('max_velocity','max_command_lead')):
            if isinstance(p[name],bool) or not isinstance(p[name],(int,float)) or not math.isfinite(p[name]) or not 0<p[name]<=10:raise ValueError('MMK2_SDK_INVALID_'+name)
        # Cruise target for forward_position goals. It is deliberately NOT
        # bounded by max_velocity: that bound turned a bulk "smooth everything"
        # edit into MMK2_SDK_INVALID_goal_max_velocity on the lift and head,
        # whose step caps are 0.02 and 0.1. A cruise above the step cap only
        # means "as fast as this component is allowed to go".
        value=p.get('goal_max_velocity')
        if value is not None and (isinstance(value,bool) or not isinstance(value,(int,float)) or
                                  not math.isfinite(value) or not 0<value<=10):
            raise ValueError('MMK2_SDK_INVALID_goal_max_velocity')
        value=p.get('command_timeout_s')
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not .05<=value<=1:raise ValueError('MMK2_SDK_INVALID_COMMAND_TIMEOUT')
        if role=='base' and any(mapping(row)[1]):raise ValueError('MMK2_BASE_VELOCITY_OFFSETS_UNSUPPORTED')
    else:
        if p.get('camera') not in ('head_camera','left_camera','right_camera'):raise ValueError('MMK2_CAMERA_NAME')
        for name,lo,hi in [('fps',1,30),('depth_scale_m',0,1)]:
            value=p.get(name)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:raise ValueError('MMK2_INVALID_CAMERA_'+name)
    if row['backend']=='real':
        import re
        host=p.get('host','')
        if not isinstance(host,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]*',host):raise ValueError('MMK2_SDK_EXPLICIT_HOST_REQUIRED')
        if type(p.get('port')) is not int or not 1<=p['port']<=65535:raise ValueError('MMK2_SDK_INVALID_PORT')
        if not isinstance(p.get('robot_name'),str) or not p['robot_name']:raise ValueError('MMK2_SDK_ROBOT_NAME_REQUIRED')
        if type(p.get('domain_id')) is not int or p['domain_id']< -1:raise ValueError('MMK2_SDK_DOMAIN_ID')


def _outside_allowed(q,measured,lower,upper):
    """A joint already resting outside its range may be commanded toward it.

    The command must stay on the same side and never further out than the
    measured pose, so this can only reduce the excursion. It exists so an arm
    that settled or was nudged past a soft limit can be driven back instead of
    locking every command path (teleop and work positions included).
    """
    if measured is None or len(measured)!=len(q) or any(not math.isfinite(v) for v in measured):return False
    for x,m,lo,hi in zip(q,measured,lower,upper):
        if lo<=x<=hi:continue
        if x<lo:
            if not (m<lo and x>=m):return False
        elif not (m>hi and x<=m):return False
    return True


def validate_target(row,command,measured=None):
    role=role_of(row);mode='velocity' if role=='base' else 'position';unit=ROLES[role][2]
    if command.get('names')!=[j['name'] for j in row['joints']] or command.get('mode')!=mode:
        raise ValueError('MMK2_SDK_COMMAND_NAMES_OR_MODE')
    units=command.get('units');n=len(row['joints'])
    if units not in (unit,[unit]*n) and not (role=='base' and units==['m/s','rad/s']):
        raise ValueError('MMK2_SDK_COMMAND_UNITS')
    q=vector(command.get('values'),n);lower,upper=limits(row)
    if role=='base':measured=None
    if any(not lo<=x<=hi for x,lo,hi in zip(q,lower,upper)) and not _outside_allowed(q,measured,lower,upper):
        raise ValueError('MMK2_SDK_TARGET_OUTSIDE_LIMITS')
    signs,offsets=mapping(row)
    raw=[(v-o)/s for v,s,o in zip(q,signs,offsets)]
    raw_measured=None if measured is None else [(v-o)/s for v,s,o in zip(measured,signs,offsets)]
    hard_low=[j['lower'] for j in row['joints']];hard_high=[j['upper'] for j in row['joints']]
    if any(not lo<=v<=hi for v,lo,hi in zip(raw,hard_low,hard_high)) and not _outside_allowed(raw,raw_measured,hard_low,hard_high):
        raise ValueError('MMK2_SDK_MAPPED_TARGET_OUTSIDE_HARDWARE_LIMITS')
    return q
