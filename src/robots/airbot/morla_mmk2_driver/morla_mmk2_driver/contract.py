"""Explicit SDK names, native units and strict per-component command boundary."""
import math

PREFIX = 'morla.airbot.mmk2_sdk.'
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
CONNECTION_FIELDS = ('host','port','robot_name','domain_id','rpc_timeout_s','startup_timeout_s','feedback_timeout_s','rate_hz','command_interface','trajectory_scale')


def role_of(row):
    if not row['plugin_id'].startswith(PREFIX):raise ValueError('MMK2_SDK_PLUGIN_ID')
    role=row['plugin_id'][len(PREFIX):]
    if role not in {*ROLES,'camera'}:raise ValueError('MMK2_SDK_UNKNOWN_ROLE')
    return role


def vector(value,n):
    if not isinstance(value,(list,tuple)) or len(value)!=n or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value):
        raise ValueError('MMK2_SDK_FINITE_NAMED_VECTOR_REQUIRED')
    return list(map(float,value))


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
    for name in (() if role=='camera' else ('allow_motion','mapping_verified','hold_stop_accepted')):
        if type(p.get(name)) is not bool:raise ValueError('MMK2_SDK_BOOLEAN_REQUIRED_'+name)
    if p.get('command_interface') not in ('trajectory','forward_position'):raise ValueError('MMK2_SDK_COMMAND_INTERFACE')
    if role!='camera':
        limits(row);mapping(row)
        for name in (() if role=='base' else ('max_velocity','max_command_lead')):
            if isinstance(p[name],bool) or not isinstance(p[name],(int,float)) or not math.isfinite(p[name]) or not 0<p[name]<=10:raise ValueError('MMK2_SDK_INVALID_'+name)
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


def validate_target(row,command):
    role=role_of(row);mode='velocity' if role=='base' else 'position';unit=ROLES[role][2]
    if command.get('names')!=[j['name'] for j in row['joints']] or command.get('mode')!=mode:
        raise ValueError('MMK2_SDK_COMMAND_NAMES_OR_MODE')
    units=command.get('units');n=len(row['joints'])
    if units not in (unit,[unit]*n) and not (role=='base' and units==['m/s','rad/s']):
        raise ValueError('MMK2_SDK_COMMAND_UNITS')
    q=vector(command.get('values'),n);lower,upper=limits(row)
    if any(not lo<=x<=hi for x,lo,hi in zip(q,lower,upper)):raise ValueError('MMK2_SDK_TARGET_OUTSIDE_LIMITS')
    signs,offsets=mapping(row)
    raw=[(v-o)/s for v,s,o in zip(q,signs,offsets)]
    if any(not j['lower']<=v<=j['upper'] for v,j in zip(raw,row['joints'])):raise ValueError('MMK2_SDK_MAPPED_TARGET_OUTSIDE_HARDWARE_LIMITS')
    return q
