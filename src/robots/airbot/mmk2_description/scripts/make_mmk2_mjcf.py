"""Deterministically derive the complete MMK2 scene from the supplied URDF.

Planar chassis is an explicit reference model, not wheel-ground dynamics.
Visual meshes do not participate in contact; fingers preserve URDF mimic.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]


def pose(origin):
    if origin is None:
        return {}
    r, p, y = [float(v) / 2 for v in origin.get('rpy', '0 0 0').split()]
    cr, cp, cy, sr, sp, sy = math.cos(r), math.cos(p), math.cos(y), math.sin(r), math.sin(p), math.sin(y)
    return dict(pos=origin.get('xyz', '0 0 0'), quat=' '.join(map(str, (
        cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy))))


def generate():
    u = E.parse(ROOT/'urdf/airbot_mmk2.urdf').getroot()
    root = E.Element('mujoco', model='mmk2_urdf_planar_reference')
    E.SubElement(root, 'compiler', angle='radian', autolimits='true', meshdir='../meshes')
    E.SubElement(root, 'option', timestep='0.002', integrator='implicitfast')
    visual=E.SubElement(root, 'visual')
    E.SubElement(visual, 'global', offwidth='1280', offheight='960')
    E.SubElement(visual, 'headlight', ambient='.4 .4 .4', diffuse='.6 .6 .6', specular='0 0 0')
    assets, world, actuators, equality = [E.SubElement(root, tag) for tag in ('asset','worldbody','actuator','equality')]
    E.SubElement(world, 'light', pos='0 0 4', dir='0 0 -1')
    E.SubElement(world, 'geom', name='floor', type='plane', size='10 10 .01', rgba='.25 .3 .35 1')
    E.SubElement(world, 'geom', name='reference_obstacle', type='box', pos='2 0 .35', size='.15 .4 .35', rgba='.8 .4 .15 1')
    E.SubElement(world, 'camera', name='scene', pos='2.8 -2.8 2.2', xyaxes='.707 .707 0 -.3 .3 .906')
    links = {x.get('name'): x for x in u.findall('link')}
    def body(name, parent, joint=None):
        node = E.SubElement(parent, 'body', name=name, **(pose(joint.find('origin')) if joint is not None else {}))
        inertial = links[name].find('inertial')
        if inertial is not None:
            # Supplied URDF has zero inertial rotation. Do not silently ignore it.
            origin = inertial.find('origin')
            if origin is not None and any(float(v) for v in origin.get('rpy', '0 0 0').split()):
                raise ValueError('nonzero inertial RPY requires tensor rotation')
            E.SubElement(node, 'inertial', mass=inertial.find('mass').get('value'),
                         pos=origin.get('xyz','0 0 0') if origin is not None else '0 0 0',
                         fullinertia=' '.join(inertial.find('inertia').get(k) for k in ('ixx','iyy','izz','ixy','ixz','iyz')))
        if name == 'base_link':
            for axis, kind, direction in [('x','slide','1 0 0'),('y','slide','0 1 0'),('yaw','hinge','0 0 1')]:
                jname = 'base_'+axis
                E.SubElement(node, 'joint', name=jname, type=kind, axis=direction, limited='false', damping='10')
                E.SubElement(actuators, 'velocity', name=jname+'_drive', joint=jname, kv='10000')
            E.SubElement(node, 'site', name='imu', pos='0 0 .25', size='.005')
        if joint is not None and joint.get('type') != 'fixed':
            jname = joint.get('name'); wheel = 'wheel_joint' in jname
            limit = joint.find('limit'); mimic = joint.find('mimic')
            attrs = dict(name=jname, type='slide' if joint.get('type')=='prismatic' else 'hinge',
                         axis=joint.find('axis').get('xyz'), damping='2', armature='.02')
            if wheel:
                attrs['limited'] = 'false'  # right wheel ±pi URDF typo; both wheels continuous
            else:
                attrs['range'] = limit.get('lower')+' '+limit.get('upper')
            E.SubElement(node, 'joint', **attrs)
            if mimic is not None:
                E.SubElement(equality, 'joint', joint1=jname, joint2=mimic.get('joint'), polycoef='0 1 0 0 0')
            elif wheel:
                E.SubElement(actuators, 'velocity', name=jname+'_drive', joint=jname, kv='20', ctrlrange='-8 8')
            else:
                kp, kv = ('50000','1000') if jname=='slide_joint' else ('1000','60')
                E.SubElement(actuators, 'position', name=jname+'_servo', joint=jname, kp=kp, kv=kv,
                             ctrlrange=attrs['range'], forcerange='-2000 2000')
        for i, v in enumerate(links[name].findall('visual')):
            mesh = v.find('geometry/mesh')
            if mesh is None:
                continue
            key = name+'_visual_'+str(i)
            filename = mesh.get('filename').split('/meshes/',1)[1]
            mesh_attributes = dict(name=key, file=filename)
            if mesh.get('scale') is not None:
                mesh_attributes['scale'] = mesh.get('scale')
            E.SubElement(assets, 'mesh', **mesh_attributes)
            color = v.find('material/color')
            E.SubElement(node, 'geom', mesh=key, type='mesh', group='2', contype='0', conaffinity='0',
                         rgba=color.get('rgba') if color is not None else '.7 .75 .8 1', **pose(v.find('origin')))
        if name in ('head_pitch_link','left_camera_link','right_camera_link'):
            # MJ cameras look -Z. The head link has -Y up and -Z right;
            # place the lens outside its housing, pitched down by .3 rad.
            E.SubElement(node, 'camera', name={'head_pitch_link':'head','left_camera_link':'left_wrist','right_camera_link':'right_wrist'}[name],
                         pos='.11 -.1855 0' if name=='head_pitch_link' else '.06 0 0',
                         xyaxes='0 0 -1 .295520206661 -.955336489126 0' if name=='head_pitch_link' else '0 -1 0 0 0 1', fovy='60')
        for child in u.findall('joint'):
            if child.find('parent').get('link') == name:
                body(child.find('child').get('link'), node, child)
    body('base_link', world)
    sensors = E.SubElement(root, 'sensor')
    E.SubElement(sensors, 'gyro', name='imu_gyro', site='imu')
    E.SubElement(sensors, 'accelerometer', name='imu_acc', site='imu')
    E.indent(root)
    return E.tostring(root, encoding='unicode')+'\n'


if __name__ == '__main__':
    (ROOT/'mjcf/mmk2.xml').write_text(generate())
