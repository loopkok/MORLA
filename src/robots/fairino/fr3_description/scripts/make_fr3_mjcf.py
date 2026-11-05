"""Generate FR3 MJCF from the versioned IK URDF, retaining frames and inertia.

Run from source or installed fr3_description/scripts. Collision meshes are
intentionally excluded: this is an IK/dynamics bench, not collision validation.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def pose(origin):
    if origin is None:
        return {}
    r, p, y = map(float, origin.get('rpy', '0 0 0').split())
    cr, cp, cy = math.cos(r/2), math.cos(p/2), math.cos(y/2)
    sr, sp, sy = math.sin(r/2), math.sin(p/2), math.sin(y/2)
    return {'pos': origin.get('xyz', '0 0 0'), 'quat': ' '.join(map(str, (
        cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
        cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)))}


def generate():
    import hashlib, json
    path=ROOT/'urdf/fairino3_v6.urdf'
    if hashlib.sha256(path.read_bytes()).hexdigest()!=json.loads((ROOT/'PROVENANCE.json').read_text())['target_urdf_sha256']:
        raise ValueError('FR3 URDF differs from versioned provenance')
    urdf = ET.parse(ROOT/'urdf/fairino3_v6.urdf').getroot()
    root = ET.Element('mujoco', model='fairino3_v6')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true', meshdir='../meshes/fairino3_v6')
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast')
    assets = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    actuators = ET.SubElement(root, 'actuator')
    links = {link.get('name'): link for link in urdf.findall('link')}

    def body(name, parent, joint=None):
        link = links[name]
        node = ET.SubElement(parent, 'body', name=name, **(pose(joint.find('origin')) if joint is not None else {}))
        inertial = link.find('inertial')
        if inertial is not None:
            inertia = inertial.find('inertia')
            if any(float(v) != 0 for v in inertial.find('origin').get('rpy','0 0 0').split()):
                raise ValueError('FR3 generator requires the versioned zero inertial rpy')
            ET.SubElement(node, 'inertial', mass=inertial.find('mass').get('value'),
                          fullinertia=' '.join(inertia.get(k) for k in ('ixx','iyy','izz','ixy','ixz','iyz')),
                          pos=inertial.find('origin').get('xyz', '0 0 0'))
        if joint is not None and joint.get('type') != 'fixed':
            limit = joint.find('limit')
            # Intersect the native URDF with the original FR3 manual limits.
            index = int(joint.get('name')[1:])-1
            lo = max(float(limit.get('lower')), math.radians([-175,-265,-150,-265,-175,-175][index]))
            hi = min(float(limit.get('upper')), math.radians([175,85,150,85,175,175][index]))
            bounds = f'{lo} {hi}'
            ET.SubElement(node, 'joint', name=joint.get('name'), type='hinge',
                          axis=joint.find('axis').get('xyz'), range=bounds, damping='5', armature='0.05')
            ET.SubElement(actuators, 'position', name=joint.get('name')+'_servo', joint=joint.get('name'),
                          kp='1200', kv='120', ctrlrange=bounds, forcerange='-300 300')
        for index, visual in enumerate(link.findall('visual')):
            filename = Path(visual.find('geometry/mesh').get('filename')).name
            mesh = name+'_visual_'+str(index)
            ET.SubElement(assets, 'mesh', name=mesh, file=filename)
            ET.SubElement(node, 'geom', type='mesh', mesh=mesh, contype='0', conaffinity='0',
                          rgba=(visual.find('material/color').get('rgba') if visual.find('material/color') is not None else '.7 .75 .8 1'), **pose(visual.find('origin')))
        for child in urdf.findall('joint'):
            if child.find('parent').get('link') == name:
                body(child.find('child').get('link'), node, child)
    body('base_link', world)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')+'\n'


if __name__ == '__main__':
    target = ROOT/'mjcf/fr3.xml'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(generate())
