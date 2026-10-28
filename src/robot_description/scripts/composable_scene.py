"""Compose selected component MJCF subtrees into one physical MuJoCo scene.

Asset coordinates and inertias are retained. Imported legacy arm meshes disable
collision and therefore this builder only claims kinematic/dynamic integration,
not collision safety. Mount transforms use [x,y,z,qx,qy,qz,qw] and require explicit
values; a calibration name alone is not interpreted as an identity transform.
"""
from copy import deepcopy
from pathlib import Path
import argparse
import json
import re
import xml.etree.ElementTree as ET

_DESCRIPTION=Path(__file__).resolve().parents[1]
# Only trusted installed assets: an assembly cannot provide an arbitrary filename.
_ASSETS={'morla.nero.arm':'mjcf/xnero/xnero_dual.xml','morla.xhand.hand':'mjcf/xnero/xnero_dual.xml','morla.astral.arm':'mjcf/astral/astral_dual_clean.xml','morla.wuji.hand':'mjcf/wuji/{side}.xml'}
_REFERENCES={'joint','joint1','joint2','body','body1','body2','site','site1','site2','geom','geom1','geom2','mesh','material','texture','tendon','tendon1','tendon2','actuator','target','objname','refname','class','childclass','camera'}

def _prefix_tree(node,prefix,names):
    for item in node.iter():
        for key,value in list(item.attrib.items()):
            if key=='name': item.set(key,prefix+value)
            elif key in _REFERENCES and value in names: item.set(key,prefix+value)

def _clone_assets(root,path,prefix):
    compiler=root.find('compiler')
    meshdir=path.parent/(compiler.get('meshdir','') if compiler is not None else '')
    texturedir=path.parent/(compiler.get('texturedir','') if compiler is not None else '')
    assets=[]
    asset=root.find('asset')
    if asset is None:return assets
    names={n.get('name') for n in root.iter() if n.get('name')}
    for original in asset:
        clone=deepcopy(original)
        for item in clone.iter():
            if item.get('file'):
                directory=meshdir if item.tag in {'mesh','skin'} else texturedir
                item.set('file',str((directory/item.get('file')).resolve()))
        _prefix_tree(clone,prefix,names);assets.append(clone)
    return assets

def _source_component(plugin,side):
    if plugin not in _ASSETS:raise ValueError('No validated physical asset for '+plugin)
    path=_DESCRIPTION/_ASSETS[plugin].format(side=side)
    root=ET.parse(path).getroot()
    if plugin=='morla.nero.arm':
        body=deepcopy(root.find(".//body[@name='"+side+"_link1']"))
        for parent in body.iter():
            for child in list(parent):
                if child.tag=='body' and '_hand_' in child.get('name',''):parent.remove(child)
        flange=side+'_link7'
    elif plugin=='morla.astral.arm':
        body=deepcopy(root.find(".//body[@name='"+side+"_base_link']"));flange=side+'_link7'
    elif plugin=='morla.wuji.hand':
        body=deepcopy(root.find('worldbody/body'));flange=side+'_palm_link'
    else:
        # XHand is embedded in link7 in the established source model. Retain only
        # its own fixed geometry and finger subtrees, preserving original poses.
        original=root.find(".//body[@name='"+side+"_link7']")
        body=ET.Element('body',{'name':side+'_hand_mount'})
        for child in original:
            if child.tag=='body' and '_hand_' in child.get('name',''):body.append(deepcopy(child))
            elif child.tag=='geom' and ('hand' in child.get('mesh','') or 'connector' in child.get('mesh','')):body.append(deepcopy(child))
        flange=side+'_hand_mount'
    if body is None:raise ValueError('Source component subtree missing')
    return root,path,body,flange

def build_scene(resolved,output=None,timestep=0.002):
    import math
    if not math.isfinite(timestep) or timestep<=0:raise ValueError('timestep must be finite and positive')
    root=ET.Element('mujoco',{'model':resolved['assembly_id']})
    ET.SubElement(root,'compiler',{'angle':'radian','autolimits':'true'})
    ET.SubElement(root,'option',{'timestep':str(timestep),'gravity':'0 0 -9.81'})
    assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');actuators=ET.SubElement(root,'actuator');sensors=ET.SubElement(root,'sensor');contacts=ET.SubElement(root,'contact');equalities=ET.SubElement(root,'equality')
    ET.SubElement(world,'light',{'pos':'1 1 3','dir':'-0.3 -0.3 -1'})
    ET.SubElement(world,'geom',{'name':'scene_floor','type':'plane','size':'3 3 .1','rgba':'.2 .2 .25 1'})
    wrappers={};frames={'world':world,'workcell':world};joint_map={};source_hashes={}
    import hashlib
    for instance in resolved['instances']:
        iid=instance['instance_id'];plugin=instance['plugin_id'];side=instance.get('parameters',{}).get('side','left')
        if not re.fullmatch('[a-z][a-z0-9_-]*',iid):raise ValueError('Invalid instance name')
        if plugin.startswith('morla.camera'):
            wrapper=ET.Element('body',{'name':iid+'__mount'});ET.SubElement(wrapper,'camera',{'name':iid+'__rgb','pos':'0 0 0','quat':'1 0 0 0'})
            wrappers[iid]=wrapper;frames[iid+'.optical']=wrapper;world.append(wrapper);continue
        if instance.get('kind')!='actuator':continue
        source,path,body,flange=_source_component(plugin,side)
        source_hashes[iid]=hashlib.sha256(path.read_bytes()).hexdigest()
        prefix=iid+'__';names={n.get('name') for n in source.iter() if n.get('name')}
        selected={n.get('name') for n in body.iter() if n.get('name')}
        source_joints=[n.get('name') for n in body.iter('joint')]
        for asset in _clone_assets(source,path,prefix):assets.append(asset)
        _prefix_tree(body,prefix,names)
        wrapper=ET.Element('body',{'name':prefix+'mount'});wrapper.append(body);world.append(wrapper);wrappers[iid]=wrapper
        frames[iid+'.base']=wrapper;frames[iid+'.mount']=wrapper
        frames[iid+'.flange']=next(n for n in body.iter('body') if n.get('name')==prefix+flange)
        for section,target in (('actuator',actuators),('sensor',sensors),('contact',contacts),('equality',equalities)):
            source_section=source.find(section)
            if source_section is None:continue
            for element in source_section:
                references=[value for key,value in element.attrib.items() if key in _REFERENCES and key not in {'class','childclass'}]
                if any(value not in selected for value in references):continue
                clone=deepcopy(element);_prefix_tree(clone,prefix,names);target.append(clone)
        canonical=instance['joints']
        if len(canonical)!=len(source_joints):raise ValueError(f'{iid}: source joint count {len(source_joints)} differs from contract {len(canonical)}')
        joint_map[iid]={}
        for joint in canonical:
            source_name=joint.get('mjcf_name')
            if source_name not in source_joints: raise ValueError(f"{iid}: explicit MJCF joint mapping missing/invalid for {joint['name']}")
            joint_map[iid][joint['name']]=prefix+source_name
        if len(set(joint_map[iid].values()))!=len(source_joints):raise ValueError('MJCF mapping must be bijective')
    actuated={element.get('joint') for element in actuators}
    for instance in resolved['instances']:
        iid=instance['instance_id']
        if iid not in joint_map: continue
        for joint in instance['joints']:
            target=joint_map[iid][joint['name']]
            if target not in actuated:
                # Established arm-only assets may provide no actuator. Reference
                # position servos are explicit generated simulation controllers,
                # not claimed to reproduce real actuator tuning.
                source_joint=next(element for element in root.iter('joint') if element.get('name')==target)
                limits=source_joint.get('range')
                attrs={'name':target+'__servo','joint':target,'kp':'100','kv':'10'}
                if limits: attrs['ctrlrange']=limits
                ET.SubElement(actuators,'position',attrs)
            ET.SubElement(sensors,'jointpos',{'name':target+'__position','joint':target})
    # All nodes exist before applying mounts so ordering of user mounts is irrelevant.
    parents={}
    for mount in resolved.get('mounts',[]):
        child,endpoint=mount['child'].split('.',1);parent=mount['parent']
        if endpoint not in {'base','mount'} and not (endpoint=='optical' and child+'.optical' in frames):
            raise ValueError('Mount child endpoint must be component base/mount; nonroot frame inversion is not supported')
        if child not in wrappers:raise ValueError('No geometry for mounted child '+child)
        if parent not in frames:raise ValueError('Unknown mount parent '+parent)
        if child in parents:raise ValueError('Multiple parents for '+child)
        parents[child]=parent.split('.')[0]
    for child in parents:
        seen=set();cur=child
        while cur in parents:
            if cur in seen:raise ValueError('Mount cycle')
            seen.add(cur);cur=parents[cur]
    for mount in resolved.get('mounts',[]):
        child=mount['child'].split('.')[0];wrapper=wrappers[child];transform=mount.get('transform')
        if transform is None:raise ValueError('Physical scene requires resolved transform for '+mount['child'])
        if len(transform)!=7:raise ValueError('Physical scene supports xyz + xyzw quaternion transforms')
        import math
        if not all(isinstance(v,(float,int)) and math.isfinite(v) for v in transform):raise ValueError('Nonfinite transform')
        if abs(sum(v*v for v in transform[3:])-1)>1e-5:raise ValueError('Nonunit quaternion')
        wrapper.set('pos',' '.join(map(str,transform[:3])));wrapper.set('quat',' '.join(map(str,[transform[6],*transform[3:6]])))
        world.remove(wrapper);frames[mount['parent']].append(wrapper)
    xml=ET.tostring(root,encoding='unicode')
    if output:Path(output).write_text(xml)
    return {'xml':xml,'joint_map':joint_map,'source_asset_hashes':source_hashes,'simulation_level':'legacy_asset_dynamics_collision_unverified','timestep':timestep}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('resolved');parser.add_argument('--output',required=True);args=parser.parse_args()
    result=build_scene(json.loads(Path(args.resolved).read_text()),args.output)
    print(json.dumps({k:v for k,v in result.items() if k!='xml'},indent=2))
if __name__=='__main__':main()
