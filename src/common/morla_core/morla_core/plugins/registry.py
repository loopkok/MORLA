"""Static descriptors only: catalogue discovery never imports device code."""
from copy import deepcopy
from morla_core.composition.parser import fields, finite_json
from morla_core.composition.diagnostics import CompilationError
from morla_core.composition.hashing import digest

class Catalog:
    def __init__(self, descriptors=()):
        self._descriptors={}
        for descriptor in descriptors: self.register(descriptor)
    def register(self, descriptor):
        fields(descriptor,{"descriptor_version","plugin_id","plugin_version","api_version","kind","ports","joints","features","backend_variants","resources","operations","geometry","configuration_schema","defaults","validation_artifacts","source","command_modes","controller_types","joint_variants"},("descriptor_version","plugin_id","plugin_version","kind","ports","backend_variants"))
        finite_json(descriptor)
        import re
        if not isinstance(descriptor['plugin_id'],str) or not re.fullmatch(r'[a-z][a-z0-9_]*(?:[.-][a-z0-9_]+)*',descriptor['plugin_id']):raise CompilationError('invalid plugin stable identifier')
        if not isinstance(descriptor['plugin_version'],str) or not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+',descriptor['plugin_version']):raise CompilationError('plugin version must be exact major.minor.patch')
        if descriptor['kind'] not in {'input','actuator','sensor','controller','device_session'}: raise CompilationError("unknown component kind")
        if type(descriptor['descriptor_version']) is not int or descriptor['descriptor_version']!=1: raise CompilationError("unsupported descriptor version")
        key=(descriptor['plugin_id'],descriptor['plugin_version'])
        if key in self._descriptors: raise CompilationError("ambiguous plugin registration: "+str(key))
        ports=descriptor['ports']
        if not isinstance(ports,dict): raise CompilationError("ports must be mapping")
        for name,port in ports.items():
            fields(port,{'direction','semantic','dim','shape','dtype','unit','codec','frame','clock','required','qos','validity','update_range','max_age_ms'})
            if port.get('direction') not in {'input','output'} or not port.get('semantic'): raise CompilationError("port requires direction and semantic")
            if type(port.get('dim',1)) is not int or port.get('dim',1)<1: raise CompilationError("port dimension must be positive")
        self._descriptors[key]=deepcopy(descriptor)
    def get(self, plugin_id, version="1.0.0"):
        try: return deepcopy(self._descriptors[(plugin_id,version)])
        except KeyError: raise CompilationError("uninstalled plugin/version: "+plugin_id+"@"+version,code="PLUGIN_NOT_FOUND")
    def list(self): return [deepcopy(self._descriptors[k]) for k in sorted(self._descriptors)]
    def to_dict(self): return {"components":self.list(),"catalog_hash":digest(self.list())}

def _port(direction,semantic,dim=1,unit="ratio",**extra):
    return {"direction":direction,"semantic":semantic,"dim":dim,"unit":unit,"codec":"numeric","dtype":"float64",**extra}

def _descriptor(plugin,kind,dim=0,unit="rad",family=None):
    joints=[{"name":f"joint{i+1}","unit":unit,"type":"revolute" if unit=="rad" else "scalar","limits":[-3.141593,3.141593] if unit=="rad" else [0,1],"max_velocity":1.0,"lower":-3.141593 if unit=="rad" else 0,"upper":3.141593 if unit=="rad" else 1,"limit_source":"fake_only_defaults"} for i in range(dim)]
    ports={}
    if kind=='actuator':
        ports={"position":_port("output","joint_state",dim,unit),"velocity":_port("output","joint_velocity",dim,"rad/s" if unit=="rad" else "ratio/s"),"joint_target":_port("input","joint_targets",dim,unit),"position_target":_port("input","joint_targets",dim,unit)}
        if unit=='ratio': ports['closure']=_port('input','scalar',dim,unit)
    if kind=='sensor': ports={"rgb":_port("output","image",3,"rgb",codec="image",dtype="uint8")} if 'camera' in plugin else {"values":_port("output","numeric",3,"N")}
    if kind=='input':
        ports={"axes":_port("output","axes_buttons",6),"scalar":_port("output","scalar"),"pose":_port("output","pose6d",7,"m_rad"),"left_pose":_port("output","pose6d",7,"m_rad"),"right_pose":_port("output","pose6d",7,"m_rad"),"left_landmarks":_port("output","hand_landmarks",63),"right_landmarks":_port("output","hand_landmarks",63)}
        for name in ('deadman','left_grip','right_grip','right_shoulder','hold_to_control'): ports[name]=_port('output','scalar')
    if kind=='input' and ('gamepad' in plugin or 'xbox' in plugin or 'mobile' in plugin):
        ports={key:value for key,value in ports.items() if value['semantic'] not in {'pose6d','hand_landmarks'}}
    if kind=='input' and 'glove' in plugin:
        ports={key:value for key,value in ports.items() if value['semantic'] in {'hand_landmarks','scalar'}}
    return {"descriptor_version":1,"api_version":"1.0","plugin_id":plugin,"plugin_version":"1.0.0","kind":kind,"ports":ports,"joints":joints,"command_modes":["position"] if dim else [],
        "backend_variants":{"fake":{"available":True,"validation_level":"protocol"},"sim":{"available":False,"validation_level":"unverified"},"real":{"available":False,"validation_level":"unverified"}},
        "resources":{"session_family":family or plugin,"sharing":"session" if family=='astral_board' else 'exclusive',"stop_policy":"hold"},
        "operations":{"prepare":{"moves":False},"enable":{"moves":False},"stop":{"moves":False}},"geometry":{"frames":["base","flange","mount","optical"],"collision_verified":False,"calibration_required":kind=='actuator'},
        "configuration_schema":{"type":"object","properties":{"board_scope_verified":{"type":"boolean"},"unselected_joints_policy":{"type":"string","enum":["disabled","held"]},"initial_position":{"type":"array","items":{"type":"number"}},"can_channel":{"type":"string"},"control_board_ip":{"type":"string"},"local_port":{"type":"integer","minimum":1,"maximum":65535},"serial_number":{"type":"string"},"port_name":{"type":"string"},"hand_id":{"type":"integer","minimum":0},"rate_hz":{"type":"number","exclusiveMinimum":0},"side":{"type":"string","enum":["left","right"]},"device_id":{"type":"string"},"max_sample_age_ms":{"type":"number","exclusiveMinimum":0}},"additionalProperties":False},"defaults":{"rate_hz":100},"validation_artifacts":[],"source":{"package":"morla_core","version":"1.0.0"}}

def default_catalog():
    specs=[('morla.nero.arm','actuator',7,'rad',None),('morla.astral.arm','actuator',7,'rad','astral_board'),('morla.astral.gripper','actuator',1,'ratio','astral_board'),('morla.xhand.hand','actuator',12,'rad',None),('morla.wuji.hand','actuator',20,'rad',None),('morla.astral.head','actuator',2,'rad','astral_board'),('morla.astral.waist','actuator',2,'rad','astral_board'),('morla.ros2_control.mock','actuator',7,'rad',None),('morla.fake.arm','actuator',7,'rad',None),('morla.fake.sensor','sensor',0,'N',None),('morla.fake.input','input',0,'ratio',None)]
    descriptors=[_descriptor(*row) for row in specs]
    for plugin in ('morla.camera','morla.camera.ros_image','morla.camera.uvc','morla.fake.camera'): descriptors.append(_descriptor(plugin,'sensor'))
    for plugin in ('morla.input.quest','morla.input.gamepad','morla.input.xbox','morla.input.mobile','morla.input.glove'): descriptors.append(_descriptor(plugin,'input'))
    import json
    from pathlib import Path
    metadata=json.loads((Path(__file__).parent/'data'/'joint_metadata.json').read_text())
    geometry_metadata=json.loads((Path(__file__).parent/'data'/'geometry_metadata.json').read_text())
    for descriptor in descriptors:
        descriptor['geometry'].update(geometry_metadata.get(descriptor['plugin_id'],{}))
        if descriptor['plugin_id'] in {'morla.nero.arm','morla.astral.arm','morla.astral.gripper','morla.xhand.hand','morla.wuji.hand'}:
            descriptor['backend_variants']['real']={'available':True,'requires':['rclpy','ROS Humble','native driver package','vendor SDK'],'validation_level':'requires_ros_sdk'}
        if descriptor['plugin_id'] in {'morla.nero.arm','morla.astral.arm'}:
            descriptor['operations'].update({'home':{'moves':True,'scope':'device_session'},'workpos':{'moves':True,'scope':'device_session'},'disable':{'moves':False,'scope':'device_session'}})
        elif descriptor['plugin_id'] in {'morla.xhand.hand','morla.wuji.hand'}:
            descriptor['operations']['disable']={'moves':False,'scope':'component'}
        if descriptor['plugin_id'].startswith('morla.camera'):
            descriptor['configuration_schema']['properties'].update({'image_topic':{'type':'string'},'compressed':{'type':'boolean'},'device_path':{'type':'string'},'width':{'type':'integer','minimum':1},'height':{'type':'integer','minimum':1},'fps':{'type':'number','exclusiveMinimum':0},'camera_backend':{'type':'string','enum':['ros_image','uvc']}})
            descriptor['backend_variants']['real']={'available':True,'requires':['rclpy or cv2','camera driver','stable device fingerprint'],'validation_level':'requires_camera_driver'}
        if descriptor['kind']=='input':
            descriptor['configuration_schema']['properties'].update({'input_topic':{'type':'string'},'device_name':{'type':'string'},'deadzone':{'type':'number','minimum':0,'exclusiveMaximum':1},'protocol':{'type':'string','enum':['udp','tcp_wired','tcp_wireless']},'udp_port':{'type':'integer','minimum':1,'maximum':65535},'tcp_port':{'type':'integer','minimum':1,'maximum':65535},'source_namespace':{'type':'string'}})
        if descriptor['plugin_id'] in {'morla.xhand.hand','morla.wuji.hand'}:
            descriptor['configuration_schema']['properties'].update({'retarget_backend':{'type':'string','enum':['official','dexpilot']},'preprocess_mode':{'type':'string','enum':['legacy_xhand','legacy_xhand_pinky']},'index_bend_sign':{'type':'number','enum':[-1,1]},'enable_thumb_fix':{'type':'boolean'}})
        if descriptor['plugin_id'] in {'morla.nero.arm','morla.astral.arm','morla.xhand.hand','morla.wuji.hand','morla.camera','morla.camera.ros_image','morla.camera.uvc'}:
            descriptor['backend_variants']['sim']={'available':True,'requires':['mujoco','robot_description_assets'],'validation_level':'legacy_asset_dynamics_collision_unverified'}
        variants=metadata.get(descriptor['plugin_id'])
        if variants:
            descriptor['joint_variants']=variants
            descriptor['joints']=deepcopy(variants.get('left',variants.get('default',next(iter(variants.values())))))
    from .manifest_loader import discover_descriptors
    descriptors.extend(discover_descriptors())
    return Catalog(descriptors)
