"""Generate SDK-native descriptors. No vendor import or network discovery."""
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from morla_mmk2_driver.contract import PREFIX,ROLES,ARM_LIMITS


def port(direction,semantic,n,unit,**kw):
    return dict(direction=direction,semantic=semantic,dim=n,unit=unit,**dict(codec='numeric',dtype='float64')|kw)


def generate():
    connection={
        'host':{'type':'string','title':'MMK2 gRPC 主机'},'port':{'type':'integer','minimum':1,'maximum':65535,'title':'gRPC 端口'},
        'robot_name':{'type':'string','title':'SDK 客户端注册名称'},'domain_id':{'type':'integer','minimum':-1},
        'rpc_timeout_s':{'type':'number','minimum':.02,'maximum':2},
        'command_rpc_timeout_s':{'type':'number','minimum':.05,'maximum':10,'title':'运动指令 set_goal 超时 / s'},
        'startup_timeout_s':{'type':'number','minimum':.1,'maximum':30},
        'feedback_timeout_s':{'type':'number','minimum':.02,'maximum':2},'rate_hz':{'type':'number','minimum':1,'maximum':100},
        'allow_motion':{'type':'boolean','title':'允许写入运动目标（默认只读）'},
        'mapping_verified':{'type':'boolean','title':'已完成 SDK 名称/单位/方向核验（写入门禁）'},
        'hold_stop_accepted':{'type':'boolean','title':'接受 SDK 保持目标/零底盘速度停止策略（非失能）'},
        'command_interface':{'type':'string','enum':['trajectory','forward_position']},
        'trajectory_scale':{'type':'number','minimum':.001,'maximum':.2},
    }
    defaults=dict(host='',port=50055,robot_name='morla_mmk2',domain_id=-1,rpc_timeout_s=.25,command_rpc_timeout_s=3.,startup_timeout_s=5.,feedback_timeout_s=.3,rate_hz=20.,allow_motion=False,mapping_verified=False,hold_stop_accepted=False,command_interface='trajectory',trajectory_scale=.05)
    descriptors=[]
    for role,(sdk,names,unit) in ROLES.items():
        n=len(names)
        bounds=ARM_LIMITS if role.endswith('_arm') else [(-.47,.47),(-.98,.32)] if role=='head' else [(-.04,.87)] if role=='lift' else [(-.15,.15),(-.3,.3)] if role=='base' else [(0.,1.)]
        joints=[dict(name=name,unit=(['m/s','rad/s'][i] if role=='base' else unit),type='scalar' if role=='base' or 'gripper' in role else 'prismatic' if role=='lift' else 'revolute',lower=lo,upper=hi,limits=[lo,hi],limit_source='MMK2 SDK JointLimits; base deployment speed envelope') for i,(name,(lo,hi)) in enumerate(zip(names,bounds))]
        ports={'velocity':port('output','joint_velocity',n,unit),'velocity_target':port('input','joint_targets',n,unit),'odometry':port('output','pose6d',7,'m_rad')} if role=='base' else {'position':port('output','joint_state',n,unit),'position_target':port('input','joint_targets',n,unit),'velocity':port('output','joint_velocity',n,'rad/s' if unit=='rad' else 'm/s' if unit=='m' else 'sdk_gripper/s')}
        fields=dict(connection)
        for name in ('joint_lower','joint_upper','joint_signs','joint_offsets'):
            label_unit='通道 1 m/s、通道 2 rad/s' if role=='base' else 'SDK 原始量' if unit=='sdk_gripper' else unit
            title={'joint_lower':'目标下限 / '+label_unit,'joint_upper':'目标上限 / '+label_unit,'joint_signs':'SDK 到逻辑关节方向（±1）','joint_offsets':'SDK 到逻辑关节偏置 / '+label_unit}[name]
            fields[name]=dict(type='array',title=title,items={'type':'number'},minItems=n,maxItems=n)
        fields.update(max_velocity={'type':'number','exclusiveMinimum':0,'maximum':10},goal_max_velocity={'type':'number','exclusiveMinimum':0,'maximum':10},goal_velocity_tracking={'type':'boolean'},max_command_lead={'type':'number','exclusiveMinimum':0,'maximum':10},command_timeout_s={'type':'number','minimum':.05,'maximum':1})
        params=dict(defaults,joint_lower=[x[0] for x in bounds],joint_upper=[x[1] for x in bounds],joint_signs=[1]*n,joint_offsets=[0]*n,max_velocity=.1 if unit=='rad' else .02 if unit=='m' else .2,max_command_lead=.15 if unit=='rad' else .03 if unit=='m' else 1. if unit=='sdk_gripper' else .2,command_timeout_s=.25)
        fields['max_velocity']['title']='目标限速 / '+('SDK 原始量/s' if unit=='sdk_gripper' else unit+'/s')
        fields['goal_max_velocity']['title']='控制器巡航速度 / '+('SDK 原始量/s' if unit=='sdk_gripper' else unit+'/s')
        params['goal_max_velocity']=params['max_velocity']
        fields['goal_velocity_tracking']['title']='按目标速度自动调整每个目标的巡航速度'
        params['goal_velocity_tracking']=False
        fields['max_command_lead']['title']='目标领先上限 / '+label_unit
        fields['command_interface']['title']='SDK 目标接口'
        fields['trajectory_scale']['title']='SDK 轨迹速度与加速度缩放'
        if role.endswith('_arm'):
            for name, default, maximum, title in (
                ('position_lead_m', .02, .25, '每拍平移目标允许超前 / m'),
                ('rotation_lead_rad', .35, 1., '每拍旋转目标允许超前 / rad'),
                ('damping_min', .005, 1., '奇异点外 DLS 阻尼'),
                ('damping_max', .145, 1., '奇异点处 DLS 阻尼'),
                ('singularity_threshold', .05, 1., '阻尼起作用的 σ_min 阈值'),
            ):
                fields[name] = dict(type='number', exclusiveMinimum=0, maximum=maximum, title=title)
                params[name] = default
        if role=='base':
            for name in ('max_velocity','goal_max_velocity','goal_velocity_tracking','max_command_lead'):
                fields.pop(name);params.pop(name)
        geometry={'frames':['base','flange','mount'],'sdk_component':sdk,'calibration_required':True,'collision_verified':False}
        if role.endswith('_arm'):geometry.update(controller_capabilities=['cartesian_ik'],solver_base_frame=role.split('_')[0]+'_base_link',solver_tip_frame=role.split('_')[0]+'_gripper_center')
        label={'left_arm':'左臂','right_arm':'右臂','left_gripper':'左夹爪','right_gripper':'右夹爪','head':'头部','lift':'升降','base':'底盘原生速度'}[role]
        descriptors.append(dict(descriptor_version=1,api_version='1.0',plugin_id=PREFIX+role,plugin_version='1.0.0',kind='actuator',ports=ports,joints=joints,command_modes=['velocity' if role=='base' else 'position'],backend_variants={'fake':{'available':True,'validation_level':'synthetic_sdk_protocol'},'real':{'available':True,'requires':['grpcio>=1.73.1','protobuf>=6.31,<7','MMK2 gRPC service'],'validation_level':'NOT_RUN_hardware_readonly_default'}},resources={'session_family':'mmk2_sdk','sharing':'session','stop_policy':'hold_and_zero_base'},operations={x:{'moves':False,'scope':'device_session'} for x in ('prepare','enable','stop')},geometry=geometry,configuration_schema={'type':'object','additionalProperties':False,'properties':fields},defaults=params,source={'package':'morla_mmk2_driver','version':'0.1.0'},web={'device_category':'arm' if role.endswith('_arm') else 'end' if 'gripper' in role else 'whole','label':'MMK2 SDK '+label,'connection_fields':['host','port','robot_name','domain_id'],'fields':[name for name in fields if name not in ('position_lead_m','damping_min','damping_max','singularity_threshold')],'preferred_controllers':{'axes_buttons':'morla.mmk2_sdk.base_twist'} if role=='base' else {'pose6d':'morla.mmk2.cartesian_delta'} if role.endswith('_arm') else {}}))
    camera_connection={k:v for k,v in connection.items() if k not in ('allow_motion','mapping_verified','hold_stop_accepted')}
    camera_defaults={k:v for k,v in defaults.items() if k in camera_connection}
    descriptors.append(dict(descriptor_version=1,api_version='1.0',plugin_id=PREFIX+'camera',plugin_version='1.0.0',kind='sensor',ports={'rgb':port('output','image',3,'rgb',codec='image',dtype='uint8'),'depth':port('output','image',1,'sdk_depth',codec='image',dtype='uint16')},joints=[],backend_variants={'fake':{'available':True},'real':{'available':True,'requires':['grpcio','protobuf','Pillow']}},resources={'session_family':'mmk2_sdk','sharing':'session'},geometry={'frames':['optical'],'calibration_required':True},configuration_schema={'type':'object','additionalProperties':False,'properties':dict(camera_connection,camera={'type':'string','enum':['head_camera','left_camera','right_camera']},fps={'type':'number','minimum':1,'maximum':30},depth_scale_m={'type':'number','minimum':0,'maximum':1})},defaults=dict(camera_defaults,camera='head_camera',fps=10.,depth_scale_m=0.),source={'package':'morla_mmk2_driver','version':'0.1.0'},web={'label':'MMK2 SDK 相机','connection_fields':['host','port','robot_name','domain_id']}))
    descriptors.append(dict(descriptor_version=1,api_version='1.0',plugin_id='morla.mmk2_sdk.base_twist',plugin_version='1.0.0',kind='controller',ports={},controller_types={'inputs':['axes_buttons'],'outputs':['joint_targets'],'target_plugins':[PREFIX+'base'],'target_dim':2},backend_variants={'fake':{'available':True},'real':{'available':True}},defaults={'linear_speed':.05,'angular_speed':.15},configuration_schema={'type':'object','additionalProperties':False,'properties':{'linear_speed':{'type':'number','exclusiveMinimum':0,'maximum':.15},'angular_speed':{'type':'number','exclusiveMinimum':0,'maximum':.3}}},source={'package':'morla_mmk2_driver','version':'0.1.0'},web={'label':'MMK2 SDK 底盘速度'}))
    descriptors.append(dict(
        descriptor_version=1, api_version='1.0', plugin_id='morla.mmk2_sdk.lift_button_jog',
        plugin_version='1.0.0', kind='controller', ports={},
        controller_types={'inputs': ['button_pair'], 'outputs': ['joint_targets'],
                          'target_plugins': [PREFIX + 'lift'], 'target_dim': 1},
        backend_variants={'fake': {'available': True}, 'real': {'available': True}},
        defaults={'speed': .01},
        configuration_schema={'type': 'object', 'additionalProperties': False, 'properties': {
            'speed': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': .02, 'title': '升降点动速度 / m/s'},
        }},
        source={'package': 'morla_mmk2_driver', 'version': '0.1.0'},
        web={'label': 'MMK2 升降按键点动'},
    ))
    descriptors[0]['web']['templates']=[{'id':'mmk2-sdk-fake','title':'AIRBOT MMK2 SDK 整机 / Quest（协议测试）','package':'morla_mmk2_driver','path':'config/quest.fake.yaml'},{'id':'mmk2-sdk-real','title':'AIRBOT MMK2 SDK 整机（真实设备，默认只读）','package':'morla_mmk2_driver','path':'config/assembly.real.yaml'}]
    quest = json.loads((ROOT / 'config/quest.fake.yaml').read_text())
    for binding in quest['bindings']:
        if binding['controller'] == 'morla.mmk2.cartesian_delta':
            role = binding['target'].split('.')[0]
            descriptor = next(d for d in descriptors if d['plugin_id'] == PREFIX + role)
            descriptor['web']['binding_presets'] = [
                dict(controller=binding['controller'], source_plugin='morla.input.quest', defaults=binding['parameters'])
            ]
    return descriptors


if __name__=='__main__':(ROOT/'descriptors.json').write_text(json.dumps(generate(),indent=2,ensure_ascii=False)+'\n')
