export type Schema = { type?: string; title?: string; description?: string; enum?: (string | number | boolean)[]; minimum?: number; maximum?: number; exclusiveMinimum?: number; exclusiveMaximum?: number; minItems?: number; maxItems?: number; items?: Schema; properties?: Record<string, Schema>; default?: unknown }
export type Port = { direction: string; semantic: string; dim: number; unit: string; axis_names?: string[] }
export type Component = { plugin_id: string; plugin_version: string; kind: string; resources?: {sharing?: string}; ports: Record<string, Port>; joints?: { name: string; unit: string; lower?: number; upper?: number }[]; geometry?: { frames?: string[] }; defaults?: Record<string, unknown>; configuration_schema?: Schema; controller_types?: { inputs?: string[]; outputs?: string[] }; backend_variants?: Record<string, { available?: boolean }> }
export type Instance = { plugin: string; version: string; backend?: string; role?: string; parameters?: Record<string, unknown>; display_name?: string; [key: string]: unknown }
export type Binding = { id: string; source: string; target: string; deadman?: string; controller: string; max_age_ms?: number; input_basis_id?: string; parameters: Record<string, unknown>; [key: string]: unknown }
export type LogOptions = { enabled?:boolean; feedback_hz?:number; queue_size?:number; segment_mb?:number; max_run_mb?:number }
export type OperatorOptions = { work_positions?:Record<string,number[]>; work_speed?:number }
export type SimOptions = { mode: 'physics' | 'kinematic'; display: 'web' | 'local' | 'both' }
export type Mount = { child: string; parent: string; transform?: number[]; calibration?: string }
export type Group = { members: string[]; fault_policy: string; [key: string]: unknown }
export type Assembly = { assembly_version: number; assembly_id: string; revision: number; title?: string; metadata?: Record<string, unknown> & { simulation?: SimOptions; diagnostic_logging?:LogOptions; operator_control?:OperatorOptions }; instances: Record<string, Instance>; mounts: Mount[]; bindings: Binding[]; control_groups: Record<string, Group>; capture: { state: string[]; action: string[]; images: Record<string, string>; auxiliary?: string[]; action_source: string; dataset_fps?: number; [key: string]: unknown }; [key: string]: unknown }
export type Device = { backend?: string; device_id?: string; resource_key?: string; verified?: boolean; stop_verified?: boolean; parameters?: Record<string, unknown>; [key: string]: unknown }
export type Deployment = { deployment_version?: number; deployment_id?: string; instances?: Record<string, Device>; calibrations?: Record<string, unknown>; [key: string]: unknown }
export const initial = (): Assembly => ({ assembly_version: 1, assembly_id: 'my_workcell', revision: 1, metadata: { simulation: { mode: 'kinematic', display: 'web' } }, instances: {}, mounts: [], bindings: [], control_groups: {}, capture: { state: [], action: [], images: {}, auxiliary: [], action_source: 'command', dataset_fps: 30 } })
export function normalise(raw: Assembly): Assembly {
  const value = { ...initial(), ...raw, metadata: raw.metadata, capture: { ...initial().capture, ...raw.capture },instances:structuredClone(raw.instances) }
  for(const item of Object.values(value.instances)) {
    if(item.plugin!=='morla.input.vader5')continue
    const p=item.parameters||{},actions=p.action_buttons
    const legacy:Record<string,string>[]=[{START:'teleop_start',B:'teleop_pause',Y:'record_start',X:'record_stop'}, {Z:'teleop_start',B:'teleop_pause',Y:'record_start',X:'record_stop'}, {Z:'teleop_start',C:'teleop_pause',Y:'record_start',X:'record_stop'}]
    const equal=(a:unknown,b:Record<string,string>)=>!!a&&typeof a==='object'&&Object.keys(a).length===Object.keys(b).length&&Object.entries(b).every(([k,v])=>(a as Record<string,unknown>)[k]===v)
    if(actions===undefined||legacy.some(map=>equal(actions,map))) {
      if(actions!==undefined)p.action_buttons={M3:'teleop_start',A:'record_or_release',B:'record_stop',R3:'record_discard',X:'workpos'}
      if(p.deadman_button==='M1')p.deadman_button='M3'
      if(item.parameters)item.parameters=p
    }
  }
  return value
}
export const labels: Record<string, string> = {
  'morla.input.vader5': 'Vader 5 手柄', 'morla.input.quest': 'Quest 3', 'morla.input.gamepad': '通用手柄', 'morla.input.xbox': 'Xbox 手柄', 'morla.input.glove': '数据手套', 'morla.input.mobile': '手机体感', 'morla.input.standard': 'ROS 输入', 'morla.fake.input': '测试输入',
  'morla.dobot.cr5': 'Dobot CR5', 'morla.fairino.fr3': 'FAIRINO FR3', 'morla.astral.arm': 'Astral 机械臂', 'morla.nero.arm': 'Nero 机械臂', 'morla.astral.waist': 'Astral 腰部', 'morla.astral.head': 'Astral 头部', 'morla.fake.arm': '测试机械臂', 'morla.ros2_control.mock': 'ros2_control 测试臂', 'morla.astral.gripper': 'Astral 夹爪', 'morla.jodell.epg50_060': 'Jodell EPG50', 'morla.wuji.hand': 'Wuji 灵巧手', 'morla.xhand.hand': 'XHand 灵巧手', 'morla.loophand.compact': 'LoopHand', 'morla.camera': '场景相机', 'morla.camera.ros_image': 'ROS 相机', 'morla.camera.uvc': 'USB 相机', 'morla.fake.camera': '测试相机', 'morla.fake.sensor': '测试传感器',
  'morla.cr5.cartesian_delta': '末端位姿跟随 · CR5', 'morla.cr5.cartesian_jog': '末端速度控制 · CR5', 'morla.cartesian_delta_ik': '末端位姿跟随', 'morla.gamepad_cartesian_jog': '末端速度控制', 'morla.joint_jog': '关节点动', 'morla.identity': '直接传递', 'morla.scalar_map': '线性映射', 'morla.trigger_filtered': '扳机控制', 'morla.pinch_filtered': '捏合控制（滤波）', 'morla.pinch_scalar': '捏合控制', 'morla.xhand.retarget': 'XHand 手势重定向', 'morla.wuji.retarget': 'Wuji 手势重定向',
}
export const label = (key: string) => labels[key] || key
export const backendNames: Record<string, string> = { fake: '测试数据', sim: 'MuJoCo 仿真', real: '真实设备' }
export const portNames: Record<string, string> = { position: '关节位置', velocity: '关节速度', position_target: '位置指令', joint_target: '关节指令', right_pose: '右手位姿', left_pose: '左手位姿', right_buttons: '右手柄按键', left_buttons: '左手柄按键', right_hold: '右手控制键', left_hold: '左手控制键', axes: '摇杆轴', scalar: '扳机输入', closure: '夹爪开合', rgb: '彩色图像', hold_to_control: '使能按键', landmarks: '手部关键点', right_landmarks: '右手关键点', left_landmarks: '左手关键点' }
export function portLabel(ref: string) { const [id, port] = ref.split('.'); return `${id} · ${portNames[port] || port}` }
export const controllerCapabilities: Record<string, [string[], string[]]> = {
 'morla.pinch_filtered': [['hand_landmarks'], ['scalar','joint_targets']], 'morla.trigger_filtered': [['scalar'], ['scalar','joint_targets']], 'morla.pinch_scalar': [['hand_landmarks'], ['scalar','joint_targets']], 'morla.cartesian_delta_ik': [['pose6d'], ['joint_targets']], 'morla.gamepad_cartesian_jog': [['axes_buttons'], ['joint_targets']], 'morla.joint_jog': [['axes_buttons','joint_targets'], ['joint_targets']], 'morla.scalar_map': [['scalar'], ['scalar','joint_targets']], 'morla.identity': [['joint_targets','scalar','numeric'], ['joint_targets','scalar','numeric']], 'morla.xhand.retarget': [['hand_landmarks'], ['joint_targets']], 'morla.wuji.retarget': [['hand_landmarks'], ['joint_targets']],
}
export function compatible(catalog: Component[], source?: Port, target?: Port, targetPlugin?: string) {
 if (!source || !target) return []
 const caps = { ...controllerCapabilities, ...Object.fromEntries(catalog.filter(c => c.kind==='controller').map(c => [c.plugin_id, [c.controller_types?.inputs || [], c.controller_types?.outputs || []] as [string[], string[]]])) }
 return Object.entries(caps).filter(([id,[ins,outs]]) => ins.includes(source.semantic) && outs.includes(target.semantic) && (!['morla.scalar_map','morla.pinch_scalar','morla.pinch_filtered','morla.trigger_filtered'].includes(id) || target.dim===1) && (id!=='morla.identity' || source.dim===target.dim && source.unit===target.unit)).map(([id])=>id).filter(id => {
  if (!targetPlugin) return true
  if (id === 'morla.xhand.retarget') return targetPlugin === 'morla.xhand.hand'
  if (id === 'morla.wuji.retarget') return targetPlugin === 'morla.wuji.hand'
  if (id.startsWith('morla.cr5.')) return targetPlugin === 'morla.dobot.cr5'
  if (['morla.cartesian_delta_ik', 'morla.gamepad_cartesian_jog'].includes(id))
   return ['morla.nero.arm', 'morla.astral.arm', 'morla.fairino.fr3'].includes(targetPlugin)
  return true
 })
}
export const identity = () => [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
export function bindingDefaults(controller: string, source: Port, target: Component, quest = false): Record<string, unknown> {
 const axes=source.axis_names || []
 if (controller==='morla.joint_jog') return { axes_map: Array.from({length:target.joints?.length || 1},(_,i)=>axes[i] || null), speed:.25 }
 if (controller.includes('cartesian')) {
   const p: Record<string, unknown> = { input_rotation: quest ? [[0,0,-1],[1,0,0],[0,1,0]] : [[1,0,0],[0,1,0],[0,0,1]], solver_tip_from_tool: identity() }
   if (controller.endsWith('jog')) Object.assign(p,{axes_map:Object.fromEntries(['x','y','z','roll','pitch','yaw'].map((name,i)=>[name, axes.includes(name)?name:axes[i] || null])),linear_speed:.02,angular_speed:.15})
   else p.position_scale=.8
   if(controller.startsWith('morla.cr5.')) Object.assign(p,{ik_mode:'differential',max_joint_velocity:.3,smoothing_alpha:.35})
   return p
 }
 const output_range=[target.joints?.[0]?.lower ?? 0,target.joints?.[0]?.upper ?? 1]
 if(controller==='morla.scalar_map') return {input_range:[0,1],output_range}
 if(controller==='morla.pinch_scalar') return {pinch_range_m:[.015,.08],output_range}
 if(controller==='morla.trigger_filtered') return {trigger_invert:false,trigger_deadzone:.05,trigger_gamma:1.4,trigger_ema_alpha:.4,max_ratio_rate:2.5,output_range}
 if(controller==='morla.pinch_filtered') return {close_dist_m:.015,open_dist_m:.08,auto_range:true,auto_range_forget_s:8,auto_range_min_span_m:.01,ema_alpha:.4,max_ratio_rate:2.5,output_range}
 return {}
}
// Rz(yaw) Ry(pitch) Rx(roll); positions are metres, UI angles are degrees.
export function poseToTransform(pose: number[]) {
 const [x,y,z]=pose; const [r,p,yaw]=pose.slice(3).map(v=>v*Math.PI/180)
 const cr=Math.cos(r/2),sr=Math.sin(r/2),cp=Math.cos(p/2),sp=Math.sin(p/2),cy=Math.cos(yaw/2),sy=Math.sin(yaw/2)
 return [x,y,z,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
}
export function transformToPose(t: number[]) {
 if (t.length===16) { const m=t; return [m[3],m[7],m[11],Math.atan2(m[9],m[10])*180/Math.PI,Math.asin(Math.max(-1,Math.min(1,-m[8])))*180/Math.PI,Math.atan2(m[4],m[0])*180/Math.PI] }
 const [x,y,z,qx,qy,qz,qw]=t
 return [x,y,z,Math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))*180/Math.PI,Math.asin(Math.max(-1,Math.min(1,2*(qw*qy-qz*qx))))*180/Math.PI,Math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))*180/Math.PI]
}
export function transformToMatrix(t: number[]) {
 const [x,y,z,a,b,c,d]=t
 return [[1-2*(b*b+c*c),2*(a*b-c*d),2*(a*c+b*d),x],[2*(a*b+c*d),1-2*(a*a+c*c),2*(b*c-a*d),y],[2*(a*c-b*d),2*(b*c+a*d),1-2*(a*a+b*b),z],[0,0,0,1]]
}
export function matrixToTransform(m: number[][]) { return poseToTransform(transformToPose(m.flat())) }
export const parameterLabels: Record<string,string> = {
 rpc_timeout_s:'SDK 调用超时 / s',startup_timeout_s:'SDK 连接超时 / s',operation_timeout_s:'使能与停止超时 / s',feedback_timeout_s:'反馈有效期 / s',max_velocity_rad_s:'驱动关节速度上限 / rad/s',max_command_lead_rad:'目标相对实测领先上限 / rad',
 close_dist_m:'捏合闭合距离 / m',open_dist_m:'捏合张开距离 / m',auto_range:'自适应捏合范围',auto_range_forget_s:'捏合范围遗忘时间 / s',auto_range_min_span_m:'捏合最小跨度 / m',ema_alpha:'捏合滤波系数',trigger_invert:'反转扳机方向',trigger_deadzone:'扳机死区',trigger_gamma:'扳机响应曲线指数',trigger_ema_alpha:'扳机滤波系数',max_ratio_rate:'开合比例变化率 / s⁻¹',rate_hz:'控制频率 / Hz',host:'控制器 IP',port:'端口',port_name:'串口设备',device_path:'设备路径',serial_number:'序列号',device_id:'设备标识',can_interface:'CAN 驱动接口',firmware_version:'固件协议版本',native_j7_first:'原生 J7 优先排序',workpos_waypoints_deg:'驱动工作位途径点 / °',publish_rate:'状态发布频率 / Hz',filter_cutoff_freq:'反馈滤波截止频率 / Hz',control_board_ip:'控制板 IP',board_cmd_port:'控制板指令端口',local_ip:'本机 IP',local_port:'本机接收端口',can_channel:'CAN 通道',timeout_s:'通信超时 / s',enable_timeout_s:'使能超时 / s',command_timeout_s:'指令超时 / s',servo_t:'ServoJ 插补时间 / s',servo_lookahead_time:'ServoJ 前瞻平滑',servo_gain:'ServoJ 跟随增益',max_step_rad:'关节步长上限 / rad',joint_lower:'关节下限 / rad',joint_upper:'关节上限 / rad',urdf_joint_signs:'关节方向',urdf_joint_offsets:'关节零点偏置 / rad',initial_positions:'仿真初始关节位置 / rad',initial_position:'驱动初始关节位置 / rad',kinematics_verified:'已核对运动学约定',board_scope_verified:'已核对控制板范围',unselected_joints_policy:'未选中关节处理',side:'控制侧别',input_topic:'输入话题',input_basis_id:'输入坐标约定',image_topic:'图像话题',compressed:'压缩图像',width:'图像宽度 / px',height:'图像高度 / px',fps:'图像帧率 / Hz',process_mode:'输入接收进程',protocol:'通信协议',udp_port:'UDP 端口',tcp_port:'TCP 端口',deadzone:'摇杆死区',device_name:'设备名称',camera_backend:'相机接口',baudrate:'波特率',baud_rate:'波特率',address:'设备地址',hand_id:'手部地址',input_rotation:'输入坐标变换',solver_tip_from_tool:'工具中心点 TCP',position_scale:'位移倍率',smoothing_alpha:'输入滤波系数',max_joint_step:'关节步长上限 / rad',max_joint_velocity:'关节速度上限 / rad/s',linear_speed:'平移速度 / m/s',angular_speed:'转动速度 / rad/s',ik_mode:'逆运动学求解',position_lead_m:'位置目标领先上限 / m',rotation_lead_rad:'姿态目标领先上限 / rad',damping_min:'最小阻尼',damping_max:'最大阻尼',singularity_threshold:'奇异值阈值',differential_step_rad:'微分步长上限 / rad',speed:'关节点动速度 / rad/s',input_range:'输入范围',output_range:'输出范围',pinch_range_m:'捏合距离范围 / m',calibration_verified:'已核对输入方向与 TCP',input_basis:'手部坐标变换',max_age_ms:'输入有效期 / ms',max_sample_age_ms:'反馈有效期 / ms',retarget_backend:'手势重定向算法',preprocess_mode:'手部预处理',index_bend_sign:'食指弯曲方向',enable_thumb_fix:'拇指补偿',source_namespace:'输入命名空间',deadman_button:'使能按键',rotation_button:'旋转按键',action_buttons:'功能按键',axis_signs:'摇杆方向',resource_key:'设备资源标识',secret_ref:'凭据引用',serial:'设备序列号',bus:'通信总线',session_id:'共享驱动标识',resources:'占用资源',calibration:'标定记录',verified:'已核对设备身份',stop_verified:'已完成停止测试',
}
export const enumLabels: Record<string,string> = { left:'左侧',right:'右侧',both:'双侧',auto:'自动管理',managed:'由 MORLA 启动',external:'使用已有进程',differential:'连续微分跟随',strict:'严格位姿求解',udp:'UDP',tcp_wired:'TCP 有线',tcp_wireless:'TCP 无线',disabled:'禁用',held:'保持位置',official:'官方算法',dexpilot:'DexPilot',ros_image:'ROS 图像',uvc:'USB / UVC' }
export function parameterSchema(key: string): Schema {
 if(['input_range','output_range','pinch_range_m'].includes(key))return {type:'array',minItems:2,maxItems:2,items:{type:'number'}}
 if(key==='ik_mode') return {type:'string',enum:['differential','strict']}
 const defaults:Record<string,number>={position_scale:.8,max_joint_velocity:.3,smoothing_alpha:.35,max_joint_step:.15,linear_speed:.02,angular_speed:.15,speed:.25,position_lead_m:.05,rotation_lead_rad:.35,damping_min:.005,damping_max:.15,singularity_threshold:.05,differential_step_rad:.04}
 return key in defaults?{type:'number',default:defaults[key],minimum:0}:{}
}
export function parseWorkcellDocument(raw:unknown): {assembly:Assembly;deployment:Deployment} {
 const object=(v:unknown):v is Record<string,unknown> => !!v&&typeof v==='object'&&!Array.isArray(v)
 if(!object(raw))throw new Error('配置文件必须包含工作站对象。')
 const candidate=object(raw.assembly)?raw.assembly:raw
 const a:Record<string,unknown>={bindings:[],mounts:[],control_groups:{},...candidate}
 const strings=(v:unknown):v is string[]=>Array.isArray(v)&&v.every(x=>typeof x==='string')
 if(a.assembly_version!==1||typeof a.assembly_id!=='string'||!Number.isInteger(a.revision)||Number(a.revision)<1||!object(a.instances)||!Array.isArray(a.bindings)||!Array.isArray(a.mounts)||!object(a.control_groups)||!object(a.capture))throw new Error('配置缺少有效的设备、控制映射、安装关系或记录设置。')
 if(Object.values(a.instances).some(i=>!object(i)||typeof i.plugin!=='string')||a.bindings.some(b=>!object(b)||!['id','source','target','controller'].every(k=>typeof b[k]==='string')||!object(b.parameters)))throw new Error('设备或控制映射格式不正确。')
 if(a.mounts.some(m=>!object(m)||typeof m.child!=='string'||typeof m.parent!=='string'||m.transform!==undefined&&(!Array.isArray(m.transform)||![7,16].includes(m.transform.length)||m.transform.some(v=>typeof v!=='number'||!Number.isFinite(v)))))throw new Error('安装位姿必须包含有限数值的坐标变换。')
 if(!strings(a.capture.state)||!strings(a.capture.action)||!object(a.capture.images)||Object.values(a.capture.images).some(v=>typeof v!=='string')||a.capture.auxiliary!==undefined&&!strings(a.capture.auxiliary)||Object.values(a.control_groups).some(g=>!object(g)||!strings(g.members)))throw new Error('数据记录或控制组格式不正确。')
 if(raw.deployment!==undefined&&!object(raw.deployment))throw new Error('硬件部署配置必须是对象。')
 return {assembly:normalise(a as Assembly),deployment:(raw.deployment || {}) as Deployment}
}

const builtinDeviceFields: Record<string, string[]> = {
 'morla.fake.arm': ['rate_hz','initial_positions'],
 'morla.ros2_control.mock': ['rate_hz','initial_positions'],
 'morla.fake.sensor': ['rate_hz'],
 'morla.fake.input': ['rate_hz'],
 'morla.nero.arm': ['side','rate_hz','max_sample_age_ms','can_channel','can_interface','firmware_version','native_j7_first','initial_position','workpos_waypoints_deg','initial_positions'],
 'morla.astral.arm': ['side','rate_hz','max_sample_age_ms','control_board_ip','local_ip','local_port','board_cmd_port','board_scope_verified','unselected_joints_policy','initial_positions'],
 'morla.astral.gripper': ['side','rate_hz','max_sample_age_ms','control_board_ip','local_ip','local_port','board_cmd_port','board_scope_verified','unselected_joints_policy','initial_positions'],
 'morla.xhand.hand': ['side','rate_hz','max_sample_age_ms','port_name','hand_id','initial_positions','preprocess_mode','index_bend_sign','enable_thumb_fix'],
 'morla.wuji.hand': ['side','rate_hz','max_sample_age_ms','serial_number','publish_rate','filter_cutoff_freq','initial_positions','retarget_backend'],
 'morla.input.quest': ['side','process_mode','protocol','udp_port','tcp_port','input_topic','source_namespace','pinch_range_m'],
 'morla.input.gamepad': ['process_mode','input_topic','device_name','deadzone'],
 'morla.input.xbox': ['process_mode','input_topic','device_name','deadzone'],
 'morla.input.glove': ['side','process_mode','input_topic','source_namespace'],
 'morla.input.mobile': ['process_mode','input_topic'],
}
const cameraFields = ['width','height','fps','camera_backend','image_topic','compressed','device_path']
const hardwareFields = new Set([
 'can_channel','can_interface','firmware_version','native_j7_first','initial_position','workpos_waypoints_deg',
 'control_board_ip','local_ip','local_port','board_cmd_port','board_scope_verified','unselected_joints_policy',
 'serial_number','publish_rate','filter_cutoff_freq','port_name','hand_id',
 'udp_port','tcp_port','input_topic','source_namespace','host','port','device_path','baudrate','baud_rate','address',
])
export function visibleDeviceFields(
 component: Component, backend: string,
 parameters?: Record<string, unknown>, deployment?: Record<string, unknown>,
 connection = false,
): Record<string, Schema> {
 const plugin=component.plugin_id
 const settings={...component.defaults,...parameters,...deployment}
 const camera=plugin.startsWith('morla.camera')||plugin==='morla.fake.camera'
 const allowed=camera?cameraFields:builtinDeviceFields[plugin]
 return Object.fromEntries(Object.entries(component.configuration_schema?.properties||{}).filter(([key])=>{
  if(allowed&&!allowed.includes(key))return false
  if(key==='device_id')return false
  if(['morla.astral.head','morla.astral.waist'].includes(plugin)&&!['rate_hz','max_sample_age_ms','initial_positions','control_board_ip','local_ip','local_port','board_cmd_port'].includes(key))return false
  if(key==='initial_positions')return backend!=='real'&&!connection&&component.kind==='actuator'
  if(hardwareFields.has(key)) {
   if(backend!=='real'||!connection)return false
   if(plugin==='morla.input.quest') {
    const tcp=String(settings.protocol||'udp').startsWith('tcp')
    if(key==='udp_port')return !tcp
    if(key==='tcp_port')return tcp
   }
   if(camera&&key==='device_path')return settings.camera_backend==='uvc'||plugin==='morla.camera.uvc'
   return true
  }
  if(camera) {
   const uvc=settings.camera_backend==='uvc'||plugin==='morla.camera.uvc'
   if(['image_topic','compressed'].includes(key))return backend==='real'&&!uvc&&connection
   if(key==='camera_backend')return backend==='real'&&!connection
  }
  if(backend==='real'&&key in (deployment||{}))return connection
  return !connection
 }))
}

// Public CartesianController nests device solver details; CR5 reports them flat.
export function presentIkReport(raw: Record<string, unknown>): Record<string, unknown> {
 const nested=raw.solver_report&&typeof raw.solver_report==='object'?raw.solver_report as Record<string,unknown>:{}
 return {...nested,...raw,angle_error_rad:raw.angle_error_rad??nested.angle_error_rad??nested.rotation_error_rad,
  success:raw.success??(raw.stage==='solved'?true:undefined)}
}
