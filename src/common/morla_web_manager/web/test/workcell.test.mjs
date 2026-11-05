import test from 'node:test'
import assert from 'node:assert/strict'
import { readFile } from 'node:fs/promises'
import ts from 'typescript'
const source=await readFile(new URL('../src/lib/workcell.ts',import.meta.url),'utf8')
const javascript=ts.transpileModule(source,{compilerOptions:{module:ts.ModuleKind.ES2022,target:ts.ScriptTarget.ES2020}}).outputText
const model=await import('data:text/javascript;base64,'+Buffer.from(javascript).toString('base64'))
const arm={plugin_id:'morla.dobot.cr5',joints:Array.from({length:6},(_,i)=>({name:'joint'+(i+1),lower:-2,upper:2}))}

test('FR3 offers its installed common IK and joint jog, not CR5 controllers',()=>{
 const target={semantic:'joint_targets',dim:6,unit:'rad'}
 const catalog=[{kind:'controller',plugin_id:'morla.cr5.cartesian_delta',controller_types:{inputs:['pose6d'],outputs:['joint_targets']}}]
 assert.deepEqual(model.compatible(catalog,{semantic:'pose6d',dim:7,unit:'m_rad'},target,'morla.fairino.fr3'),['morla.cartesian_delta_ik'])
 assert.ok(model.compatible(catalog,{semantic:'axes_buttons',dim:6},target,'morla.fairino.fr3').includes('morla.joint_jog'))
 assert.equal(model.label('morla.fairino.fr3'),'FAIRINO FR3')
})

test('Cartesian jog defaults generate a named map, never joint-jog parameters',()=>{
 const defaults=model.bindingDefaults('morla.cr5.cartesian_jog',{axis_names:['x','y','z','roll','pitch','yaw']},arm)
 assert.deepEqual(defaults.axes_map,{x:'x',y:'y',z:'z',roll:'roll',pitch:'pitch',yaw:'yaw'})
 assert.equal(defaults.ik_mode,'differential');assert.equal(defaults.max_joint_velocity,.3)
 assert(!('speed' in defaults));assert.equal(defaults.linear_speed,.02)
})
test('joint jog has one assignment per joint, leaving unavailable axes unassigned',()=>{
 const defaults=model.bindingDefaults('morla.joint_jog',{axis_names:['left_x','left_y']},arm)
 assert.deepEqual(defaults.axes_map,['left_x','left_y',null,null,null,null]);assert.equal(defaults.speed,.25)
})
test('Quest pose mapping has an orthonormal left-handed basis',()=>{
 const p=model.bindingDefaults('morla.cr5.cartesian_delta',{},arm,true),m=p.input_rotation
 const determinant=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])
 assert.equal(determinant,-1);assert.deepEqual(p.solver_tip_from_tool,model.identity())
})
test('XYZ and RPY forms round-trip to normalized quaternion and TCP matrix',()=>{
 for(const pose of [[.02,-.03,.4,20,-30,70],[0,0,0,0,90,0],[-.5,1.5,2,-45,60,-120]]){
  const transform=model.poseToTransform(pose);assert(Math.abs(transform.slice(3).reduce((sum,v)=>sum+v*v,0)-1)<1e-12)
  const matrix=model.transformToMatrix(transform),roundtrip=model.transformToMatrix(model.matrixToTransform(matrix))
  for(let i=0;i<4;i++)for(let j=0;j<4;j++)assert(Math.abs(matrix[i][j]-roundtrip[i][j])<1e-7)
  assert.deepEqual(matrix.slice(0,3).map(row=>row[3]),pose.slice(0,3))
 }
})
test('new configurations are explicitly kinematic, imported defaults are preserved',()=>{
 const fresh=model.initial();assert.equal(fresh.metadata.simulation.mode,'kinematic')
 const legacy={...fresh,metadata:undefined,legacy:{keep:true},capture:{...fresh.capture,hold_frames:7}}
 const restored=model.normalise(legacy);assert.equal(restored.metadata,undefined);assert.equal(restored.capture.hold_frames,7);assert.deepEqual(restored.legacy,{keep:true})
})
test('controller suggestions respect dimensions and scalar target restrictions',()=>{
 const scalar={semantic:'scalar',dim:1,unit:'ratio'},joint={semantic:'joint_targets',dim:6,unit:'rad'}
 assert(!model.compatible([],scalar,joint).includes('morla.scalar_map'))
 assert(!model.compatible([],scalar,joint).includes('morla.identity'))
 assert(model.compatible([],{semantic:'axes_buttons',dim:6,unit:'ratio'},joint).includes('morla.joint_jog'))
})

test('pinch and trigger defaults match their actual controller parameters',()=>{
 const gripper={joints:[{lower:0,upper:.05}]}
 const pinch=model.bindingDefaults('morla.pinch_scalar',{},gripper)
 assert.deepEqual(pinch.pinch_range_m,[.015,.08]);assert(!('input_range' in pinch));assert.deepEqual(pinch.output_range,[0,.05])
 const trigger=model.bindingDefaults('morla.trigger_filtered',{},gripper)
 assert.equal(trigger.trigger_deadzone,.05);assert.equal(trigger.trigger_invert,false);assert(!('input_range' in trigger))
})

test('file import rejects malformed values before rendering, preserves extra fields',()=>{
 const a=model.initial();a.extra={preserved:true}
 assert.equal(model.parseWorkcellDocument({assembly:a,deployment:{metadata:{asset:'bench'}}}).assembly.extra.preserved,true)
 for(const bad of [null,{...a,instances:{arm:null}},{...a,capture:{...a.capture,images:[]}},{...a,bindings:[null]},{...a,mounts:[{child:'arm.base',parent:'world',transform:[NaN,0,0,0,0,0,1]}]}])assert.throws(()=>model.parseWorkcellDocument(bad))
})

test('shipped Vader maps migrate to M3/A/B/R3/X, custom maps remain unchanged',()=>{
 const old=model.initial();old.instances.pad={plugin:'morla.input.vader5',version:'1.0.0',parameters:{deadman_button:'M1',action_buttons:{Z:'teleop_start',C:'teleop_pause',Y:'record_start',X:'record_stop'}}}
 const next=model.normalise(old)
 assert.equal(next.instances.pad.parameters.deadman_button,'M3')
 assert.deepEqual(next.instances.pad.parameters.action_buttons,{M3:'teleop_start',A:'record_or_release',B:'record_stop',R3:'record_discard',X:'workpos'})
 assert.equal(old.instances.pad.parameters.deadman_button,'M1')
 old.instances.pad.parameters.action_buttons={M4:'teleop_start'}
 assert.deepEqual(model.normalise(old).instances,old.instances)
})

test('Quest hides actuator parameters and exposes only the selected protocol port', () => {
 const properties = Object.fromEntries([
  'side', 'protocol', 'pinch_range_m', 'initial_position', 'initial_positions',
  'rate_hz', 'max_sample_age_ms', 'can_channel', 'board_scope_verified', 'udp_port', 'tcp_port',
 ].map(key => [key, {}]))
 const quest = {plugin_id:'morla.input.quest', kind:'input', configuration_schema:{properties}}
 assert.deepEqual(Object.keys(model.visibleDeviceFields(quest, 'fake')), ['side', 'protocol', 'pinch_range_m'])
 assert.deepEqual(Object.keys(model.visibleDeviceFields(quest, 'real', {protocol:'tcp_wired'}, undefined, true)), ['tcp_port'])
 assert.deepEqual(Object.keys(model.visibleDeviceFields(quest, 'real', {protocol:'udp'}, undefined, true)), ['udp_port'])
 const nero = {...quest, plugin_id:'morla.nero.arm', kind:'actuator'}
 assert.ok('initial_position' in model.visibleDeviceFields(nero, 'real', undefined, undefined, true))
 assert.ok('can_channel' in model.visibleDeviceFields(nero, 'real', undefined, undefined, true))
})

test('controllers respect robot identity as well as port semantics', () => {
 const pose={semantic:'pose6d',dim:7,unit:'m_rad'}
 const landmarks={semantic:'hand_landmarks',dim:63}
 const joints={semantic:'joint_targets',dim:6,unit:'rad'}
 const catalog=[{kind:'controller',plugin_id:'morla.cr5.cartesian_delta',controller_types:{inputs:['pose6d'],outputs:['joint_targets']}}]
 assert.deepEqual(model.compatible(catalog,pose,joints,'morla.dobot.cr5'),['morla.cr5.cartesian_delta'])
 assert.equal(model.compatible(catalog,landmarks,joints,'morla.dobot.cr5').length,0)
 assert.ok(model.compatible(catalog,pose,{...joints,dim:7},'morla.nero.arm').includes('morla.cartesian_delta_ik'))
 assert.deepEqual(model.compatible(catalog,landmarks,{...joints,dim:12},'morla.xhand.hand'),['morla.xhand.retarget'])
})

test('camera fields exclude arm and joint configuration in every backend', () => {
 const properties=Object.fromEntries(['side','rate_hz','initial_position','initial_positions','width','height','fps','camera_backend','image_topic','compressed','device_path'].map(key=>[key,{}]))
 const camera={plugin_id:'morla.camera',kind:'sensor',configuration_schema:{properties}}
 for(const backend of ['fake','sim','real']) {
  for(const connection of [true,false]) {
   const fields=model.visibleDeviceFields(camera,backend,undefined,undefined,connection)
   for(const key of ['side','initial_position','initial_positions','rate_hz'])assert.ok(!(key in fields))
  }
 }
 assert.ok('device_path' in model.visibleDeviceFields(camera,'real',{camera_backend:'uvc'},undefined,true))
})

test('public Cartesian IK diagnostics show nested solver residuals and preserve CR5 flat reports',()=>{
 const common=model.presentIkReport({stage:'solved',solver_report:{position_error_m:.0001,rotation_error_rad:.002}})
 assert.equal(common.success,true);assert.equal(common.position_error_m,.0001);assert.equal(common.angle_error_rad,.002)
 const flat=model.presentIkReport({success:false,position_error_m:.01,angle_error_rad:.02})
 assert.equal(flat.success,false);assert.equal(flat.position_error_m,.01);assert.equal(flat.angle_error_rad,.02)
 assert.equal(model.presentIkReport({stage:'held'}).success,undefined)
})


test('FR3 dedicated URDF controllers are robot-specific with differential and lead defaults',()=>{
 const catalog=['morla.fr3.cartesian_delta','morla.fr3.cartesian_jog'].map(plugin_id=>({kind:'controller',plugin_id,controller_types:{inputs:[plugin_id.endsWith('delta')?'pose6d':'axes_buttons'],outputs:['joint_targets']}}))
 const target={semantic:'joint_targets',dim:6,unit:'rad'}
 assert(model.compatible(catalog,{semantic:'pose6d',dim:7,unit:'m_rad'},target,'morla.fairino.fr3').includes('morla.fr3.cartesian_delta'))
 assert(!model.compatible(catalog,{semantic:'pose6d',dim:7,unit:'m_rad'},target,'morla.dobot.cr5').includes('morla.fr3.cartesian_delta'))
 for(const id of ['morla.fr3.cartesian_delta','morla.fr3.cartesian_jog']){
  const p=model.bindingDefaults(id,{axis_names:['x','y','z','roll','pitch','yaw']},{...arm,plugin_id:'morla.fairino.fr3'},true)
  assert.equal(p.ik_mode,'differential');assert.equal(p.command_lead_rad,.08);assert.equal(p.max_joint_velocity,.3)
 }
})


test('recording completion follows validation status even when failure detail is empty',()=>{
 assert.equal(model.recordingSaveMessage({status:'complete',failure:undefined}),'数据记录已保存。')
 assert.match(model.recordingSaveMessage({status:'failed',failure:undefined}),/未通过完整性检查/)
 assert.match(model.recordingSaveMessage({status:'complete',failure:'queue overflow'}),/未通过完整性检查/)
})
