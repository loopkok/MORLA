import test from 'node:test'
import assert from 'node:assert/strict'
import { readFile } from 'node:fs/promises'
import ts from 'typescript'
const source=await readFile(new URL('../src/lib/workcell.ts',import.meta.url),'utf8')
const javascript=ts.transpileModule(source,{compilerOptions:{module:ts.ModuleKind.ES2022,target:ts.ScriptTarget.ES2020}}).outputText
const model=await import('data:text/javascript;base64,'+Buffer.from(javascript).toString('base64'))
const arm={plugin_id:'morla.dobot.cr5',joints:Array.from({length:6},(_,i)=>({name:'joint'+(i+1),lower:-2,upper:2}))}

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
