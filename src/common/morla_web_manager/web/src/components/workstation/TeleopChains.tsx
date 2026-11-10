import { InlineIssue, BasisFields, NumberInput, ObjectFields, PoseFields, ValueField } from '../WorkcellFields'
import { parameterSchema } from '../../lib/workcell'

import { capSummary } from '../../lib/workstationPresentation'

import type { WorkstationModel } from '../ComposableTab'
import { Section, Check } from './shared'
export function TeleopChains({actuators, addBinding, assembly, catalog, changed, choices, deployment, descriptor, device, endpoints, holdPort, inputs, isQuest, label, locked, patchBinding, portName, questVisible, selectedBinding, setSelectedChain, setSelectedDevice, setStage, signalLabel, sources, stage, standardHold, targets, updateBinding}:Pick<WorkstationModel,'actuators'|'addBinding'|'assembly'|'catalog'|'changed'|'choices'|'deployment'|'descriptor'|'device'|'endpoints'|'holdPort'|'inputs'|'isQuest'|'label'|'locked'|'patchBinding'|'portName'|'questVisible'|'selectedBinding'|'setSelectedChain'|'setSelectedDevice'|'setStage'|'signalLabel'|'sources'|'stage'|'standardHold'|'targets'|'updateBinding'>){return <>{stage==='control'&&<><div className="wc-chain-add"><button className="wc-primary" disabled={locked||!sources.length||!targets.length} onClick={addBinding}>＋ 添加控制映射</button></div><div className="wc-chain-list">{assembly.bindings.map(b=><button key={b.id} aria-pressed={selectedBinding===b.id} onClick={()=>setSelectedChain(b.id)}>{b.source.split('.')[0]} → {label(b.controller)} → {b.target.split('.')[0]}</button>)}</div><fieldset disabled={locked} className="wc-unframed">
   <Section title="遥操控制链" note="为执行器配置控制模式、遥操输入和跟随门控。切换输入或控制模式后，请核对坐标标定。">

    {assembly.bindings.map((binding,index)=>{const source=endpoints.find(e=>e.ref===binding.source),target=assembly.instances[binding.target.split('.')[0]],d=target&&descriptor(target),controllerDescriptor=catalog.find(c=>c.plugin_id===binding.controller),family=controllerDescriptor?.web?.control_family,cartesian=family?.startsWith('cartesian'),jog=family==='cartesian_jog',joint=family==='joint_jog',params=binding.parameters
     const available=sources.filter(e=>questVisible(e)&&!holdPort(e)&&!portName(e).endsWith('_buttons')&&choices(e.ref,binding.target).length>0)
     const hold=source&&standardHold(source)
     const targetOptions=actuators.flatMap(([id])=>{
      if(assembly.bindings.some(other=>other!==binding&&other.target.split('.')[0]===id))return []
      const ports=targets.filter(t=>t.instance===id)
      const compatiblePorts=ports.filter(t=>choices(binding.source,t.ref).length>0)
      const primary=ports.find(t=>t.ref===binding.target)||compatiblePorts.find(t=>['closure','position_target'].includes(portName(t)))||compatiblePorts[0]
      return primary?[primary]:[]
     })
     const setParams=(parameters:Record<string,unknown>)=>patchBinding(index,{parameters})
     const deviceIk=!!catalog.find(c=>c.plugin_id===binding.controller)?.web?.ik_modes?.length
     const common=cartesian?['position_scale','rotation_scale','smoothing_alpha','linear_speed','angular_speed',...(deviceIk?['max_joint_velocity']:[])].filter(key=>jog?!['position_scale','rotation_scale','smoothing_alpha'].includes(key):!['linear_speed','angular_speed'].includes(key)):joint?['speed']:[]
     const hidden=['input_position_rotation','input_rotation','input_basis','solver_tip_from_tool','axes_map',...common]
     const advanced=deviceIk?['command_lead_rad','max_joint_step','position_lead_m','rotation_lead_rad','damping_min','damping_max','singularity_threshold','differential_step_rad']:[]
     return binding.id===selectedBinding&&<article className="wc-device" key={binding.id}><div className="wc-device-heading"><h4>{label(binding.controller)} <code>{binding.id}</code></h4><button className="wc-text-button wc-destructive" onClick={()=>changed({...assembly,bindings:assembly.bindings.filter((_,i)=>i!==index)})}>移除映射</button></div><div className="wc-form-grid">
      <label data-field="target"><InlineIssue name="target" />控制目标<select aria-label={binding.id+'控制目标'} value={binding.target} onChange={e=>updateBinding(index,'target',e.target.value)}>{targetOptions.map(e=>{
       const item=assembly.instances[e.instance]
       return <option key={e.ref} value={e.ref}>{item.display_name||label(item.plugin)} · {e.instance}</option>
      })}</select></label>
      <label data-field="controller"><InlineIssue name="controller" />控制模式<select value={binding.controller} onChange={e=>void updateBinding(index,'controller',e.target.value)}>{[...new Set([binding.controller,...available.flatMap(e=>choices(e.ref,binding.target))])].map(id=><option key={id} value={id}>{(choices(binding.source,binding.target).includes(id)||available.some(e=>choices(e.ref,binding.target).includes(id)))?label(id):label(id)+'（已有配置，请核对）'}</option>)}</select></label>
      <label data-field="source"><InlineIssue name="source" />遥操输入<select aria-label={binding.id+'输入信号'} value={binding.source} onChange={e=>updateBinding(index,'source',e.target.value)}>{!source&&<option value={binding.source}>{binding.source}（待修正）</option>}{sources.filter(e=>e.ref===binding.source||available.includes(e)).map(e=><option key={e.ref} value={e.ref}>{signalLabel(e)}</option>)}</select></label>
      <label>跟随门控<select value={!binding.deadman?'continuous':hold?.ref===binding.deadman?'hold':'custom'} onChange={e=>{
       if(e.target.value==='continuous')patchBinding(index,{deadman:''})
       if(e.target.value==='hold'&&hold)patchBinding(index,{deadman:hold.ref})
      }}><option value="continuous">持续跟随</option>{hold&&<option value="hold">{source&&isQuest(source)?'按住跟随':'按住跟随'}</option>}{binding.deadman&&hold?.ref!==binding.deadman&&<option value="custom">已有自定义门控</option>}</select></label>
     </div>
     <div className="wc-cap-summary">{capSummary(binding,assembly,deployment,catalog).map(cap=><div key={cap.key}><strong>{cap.title} · {cap.effective??'检查后查看'} {cap.unit}</strong><span>绑定 {String(cap.bound??'未设置')} · 设备 {String(cap.source.value??'未设置')}（{cap.source.label}）</span><button onClick={()=>{setSelectedDevice(cap.id);setStage(cap.source.label==='部署覆盖'?'geometry':'devices')}}>查看设备上限</button><small>配置约束，不代表实测速度或跟随精度。</small></div>)}</div>
     <p className="wc-note">{binding.deadman
      ? '按住跟随接受当前输入的控制键，以及工作站的公共按住信号。键盘 S 需在运行页启用键盘控制。'
      : '开始遥操后持续跟随，工作站暂停时停止。'}
     </p>
     {binding.deadman&&<p className="wc-note">跟随按键 · {[
      ...inputs.filter(([,item])=>item.plugin==='morla.input.quest').map(()=> 'Quest 右 Grip / 捏合'),
      ...inputs.filter(([,item])=>item.plugin==='morla.input.vader5').map(([id,item])=>'Vader '+String(device(id).parameters?.deadman_button||item.parameters?.deadman_button||descriptor(item)?.defaults?.deadman_button||'M3')),
      ...(source&&isQuest(source)&&portName(source).startsWith('left_')?['Quest 左 Grip / 捏合']:[]),
      '键盘 S',
     ].join(' · ')}</p>}
     <details className="wc-details"><summary>自定义门控</summary><label data-field="deadman"><InlineIssue name="deadman" />门控端口<input value={binding.deadman||''} onChange={e=>patchBinding(index,{deadman:e.target.value})} /></label></details>
     <details className="wc-details"><summary>输入时效与求解设置</summary><div className="wc-form-grid">
      <label data-field="max_age_ms"><InlineIssue name="max_age_ms" />输入有效期 / ms<NumberInput label={binding.id+'输入有效期'} schema={{minimum:1}} value={binding.max_age_ms||250} onChange={max_age_ms=>patchBinding(index,{max_age_ms})} /></label>
      {deviceIk&&<ValueField name="ik_mode" value={params.ik_mode||'differential'} schema={parameterSchema('ik_mode')} onChange={v=>setParams({...params,ik_mode:v})} />}
     </div></details>
     {common.length>0&&<ObjectFields value={Object.fromEntries(Object.entries(params).filter(([key])=>common.includes(key)))} defaults={controllerDescriptor?.defaults} properties={Object.fromEntries(common.map(key=>[key,{...parameterSchema(key),...controllerDescriptor?.configuration_schema?.properties?.[key]}]))} onChange={value=>{const next={...params};for(const key of common)delete next[key];setParams({...next,...value})}} />}
     {(jog||joint)&&<fieldset className="wc-pose"><legend>摇杆轴分配</legend><div className="wc-form-grid">{(joint?(d?.joints||[]).map(j=>j.name):['x','y','z','roll','pitch','yaw']).map((axis,i)=>{const map=params.axes_map as string[]|Record<string,string|null>|undefined;const current=joint?Array.isArray(map)?map[i]:null:!Array.isArray(map)?map?.[axis]:null;return <label key={axis}>{joint?`关节 ${i+1}`:({x:'沿 X 平移',y:'沿 Y 平移',z:'沿 Z 平移',roll:'绕 X 转动',pitch:'绕 Y 转动',yaw:'绕 Z 转动'}[axis])}<select aria-label={binding.id+' '+axis+'输入轴'} value={current||''} onChange={e=>{const next=joint?Array.from({length:d?.joints?.length||1},(_,j)=>Array.isArray(map)?map[j]??null:null):{...(!Array.isArray(map)?map:{})};if(Array.isArray(next))next[i]=e.target.value||null;else next[axis]=e.target.value||null;setParams({...params,axes_map:next})}}><option value="">不使用</option>{(source?.port.axis_names||[]).map(name=><option key={name}>{name}</option>)}</select></label>})}</div></fieldset>}
     {(cartesian||Boolean(params.input_basis))&&<BasisFields label={cartesian?'输入方向与基座坐标系':'手部输入坐标系'} value={(params.input_rotation||params.input_basis) as number[][]|undefined} onChange={v=>setParams({...params,[cartesian?'input_rotation':'input_basis']:v})} />}
     {cartesian&&params.input_position_rotation!==undefined&&<BasisFields label="平移输入轴映射（仅影响位移）" value={params.input_position_rotation as number[][]} onChange={v=>setParams({...params,input_position_rotation:v})} />}
     {cartesian&&<PoseFields label="工具中心点 TCP 相对求解末端" matrix value={params.solver_tip_from_tool as number[][]|undefined} onChange={v=>setParams({...params,solver_tip_from_tool:v})} />}
     {cartesian&&<Check text="已核对实际输入方向与工具坐标系（仅记录）" value={!!params.calibration_verified} onChange={v=>setParams({...params,calibration_verified:v})} />}
     <details className="wc-details" open={!cartesian&&!joint}><summary>{cartesian?'求解与跟随参数':'控制参数'}</summary><ObjectFields value={Object.fromEntries(Object.entries(params).filter(([key])=>!hidden.includes(key)&&key!=='calibration_verified'&&key!=='ik_mode'))} defaults={controllerDescriptor?.defaults} properties={Object.fromEntries(Object.entries({...Object.fromEntries([...advanced,...Object.keys(params).filter(key=>['input_range','output_range','pinch_range_m'].includes(key))].map(key=>[key,parameterSchema(key)])),...controllerDescriptor?.configuration_schema?.properties}).filter(([key])=>!hidden.includes(key)&&key!=='calibration_verified'&&key!=='ik_mode'))} onChange={value=>{const keep=Object.fromEntries(Object.entries(params).filter(([key])=>hidden.includes(key)||key==='calibration_verified'||key==='ik_mode'));setParams({...keep,...value})}} allowNew={!cartesian&&!joint} /></details>
     </article>})}
    {inputs.some(([,item])=>item.plugin==='morla.input.quest')&&<details className="wc-details"><summary>Quest 工作站快捷键</summary><p className="wc-note">右 Grip 按下开始遥操或接管。右 A 开始录制或交还控制，右 B 结束并保存，右摇杆按下丢弃当前录制，左 X 回工作位。也可在运行页点击「开始遥操」或按 S。</p></details>}
    {!assembly.bindings.length&&<p className="wc-empty">尚无控制映射。先添加输入设备与执行器，再建立映射。</p>}
   </Section>
  </fieldset></>}</>}
