import type { Assembly, Binding, Component, Deployment } from './workcell'
export const stages=[{id:'devices',title:'设备选型',note:'选择设备与运行方式'},{id:'geometry',title:'连接与标定',note:'连接身份、安装与坐标'},{id:'control',title:'遥操配置',note:'输入 → 控制模式 → 执行器'},{id:'capture',title:'数据记录',note:'观测、动作与图像'},{id:'check',title:'检查',note:'检查配置并保存版本'},{id:'run',title:'运行',note:'初始化、遥操与运行反馈'}]
export function snapshot(assembly:Assembly,deployment:Deployment){return JSON.stringify({assembly,deployment},(_key,value)=>value&&typeof value==='object'&&!Array.isArray(value)?Object.fromEntries(Object.entries(value).sort(([a],[b])=>a.localeCompare(b))):value)}
export function parameterSource(key:string,explicit:Record<string,unknown>={},defaults:Record<string,unknown>={},overrides:Record<string,unknown>={}) {
 return key in overrides?{label:'部署覆盖',value:overrides[key]}:key in explicit?{label:'装配显式值',value:explicit[key]}:{label:'插件默认',value:defaults[key]}
}
const obsolete:Record<string,string[]>={'morla.dobot.cr5':['command_timeout_s','max_step_rad'],'morla.jodell.epg50_060':['max_step_m','command_timeout_s'],'morla.loophand.compact':['command_timeout_s','max_step_rad']}
export function obsoleteFields(assembly:Assembly,deployment:Deployment){
 return Object.entries(assembly.instances).flatMap(([id,item])=>{
  const keys=obsolete[item.plugin]||[]
  return (['assembly','deployment'] as const).flatMap(layer=>keys.filter(key=>key in (layer==='assembly'?item.parameters||{}:deployment.instances?.[id]?.parameters||{})).map(key=>({id,key,layer})))
 })
}
export function cleanObsolete(assembly:Assembly,deployment:Deployment){
 const a=structuredClone(assembly),d=structuredClone(deployment)
 for(const {id,key,layer} of obsoleteFields(a,d))delete (layer==='assembly'?a.instances[id].parameters!:d.instances![id].parameters!)[key]
 return {assembly:a,deployment:d}
}
export function changeBackend(assembly:Assembly,deployment:Deployment,id:string,backend:string){
 const a=structuredClone(assembly),d=structuredClone(deployment)
 a.instances[id].backend=backend
 if(d.instances?.[id])d.instances[id].backend=backend
 return {assembly:a,deployment:d}
}
export function preserveBindingParameters(old:Binding,next:Binding,catalog:Component[]) {
 if(old.controller===next.controller)return {...next,parameters:structuredClone(old.parameters)}
 const schema=catalog.find(c=>c.plugin_id===next.controller)?.configuration_schema?.properties||{}
 const supported=new Set([...Object.keys(schema),...Object.keys(next.parameters),'calibration_verified'])
 return {...next,parameters:{...next.parameters,...Object.fromEntries(Object.entries(old.parameters).filter(([key])=>supported.has(key)))}}
}
export function describeChanges(before:unknown,after:unknown):string {
 const lines:string[]=[]
 const walk=(a:unknown,b:unknown,path:string)=>{
  if(JSON.stringify(a)===JSON.stringify(b))return
  if(a&&b&&typeof a==='object'&&typeof b==='object'&&!Array.isArray(a)&&!Array.isArray(b)) {
   const x=a as Record<string,unknown>,y=b as Record<string,unknown>
   for(const key of new Set([...Object.keys(x),...Object.keys(y)]))walk(x[key],y[key],path?path+'.'+key:key)
  } else lines.push(`${path}：${JSON.stringify(a)??'未设置'} → ${JSON.stringify(b)??'移除'}`)
 }
 walk(before,after,'');return lines.join('\n')||'无参数变化'
}
export function issueLocation(path:string,assembly:Assembly) {
 const normalized=path.replace(/\//g,'.').replace(/^\./,'').replace(/^assembly\./,'')
 const parts=normalized.split('.'),last=parts[parts.length-1]||''
 const binding=assembly.bindings.find((b,i)=>parts.includes(b.id)||normalized.startsWith('bindings.'+i+'.')||[b.source,b.target,b.deadman].includes(path))
 if(binding){const field=(['source','target','deadman'] as const).find(key=>binding[key]===path)||last;return {stage:'control',id:binding.target.split('.')[0],binding:binding.id,field}}
 const mountIndex=parts.indexOf('mounts'),mount=mountIndex>=0?assembly.mounts[Number(parts[mountIndex+1])]:undefined
 const id=mount?.child.split('.')[0]||Object.keys(assembly.instances).find(id=>parts.includes(id))||''
 return {stage:/deployment|mount|calibration|control_groups/.test(normalized)?'geometry':/capture/.test(normalized)?'capture':'devices',id,binding:'',field:last}
}
export function capSummary(binding:Binding,assembly:Assembly,deployment:Deployment,catalog:Component[]) {
 if(!/^morla\.(cr5|fr3)\.cartesian_(delta|jog)$/.test(binding.controller))return []
 const id=binding.target.split('.')[0],item=assembly.instances[id]
 if(!item)return []
 const d=catalog.find(c=>c.plugin_id===item.plugin),c=catalog.find(c=>c.plugin_id===binding.controller)
 return [['max_velocity_rad_s','max_joint_velocity','速度约束','rad/s'],['max_command_lead_rad','command_lead_rad','领先约束','rad']].map(([deviceKey,key,title,unit])=>{
  const source=parameterSource(deviceKey,item.parameters,d?.defaults,deployment.instances?.[id]?.parameters)
  const bound=binding.parameters[key]??c?.defaults?.[key]
  return {id,deviceKey,key,title,unit,source,bound,effective:typeof source.value==='number'&&typeof bound==='number'?Math.min(source.value,bound):undefined}
 })
}
