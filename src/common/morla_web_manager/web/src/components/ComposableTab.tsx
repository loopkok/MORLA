import { useEffect, useRef, useState } from 'react'
import { BasisFields, NumberInput, ObjectFields, PoseFields, ValueField } from './WorkcellFields'
import { recordingSaveMessage, backendNames, pluginBindingDefaults, preferredController, compatible, initial, label as componentLabel, normalise, presentIkReport, parseWorkcellDocument, parameterSchema, portLabel, visibleDeviceFields } from '../lib/workcell'
import type { Assembly, Binding, Component, Deployment, Device, Instance, Port, Template, SimOptions } from '../lib/workcell'
import './workcell.css'
import { OperatorKeyboard } from './OperatorKeyboard'

type Resolved = { assembly_hash:string; feature_contract:{state_dim:number;action_dim:number;cameras:Record<string,unknown>};diagnostics: {message?:string;severity?:string;path?:string}[];status?:Record<string,unknown> }
type IkReport = {solver?:string;reason?:string;error?:string;age_ms?:number;raw_position_error_m?:number;raw_angle_error_rad?:number;position_error_m?:number;angle_error_rad?:number;damping?:number;singularity_proximity?:number;absorbed_position_m?:number;success?:boolean}
type LogStatus = {enabled:boolean;state:string;directory?:string;files:string[];written:Record<string,number>;dropped:Record<string,number>;queued:number;error?:string;feedback_hz:number}
type LogRun = {session_id:string;assembly_id:string;started_at_utc:string;state:string;files:string[];dropped:Record<string,number>;error?:string}
type Session = {operator?:{activity:string;paused?:boolean;transition?:{status:string;phase:string;kind:string;acks?:Record<string,unknown>;error?:string};return_to?:string;error?:string;work_position?:{status:string}};diagnostic_log?:LogStatus;session_id:string;status:string;assembly_id?:string;revision?:number;recording?:boolean;simulation?:SimOptions & {viewer?:{status:string;error?:string};render_error?:string};controllers?:Record<string,Record<string,IkReport>>;components?:Record<string,{ready?:boolean;reason?:string;values?:number[]}>;inputs?:Record<string,{connected?:boolean;fresh?:boolean}>;[key:string]:unknown}
type Endpoint = {ref:string;instance:string;port:Port}
const stages = [{id:'devices',title:'设备选型',note:'输入、机器人与相机'},{id:'geometry',title:'安装与标定',note:'坐标系、工具与连接'},{id:'control',title:'遥操作设置',note:'控制映射与运动参数'},{id:'capture',title:'数据记录',note:'观测、动作与图像'},{id:'run',title:'检查与运行',note:'保存配置并启动'}]
const states:Record<string,string>={ACTIVE:'运动已使能',READY_DISARMED:'就绪 · 待使能',STOPPED:'已停止',FAULTED:'故障',ESTOP_LATCHED:'急停已锁存'}
const reasons:Record<string,string>={tracking:'跟踪中',converged:'已达到求解精度',holding:'保持位置',deadline_hold:'计算超时，保持位置',absorbed_target_hold:'已吸收超量，保持位置'}
const controlNames:Record<string,string>={human:'人工遥操',policy:'策略控制',playback:'轨迹回放',workpos:'工作位移动',paused:'已暂停'}
const handoffPhases:Record<string,string>={reanchoring:'确认跟随零点',preparing:'准备交还控制',activating:'确认设备激活',committed:'切换完成',rolled_back:'切换失败，已回滚',rollback_failed:'无法恢复原控制，已停止输出',cancelled:'切换已取消'}
const workStates:Record<string,string>={RUNNING:'移动中',SUCCEEDED:'已到达',FAILED:'失败',CANCELLED:'已取消'}
const metric=(n:number|undefined,scale=1,digits=2)=>n==null?'—':(n*scale).toFixed(digits)
async function request<T>(path:string,body?:unknown):Promise<T>{
 const response=await fetch('/api/v2'+path,{method:body===undefined?'GET':'POST',headers:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body)})
 // A crash page or a proxy error is plain text; report it as such instead of
 // letting JSON.parse raise "unexpected character at line 1 column 1".
 const text=await response.text()
 let result:any=null,parsed=false
 if(text){try{result=JSON.parse(text);parsed=true}catch{parsed=false}}
 // JSON `null` is a valid answer (/active-session uses it); only a parse
 // failure means the body was not JSON at all.
 if(!parsed){
  const error=new Error(`接口 ${path} 返回了非 JSON 响应（HTTP ${response.status}）：${text.slice(0,200)||'空响应'}`) as Error & {status?:number}
  error.status=response.status;throw error
 }
 if(!response.ok){const error=new Error(typeof result.detail==='string'?result.detail:JSON.stringify(result.detail??result)) as Error & {status?:number};error.status=response.status;throw error}
 return result
}
function Section({title,note,children}:{title:string;note?:string;children:React.ReactNode}){return <section className="wc-section"><div className="wc-section-heading"><h3>{title}</h3>{note&&<p>{note}</p>}</div>{children}</section>}
function Check({text,value,onChange}:{text:string;value:boolean;onChange:(v:boolean)=>void}){return <label className="wc-check"><input type="checkbox" checked={value} onChange={e=>onChange(e.target.checked)} />{text}</label>}
function SimPreview({ sessionId, cameraId }: { sessionId: string; cameraId: string }) {
  const image = useRef<HTMLImageElement | null>(null)
  const drag = useRef<{ action: 'orbit' | 'pan'; x: number; y: number } | null>(null)
  const queued = useRef<{ action: string; dx: number; dy: number } | null>(null)
  const timer = useRef<number | null>(null)
  // Pointer moves arrive faster than the renderer and the preview stream; keep
  // one request per ~30 ms and let the server queue the remainder.
  const flush = () => {
    timer.current = null
    const item = queued.current
    queued.current = null
    if (item) void request(`/sessions/${sessionId}/cameras/${cameraId}/view`, item).catch(() => {})
  }
  const queue = (action: string, dx: number, dy: number) => {
    if (!dx && !dy) return
    const item = queued.current
    if (item && item.action === action) { item.dx += dx; item.dy += dy }
    else { if (timer.current !== null) flush(); queued.current = { action, dx, dy } }
    if (timer.current === null) timer.current = window.setTimeout(flush, 33)
  }
  useEffect(() => {
    const node = image.current
    if (!node) return
    // React registers wheel handlers as passive; zoom needs preventDefault so
    // the page does not scroll while the view zooms.
    const wheel = (event: WheelEvent) => { event.preventDefault(); queue('zoom', 0, event.deltaY) }
    node.addEventListener('wheel', wheel, { passive: false })
    return () => node.removeEventListener('wheel', wheel)
  }, [sessionId, cameraId])
  const down = (event: React.PointerEvent<HTMLImageElement>) => {
    const pan = event.button === 2 || event.button === 1 || event.shiftKey
    if (event.button !== 0 && !pan) return
    event.preventDefault()
    drag.current = { action: pan ? 'pan' : 'orbit', x: event.clientX, y: event.clientY }
    event.currentTarget.setPointerCapture(event.pointerId)
  }
  const move = (event: React.PointerEvent<HTMLImageElement>) => {
    if (!drag.current) return
    queue(drag.current.action, event.clientX - drag.current.x, event.clientY - drag.current.y)
    drag.current.x = event.clientX; drag.current.y = event.clientY
  }
  const up = (event: React.PointerEvent<HTMLImageElement>) => {
    if (!drag.current) return
    drag.current = null
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
  }
  return <figure style={{ margin: 0, ...subCard }}>
    <figcaption style={{ ...mutedSmall, display: 'flex', alignItems: 'center', gap: 8 }}>
      <span>MuJoCo · {cameraId}</span>
      <span style={{ flex: 1 }} />
      <button type="button" style={{ ...ghostBtn, padding: '3px 10px', fontSize: 12 }} onClick={() => void request(`/sessions/${sessionId}/cameras/${cameraId}/view`, { action: 'reset' }).catch(() => {})}>复位视角</button>
    </figcaption>
    <img ref={image} alt={`${cameraId} 仿真画面`} src={`/api/v2/sessions/${sessionId}/cameras/${cameraId}/stream`} draggable={false}
      onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up}
      onContextMenu={event => event.preventDefault()}
      style={{ maxWidth: '100%', width: 640, borderRadius: 8, touchAction: 'none', cursor: 'grab', userSelect: 'none' }} />
    <p style={{ ...mutedSmall, margin: 0 }}>左键拖动旋转 · 右键或 Shift 拖动平移 · 滚轮缩放</p>
  </figure>
}

export function ComposableTab(){
 const [catalog,setCatalog]=useState<Component[]>([]),[assembly,setAssembly]=useState<Assembly>(initial),[deployment,setDeployment]=useState<Deployment>({})
 const [resolved,setResolved]=useState<Resolved|null>(null),[preflight,setPreflight]=useState<unknown>(null),[session,setSession]=useState<Session|null>(null)
 const [savedRevision,setSavedRevision]=useState(0),[savedHash,setSavedHash]=useState(''),[saved,setSaved]=useState<{assembly_id:string;revision:number}[]>([])
 const [stage,setStage]=useState('devices'),[message,setMessage]=useState(''),[error,setError]=useState(false),[busy,setBusy]=useState(false)
 const [template,setTemplate]=useState(''),[pairLink,setPairLink]=useState(''),[task,setTask]=useState('demonstration')
 const [picks,setPicks]=useState<Record<string,string>>({})
 const [importText,setImportText]=useState('')
 const [logRuns,setLogRuns]=useState<LogRun[]>([]),[selectedRun,setSelectedRun]=useState(''),[logTail,setLogTail]=useState<{records:{kind:string;wall_time_ns:number;component_id?:string;binding_id?:string;data:unknown}[];partial:boolean}|null>(null)
 const label=(key:string)=>componentLabel(key,catalog)
 const [templates,setTemplates]=useState<Template[]>([])
 const [templateCache,setTemplateCache]=useState<Record<string,Assembly>>({})
 const active=!!session&&session.status!=='STOPPED', locked=active||busy
 const descriptor=(item:Instance)=>catalog.find(d=>d.plugin_id===item.plugin)
 const instances=Object.entries(assembly.instances), actuators=instances.filter(([,i])=>descriptor(i)?.kind==='actuator'),inputs=instances.filter(([,i])=>descriptor(i)?.kind==='input')
 const endpoints:Endpoint[]=instances.flatMap(([id,item])=>Object.entries(descriptor(item)?.ports||{}).map(([name,port])=>({ref:id+'.'+name,instance:id,port})))
 const sources=endpoints.filter(e=>descriptor(assembly.instances[e.instance])?.kind==='input'&&e.port.direction==='output')
 const targets=endpoints.filter(e=>descriptor(assembly.instances[e.instance])?.kind==='actuator'&&e.port.direction==='input')
 const rootFrame=(item:Instance)=>{const d=descriptor(item);return d?.kind==='sensor'&&d.ports.rgb?'optical':d?.geometry?.frames?.includes('base')?'base':d?.geometry?.frames?.includes('mount')?'mount':undefined}
 const mountable=instances.filter(([,item])=>descriptor(item)?.kind!=='input').flatMap(([id,item])=>rootFrame(item)?[id+'.'+rootFrame(item)]:[])
 const frames=['world',...instances.filter(([,i])=>descriptor(i)?.kind!=='input').flatMap(([id,item])=>(descriptor(item)?.geometry?.frames||[]).map(frame=>id+'.'+frame))]
 const changed=(next:Assembly)=>{if(active)return;setAssembly(next);setResolved(null);setPreflight(null);setPairLink('')}
 const changeDeployment=(next:Deployment)=>{if(active)return;setDeployment(next);setResolved(null);setPreflight(null)}
 const run=async(fn:()=>Promise<void>)=>{setBusy(true);setError(false);try{await fn()}catch(e){setError(true);setMessage((e as Error).message)}finally{setBusy(false)}}
 const applySaved=(item:{assembly:Assembly;deployment?:Deployment;resolved:Resolved})=>{const next=normalise(item.assembly),migrated=JSON.stringify(next.instances)!==JSON.stringify(item.assembly.instances);setAssembly(next);setDeployment(item.deployment||{});setResolved(migrated?null:item.resolved);setSavedRevision(item.assembly.revision);setSavedHash(migrated?'':item.resolved.assembly_hash);setTemplate(templates.find(t=>t.assembly_id===next.assembly_id)?.id||'');if(migrated)setMessage('已更新为统一手柄按键，请保存新版本后启动。')}
 useEffect(()=>{
  request<{components:Component[]}>('/components').then(x=>setCatalog(x.components)).catch(e=>{setError(true);setMessage(e.message)})
  request<typeof saved>('/assemblies').then(setSaved).catch(()=>{})
  request<Template[]>('/templates').then(items=>{setTemplates(items);setTemplate(current=>current||items[0]?.id||'')}).catch(e=>{setError(true);setMessage(e.message)})
  void request<Session|null>('/active-session').then(async current=>{if(!current||current.status==='STOPPED'||!current.assembly_id||!current.revision)return;const item=await request<{assembly:Assembly;deployment?:Deployment;resolved:Resolved}>(`/assemblies/${current.assembly_id}/revisions/${current.revision}`);applySaved(item);setSession(current);setStage('run')}).catch(e=>{setError(true);setMessage(e.message)})
 },[])
 useEffect(()=>{setTemplate(templates.find(t=>t.assembly_id===assembly.assembly_id)?.id||'')},[templates,assembly.assembly_id])
 useEffect(()=>{if(!session||session.status==='STOPPED')return;const timer=window.setInterval(()=>{request<Session>(`/sessions/${session.session_id}/state`).then(setSession).catch((e:Error&{status?:number})=>{if(e.status===404){setSession(null);setMessage('运行已结束，请重新启动。')}else{setError(true);setMessage(e.message)}})},500);return()=>clearInterval(timer)},[session?.session_id,session?.status])
 const setInstance=(id:string,item:Instance)=>changed({...assembly,instances:{...assembly.instances,[id]:item}})
 function add(kind:string){
  const list=catalog.filter(d=>d.kind===kind),d=list.find(d=>d.plugin_id===picks[kind])||list[0];if(!d)return
  const root=d.kind==='input'?(d.plugin_id.includes('quest')?'quest':d.plugin_id.includes('vader')?'pad':'input'):d.kind==='sensor'?'camera':d.plugin_id.includes('hand')?'hand':d.plugin_id.includes('gripper')||d.plugin_id.includes('epg')?'gripper':'arm'
  let id=root,n=2;while(assembly.instances[id])id=root+'_'+n++
  const next=structuredClone(assembly);next.instances[id]={plugin:d.plugin_id,version:d.plugin_version,backend:d.backend_variants?.fake?'fake':Object.keys(d.backend_variants||{})[0]}
  if(d.kind==='actuator'){next.capture.state.push(id+'.position');const target=d.ports.closure?'closure':d.ports.position_target?'position_target':Object.entries(d.ports).find(([,p])=>p.direction==='input')?.[0];if(target)next.capture.action.push(id+'.'+target)}
  if(d.ports.rgb){next.capture.images[id]=id+'.rgb';next.instances[id].parameters={width:640,height:480,fps:30}}
  const rootFrame=d.kind==='sensor'&&d.ports.rgb?'optical':d.geometry?.frames?.includes('base')?'base':d.geometry?.frames?.includes('mount')?'mount':d.geometry?.frames?.includes('optical')?'optical':null
  if(rootFrame&&d.kind!=='input') next.mounts.push({child:id+'.'+rootFrame,parent:'world',transform:rootFrame==='optical'?[1.5,1.5,1.4,0,0,0,1]:[0,0,0,0,0,0,1]})
  changed(next)
 }
 function remove(id:string){if(!window.confirm(`移除设备 ${id}？相关控制映射和安装关系会一并删除。`))return
  const next=structuredClone(assembly);delete next.instances[id];next.bindings=next.bindings.filter(b=>![b.source,b.target,b.deadman||''].some(x=>x.startsWith(id+'.')));next.mounts=next.mounts.filter(m=>![m.child,m.parent].some(x=>x.startsWith(id+'.')))
  next.capture.state=next.capture.state.filter(x=>!x.startsWith(id+'.'));next.capture.action=next.capture.action.filter(x=>!x.startsWith(id+'.'));next.capture.auxiliary=next.capture.auxiliary?.filter(x=>!x.startsWith(id+'.'));next.capture.images=Object.fromEntries(Object.entries(next.capture.images).filter(([,v])=>!v.startsWith(id+'.')))
  next.control_groups=Object.fromEntries(Object.entries(next.control_groups).map(([key,g])=>[key,{...g,members:g.members.filter(x=>x!==id)}] as const).filter(([,g])=>g.members.length))
  const dep=structuredClone(deployment);if(dep.instances)delete dep.instances[id];changeDeployment(dep);changed(next)
 }
 const portName=(endpoint:Endpoint)=>endpoint.ref.slice(endpoint.instance.length+1)
 const isQuest=(endpoint:Endpoint)=>assembly.instances[endpoint.instance]?.plugin==='morla.input.quest'
 const questVisible=(endpoint:Endpoint)=>{
  if(!isQuest(endpoint))return true
  const side=deployment.instances?.[endpoint.instance]?.parameters?.side||assembly.instances[endpoint.instance].parameters?.side||'both'
  const match=portName(endpoint).match(/^(left|right)_(pose|axes|scalar|hold|landmarks|buttons)$/)
  return !!match&&(side==='both'||side===match[1])
 }
 const currentQuestPort=(endpoint:Endpoint)=>/^(left|right)_(pose|axes|scalar|hold|landmarks|buttons)$/.test(portName(endpoint))
 const holdPort=(endpoint:Endpoint)=>/^(left_hold|right_hold|hold_to_control|deadman)$/.test(portName(endpoint))
 const standardHold=(endpoint:Endpoint)=>sources.find(candidate=>candidate.instance===endpoint.instance&&(
  isQuest(endpoint)?portName(candidate)===portName(endpoint).replace(/_(pose|axes|scalar|landmarks|buttons)$/,'_hold'):portName(candidate)==='hold_to_control'))
 const signalLabel=(endpoint:Endpoint)=>{
  if(!isQuest(endpoint))return portLabel(endpoint.ref)
  const [side,kind]=portName(endpoint).split('_')
  const names:Record<string,string>={pose:'侧位姿（手柄 / 裸手）',axes:'摇杆',scalar:'扳机 / 捏合开合量',hold:' Grip / 捏合',landmarks:'手关键点',buttons:'手柄按键'}
  return names[kind]?endpoint.instance+' · '+(side==='left'?'左':'右')+names[kind]:portLabel(endpoint.ref)+'（兼容）'
 }
 const choices=(source:string,target:string)=>compatible(catalog,endpoints.find(e=>e.ref===source)?.port,endpoints.find(e=>e.ref===target)?.port,assembly.instances[target.split('.')[0]]?.plugin,device(target.split('.')[0]).backend||assembly.instances[target.split('.')[0]]?.backend||'fake')
 function defaults(id:string,source:string,target:string){
  const sp=endpoints.find(e=>e.ref===source)?.port,ti=assembly.instances[target.split('.')[0]],td=ti&&descriptor(ti);if(!sp||!td)return{}
  return pluginBindingDefaults(catalog,id,sp,td,assembly.instances[source.split('.')[0]]?.plugin||'',ti.parameters)
 }
 function preferred(source:string,target:string){const list=choices(source,target),td=descriptor(assembly.instances[target.split('.')[0]]),semantic=endpoints.find(e=>e.ref===source)?.port.semantic
  return td?preferredController(list,td,semantic||''):list[0]||''
 }
 function buildBinding(source:string,target:string,id:string):Binding{
  const controller=preferred(source,target),sourcePort=endpoints.find(e=>e.ref===source)?.port,quest=assembly.instances[source.split('.')[0]]?.plugin==='morla.input.quest'
  const deadman=sourcePort?.semantic==='axes_buttons'?(sources.find(s=>s.instance===source.split('.')[0]&&s.ref.endsWith('.hold_to_control'))?.ref||''):''
  return {id,source,target,controller,deadman,max_age_ms:250,...(quest&&['pose6d','hand_landmarks'].includes(sourcePort?.semantic||'')?{input_basis_id:'unity_left_handed_v1'}:{}),parameters:defaults(controller,source,target)}
 }
 function addBinding(){
  const target=targets.find(t=>!assembly.bindings.some(b=>b.target.split('.')[0]===t.instance));if(!target){setError(true);setMessage('每个执行器只能有一条控制映射。请添加设备，或编辑已有映射。');return}
  const available=sources.filter(s=>questVisible(s)&&!holdPort(s)&&!portName(s).endsWith('_buttons')&&choices(s.ref,target.ref).length)
  const source=available.find(s=>isQuest(s)&&portName(s)==='right_pose')||available.find(s=>s.port.semantic==='pose6d')||available[0];if(!source){setError(true);setMessage('当前设备没有兼容的输入与控制目标。');return}
  let n=1;while(assembly.bindings.some(b=>b.id==='control_'+n))n++
  const binding=buildBinding(source.ref,target.ref,'control_'+n)
  const robot=assembly.instances[target.instance]
  if((device(target.instance).backend||robot.backend)==='real'&&source.port.semantic==='pose6d')binding.deadman=standardHold(source)?.ref||binding.deadman
  changed({...assembly,bindings:[...assembly.bindings,binding]})
 }
 const patchBinding=(index:number,patch:Partial<Binding>)=>changed({...assembly,bindings:assembly.bindings.map((b,i)=>i===index?{...b,...patch}:b)})
 function updateBinding(index:number,key:'source'|'target'|'controller',value:string){
  const old=assembly.bindings[index],source=key==='source'?value:old.source,target=key==='target'?value:old.target
  const next=buildBinding(source,target,old.id),controller=key==='controller'?value:choices(source,target).includes(old.controller)?old.controller:next.controller
  if(source===old.source&&target===old.target&&controller===old.controller)return
  const selected=sources.find(e=>e.ref===source)
  const previous=sources.find(e=>e.ref===old.source)
  const previousHold=previous&&standardHold(previous)
  const deadman=selected&&source!==old.source&&old.deadman&&old.deadman===previousHold?.ref
   ?standardHold(selected)?.ref||old.deadman:old.deadman
  patchBinding(index,{source,target,controller,deadman,input_basis_id:next.input_basis_id,parameters:defaults(controller,source,target)})
 }
 const device=(id:string)=>deployment.instances?.[id]||{}
 const patchDevice=(id:string,patch:Partial<Device>)=>changeDeployment({...deployment,deployment_version:1,deployment_id:deployment.deployment_id||assembly.assembly_id+'_hardware',instances:{...deployment.instances,[id]:{...device(id),...patch}}})
 function backend(id:string,value:string){const dep=structuredClone(deployment);if(dep.instances?.[id]){delete dep.instances[id].backend;if(value!=='real')delete dep.instances[id]}changeDeployment(dep);setInstance(id,{...assembly.instances[id],backend:value})}
 const nextStage=()=>setStage(stages[Math.min(stages.findIndex(s=>s.id===stage)+1,stages.length-1)].id)
 async function loadTemplate(){if(!template)return;const a=templateCache[template]||normalise(await request<Assembly>('/templates/'+template));setTemplateCache(cache=>({...cache,[template]:a}));const revision=saved.find(x=>x.assembly_id===a.assembly_id)?.revision||0;changed({...structuredClone(a),revision:revision+1});setDeployment({});setSavedRevision(revision);setSavedHash('');setMessage('已载入机器人与输入标定，可继续检查设备设置。')}
 async function check(){const result=await request<Resolved>('/assemblies/compile',{assembly,deployment});setResolved(result);setMessage('配置检查通过。请保存配置后启动。');return result}
 async function save(){const result=resolved||await check();const next={...assembly,revision:savedRevision+1};const item=await request<{assembly:Assembly;deployment:Deployment;resolved:Resolved}>('/assemblies',{assembly:next,deployment,expected_revision:savedRevision});applySaved(item);setSaved(await request<typeof saved>('/assemblies'));setMessage(`配置已保存，版本 ${next.revision}。`);return result}
 const operate=(operation:string)=>void run(async()=>{if(operation==='activate'&&instances.some(([id,item])=>descriptor(item)?.kind==='actuator'&&(device(id).backend||item.backend)==='real')&&!window.confirm('允许真实机器人响应当前控制输入？请先核对输入方向、工具坐标系与周围环境。'))return;if(['estop','stop'].includes(operation)&&!window.confirm(operation==='estop'?'急停当前机器人？':'停止当前运行并释放设备连接？'))return;const reply=await request<{status?:string;error?:string}>(`/sessions/${session!.session_id}/${operation==='activate'?'operator/action':'operations'}`,operation==='activate'?{action:'teleop_start',request_id:crypto.randomUUID()}:{operation,request_id:crypto.randomUUID()});setSession(await request<Session>(`/sessions/${session!.session_id}/state`));setMessage(reply.error||'操作已提交，运行状态将自动更新。')})
 const sim=session?.simulation||assembly.metadata?.simulation||{mode:'physics' as const,display:'web' as const}
 function simOptions(patch:Partial<SimOptions>){if(active)void run(async()=>{await request(`/sessions/${session!.session_id}/simulation`,patch);setSession(await request<Session>(`/sessions/${session!.session_id}/state`))});else changed({...assembly,metadata:{...assembly.metadata,simulation:{...sim,...patch}}})}
 const logOptions={enabled:true,feedback_hz:50,queue_size:4096,segment_mb:16,max_run_mb:1024,...assembly.metadata?.diagnostic_logging}
 const currentLogRun=selectedRun||session?.session_id||logRuns[0]?.session_id||''
 const logEntry=logRuns.find(r=>r.session_id===currentLogRun)
 const logFiles=currentLogRun===session?.session_id&&session?.diagnostic_log?.directory?['manifest.json',...session.diagnostic_log.files,...(session.status==='STOPPED'?['summary.json']:[])]:logEntry?.files||[]
 useEffect(()=>{if(stage!=='run')return;void request<LogRun[]>('/diagnostics/runs').then(setLogRuns).catch(()=>{})},[stage,session?.session_id,session?.status])
 const count={input:inputs.length,actuator:actuators.length,sensor:instances.filter(([,i])=>descriptor(i)?.kind==='sensor').length}
 const simPresent=instances.some(([id,i])=>(device(id).backend||i.backend)==='sim')
 return <div className="workcell">
  <header className="wc-header"><div><div className="wc-eyebrow">MORLA / WORKCELL</div><h2>机器人工作站</h2><p>配置设备与坐标关系，建立遥操作映射，然后检查并运行。</p></div><div className="wc-header-status"><span className={'wc-status '+(active?'is-live':'')}>{session?.operator?.transition?.status==='RUNNING'?'控制权切换中':session?states[session.status]||session.status:'配置中'}</span><span>{savedRevision?'已保存 v'+savedRevision:'尚未保存'}</span></div></header>
  <div className="wc-config-bar"><label>配置名称<input aria-label="配置名称" disabled={locked} value={assembly.assembly_id} onChange={e=>{setSavedRevision(0);setSavedHash('');changed({...assembly,assembly_id:e.target.value,revision:1})}} /></label><label>已保存的工作站<select aria-label="打开已保存配置" disabled={locked} value="" onChange={e=>void run(async()=>{const item=await request<{assembly:Assembly;deployment?:Deployment;resolved:Resolved}>(`/assemblies/${e.target.value}/revisions/${saved.find(x=>x.assembly_id===e.target.value)!.revision}`);applySaved(item);setSession(null);setMessage('已打开保存的配置。')})}><option value="">选择配置…</option>{saved.map(s=><option key={s.assembly_id} value={s.assembly_id}>{s.assembly_id} · v{s.revision}</option>)}</select></label><div className="wc-template"><label>设备预设<select aria-label="仿真机器人模板" disabled={locked} value={template} onChange={e=>setTemplate(e.target.value)}><option value="">选择预设…</option>{templates.map(t=><option key={t.id} value={t.id}>{t.title}</option>)}</select></label><button disabled={locked||!template} onClick={()=>void run(loadTemplate)}>载入预设</button></div></div>
  <nav className="wc-stage-nav" aria-label="工作站配置流程">{stages.map(s=><button key={s.id} className={stage===s.id?'is-current':''} aria-current={stage===s.id?'step':undefined} onClick={()=>{setStage(s.id);setMessage('')}}>{s.title}</button>)}</nav>
  {message&&<div role="status" className={'wc-message '+(error?'is-error':'')}>{message}</div>}
  {active&&stage!=='run'&&<div className="wc-message">当前配置正在运行，可查看设置。停止运行后才能修改设备与控制参数。</div>}
  <div className="wc-stage-heading"><h3>{stages.find(s=>s.id===stage)?.title}</h3><p>{stages.find(s=>s.id===stage)?.note}</p></div>

  {stage==='devices'&&<fieldset disabled={locked} className="wc-unframed">
   {(['input','actuator','sensor'] as const).map(kind=><Section key={kind} title={{input:'输入设备',actuator:'机器人与末端执行器',sensor:'相机与传感器'}[kind]} note={{input:'选择手柄、头显、手套或标准输入接口。',actuator:'机械臂、夹爪和灵巧手分别配置，可组合使用。',sensor:'图像采集为可选项，仿真相机与机械臂共用场景。'}[kind]}>
    <div className="wc-add-row"><select aria-label={kind+'组件'} value={catalog.filter(c=>c.kind===kind).some(c=>c.plugin_id===picks[kind])?picks[kind]:catalog.find(c=>c.kind===kind)?.plugin_id||''} onChange={e=>setPicks({...picks,[kind]:e.target.value})}>{catalog.filter(c=>c.kind===kind).map(c=><option key={c.plugin_id} value={c.plugin_id}>{label(c.plugin_id)}{c.joints?.length?` · ${c.joints.length} 轴`:''}</option>)}</select><button onClick={()=>add(kind)} disabled={!catalog.some(c=>c.kind===kind)}>添加设备</button><span className="wc-note">{count[kind]} 个设备</span></div>
    {instances.filter(([,i])=>descriptor(i)?.kind===kind).map(([id,item])=>{const d=descriptor(item)!;const mode=device(id).backend||item.backend||'fake';const fields=d.configuration_schema?.properties||{};const props=visibleDeviceFields(d,mode,item.parameters,device(id).parameters);const basic=kind==='sensor'?(mode==='real'?['camera_backend','width','height','fps','image_topic','compressed']:['width','height','fps']):kind==='input'?['side','process_mode','protocol','deadzone']:['side','rate_hz'];const rest=Object.fromEntries(Object.entries(props).filter(([key])=>!basic.includes(key)));return <article className="wc-device" key={id}><div className="wc-device-heading"><div><h4>{item.display_name||label(item.plugin)}</h4><code>{id}</code>{!!d.joints?.length&&<span className="wc-note">{d.joints.length} 轴</span>}</div><button className="wc-text-button wc-destructive" onClick={()=>remove(id)}>移除</button></div>
     <fieldset className="wc-pose"><legend>通用设置</legend><div className="wc-form-grid"><label>显示名称<input value={String(item.display_name||'')} placeholder={label(item.plugin)} onChange={e=>setInstance(id,{...item,display_name:e.target.value})} /></label><label>运行方式<select aria-label={id+'运行后端'} value={mode} onChange={e=>backend(id,e.target.value)}>{Object.entries(d.backend_variants||{}).map(([key,status])=><option key={key} value={key} disabled={status.available===false}>{backendNames[key]||key}{status.available===false?'（未提供）':''}</option>)}</select></label></div></fieldset>
     <ObjectFields value={Object.fromEntries(Object.entries(item.parameters||{}).filter(([key])=>basic.includes(key)))} properties={Object.fromEntries(basic.filter(key=>props[key]).map(key=>[key,props[key]]))} defaults={d.defaults} onChange={value=>{const params={...item.parameters};for(const key of basic)delete params[key];setInstance(id,{...item,parameters:{...params,...value}})}} />
     {(Object.keys(rest).length>0||Object.keys(item.parameters||{}).some(key=>!(key in fields)))&&<details className="wc-details"><summary>设备专用参数</summary><ObjectFields value={Object.fromEntries(Object.entries(item.parameters||{}).filter(([key])=>key in rest||!(key in fields)))} properties={rest} defaults={d.plugin_id==='morla.input.quest'?{pinch_range_m:[.015,.08],...d.defaults}:d.defaults} onChange={value=>setInstance(id,{...item,parameters:{...Object.fromEntries(Object.entries(item.parameters||{}).filter(([key])=>basic.includes(key)||(key in fields&&!(key in rest)))),...value}})} /></details>}
     {mode==='real'&&<fieldset className="wc-pose"><legend>硬件连接</legend>
      <div className="wc-form-grid">
       <label>设备标识<input value={device(id).device_id||''} placeholder="设备序列号或资产标识" onChange={e=>patchDevice(id,{device_id:e.target.value})} /></label>
       <label>设备资源标识<input value={device(id).resource_key||''} placeholder="默认使用设备标识" onChange={e=>patchDevice(id,{resource_key:e.target.value||undefined})} /></label>
      </div>
      <ObjectFields value={Object.fromEntries(Object.entries(device(id).parameters||{}).filter(([key])=>key in visibleDeviceFields(d,mode,item.parameters,device(id).parameters,true)))}
       properties={visibleDeviceFields(d,mode,item.parameters,device(id).parameters,true)}
       defaults={{...d.defaults,...item.parameters}}
       onChange={parameters=>{
        const shown=visibleDeviceFields(d,mode,item.parameters,device(id).parameters,true)
        const retained=Object.fromEntries(Object.entries(device(id).parameters||{}).filter(([key])=>!(key in shown)))
        patchDevice(id,{parameters:{...retained,...parameters}})
       }} />
      <div className="wc-inline"><Check text="已核对设备身份" value={!!device(id).verified} onChange={verified=>patchDevice(id,{verified})} /><Check text="已完成停止测试" value={!!device(id).stop_verified} onChange={stop_verified=>patchDevice(id,{stop_verified})} /></div>
      {(d.resources?.sharing==='session'||!!device(id).session_id)&&<label>共享驱动标识<input value={String(device(id).session_id||'')} onChange={e=>patchDevice(id,{session_id:e.target.value||undefined})} /></label>}
     </fieldset>}
     {mode!=='real'&&<p className="wc-note">当前为{backendNames[mode]}。切换为真实设备后可配置硬件连接与驱动参数。</p>}
    </article>})}
    {!count[kind]&&<p className="wc-empty">尚未添加{kind==='input'?'输入设备':kind==='actuator'?'机器人或末端':'相机'}。</p>}
   </Section>)}
  </fieldset>}

  {stage==='geometry'&&<fieldset disabled={locked} className="wc-unframed">
   <Section title="安装关系" note="定义基座、末端和相机在场景中的位置。位姿使用父坐标系表达；平移单位 m，转角单位 °。">
    {assembly.mounts.map((mount,index)=><article className="wc-device" key={index}><div className="wc-form-grid"><label>安装对象<select aria-label={'安装对象 '+index} value={mount.child} onChange={e=>changed({...assembly,mounts:assembly.mounts.map((m,i)=>i===index?{...m,child:e.target.value}:m)})}>{mountable.map(f=><option key={f}>{f}</option>)}</select></label><label>父坐标系<select aria-label={'父坐标系 '+index} value={mount.parent} onChange={e=>changed({...assembly,mounts:assembly.mounts.map((m,i)=>i===index?{...m,parent:e.target.value}:m)})}>{frames.filter(f=>f.split('.')[0]!==mount.child.split('.')[0]).map(f=><option key={f}>{f}</option>)}</select></label></div>
     <PoseFields label={mount.child+' 安装位姿'} value={mount.transform} onChange={v=>changed({...assembly,mounts:assembly.mounts.map((m,i)=>i===index?{...m,transform:v as number[]}:m)})} />
     {!mount.transform&&<div className="wc-inline"><span className="wc-note">此关系尚无明确位姿。</span><button onClick={()=>changed({...assembly,mounts:assembly.mounts.map((m,i)=>i===index?{...m,transform:[0,0,0,0,0,0,1]}:m)})}>设为重合坐标系</button></div>}
     <div className="wc-inline"><label>标定记录名称（可选）<input value={mount.calibration||''} onChange={e=>changed({...assembly,mounts:assembly.mounts.map((m,i)=>{if(i!==index)return m;const next={...m};if(e.target.value)next.calibration=e.target.value;else delete next.calibration;return next})})} /></label><button className="wc-text-button wc-destructive" onClick={()=>changed({...assembly,mounts:assembly.mounts.filter((_,i)=>i!==index)})}>删除安装关系</button></div>
    </article>)}
    <button disabled={!mountable.some(f=>!assembly.mounts.some(m=>m.child.split('.')[0]===f.split('.')[0]))} onClick={()=>{const child=mountable.find(f=>!assembly.mounts.some(m=>m.child.split('.')[0]===f.split('.')[0]));if(child)changed({...assembly,mounts:[...assembly.mounts,{child,parent:'world',transform:[0,0,0,0,0,0,1]}]})}}>添加安装关系</button>
   </Section>
   <Section title="协同控制组" note="需要联动停止的执行器可以编为一组。未分组的执行器使用独立控制组。">
    {Object.entries(assembly.control_groups).map(([id,group])=><article className="wc-device" key={id}><div className="wc-device-heading"><h4>{id}</h4><button className="wc-text-button wc-destructive" onClick={()=>{const next={...assembly.control_groups};delete next[id];changed({...assembly,control_groups:next})}}>删除组</button></div><div className="wc-inline">{actuators.map(([iid,item])=><Check key={iid} text={(item.display_name||label(item.plugin))+' · '+iid} value={group.members.includes(iid)} onChange={v=>changed({...assembly,control_groups:{...assembly.control_groups,[id]:{...group,members:v?[...group.members,iid]:group.members.filter(x=>x!==iid)}}})} />)}</div><p className="wc-note">组内故障处理：{group.fault_policy==='stop_group'?'停止该组':group.fault_policy}。同一执行器只能属于一个控制组。</p></article>)}
    <button disabled={!actuators.some(([id])=>!Object.values(assembly.control_groups).some(g=>g.members.includes(id)))} onClick={()=>{let n=1;while(assembly.control_groups['group_'+n])n++;const member=actuators.find(([id])=>!Object.values(assembly.control_groups).some(g=>g.members.includes(id)))?.[0];if(member)changed({...assembly,control_groups:{...assembly.control_groups,['group_'+n]:{members:[member],fault_policy:'stop_group'}}})}}>添加控制组</button>
   </Section>
  </fieldset>}

  {stage==='control'&&<fieldset disabled={locked} className="wc-unframed">
   <Section title="控制映射" note="为执行器配置控制模式、遥操输入和跟随门控。切换输入或控制模式后，请核对坐标标定。">
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
     const common=cartesian?['position_scale','linear_speed','angular_speed',...(deviceIk?['max_joint_velocity','smoothing_alpha']:[])].filter(key=>jog?!['position_scale','smoothing_alpha'].includes(key):!['linear_speed','angular_speed'].includes(key)):joint?['speed']:[]
     const hidden=['input_rotation','input_basis','solver_tip_from_tool','axes_map',...common]
     const advanced=deviceIk?['command_lead_rad','max_joint_step','position_lead_m','rotation_lead_rad','damping_min','damping_max','singularity_threshold','differential_step_rad']:[]
     return <article className="wc-device" key={binding.id}><div className="wc-device-heading"><h4>{label(binding.controller)} <code>{binding.id}</code></h4><button className="wc-text-button wc-destructive" onClick={()=>changed({...assembly,bindings:assembly.bindings.filter((_,i)=>i!==index)})}>移除映射</button></div><div className="wc-form-grid">
      <label>控制目标<select aria-label={binding.id+'控制目标'} value={binding.target} onChange={e=>updateBinding(index,'target',e.target.value)}>{targetOptions.map(e=>{
       const item=assembly.instances[e.instance]
       return <option key={e.ref} value={e.ref}>{item.display_name||label(item.plugin)} · {e.instance}</option>
      })}</select></label>
      <label>控制模式<select value={binding.controller} onChange={e=>{
       const controller=e.target.value
       const selected=sources.find(candidate=>candidate.ref===binding.source&&choices(candidate.ref,binding.target).includes(controller))||available.find(candidate=>choices(candidate.ref,binding.target).includes(controller))
       if(!selected)return
       const next=buildBinding(selected.ref,binding.target,binding.id)
       patchBinding(index,{...next,controller,deadman:selected.ref!==binding.source&&binding.deadman&&binding.deadman===hold?.ref?standardHold(selected)?.ref||binding.deadman:binding.deadman,parameters:defaults(controller,selected.ref,binding.target)})
      }}>{[...new Set([binding.controller,...available.flatMap(e=>choices(e.ref,binding.target))])].map(id=><option key={id} value={id}>{(choices(binding.source,binding.target).includes(id)||available.some(e=>choices(e.ref,binding.target).includes(id)))?label(id):label(id)+'（已有配置，请核对）'}</option>)}</select></label>
      <label>遥操输入<select aria-label={binding.id+'输入信号'} value={binding.source} onChange={e=>updateBinding(index,'source',e.target.value)}>{sources.filter(e=>e.ref===binding.source||available.includes(e)).map(e=><option key={e.ref} value={e.ref}>{signalLabel(e)}</option>)}</select></label>
      <label>跟随门控<select value={!binding.deadman?'continuous':hold?.ref===binding.deadman?'hold':'custom'} onChange={e=>{
       if(e.target.value==='continuous')patchBinding(index,{deadman:''})
       if(e.target.value==='hold'&&hold)patchBinding(index,{deadman:hold.ref})
      }}><option value="continuous">持续跟随</option>{hold&&<option value="hold">{source&&isQuest(source)?'按住跟随':'按住跟随'}</option>}{binding.deadman&&hold?.ref!==binding.deadman&&<option value="custom">已有自定义门控</option>}</select></label>
     </div>
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
     {binding.deadman&&binding.deadman!==hold?.ref&&<p className="wc-note">已有门控绑定 · {binding.deadman}</p>}
     <details className="wc-details"><summary>输入时效与求解设置</summary><div className="wc-form-grid">
      <label>输入有效期 / ms<NumberInput label={binding.id+'输入有效期'} schema={{minimum:1}} value={binding.max_age_ms||250} onChange={max_age_ms=>patchBinding(index,{max_age_ms})} /></label>
      {deviceIk&&<ValueField name="ik_mode" value={params.ik_mode||'differential'} schema={parameterSchema('ik_mode')} onChange={v=>setParams({...params,ik_mode:v})} />}
     </div></details>
     {common.length>0&&<ObjectFields value={Object.fromEntries(Object.entries(params).filter(([key])=>common.includes(key)))} properties={Object.fromEntries(common.map(key=>[key,parameterSchema(key)]))} onChange={value=>{const next={...params};for(const key of common)delete next[key];setParams({...next,...value})}} />}
     {(jog||joint)&&<fieldset className="wc-pose"><legend>摇杆轴分配</legend><div className="wc-form-grid">{(joint?(d?.joints||[]).map(j=>j.name):['x','y','z','roll','pitch','yaw']).map((axis,i)=>{const map=params.axes_map as string[]|Record<string,string|null>|undefined;const current=joint?Array.isArray(map)?map[i]:null:!Array.isArray(map)?map?.[axis]:null;return <label key={axis}>{joint?`关节 ${i+1}`:({x:'沿 X 平移',y:'沿 Y 平移',z:'沿 Z 平移',roll:'绕 X 转动',pitch:'绕 Y 转动',yaw:'绕 Z 转动'}[axis])}<select aria-label={binding.id+' '+axis+'输入轴'} value={current||''} onChange={e=>{const next=joint?Array.from({length:d?.joints?.length||1},(_,j)=>Array.isArray(map)?map[j]??null:null):{...(!Array.isArray(map)?map:{})};if(Array.isArray(next))next[i]=e.target.value||null;else next[axis]=e.target.value||null;setParams({...params,axes_map:next})}}><option value="">不使用</option>{(source?.port.axis_names||[]).map(name=><option key={name}>{name}</option>)}</select></label>})}</div></fieldset>}
     {(cartesian||Boolean(params.input_basis))&&<BasisFields label={cartesian?'输入方向与基座坐标系':'手部输入坐标系'} value={(params.input_rotation||params.input_basis) as number[][]|undefined} onChange={v=>setParams({...params,[cartesian?'input_rotation':'input_basis']:v})} />}
     {cartesian&&<PoseFields label="工具中心点 TCP 相对求解末端" matrix value={params.solver_tip_from_tool as number[][]|undefined} onChange={v=>setParams({...params,solver_tip_from_tool:v})} />}
     {cartesian&&<Check text="已核对实际输入方向与工具坐标系" value={!!params.calibration_verified} onChange={v=>setParams({...params,calibration_verified:v})} />}
     <details className="wc-details" open={!cartesian&&!joint}><summary>{cartesian?'求解与跟随参数':'控制参数'}</summary><ObjectFields value={Object.fromEntries(Object.entries(params).filter(([key])=>!hidden.includes(key)&&key!=='calibration_verified'&&key!=='ik_mode'))} properties={{...controllerDescriptor?.configuration_schema?.properties,...Object.fromEntries([...advanced,...Object.keys(params).filter(key=>['input_range','output_range','pinch_range_m'].includes(key))].map(key=>[key,parameterSchema(key)]))}} onChange={value=>{const keep=Object.fromEntries(Object.entries(params).filter(([key])=>hidden.includes(key)||key==='calibration_verified'||key==='ik_mode'));setParams({...keep,...value})}} allowNew={!cartesian&&!joint} /></details>
     </article>})}
    <button disabled={!sources.length||!targets.length} onClick={addBinding}>添加控制映射</button>
    {inputs.some(([,item])=>item.plugin==='morla.input.quest')&&<details className="wc-details"><summary>Quest 工作站快捷键</summary><p className="wc-note">右 Grip 按下开始遥操或接管。右 A 开始录制或交还控制，右 B 结束并保存，右摇杆按下丢弃当前录制，左 X 回工作位。也可在运行页点击「开始遥操」或按 S。</p></details>}
    {!assembly.bindings.length&&<p className="wc-empty">尚无控制映射。先添加输入设备与执行器，再建立映射。</p>}
   </Section>
  </fieldset>}

  {stage==='control'&&<fieldset disabled={locked} className="wc-unframed"><Section title="直接回工作位" note="按关节直线插补返回配置位置，不经过途径点，也不进行避障规划。未配置设备不参与；录制期间禁止回位。">
    {actuators.filter(([,item])=>(descriptor(item)?.joints?.length||0)>0).map(([id,item])=>{const joints=descriptor(item)!.joints!,config=assembly.metadata?.operator_control||{},values=config.work_positions?.[id];return <div className="wc-pose" key={id}>
     <Check text={'设置 '+(item.display_name||label(item.plugin))+' · '+id+' 工作位'} value={!!values} onChange={enabled=>{const positions={...config.work_positions};if(enabled)positions[id]=joints.map(j=>Math.max(j.lower??-Infinity,Math.min(j.upper??Infinity,0)));else delete positions[id];changed({...assembly,metadata:{...assembly.metadata,operator_control:{...config,work_positions:positions}}})}} />
     {values&&<div className="wc-form-grid">{joints.map((joint,index)=><label key={joint.name}>{joint.name} / {joint.unit}<NumberInput label={id+'工作位'+joint.name} schema={{minimum:joint.lower,maximum:joint.upper}} value={values[index]} onChange={value=>{const next=[...values];next[index]=value;changed({...assembly,metadata:{...assembly.metadata,operator_control:{...config,work_positions:{...config.work_positions,[id]:next}}}})}} /></label>)}</div>}
    </div>})}
    <label>关节插补速度上限 / 关节单位每秒<NumberInput label="工作位插补速度" schema={{minimum:.001,maximum:1}} value={assembly.metadata?.operator_control?.work_speed??.15} onChange={value=>changed({...assembly,metadata:{...assembly.metadata,operator_control:{...assembly.metadata?.operator_control,work_speed:value}}})} /></label>
   </Section></fieldset>}
  {stage==='capture'&&<fieldset disabled={locked} className="wc-unframed">
   <Section title="记录设置" note="选择需要保存的观测与动作。无需采集时可保留默认投影，在运行页不启动记录。">
    <div className="wc-form-grid"><label>记录频率 / Hz<NumberInput label="记录频率" schema={{minimum:1,maximum:500}} value={assembly.capture.dataset_fps||30} onChange={v=>changed({...assembly,capture:{...assembly.capture,dataset_fps:v}})} /></label><label>动作数据来源<select value={assembly.capture.action_source} onChange={e=>changed({...assembly,capture:{...assembly.capture,action_source:e.target.value}})}>{[['command','控制指令'],['feedback','反馈位置'],['next_state','下一帧状态'],['requested','请求指令'],['authorized','授权指令'],['dispatched','已下发指令']].map(([id,title])=><option key={id} value={id}>{title}</option>)}</select></label></div>
    {(['state','action'] as const).map(kind=><fieldset className="wc-pose" key={kind}><legend>{{state:'机器人观测',action:'机器人动作'}[kind]}</legend><div className="wc-form-grid">{(kind==='action'?targets:endpoints.filter(e=>e.port.direction==='output'&&e.port.semantic!=='image'&&descriptor(assembly.instances[e.instance])?.kind!=='input')).map(e=><Check key={e.ref} text={portLabel(e.ref)+` · ${e.port.dim} 维`} value={(assembly.capture[kind]||[]).includes(e.ref)} onChange={checked=>{let list=(assembly.capture[kind]||[]).filter(ref=>ref!==e.ref&&!(kind==='action'&&checked&&ref.split('.')[0]===e.instance));if(checked)list=[...list,e.ref];changed({...assembly,capture:{...assembly.capture,[kind]:list}})}} />)}</div></fieldset>)}
    {(()=>{
     const used=new Set(assembly.bindings.flatMap(binding=>[binding.source,binding.deadman]))
     const selected=assembly.capture.auxiliary||[]
     const task=sources.filter(e=>used.has(e.ref))
     const setChannels=(refs:string[], enabled:boolean)=>{
      const next=enabled?[...new Set([...selected,...refs])]:selected.filter(ref=>!refs.includes(ref))
      changed({...assembly,capture:{...assembly.capture,auxiliary:next}})
     }
     const fields=(channels:Endpoint[])=><div className="wc-form-grid">{channels.map(e=><Check key={e.ref} text={signalLabel(e)} value={selected.includes(e.ref)} onChange={checked=>setChannels([e.ref],checked)} />)}</div>
     const extra=sources.filter(e=>!used.has(e.ref)&&(selected.includes(e.ref)||questVisible(e)))
     const groups=[
      {title:'位姿与开合量', accepts:(e:Endpoint)=>['pose6d','scalar'].includes(e.port.semantic)&&!holdPort(e)},
      {title:'按键、摇杆与跟随状态', accepts:(e:Endpoint)=>e.port.semantic==='axes_buttons'||holdPort(e)},
      {title:'手部关键点', accepts:(e:Endpoint)=>e.port.semantic==='hand_landmarks'},
      {title:'其他信号', accepts:(e:Endpoint)=>!['pose6d','scalar','axes_buttons','hand_landmarks'].includes(e.port.semantic)},
     ]
     return <>
      {task.length>0&&<fieldset className="wc-pose"><legend>遥操输入</legend><Check text="记录当前映射使用的输入" value={task.every(e=>selected.includes(e.ref))} onChange={checked=>setChannels(task.map(e=>e.ref),checked)} /><details className="wc-details"><summary>映射通道 · 已选 {task.filter(e=>selected.includes(e.ref)).length} / {task.length}</summary>{fields(task)}</details></fieldset>}
      {extra.length>0&&<details className="wc-details"><summary>附加采集通道</summary>
       {inputs.map(([id,item])=>{
        const channels=extra.filter(e=>e.instance===id)
        if(!channels.length)return null
        const legacy=channels.filter(e=>isQuest(e)&&!currentQuestPort(e))
        const deviceGroups=groups.map(group=>({...group,channels:channels.filter(e=>!legacy.includes(e)&&group.accepts(e))}))
        return <fieldset className="wc-pose" key={id}><legend>{item.display_name||label(item.plugin)} · {id}</legend>
         {deviceGroups.map(group=>group.channels.length>0&&<details key={group.title} className="wc-details"><summary>{group.title} · 已选 {group.channels.filter(e=>selected.includes(e.ref)).length}</summary>{fields(group.channels)}</details>)}
         {legacy.length>0&&<details className="wc-details"><summary>已选兼容通道</summary>{fields(legacy)}</details>}
        </fieldset>
       })}
      </details>}
     </>
    })()}
   </Section>
   <Section title="图像记录" note="为每一路图像设置数据集中的名称。相机可以显示但不录制。">
    {endpoints.filter(e=>e.port.semantic==='image'&&e.port.direction==='output').map(e=>{const entry=Object.entries(assembly.capture.images).find(([,ref])=>ref===e.ref);return <div className="wc-camera-row" key={e.ref}><Check text={portLabel(e.ref)} value={!!entry} onChange={checked=>{const images={...assembly.capture.images};if(checked){let name=e.instance,n=2;while(name in images)name=e.instance+'_'+n++;images[name]=e.ref}else if(entry)delete images[entry[0]];changed({...assembly,capture:{...assembly.capture,images}})}} />{entry&&<label>数据集图像名称<input aria-label={e.instance+'图像名称'} defaultValue={entry[0]} key={entry[0]} onBlur={event=>{const key=event.target.value.trim();if(!/^[a-z][a-z0-9_-]*$/.test(key)||key!==entry[0]&&key in assembly.capture.images){setError(true);setMessage('图像名称需唯一，使用小写字母、数字或下划线。');event.target.value=entry[0];return}const images={...assembly.capture.images};delete images[entry[0]];images[key]=e.ref;changed({...assembly,capture:{...assembly.capture,images}})}} /></label>}</div>})}
    {!endpoints.some(e=>e.port.semantic==='image')&&<p className="wc-empty">未配置相机，此工作站将只记录数值数据。</p>}
   </Section>
  </fieldset>}

  {stage==='run'&&<>
   <Section title="运行设置" note="仿真模式与逆运动学模式独立。物理模式用于动力学测试，不代表已经完成实机参数标定。">
    {simPresent?<div className="wc-form-grid"><label>仿真模式<select aria-label="仿真模式" value={sim.mode} disabled={busy||session?.status==='ACTIVE'||!!session?.recording} onChange={e=>simOptions({mode:e.target.value as SimOptions['mode']})}><option value="kinematic">运动学仿真 · 验证 IK 与映射</option><option value="physics">动力学仿真 · 执行器与物理步进</option></select></label><label>仿真显示<select aria-label="仿真显示方式" value={sim.display} disabled={busy||!!session?.recording} onChange={e=>simOptions({display:e.target.value as SimOptions['display']})}><option value="web">浏览器画面</option><option value="local">本机 MuJoCo 窗口</option><option value="both">浏览器与本机窗口</option></select></label></div>:<p className="wc-note">当前未选用 MuJoCo 设备。</p>}
    {simPresent&&<p className="wc-note">本机窗口在运行服务的电脑上打开。改变动力学模式前，先暂停遥操并结束记录。</p>}
    {session?.simulation?.viewer&&<p className="wc-note">MuJoCo 窗口：{session.simulation.viewer.status==='running'?'已打开':'未打开'} {session.simulation.viewer.error}</p>}
   </Section>
   <Section title="运行诊断日志" note="随每次运行自动建立目录，独立于数据记录。写盘异步进行，日志故障不会撤销运动许可。">
    <fieldset className="wc-unframed" disabled={locked}>
     <Check text="保存本次运行的遥操作与运动诊断" value={logOptions.enabled} onChange={enabled=>changed({...assembly,metadata:{...assembly.metadata,diagnostic_logging:{...logOptions,enabled}}})} />
     <div className="wc-form-grid"><label>反馈采样上限 / Hz<NumberInput label="日志反馈采样上限" schema={{minimum:1,maximum:200}} value={logOptions.feedback_hz} onChange={feedback_hz=>changed({...assembly,metadata:{...assembly.metadata,diagnostic_logging:{...logOptions,feedback_hz}}})} /></label><label>单次日志容量上限 / MB<NumberInput label="单次日志容量上限" schema={{minimum:16,maximum:8192}} value={logOptions.max_run_mb} onChange={max_run_mb=>changed({...assembly,metadata:{...assembly.metadata,diagnostic_logging:{...logOptions,max_run_mb}}})} /></label></div>
    </fieldset>
    <p className="wc-note">记录输入、控制映射、IK 报告、指令回执和反馈；图像二进制不写入诊断日志。反馈来自运行时缓存采样，不等同于硬件原始采样率。</p>
    {session?.diagnostic_log&&<div className="wc-log-status"><strong>{({recording:'正在记录',closed:'已结束并保存',degraded:'日志不完整，请检查',disabled:'本次日志已关闭',not_started:'尚未开始'} as Record<string,string>)[session.diagnostic_log.state]||session.diagnostic_log.state}</strong><span>已写入 {Object.values(session.diagnostic_log.written).reduce((a,b)=>a+b,0)} 条 · 丢弃 {Object.values(session.diagnostic_log.dropped).reduce((a,b)=>a+b,0)} 条 · 待写入 {session.diagnostic_log.queued} 条</span>{session.diagnostic_log.directory&&<code>{session.diagnostic_log.directory}</code>}{session.diagnostic_log.error&&<p className="wc-message is-error">{session.diagnostic_log.error}</p>}</div>}
    <div className="wc-inline"><label>运行日志<select aria-label="选择运行日志" value={currentLogRun} onChange={e=>{setSelectedRun(e.target.value);setLogTail(null)}}><option value="">选择一次运行…</option>{session?.diagnostic_log?.directory&&!logRuns.some(r=>r.session_id===session.session_id)&&<option value={session.session_id}>当前运行 · {session.session_id.slice(0,8)}</option>}{logRuns.map(r=><option key={r.session_id} value={r.session_id}>{r.assembly_id} · {new Date(r.started_at_utc).toLocaleString()} · {r.session_id.slice(0,8)}</option>)}</select></label><button onClick={()=>void run(async()=>setLogRuns(await request<LogRun[]>('/diagnostics/runs')))}>刷新日志列表</button><button disabled={!currentLogRun} onClick={()=>void run(async()=>setLogTail(await request(`/diagnostics/runs/${currentLogRun}/tail?limit=50`)))}>查看最近记录</button></div>
    {!!logFiles.length&&<details className="wc-details"><summary>下载日志文件</summary><div className="wc-log-files">{[...new Set(logFiles)].map(file=><a key={file} href={`/api/v2/diagnostics/runs/${currentLogRun}/files/${file}`} download>{file}</a>)}</div></details>}
    {logEntry?.state==='incomplete'&&<p className="wc-message is-error">该次运行没有正常结束摘要，可能是进程中断；已写入的记录仍可下载。</p>}
    {logTail&&<details className="wc-details" open><summary>最近 {logTail.records.length} 条记录{logTail.partial?'（读取窗口有限）':''}</summary><pre>{logTail.records.map(row=>JSON.stringify(row)).join('\n')}</pre></details>}
   </Section>
   <Section title="检查与启动" note="配置检查不连接硬件；依赖检查也不替代设备现场核验。启动后保持未使能，确认设备状态后再允许运动。">
    <div className="wc-review"><div><span>输入设备</span><strong>{inputs.length}</strong></div><div><span>执行器</span><strong>{actuators.length}</strong></div><div><span>控制映射</span><strong>{assembly.bindings.length}</strong></div><div><span>记录通道</span><strong>{assembly.capture.state.length+assembly.capture.action.length}</strong></div></div>
    <div className="wc-run-actions"><button disabled={locked||!actuators.length} onClick={()=>void run(async()=>{await check()})}>检查配置</button><button disabled={locked||!actuators.length} onClick={()=>void run(async()=>{setPreflight(await request('/deployments/preflight',{assembly,deployment}));setMessage('依赖检查已返回，请查看检查详情。')})}>检查运行依赖</button><button disabled={locked||!actuators.length} onClick={()=>void run(async()=>{await save()})}>保存配置</button><button className="wc-primary" disabled={locked||!resolved||resolved.assembly_hash!==savedHash||savedRevision!==assembly.revision} onClick={()=>void run(async()=>{if(instances.some(([id,item])=>(device(id).backend||item.backend)==='real')&&!window.confirm('连接配置中的真实设备？启动后仍需单独允许运动。'))return;const state=await request<Session>('/sessions',{assembly_id:assembly.assembly_id,revision:savedRevision,assembly_hash:savedHash,request_id:crypto.randomUUID()});setSession(await request<Session>(`/sessions/${state.session_id}/state`));setMessage('工作站已启动。连接输入并检查状态后，可准备设备并允许运动。')})}>启动工作站</button></div>
    <p className="wc-note">{active?'工作站正在运行。停止后可编辑配置。':resolved&&resolved.assembly_hash===savedHash&&savedRevision===assembly.revision?'配置已保存，可以启动。':resolved?'配置检查通过，尚未保存当前修改。':'尚未检查当前配置。'}</p>
    {resolved&&<><p className="wc-note" data-testid="contract-dimensions">观测 {resolved.feature_contract.state_dim} 维 · 动作 {resolved.feature_contract.action_dim} 维 · 图像 {Object.keys(resolved.feature_contract.cameras||{}).length} 路</p>{resolved.diagnostics.map((d,i)=><p className={'wc-diagnostic '+(d.severity==='warning'?'is-warning':'')} key={i}>{d.message} {d.path}</p>)}<details className="wc-details"><summary>配置检查详情</summary><pre>{JSON.stringify(resolved,null,2)}</pre></details></>}
    {preflight!=null&&<details className="wc-details" open><summary>运行依赖检查结果</summary><pre>{JSON.stringify(preflight,null,2)}</pre></details>}
   </Section>
   {session&&<Section title="运行控制" note="Quest3 右手柄 grip、Vader M3 或键盘 S 开始遥操。暂停遥操会停止跟随，停止工作站会释放设备连接。">
    <div className="wc-device-heading"><h4 data-testid="session-status">{session.operator?.transition?.status==='RUNNING'?'控制权切换中':states[session.status]||session.status}</h4><code>{session.session_id.slice(0,12)}</code></div>
    <p className="wc-note">当前控制：{session.operator?.transition?.status==='RUNNING'?'切换中，输出已冻结':controlNames[session.operator?.activity||'paused']||session.operator?.activity}{session.operator?.paused&&session.operator.activity!=='paused'?' · 已暂停':''}{session.operator?.return_to?' · 可交还 '+(controlNames[session.operator.return_to]||session.operator.return_to):''}{session.operator?.work_position?' · 工作位 '+(workStates[session.operator.work_position.status]||session.operator.work_position.status):''}</p>
    {session.operator?.activity==='human'&&!session.operator.paused&&session.operator.transition?.status!=='RUNNING'&&assembly.bindings.map(binding=><p className="wc-note" key={binding.id}>{binding.target.split('.')[0]} · {binding.deadman?'按住控制键才跟随，松开后停止跟随。':'持续跟随，点击「暂停遥操」停止。'}</p>)}
    {session.operator?.transition&&<p className="wc-note" role="status">{handoffPhases[session.operator.transition.phase]||session.operator.transition.phase} · 已确认 {Object.keys(session.operator.transition.acks||{}).length} 条控制映射{session.operator.transition.error?' · '+session.operator.transition.error:''}</p>}
    {session.operator?.error&&<p className="wc-message is-error">{session.operator.error}</p>}
    {session.status!=='STOPPED'&&<OperatorKeyboard sessionId={session.session_id} onError={text=>{setError(true);setMessage(text)}} onUpdate={()=>void request<Session>(`/sessions/${session.session_id}/state`).then(setSession)} />}
    <div className="wc-run-actions">{[['connect_inputs','连接输入'],['prepare','准备设备'],['activate','开始遥操'],['disarm','暂停遥操'],['reset','复位故障'],['estop','急停'],['stop','停止工作站']].map(([op,title])=><button key={op} className={op==='estop'||op==='stop'?'wc-danger':op==='activate'?'wc-primary':''} disabled={((busy||session.operator?.transition?.status==='RUNNING')&&!['estop','disarm','stop'].includes(op))||session.status==='STOPPED'||op==='activate'&&!['READY_DISARMED','ACTIVE'].includes(session.status)||op==='disarm'&&session.status!=='ACTIVE'&&session.operator?.transition?.status!=='RUNNING'||op==='prepare'&&session.status!=='READY_DISARMED'||op==='reset'&&!['FAULTED','ESTOP_LATCHED'].includes(session.status)} onClick={()=>operate(op)}>{title}</button>)}</div>
    <div className="wc-health-list">{Object.entries(session.inputs||{}).map(([id,input])=><div key={id}><strong>{id}</strong><span>{input.connected?'已连接':'未连接'}</span><span>{input.fresh?'输入正常':'等待新输入'}</span></div>)}{Object.entries(session.components||{}).map(([id,state])=><div key={id}><strong>{id}</strong><span>{state.ready?'反馈就绪':'未就绪'}</span><span>{state.reason||''}</span></div>)}</div>
    {inputs.filter(([,i])=>i.plugin==='morla.input.mobile').map(([id])=><button key={id} disabled={busy||!active} onClick={()=>void run(async()=>{const result=await request<{token:string}>(`/sessions/${session.session_id}/inputs/${id}/pair`,{});setPairLink(`${location.origin}/mobile#session=${session.session_id}&input=${id}&token=${encodeURIComponent(result.token)}`)})}>配对手机 · {id}</button>)}
    {pairLink&&<p><a href={pairLink} target="_blank" rel="noreferrer">打开手机控制页面</a><br/><small>{pairLink}</small></p>}
    {Object.entries(session.controllers||{}).flatMap(([id,reports])=>Object.entries(reports).filter(([,r])=>Object.keys(r).length>0).map(([plugin,raw])=>{const r=presentIkReport(raw) as IkReport;return <div className="wc-ik-report" key={id+plugin}><div><strong>{id} · {r.solver?.includes('differential')?'连续微分跟随':'严格位姿求解'}</strong><span>{r.error||reasons[r.reason||'']||(r.success?'已达到求解精度':'等待目标')}</span></div><dl><div><dt>位置误差</dt><dd>{metric(r.raw_position_error_m??r.position_error_m,1000)} <small>mm</small></dd></div><div><dt>姿态误差</dt><dd>{metric(r.raw_angle_error_rad??r.angle_error_rad,180/Math.PI)} <small>°</small></dd></div><div><dt>奇异性指标</dt><dd>{metric(r.singularity_proximity,100,0)} <small>%</small></dd></div><div><dt>已吸收位移</dt><dd>{metric(r.absorbed_position_m,1000,1)} <small>mm</small></dd></div></dl><p className="wc-note">误差由候选关节正运动学计算，不是实机测量精度。报告更新距今 {metric(r.age_ms,1,0)} ms。</p></div>}))}
    <div className="wc-inline">{session.operator?.paused&&['policy','playback'].includes(session.operator.activity)&&<button disabled={busy||session.operator.transition?.status==='RUNNING'} onClick={()=>void run(async()=>{await request(`/sessions/${session.session_id}/operator/action`,{action:'resume',request_id:crypto.randomUUID()});setSession(await request<Session>(`/sessions/${session.session_id}/state`))})}>继续策略／回放</button>}<button disabled={busy||session.operator?.transition?.status==='RUNNING'||!session.operator?.return_to} onClick={()=>void run(async()=>{await request(`/sessions/${session.session_id}/operator/action`,{action:'record_or_release',request_id:crypto.randomUUID()});setSession(await request<Session>(`/sessions/${session.session_id}/state`))})}>交还策略／回放</button><button disabled={busy||session.operator?.transition?.status==='RUNNING'||!!session.recording||!active} onClick={()=>void run(async()=>{await request(`/sessions/${session.session_id}/operator/action`,{action:'workpos',request_id:crypto.randomUUID()});setSession(await request<Session>(`/sessions/${session.session_id}/state`))})}>直接回工作位</button><button className="wc-danger" disabled={busy||session.operator?.transition?.status==='RUNNING'||!session.recording} onClick={()=>void run(async()=>{await request(`/sessions/${session.session_id}/operator/action`,{action:'record_discard',request_id:crypto.randomUUID()});setSession(await request<Session>(`/sessions/${session.session_id}/state`));setMessage('本段已丢弃。')})}>丢弃本段</button></div>
    {active&&sim.display!=='local'&&<div className="wc-preview-grid">{instances.filter(([id,i])=>descriptor(i)?.ports.rgb?.semantic==='image'&&(device(id).backend||i.backend)==='sim').map(([id])=><SimPreview key={id} sessionId={session.session_id} cameraId={id} />)}</div>}
    <div className="wc-record-row"><label>记录任务<input aria-label="记录任务" disabled={!!session.recording} value={task} onChange={e=>setTask(e.target.value)} /></label><button disabled={busy||session.operator?.transition?.status==='RUNNING'||!active||!!session.recording} onClick={()=>void run(async()=>{await request(`/sessions/${session.session_id}/recordings`,{task});setSession(await request<Session>(`/sessions/${session.session_id}/state`));setMessage('数据记录已开始。')})}>开始记录</button><button disabled={busy||session.operator?.transition?.status==='RUNNING'||!session.recording} onClick={()=>void run(async()=>{const result=await request<{status?:string;failure?:string}>(`/sessions/${session.session_id}/recordings/stop`,{});setSession(await request<Session>(`/sessions/${session.session_id}/state`));setError(result.status==='failed'||!!result.failure);setMessage(recordingSaveMessage(result))})}>结束并保存</button><span className="wc-note">{session.recording?'正在记录':'未记录'}</span></div>
    <details className="wc-details"><summary>驱动与控制器诊断</summary><pre>{JSON.stringify(session,null,2)}</pre></details>
   </Section>}
  </>}

  <footer className="wc-flow-footer"><span>{active?'运行期间设置为只读。':'选择项会自动生成装配与控制配置。'}</span>{stage!=='run'?<button className="wc-primary" onClick={nextStage}>继续：{stages[stages.findIndex(s=>s.id===stage)+1]?.title}</button>:<button className="wc-text-button" onClick={()=>setStage('devices')}>返回设备设置</button>}</footer>
  <details className="wc-details wc-file-tools"><summary>配置文件 · 导入与导出</summary><p className="wc-note">普通配置使用上方表单即可。导入会替换当前草稿；未识别的字段会保留在配置中。</p><div className="wc-inline"><button disabled={locked} onClick={()=>{const file=new Blob([JSON.stringify({assembly,deployment},null,2)],{type:'application/json'});const url=URL.createObjectURL(file);const a=document.createElement('a');a.href=url;a.download=assembly.assembly_id+'.json';a.click();URL.revokeObjectURL(url)}}>导出配置文件</button><label className="wc-file-input">导入配置文件<input disabled={locked} type="file" accept=".json,application/json" onChange={e=>{const file=e.target.files?.[0];if(file)void run(async()=>{const text=await file.text();let value:unknown;try{value=JSON.parse(text)}catch{throw new Error(`不是有效的 JSON 配置文件：${file.name}（文件开头是「${text.slice(0,40).replace(/\n/g,' ')}」）`)}loadImport(value)});e.target.value=''}} /></label></div><details><summary>开发者文本编辑</summary><textarea aria-label="高级配置文本" rows={12} value={importText} onChange={e=>setImportText(e.target.value)} /><div className="wc-inline"><button disabled={locked} onClick={()=>setImportText(JSON.stringify({assembly,deployment},null,2))}>显示当前配置</button><button disabled={locked||!importText} onClick={()=>void run(async()=>{let value:unknown;try{value=JSON.parse(importText)}catch{throw new Error('导入文本不是有效的 JSON（必须以 { 开头，且不是 YAML）')}loadImport(value)})}>应用导入配置</button></div></details></details>
 </div>
 function loadImport(raw:unknown){
  const document=parseWorkcellDocument(raw),a=document.assembly
  if(!window.confirm('载入配置并替换当前未保存的修改？'))return
  changed(normalise(a));setDeployment(document.deployment||{});setSavedRevision(saved.find(x=>x.assembly_id===a.assembly_id)?.revision||0);setSavedHash('');setMessage('配置已导入，请检查表单并保存新版本。');setStage('devices')
 }
}
const subCard:React.CSSProperties={background:'#15191d',border:'1px solid #30363b',padding:14}
const mutedSmall:React.CSSProperties={color:'#a2aab1',fontSize:12,lineHeight:1.6}
const ghostBtn:React.CSSProperties={background:'transparent',color:'#c4cbd0',border:'1px solid #3b434a',borderRadius:4,padding:'5px 10px',fontSize:12,cursor:'pointer'}
