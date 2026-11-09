import { useEffect, useRef, useState } from 'react'

import { pluginBindingDefaults, preferredController, compatible, initial, label as componentLabel, normalise, parseWorkcellDocument, portLabel } from '../lib/workcell'
import type { Assembly, Binding, Component, Deployment, Device, Instance, Template, SimOptions } from '../lib/workcell'
import './workcell.css'

import { request, awaitOperation, awaitControlTransition, WorkstationError, explainError } from '../api/workstation'
import { useWorkstationSession } from '../hooks/useWorkstationSession'
import { useConfirmation } from './workstation/OperationFeedback'
import type { OperationRecord } from './workstation/OperationFeedback'
import { stages, snapshot, changeBackend, preserveBindingParameters, describeChanges, obsoleteFields, issueLocation } from '../lib/workstationPresentation'
import { pushToast } from '../hooks/useToast'
import { Resolved, LogRun, Session, Endpoint } from './workstation/shared'
import { ReferenceWorkspace } from './workstation/ReferenceWorkspace'
export type WorkstationModel=ReturnType<typeof useWorkstationModel>
function useWorkstationModel(visible:boolean){

 const [catalog,setCatalog]=useState<Component[]>([]),[assembly,setAssembly]=useState<Assembly>(initial),[deployment,setDeployment]=useState<Deployment>({})
 const [resolved,setResolved]=useState<Resolved|null>(null),[preflight,setPreflight]=useState<{compile_valid:boolean;preflight:Record<string,unknown>;resolved:Resolved}|null>(null),[session,setSession]=useState<Session|null>(null)
 const [savedRevision,setSavedRevision]=useState(0),[savedHash,setSavedHash]=useState(''),[saved,setSaved]=useState<{assembly_id:string;revision:number}[]>([])
 const [stage,setStage]=useState('devices'),[message,setMessage]=useState(''),[error,setError]=useState(false),[busy,setBusy]=useState(false)
 const [template,setTemplate]=useState(''),[pairLink,setPairLink]=useState(''),[task,setTask]=useState('demonstration')
 const [selectedDevice,setSelectedDevice]=useState(''),[selectedChain,setSelectedChain]=useState('')
 const [savedSnapshot,setSavedSnapshot]=useState(''),[checkedSnapshot,setCheckedSnapshot]=useState('')
 const [checkError,setCheckError]=useState(''),[recovering,setRecovering]=useState(true),[errorDetails,setErrorDetails]=useState('')
 const [checkIssues,setCheckIssues]=useState<{path:string;message:string}[]>([])
 const [initialization,setInitialization]=useState<{title:string;status:string;detail?:string}[]>([])
 const initializationGeneration=useRef(0)
 const [records,setRecords]=useState<OperationRecord[]>([])
 const startRequest=useRef<{identity:string;id:string}|null>(null)
 const pendingActions=useRef(new Set<string>()),draftEpoch=useRef(0),checkAbort=useRef<AbortController|null>(null)
 const {confirm,pending,answer}=useConfirmation()
 const selected=assembly.instances[selectedDevice]?selectedDevice:Object.keys(assembly.instances)[0]||''
 const selectedBinding=assembly.bindings.some(b=>b.id===selectedChain)?selectedChain:assembly.bindings[0]?.id||''
 const {connectionError,refreshSession}=useWorkstationSession(session,setSession)
 const dirty=savedSnapshot!==snapshot(assembly,deployment)
 const staleFields=obsoleteFields(assembly,deployment)

 const [importText,setImportText]=useState(''),[copyName,setCopyName]=useState('')
 const [logRuns,setLogRuns]=useState<LogRun[]>([]),[selectedRun,setSelectedRun]=useState(''),[logTail,setLogTail]=useState<{records:{kind:string;wall_time_ns:number;component_id?:string;binding_id?:string;data:unknown}[];partial:boolean}|null>(null)
 const label=(key:string)=>componentLabel(key,catalog)
 const [templates,setTemplates]=useState<Template[]>([])
 const [templateCache,setTemplateCache]=useState<Record<string,Assembly>>({})
 const active=!!session&&session.status!=='STOPPED', locked=active||busy||recovering
 const descriptor=(item?:Instance)=>catalog.find(d=>d.plugin_id===item?.plugin)
 const instances=Object.entries(assembly.instances), actuators=instances.filter(([,i])=>descriptor(i)?.kind==='actuator'),inputs=instances.filter(([,i])=>descriptor(i)?.kind==='input')
 const endpoints:Endpoint[]=instances.flatMap(([id,item])=>Object.entries(descriptor(item)?.ports||{}).map(([name,port])=>({ref:id+'.'+name,instance:id,port})))
 const sources=endpoints.filter(e=>descriptor(assembly.instances[e.instance])?.kind==='input'&&e.port.direction==='output')
 const targets=endpoints.filter(e=>descriptor(assembly.instances[e.instance])?.kind==='actuator'&&e.port.direction==='input')
 const rootFrame=(item:Instance)=>{const d=descriptor(item);return d?.kind==='sensor'&&d.ports.rgb?'optical':d?.geometry?.frames?.includes('base')?'base':d?.geometry?.frames?.includes('mount')?'mount':undefined}
 const mountable=instances.filter(([,item])=>descriptor(item)?.kind!=='input').flatMap(([id,item])=>rootFrame(item)?[id+'.'+rootFrame(item)]:[])
 const frames=['world',...instances.filter(([,i])=>descriptor(i)?.kind!=='input').flatMap(([id,item])=>(descriptor(item)?.geometry?.frames||[]).map(frame=>id+'.'+frame))]
 const invalidate=()=>{draftEpoch.current++;checkAbort.current?.abort();setCheckedSnapshot('');setCheckError('');setCheckIssues([]);setResolved(null);setPreflight(null)}
 const changed=(next:Assembly)=>{if(active)return;invalidate();setAssembly(next);setPairLink('')}
 const changeDeployment=(next:Deployment)=>{if(active)return;invalidate();setDeployment(next)}
 const run=async(fn:(requestId:string)=>Promise<void>,title='更新工作站')=>{
  if(pendingActions.current.has(title))return
  pendingActions.current.add(title);setBusy(true);setError(false);setMessage('')
  const id=crypto.randomUUID()
  setRecords(old=>[{id,title,status:'执行中',started:Date.now(),target:session?.session_id||assembly.assembly_id},...old].slice(0,20))
  try{await fn(id);setRecords(old=>old.map(r=>r.id===id?{...r,status:'已完成'}:r))}
  catch(e){if(e instanceof WorkstationError&&e.detail==='assembly revision conflict')void request<typeof saved>('/assemblies').then(setSaved).catch(()=>{});const text=(e as Error).message;setError(true);setMessage(text);if(!pending)setErrorDetails(e instanceof WorkstationError?text+'\n\n接口详情：\n'+JSON.stringify(e.detail,null,2):text);setRecords(old=>old.map(r=>r.id===id?{...r,status:text.includes('待确认')?'待确认':'未完成',message:text}:r))}
  finally{pendingActions.current.delete(title);setBusy(pendingActions.current.size>0)}
 }
 const applySaved=(item:{assembly:Assembly;deployment?:Deployment;resolved:Resolved})=>{invalidate();const next=normalise(item.assembly),migrated=JSON.stringify(next.instances)!==JSON.stringify(item.assembly.instances);setAssembly(next);setDeployment(item.deployment||{});setResolved(migrated?null:item.resolved);setSavedRevision(item.assembly.revision);setSavedHash(migrated?'':item.resolved.assembly_hash);setSavedSnapshot(snapshot(migrated?item.assembly:next,item.deployment||{}));setTemplate(templates.find(t=>t.assembly_id===next.assembly_id)?.id||'');if(migrated)setMessage('已更新为统一手柄按键，请保存新版本后启动。')}
 useEffect(()=>{
  request<{components:Component[]}>('/components').then(x=>setCatalog(x.components)).catch(e=>{setError(true);setMessage(e.message)})
  request<typeof saved>('/assemblies').then(setSaved).catch(e=>{setError(true);setMessage('读取已保存配置失败：'+e.message)})
  request<Template[]>('/templates').then(items=>{setTemplates(items);setTemplate(current=>current||items[0]?.id||'')}).catch(e=>{setError(true);setMessage(e.message)})
  const epoch=draftEpoch.current
  void request<Session|null>('/active-session').then(async current=>{if(!current||current.status==='STOPPED'||!current.assembly_id||!current.revision)return;const item=await request<{assembly:Assembly;deployment?:Deployment;resolved:Resolved}>(`/assemblies/${current.assembly_id}/revisions/${current.revision}`);if(epoch===draftEpoch.current)applySaved(item);setSession(current);setStage('run')}).catch(e=>{setError(true);setMessage(e.message)}).finally(()=>setRecovering(false))
 },[])
 useEffect(()=>{setTemplate(templates.find(t=>t.assembly_id===assembly.assembly_id)?.id||'')},[templates,assembly.assembly_id])
 const setInstance=(id:string,item:Instance)=>changed({...assembly,instances:{...assembly.instances,[id]:item}})
 function add(plugin:string){
  const d=catalog.find(d=>d.plugin_id===plugin);if(!d||locked)return
  const root=d.kind==='input'?(d.plugin_id.includes('quest')?'quest':d.plugin_id.includes('vader')?'pad':'input'):d.kind==='sensor'?'camera':d.plugin_id.includes('hand')?'hand':d.plugin_id.includes('gripper')||d.plugin_id.includes('epg')?'gripper':'arm'
  let id=root,n=2;while(assembly.instances[id])id=root+'_'+n++
  const next=structuredClone(assembly);next.instances[id]={plugin:d.plugin_id,version:d.plugin_version,backend:d.backend_variants?.fake?.available!==false&&d.backend_variants?.fake?'fake':Object.keys(d.backend_variants||{}).find(k=>d.backend_variants![k].available!==false)}
  if(d.kind==='actuator'){next.capture.state.push(id+'.position');const target=d.ports.closure?'closure':d.ports.position_target?'position_target':Object.entries(d.ports).find(([,p])=>p.direction==='input')?.[0];if(target)next.capture.action.push(id+'.'+target)}
  if(d.ports.rgb){next.capture.images[id]=id+'.rgb';next.instances[id].parameters={width:640,height:480,fps:30}}
  const rootFrame=d.kind==='sensor'&&d.ports.rgb?'optical':d.geometry?.frames?.includes('base')?'base':d.geometry?.frames?.includes('mount')?'mount':d.geometry?.frames?.includes('optical')?'optical':null
  if(rootFrame&&d.kind!=='input') next.mounts.push({child:id+'.'+rootFrame,parent:'world',transform:rootFrame==='optical'?[1.5,1.5,1.4,0,0,0,1]:[0,0,0,0,0,0,1]})
  changed(next);setSelectedDevice(id);setStage('devices');setTimeout(()=>document.querySelector<HTMLInputElement>('.wc-editor input[aria-label="设备显示名称"]')?.focus(),0)
 }
 async function remove(id:string){if(!await confirm(`移除设备 ${id}？\n相关控制链：${assembly.bindings.filter(b=>[b.source,b.target,b.deadman||''].some(x=>x.startsWith(id+'.'))).map(b=>b.id).join('、')||'无'}\n安装关系：${assembly.mounts.filter(m=>[m.child,m.parent].some(x=>x.startsWith(id+'.'))).map(m=>m.child+' → '+m.parent).join('、')||'无'}\n记录通道：${[...assembly.capture.state,...assembly.capture.action,...(assembly.capture.auxiliary||[]),...Object.values(assembly.capture.images)].filter(x=>x.startsWith(id+'.')).join('、')||'无'}\n该设备的控制组成员与部署项也将清理。`))return
  const ids=Object.keys(assembly.instances),index=ids.indexOf(id);setSelectedDevice(ids[index+1]||ids[index-1]||'')
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
  changed({...assembly,bindings:[...assembly.bindings,binding]});setSelectedChain(binding.id)
 }
 function connect(source:string,target:string,controller:string){
  if(locked||!choices(source,target).includes(controller))return
  if(assembly.bindings.some(b=>b.target.split('.')[0]===target.split('.')[0])){setError(true);setMessage('该执行器已有控制链，请点击控制器节点修改，或先移除原连接。');return}
  let n=1;while(assembly.bindings.some(b=>b.id==='control_'+n))n++
  const binding={...buildBinding(source,target,'control_'+n),controller,parameters:defaults(controller,source,target)}
  const endpoint=sources.find(e=>e.ref===source)
  if(endpoint&&(device(target.split('.')[0]).backend||assembly.instances[target.split('.')[0]]?.backend)==='real')binding.deadman=standardHold(endpoint)?.ref||binding.deadman
  changed({...assembly,bindings:[...assembly.bindings,binding]});setSelectedChain(binding.id)
 }
 const patchBinding=(index:number,patch:Partial<Binding>)=>changed({...assembly,bindings:assembly.bindings.map((b,i)=>i===index?{...b,...patch}:b)})
 async function updateBinding(index:number,key:'source'|'target'|'controller',value:string){
  const old=assembly.bindings[index],target=key==='target'?value:old.target
  const source=key==='source'?value:key==='controller'&&!choices(old.source,target).includes(value)?sources.find(e=>questVisible(e)&&choices(e.ref,target).includes(value))?.ref||old.source:old.source
  const next=buildBinding(source,target,old.id),controller=key==='controller'?value:choices(source,target).includes(old.controller)?old.controller:next.controller
  if(source===old.source&&target===old.target&&controller===old.controller)return
  const selected=sources.find(e=>e.ref===source)
  const previous=sources.find(e=>e.ref===old.source)
  const previousHold=previous&&standardHold(previous)
  const deadman=selected&&source!==old.source&&old.deadman&&old.deadman===previousHold?.ref
   ?standardHold(selected)?.ref||old.deadman:old.deadman
  const candidate=preserveBindingParameters(old,{...old,source,target,controller,deadman,input_basis_id:next.input_basis_id,parameters:defaults(controller,source,target)},catalog)
  if(await confirm('切换控制链，请核对以下变更：\n'+describeChanges(old,candidate)))patchBinding(index,candidate)
 }
 const device=(id:string)=>deployment.instances?.[id]||{}
 const patchDevice=(id:string,patch:Partial<Device>)=>changeDeployment({...deployment,deployment_version:1,deployment_id:deployment.deployment_id||assembly.assembly_id+'_hardware',instances:{...deployment.instances,[id]:{...device(id),...patch}}})
 async function backend(id:string,value:string){
  const next=changeBackend(assembly,deployment,id,value)
  if(await confirm('切换运行后端，保留连接参数与显式覆盖。请核对新后端是否适用：\n'+describeChanges({assembly,deployment},next))){changeDeployment(next.deployment);changed(next.assembly)}
 }
 function duplicate(id:string){
  const next=structuredClone(assembly);let copy=id+'_copy',n=2;while(next.instances[copy])copy=id+'_copy'+n++
  next.instances[copy]={...structuredClone(assembly.instances[id]),display_name:(assembly.instances[id].display_name||label(assembly.instances[id].plugin))+' 副本',backend:'fake'}
  delete next.instances[copy].session_id
  for(const key of [...(descriptor(assembly.instances[id])?.web?.connection_fields||[]),'device_id','resource_key','session_id','serial'])delete next.instances[copy].parameters?.[key]
  const d=descriptor(assembly.instances[id]);if(!d?.backend_variants?.fake||d.backend_variants.fake.available===false)next.instances[copy].backend=Object.keys(d?.backend_variants||{}).find(k=>d?.backend_variants?.[k].available!==false)
  changed(next);setSelectedDevice(copy);setMessage('已复制设备设置。请单独核对连接身份、安装、控制链和记录通道。')
 }
 const nextStage=()=>setStage(stages[Math.min(stages.findIndex(s=>s.id===stage)+1,stages.length-1)].id)
 async function loadTemplate(){if(!template||!await confirm('载入预设将替换当前草稿，继续？'))return;const a=templateCache[template]||normalise(await request<Assembly>('/templates/'+template));setTemplateCache(cache=>({...cache,[template]:a}));const revision=saved.find(x=>x.assembly_id===a.assembly_id)?.revision||0;changed({...structuredClone(a),revision:revision+1});setDeployment({});setSavedRevision(revision);setSavedHash('');setMessage('已载入机器人与输入标定，可继续检查设备设置。')}
 async function check(){
  checkAbort.current?.abort();const abort=new AbortController();checkAbort.current=abort
  const epoch=draftEpoch.current,key=snapshot(assembly,deployment);setCheckError('');setPreflight(null);setCheckedSnapshot('');setCheckIssues([])
  try{const result=await request<{compile_valid:boolean;preflight:Record<string,unknown>;resolved:Resolved}>('/deployments/preflight',{assembly,deployment},abort.signal)
   if(epoch!==draftEpoch.current)throw new Error('配置已变化，本次检查结果已过期。')
   setResolved(result.resolved);setPreflight(result);setCheckedSnapshot(key);setMessage('检查已返回；配置检查不包含真机探测。');return result.resolved
  }catch(e){if(epoch===draftEpoch.current){setCheckError((e as Error).message);const detail=(e as WorkstationError).detail;if(Array.isArray(detail))setCheckIssues(detail.filter(d=>typeof d?.message==='string').map(d=>({path:typeof d.path==='string'?d.path:'',message:explainError(d.message)})))}throw e}
 }
 async function save(enterRun=false){
  const epoch=draftEpoch.current
  if(enterRun&&checkedSnapshot!==snapshot(assembly,deployment))throw new Error('请先检查当前配置。')
  const next={...assembly,revision:savedRevision+1}
  const item=await request<{assembly:Assembly;deployment:Deployment;resolved:Resolved}>('/assemblies',{assembly:next,deployment,expected_revision:savedRevision})
  if(epoch!==draftEpoch.current)throw new Error('保存已完成，但草稿已变化；保留当前编辑，请重新打开保存版本核对。')
  applySaved(item);setCheckedSnapshot(enterRun?snapshot(item.assembly,item.deployment):'');setSaved(await request<typeof saved>('/assemblies'));setMessage(`配置已保存，版本 ${next.revision}。`);pushToast('配置已保存','success');if(enterRun)setStage('run')
 }
 async function saveAs(name:string){
  const epoch=draftEpoch.current
  if(!/^[A-Za-z][A-Za-z0-9_-]*$/.test(name)||saved.some(s=>s.assembly_id===name))throw new Error('请输入未使用的配置名称（字母开头，字母、数字、下划线或连字符）。')
  const item=await request<{assembly:Assembly;deployment:Deployment;resolved:Resolved}>('/assemblies',{assembly:{...assembly,assembly_id:name,revision:1},deployment,expected_revision:0})
  if(epoch!==draftEpoch.current)throw new Error('另存为已完成，但草稿已变化；当前编辑已保留，请重新打开保存版本核对。')
  applySaved(item);setSaved(await request<typeof saved>('/assemblies'));setCheckedSnapshot('');setMessage('已另存为 '+name)
 }
 async function sessionOperation(operation:string,sid:string,requestId:string=crypto.randomUUID()){
  const reply=await request<{status?:string;error?:string}>(`/sessions/${sid}/operations`,{operation,request_id:requestId})
  if(operation==='connect_inputs'){if(reply.error||reply.status==='FAILED')throw new Error(reply.error||'输入接收服务启动失败')}
  else await awaitOperation(sid,reply,requestId)
  try{await refreshSession(sid,true)}catch(e){throw new Error('操作回执已完成，运行状态待确认：'+(e as Error).message)}
 }
 async function initialize(requestId:string=crypto.randomUUID()){
  const generation=++initializationGeneration.current
  const titles=['建立运行会话','启动输入接收','设备就绪检查']
  setInitialization(titles.map(title=>({title,status:'待执行'})))
  const step=async(index:number,fn:()=>Promise<void>)=>{
   if(generation!==initializationGeneration.current)throw new Error('启动已中断，请查看当前运行状态。')
   setInitialization(rows=>rows.map((row,i)=>i===index?{...row,status:'执行中'}:row))
   try{await fn();if(generation!==initializationGeneration.current)throw new Error('启动已中断。');setInitialization(rows=>rows.map((row,i)=>i===index?{...row,status:'已完成'}:row))}
   catch(e){setInitialization(rows=>rows.map((row,i)=>i===index?{...row,status:(e as Error).message.includes('待确认')?'待确认':'未完成',detail:(e as Error).message}:row));throw e}
  }
  let current=session
  await step(0,async()=>{
   if(!current||current.status==='STOPPED'){
    const existing=await request<Session|null>('/active-session')
    if(existing&&existing.status!=='STOPPED'){
     setSession(existing)
     if(existing.assembly_id!==assembly.assembly_id||existing.revision!==savedRevision)throw new Error('服务端已有其他工作站运行，请刷新恢复后查看。')
     current=existing
    }
   }
   if(!current||current.status==='STOPPED'){
    if(dirty||!resolved||resolved.assembly_hash!==savedHash)throw new Error('请检查并保存当前配置后启动。')
    if(instances.some(([id,item])=>(device(id).backend||item.backend)==='real')&&!await confirm('连接配置中的真实设备？启动后仍需单独开始遥操。'))throw new Error('已取消设备连接。')
    const identity=JSON.stringify([assembly.assembly_id,savedRevision,savedHash]);if(startRequest.current?.identity!==identity)startRequest.current={identity,id:requestId+'-session'}
    current=await request<Session>('/sessions',{assembly_id:assembly.assembly_id,revision:savedRevision,assembly_hash:savedHash,request_id:startRequest.current!.id})
    setSession(current)
   }
  })
  const needsReceiver=inputs.some(([id,item])=>(device(id).backend||item.backend)==='real'&&item.plugin!=='morla.input.mobile')
  if(needsReceiver)await step(1,()=>sessionOperation('connect_inputs',current!.session_id,requestId+'-inputs'))
  else setInitialization(rows=>rows.map((row,i)=>i===1?{...row,status:'无需启动',detail:'测试数据或手机配对不需要 ROS 接收服务。'}:row))
  await step(2,()=>sessionOperation('prepare',current!.session_id,requestId+'-prepare'))
  setMessage('启动已完成。等待有效输入，确认状态后单独开始遥操。')
 }
 const operate=(operation:string)=>void run(async(requestId)=>{
  const sid=session!.session_id
  if(operation==='activate'&&instances.some(([id,item])=>descriptor(item)?.kind==='actuator'&&(device(id).backend||item.backend)==='real')&&!await confirm('允许真实机器人响应当前控制输入？请核对输入方向、工具坐标系与周围环境。'))return
  if(['estop','stop'].includes(operation)&&!await confirm(operation==='estop'?'急停当前机器人？':'停止当前运行并释放设备连接？'))return
  if(['disarm','estop','stop'].includes(operation))initializationGeneration.current++
  if(operation==='activate'){
   await operatorAction('teleop_start',sid,requestId)
  }else {await sessionOperation(operation,sid,requestId);if(operation==='stop')startRequest.current=null}
  setMessage('操作已完成，已更新运行状态。')
 },({activate:'开始遥操',prepare:'设备就绪检查',connect_inputs:'连接输入',disarm:'暂停遥操',estop:'急停',stop:'停止工作站',reset:'复位故障'} as Record<string,string>)[operation]||operation)
 async function operatorAction(action:string,sid=session!.session_id,requestId:string=crypto.randomUUID()){
  const reply=await request<{status?:string;error?:string}>(`/sessions/${sid}/operator/action`,{action,request_id:requestId})
  if(reply.error||reply.status==='FAILED')throw new Error(reply.error||'操作未完成')
  if(['teleop_start','resume','record_or_release','workpos'].includes(action))await awaitControlTransition(()=>refreshSession(sid,true),action)
  else await refreshSession(sid,true)
 }
 function locate(path:string){const place=issueLocation(path,assembly);setSelectedDevice(place.id);setSelectedChain(place.binding);setStage(place.stage);setTimeout(()=>{const key=place.field;const field=Array.from(document.querySelectorAll<HTMLElement>('[data-field]')).find(el=>el.dataset.field===key);let parent=field?.parentElement;while(parent){if(parent instanceof HTMLDetailsElement)parent.open=true;parent=parent.parentElement}field?.scrollIntoView({block:'center'});field?.querySelector<HTMLElement>('input,select,button')?.focus()},50)}
 const sim=session?.simulation||assembly.metadata?.simulation||{mode:'physics' as const,display:'web' as const}
 function simOptions(patch:Partial<SimOptions>){if(active)void run(async()=>{await request(`/sessions/${session!.session_id}/simulation`,patch);await refreshSession(session!.session_id,true)});else changed({...assembly,metadata:{...assembly.metadata,simulation:{...sim,...patch}}})}
 const logOptions={enabled:true,feedback_hz:50,queue_size:4096,segment_mb:16,max_run_mb:1024,...assembly.metadata?.diagnostic_logging}
 const currentLogRun=selectedRun||session?.session_id||logRuns[0]?.session_id||''
 const logEntry=logRuns.find(r=>r.session_id===currentLogRun)
 const logFiles=currentLogRun===session?.session_id&&session?.diagnostic_log?.directory?['manifest.json',...session.diagnostic_log.files,...(session.status==='STOPPED'?['summary.json']:[])]:logEntry?.files||[]
 useEffect(()=>{if(stage!=='run')return;void request<LogRun[]>('/diagnostics/runs').then(setLogRuns).catch(()=>{})},[stage,session?.session_id,session?.status])
 const count={input:inputs.length,actuator:actuators.length,sensor:instances.filter(([,i])=>descriptor(i)?.kind==='sensor').length}
 const simPresent=instances.some(([id,i])=>(device(id).backend||i.backend)==='sim')


 async function loadImport(raw:unknown){
  const document=parseWorkcellDocument(raw),a=document.assembly
  if(!await confirm('载入配置并替换当前未保存的修改？'))return
  changed(normalise(a));setDeployment(document.deployment||{});setSavedRevision(saved.find(x=>x.assembly_id===a.assembly_id)?.revision||0);setSavedHash('');setMessage('配置已导入，请检查表单并保存新版本。');setStage('devices')
 }

return {connect,operatorAction,checkIssues, initialization, refreshSession, visible, active, actuators, add, addBinding, answer, applySaved, assembly, backend, busy, catalog, changeDeployment, changed, check, checkError, checkedSnapshot, choices, confirm, connectionError, copyName, count, currentLogRun, currentQuestPort, deployment, descriptor, device, dirty, duplicate, endpoints, error, errorDetails, frames, holdPort, importText, initialize, inputs, instances, isQuest, label, loadImport, loadTemplate, locate, locked, logEntry, logFiles, logOptions, logRuns, logTail, message, mountable, nextStage, operate, pairLink, patchBinding, patchDevice, pending, portName, preflight, questVisible, records, recovering, remove, resolved, run, save, saveAs, saved, savedRevision, selected, selectedBinding, session, setCopyName, setError, setErrorDetails, setImportText, setInstance, setLogRuns, setLogTail, setMessage, setPairLink, setSavedHash, setSavedRevision, setSelectedChain, setSelectedDevice, setSelectedRun, setSession, setStage, setTask, setTemplate, signalLabel, sim, simOptions, simPresent, sources, stage, staleFields, standardHold, targets, task, template, templates, updateBinding}
}
export function ComposableTab({visible=true,onOpenLegacy}:{visible?:boolean;onOpenLegacy?:(id:string)=>void}){const model=useWorkstationModel(visible);return <ReferenceWorkspace m={model} onOpenLegacy={onOpenLegacy}/>}
