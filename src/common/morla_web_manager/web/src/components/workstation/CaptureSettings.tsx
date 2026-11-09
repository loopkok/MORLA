import { NumberInput } from '../WorkcellFields'
import { portLabel } from '../../lib/workcell'

import type { WorkstationModel } from '../ComposableTab'
import { Endpoint, Section, Check } from './shared'
export function CaptureSettings({selected:deviceId, task:recordTask, setTask, session, logOptions,assembly, changed, currentQuestPort, descriptor, endpoints, holdPort, inputs, isQuest, label, locked, questVisible, setError, setMessage, signalLabel, sources, stage, targets}:Pick<WorkstationModel,'selected'|'task'|'setTask'|'session'|'logOptions'|'assembly'|'changed'|'currentQuestPort'|'descriptor'|'endpoints'|'holdPort'|'inputs'|'isQuest'|'label'|'locked'|'questVisible'|'setError'|'setMessage'|'signalLabel'|'sources'|'stage'|'targets'>){return <>{stage==='capture'&&<fieldset disabled={locked} className="wc-unframed">
   <div className="wc-capture-summary"><strong>已选：{assembly.capture.state.length} 组观测 · {assembly.capture.action.length} 组动作 · {Object.keys(assembly.capture.images).length} 路图像 · {assembly.capture.auxiliary?.length||0} 路附加信号</strong><p>编辑设备：{deviceId||'从设备列表选择'}。切换设备不会更改已选通道。</p></div>
   <label>下一段记录任务<input value={recordTask} disabled={!!session?.recording} onChange={e=>setTask(e.target.value)} /></label>
   <Section title="记录设置" note="选择需要保存的观测与动作。无需采集时可保留默认投影，在运行页不启动记录。">
    <div className="wc-form-grid"><label>记录频率 / Hz<NumberInput label="记录频率" schema={{minimum:1,maximum:500}} value={assembly.capture.dataset_fps||30} onChange={v=>changed({...assembly,capture:{...assembly.capture,dataset_fps:v}})} /></label><label>动作数据来源<select value={assembly.capture.action_source} onChange={e=>changed({...assembly,capture:{...assembly.capture,action_source:e.target.value}})}>{[['command','控制指令'],['feedback','反馈位置'],['next_state','下一帧状态'],['requested','请求指令'],['authorized','授权指令'],['dispatched','已下发指令']].map(([id,title])=><option key={id} value={id}>{title}</option>)}</select></label></div>
    {(['state','action'] as const).filter(kind=>(kind==='action'?targets:endpoints.filter(e=>e.port.direction==='output'&&e.port.semantic!=='image'&&descriptor(assembly.instances[e.instance])?.kind!=='input')).some(e=>e.instance===deviceId)).map(kind=><fieldset className="wc-pose" key={kind}><legend>{{state:'机器人观测',action:'机器人动作'}[kind]}</legend><div className="wc-form-grid">{(kind==='action'?targets:endpoints.filter(e=>e.port.direction==='output'&&e.port.semantic!=='image'&&descriptor(assembly.instances[e.instance])?.kind!=='input')).filter(e=>e.instance===deviceId).map(e=><Check key={e.ref} text={portLabel(e.ref)+` · ${e.port.dim} 维 · ${e.port.unit||'无量纲'}`} value={(assembly.capture[kind]||[]).includes(e.ref)} onChange={checked=>{let list=(assembly.capture[kind]||[]).filter(ref=>ref!==e.ref&&!(kind==='action'&&checked&&ref.split('.')[0]===e.instance));if(checked)list=[...list,e.ref];changed({...assembly,capture:{...assembly.capture,[kind]:list}})}} />)}</div></fieldset>)}
    {(()=>{
     const used=new Set(assembly.bindings.flatMap(binding=>[binding.source,binding.deadman]))
     const selected=assembly.capture.auxiliary||[]
     const task=sources.filter(e=>used.has(e.ref)&&e.instance===deviceId)
     const setChannels=(refs:string[], enabled:boolean)=>{
      const next=enabled?[...new Set([...selected,...refs])]:selected.filter(ref=>!refs.includes(ref))
      changed({...assembly,capture:{...assembly.capture,auxiliary:next}})
     }
     const fields=(channels:Endpoint[])=><div className="wc-form-grid">{channels.map(e=><Check key={e.ref} text={signalLabel(e)} value={selected.includes(e.ref)} onChange={checked=>setChannels([e.ref],checked)} />)}</div>
     const extra=sources.filter(e=>e.instance===deviceId&&!used.has(e.ref)&&(selected.includes(e.ref)||questVisible(e)))
     const groups=[
      {title:'位姿与开合量', accepts:(e:Endpoint)=>['pose6d','scalar'].includes(e.port.semantic)&&!holdPort(e)},
      {title:'按键、摇杆与跟随状态', accepts:(e:Endpoint)=>e.port.semantic==='axes_buttons'||holdPort(e)},
      {title:'手部关键点', accepts:(e:Endpoint)=>e.port.semantic==='hand_landmarks'},
      {title:'其他信号', accepts:(e:Endpoint)=>!holdPort(e)&&!['pose6d','scalar','axes_buttons','hand_landmarks'].includes(e.port.semantic)},
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
    {endpoints.filter(e=>e.instance===deviceId&&e.port.semantic==='image'&&e.port.direction==='output').map(e=>{const entry=Object.entries(assembly.capture.images).find(([,ref])=>ref===e.ref);return <div className="wc-camera-row" key={e.ref}><Check text={portLabel(e.ref)} value={!!entry} onChange={checked=>{const images={...assembly.capture.images};if(checked){let name=e.instance,n=2;while(name in images)name=e.instance+'_'+n++;images[name]=e.ref}else if(entry)delete images[entry[0]];changed({...assembly,capture:{...assembly.capture,images}})}} />{entry&&<label>数据集图像名称<input aria-label={e.instance+'图像名称'} defaultValue={entry[0]} key={entry[0]} onBlur={event=>{const key=event.target.value.trim();if(!/^[a-z][a-z0-9_-]*$/.test(key)||key!==entry[0]&&key in assembly.capture.images){setError(true);setMessage('图像名称需唯一，使用小写字母、数字或下划线。');event.target.value=entry[0];return}const images={...assembly.capture.images};delete images[entry[0]];images[key]=e.ref;changed({...assembly,capture:{...assembly.capture,images}})}} /></label>}</div>})}
    {!endpoints.some(e=>e.instance===deviceId&&e.port.semantic==='image')&&<p className="wc-empty">当前设备没有图像端口。选择相机可编辑图像记录。</p>}
   </Section>
   <Section title="诊断日志设置" note="随工作站运行记录，独立于数据集录制。">    <fieldset className="wc-unframed" disabled={locked}>
     <Check text="保存本次运行的遥操作与运动诊断" value={logOptions.enabled} onChange={enabled=>changed({...assembly,metadata:{...assembly.metadata,diagnostic_logging:{...logOptions,enabled}}})} />
     <div className="wc-form-grid"><label>反馈采样上限 / Hz<NumberInput label="日志反馈采样上限" schema={{minimum:1,maximum:200}} value={logOptions.feedback_hz} onChange={feedback_hz=>changed({...assembly,metadata:{...assembly.metadata,diagnostic_logging:{...logOptions,feedback_hz}}})} /></label><label>单次日志容量上限 / MB<NumberInput label="单次日志容量上限" schema={{minimum:16,maximum:8192}} value={logOptions.max_run_mb} onChange={max_run_mb=>changed({...assembly,metadata:{...assembly.metadata,diagnostic_logging:{...logOptions,max_run_mb}}})} /></label></div>
    </fieldset>
</Section>
  </fieldset>}</>}
