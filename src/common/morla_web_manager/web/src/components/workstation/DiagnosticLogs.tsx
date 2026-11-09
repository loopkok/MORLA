import { useEffect, useRef, useState } from 'react'
import type { WorkstationModel } from '../ComposableTab'
import { request } from '../../api/workstation'
import { useRealtime, useWsConnected } from '../../hooks/useRealtime'
import { NumberInput } from '../WorkcellFields'
import type { LogRun } from './shared'
type LaunchLogs={teleop:string[];collect:string[];infer:string[]}
export function DiagnosticLogs({m}:{m:WorkstationModel}){
 const realtime=useRealtime(),connected=useWsConnected(),[launch,setLaunch]=useState<LaunchLogs>({teleop:[],collect:[],infer:[]}),[runs,setRuns]=useState<LogRun[]>([]),[selected,setSelected]=useState(''),[source,setSource]=useState('workstation'),[live,setLive]=useState(true),[follow,setFollow]=useState(true),[filter,setFilter]=useState(''),[lines,setLines]=useState<string[]>([]),[error,setError]=useState(''),[launchError,setLaunchError]=useState(''),[updated,setUpdated]=useState('')
 const output=useRef<HTMLPreElement>(null),sid=selected||m.session?.session_id||runs[0]?.session_id||'',entry=runs.find(r=>r.session_id===sid)
 const options=m.logOptions,patch=(value:Partial<typeof options>)=>m.changed({...m.assembly,metadata:{...m.assembly.metadata,diagnostic_logging:{...options,...value}}})
 useEffect(()=>{let disposed=false;void request<LogRun[]>('/diagnostics/runs').then(rows=>{if(!disposed)setRuns(rows)}).catch(e=>{if(!disposed)setError(e.message)});return()=>{disposed=true}},[m.session?.session_id,m.session?.status])
 useEffect(()=>{if(live&&connected&&realtime){setLaunch({teleop:realtime.logTail||[],collect:realtime.collectLaunch?.log_tail||[],infer:realtime.inferLaunch?.log_tail||[]});setLaunchError('')}},[realtime,connected,live])
 useEffect(()=>{
  if(!live)return
  let disposed=false,timer:ReturnType<typeof setTimeout>;const abort=new AbortController()
  async function poll(){
   if(source==='workstation'){
    if(!sid){setLines([]);setError('')}else try{const result=await request<{records:unknown[]}>(`/diagnostics/runs/${sid}/tail?limit=200`,undefined,abort.signal);if(!disposed){setLines(result.records.map(row=>JSON.stringify(row)));setError('');setUpdated(new Date().toLocaleTimeString())}}catch(e){if(!disposed)setError((e as Error).message)}
   }else if(!connected){
    try{const response=await fetch('/api/v1/logs',{signal:abort.signal}),body=await response.json();if(!response.ok||!body.ok)throw new Error(response.status===404?'当前服务未提供 launch 日志接口；完整 Web 服务启动后将显示进程日志。':body.detail||body.message||'读取 launch 日志失败');if(!disposed){setLaunch(body.data);setLaunchError('');setUpdated(new Date().toLocaleTimeString())}}catch(e){if(!disposed)setLaunchError((e as Error).message)}
   }
   if(!disposed)timer=setTimeout(poll,document.hidden?4000:1500)
  }
  void poll();return()=>{disposed=true;clearTimeout(timer);abort.abort()}
 },[sid,source,live,connected])
 const text=(source==='workstation'?lines:launch[source as keyof LaunchLogs]||[]).filter(line=>line.toLowerCase().includes(filter.toLowerCase())).join('\n')
 useEffect(()=>{if(follow&&output.current)output.current.scrollTop=output.current.scrollHeight},[text,follow])
 const files=sid===m.session?.session_id?m.logFiles:entry?.files||[]
 return <div className="rw-diagnostics"><section className="rw-card"><header><h3>日志文件配置</h3><span>{m.active?'当前运行配置只读':'修改后在系统编排保存，下一次启动生效'}</span></header><div className="rw-card-body"><fieldset disabled={m.locked} className="wc-unframed"><label className="wc-check"><input type="checkbox" checked={options.enabled} onChange={e=>patch({enabled:e.target.checked})}/>保存工作站运行诊断日志</label><div className="rw-config-grid"><label>反馈采样上限 / Hz<NumberInput label="日志反馈采样上限" schema={{minimum:1,maximum:200}} value={options.feedback_hz} onChange={feedback_hz=>patch({feedback_hz})}/></label><label>日志分段大小 / MB<NumberInput label="日志分段大小" schema={{minimum:1}} value={options.segment_mb} onChange={segment_mb=>patch({segment_mb})}/></label><label>单次日志容量 / MB<NumberInput label="单次日志容量" schema={{minimum:16}} value={options.max_run_mb} onChange={max_run_mb=>patch({max_run_mb})}/></label></div></fieldset>{m.session?.diagnostic_log?.directory&&<p>当前目录：<code>{m.session.diagnostic_log.directory}</code></p>}<p>诊断日志状态：<strong>{m.session?.diagnostic_log?.state||'暂无活动日志'}</strong></p></div></section>
 <section className="rw-card"><header><h3>实时运行日志</h3><span role="status">{live?'实时更新中':'已暂停更新'}{updated?' · '+updated:''}</span></header><div className="rw-card-body"><div className="rw-log-toolbar"><label>日志来源<select aria-label="日志来源" value={source} onChange={e=>setSource(e.target.value)}><option value="workstation">工作站诊断</option><option value="teleop">机器人 / teleop launch</option><option value="collect">采集 / collect launch</option><option value="infer">推理 / infer launch</option></select></label>{source==='workstation'&&<label>运行记录<select aria-label="诊断运行记录" value={sid} onChange={e=>{setSelected(e.target.value);setLines([])}}><option value="">选择运行记录</option>{m.session&&!runs.some(r=>r.session_id===m.session!.session_id)&&<option value={m.session.session_id}>当前会话 · {m.session.session_id.slice(0,8)}</option>}{runs.map(r=><option key={r.session_id} value={r.session_id}>{r.assembly_id} · {new Date(r.started_at_utc).toLocaleString()}</option>)}</select></label>}<label>筛选日志<input aria-label="筛选日志" placeholder="输入关键字" value={filter} onChange={e=>setFilter(e.target.value)}/></label><button onClick={()=>setLive(v=>!v)}>{live?'暂停更新':'恢复更新'}</button><label className="wc-check"><input type="checkbox" checked={follow} onChange={e=>setFollow(e.target.checked)}/>自动滚动</label></div>{(source==='workstation'?error:launchError)&&<p className="rw-notice error" role="status">{source==='workstation'?error:launchError}</p>}<pre className="rw-log-output" ref={output} aria-label="实时日志内容" tabIndex={0}>{text||'暂无日志。启动相关服务或选择已有运行记录后将在这里显示。'}</pre><div className="rw-actions"><button disabled={!text} onClick={()=>{const url=URL.createObjectURL(new Blob([text],{type:'text/plain'})),link=document.createElement('a');link.href=url;link.download=source+'-logs.txt';link.click();URL.revokeObjectURL(url)}}>下载当前日志</button>{source==='workstation'&&[...new Set(files)].map(file=><a key={file} download href={`/api/v2/diagnostics/runs/${sid}/files/${file}`}>{file}</a>)}</div></div></section></div>
}
