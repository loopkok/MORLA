import { useEffect, useRef, useState } from 'react'
import { request } from '../../api/workstation'
import type { SimOptions, Port } from '../../lib/workcell'
export type Resolved = { assembly_hash:string; feature_contract:{state_dim:number;action_dim:number;cameras:Record<string,unknown>};diagnostics: {message?:string;severity?:string;path?:string}[];status?:Record<string,unknown> }
export type IkReport = {solver?:string;reason?:string;error?:string;age_ms?:number;raw_position_error_m?:number;raw_angle_error_rad?:number;position_error_m?:number;angle_error_rad?:number;damping?:number;singularity_proximity?:number;absorbed_position_m?:number;success?:boolean}
export type LogStatus = {enabled:boolean;state:string;directory?:string;files:string[];written:Record<string,number>;dropped:Record<string,number>;queued:number;error?:string;feedback_hz:number}
export type LogRun = {session_id:string;assembly_id:string;started_at_utc:string;state:string;files:string[];dropped:Record<string,number>;error?:string}
export type Session = {input_errors?:Record<string,string>;input_processes?:{instance_id:string;mode:string;status?:string;exit_code?:number;pid?:number}[];operator?:{activity:string;paused?:boolean;transition?:{status:string;phase:string;kind:string;acks?:Record<string,unknown>;error?:string};return_to?:string;error?:string;work_position?:{status:string}};diagnostic_log?:LogStatus;session_id:string;status:string;assembly_id?:string;revision?:number;recording?:boolean;simulation?:SimOptions & {viewer?:{status:string;error?:string};render_error?:string};controllers?:Record<string,Record<string,IkReport>>;components?:Record<string,{ready?:boolean;fresh?:boolean;reason?:string;values?:number[]}>;inputs?:Record<string,{connected?:boolean;fresh?:boolean;mode?:string|Record<string,string>}>;[key:string]:unknown}
export type Endpoint = {ref:string;instance:string;port:Port}
export const states:Record<string,string>={ACTIVE:'会话运行中',READY_DISARMED:'就绪 · 待使能',STOPPED:'已停止',FAULTED:'故障',ESTOP_LATCHED:'急停已锁存'}
export const reasons:Record<string,string>={tracking:'跟踪中',converged:'已达到求解精度',holding:'保持位置',deadline_hold:'计算超时，保持位置',absorbed_target_hold:'已吸收超量，保持位置'}
export const controlNames:Record<string,string>={human:'人工遥操',policy:'策略控制',playback:'轨迹回放',workpos:'工作位移动',paused:'已暂停'}
export const handoffPhases:Record<string,string>={reanchoring:'确认跟随零点',preparing:'准备交还控制',activating:'确认设备激活',committed:'切换完成',rolled_back:'切换失败，已回滚',rollback_failed:'无法恢复原控制，已停止输出',cancelled:'切换已取消'}
export const workStates:Record<string,string>={RUNNING:'移动中',SUCCEEDED:'已到达',FAILED:'失败',CANCELLED:'已取消'}
export const metric=(n:number|undefined,scale=1,digits=2)=>n==null?'—':(n*scale).toFixed(digits)
export function Section({title,note,children}:{title:string;note?:string;children:React.ReactNode}){return <section className="wc-section"><div className="wc-section-heading"><h3>{title}</h3>{note&&<p>{note}</p>}</div>{children}</section>}
export function Check({text,value,onChange}:{text:string;value:boolean;onChange:(v:boolean)=>void}){return <label className="wc-check"><input type="checkbox" checked={value} onChange={e=>onChange(e.target.checked)} />{text}</label>}
export function SimPreview({ sessionId, cameraId }: { sessionId: string; cameraId: string }) {
  const [error,setError]=useState(''),[version,setVersion]=useState(0)
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
    if (item) void request(`/sessions/${sessionId}/cameras/${cameraId}/view`, item).catch(e => setError(e.message))
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
    return () => {node.removeEventListener('wheel', wheel);if(timer.current!==null)window.clearTimeout(timer.current);queued.current=null}
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
      <button type="button" style={{ ...ghostBtn, padding: '3px 10px', fontSize: 12 }} onClick={() => void request(`/sessions/${sessionId}/cameras/${cameraId}/view`, { action: 'reset' }).catch(e => setError(e.message))}>复位视角</button>
    </figcaption>
    <img ref={image} alt={`${cameraId} 仿真画面`} src={`/api/v2/sessions/${sessionId}/cameras/${cameraId}/stream?retry=${version}`} onLoad={()=>setError('')} onError={()=>setError('摄像头画面暂不可用，请检查运行诊断或重试。')} draggable={false}
      onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up}
      onContextMenu={event => event.preventDefault()}
      style={{ maxWidth: '100%', width: 640, borderRadius: 8, touchAction: 'none', cursor: 'grab', userSelect: 'none' }} />
    {error&&<p role="status">{error}<button onClick={()=>{setError('');setVersion(v=>v+1)}}>重试画面</button></p>}
    <p style={{ ...mutedSmall, margin: 0 }}>左键拖动旋转 · 右键或 Shift 拖动平移 · 滚轮缩放</p>
  </figure>
}
export const subCard:React.CSSProperties={background:'#15191d',border:'1px solid #30363b',padding:14}
export const mutedSmall:React.CSSProperties={color:'#a2aab1',fontSize:12,lineHeight:1.6}
export const ghostBtn:React.CSSProperties={background:'transparent',color:'#c4cbd0',border:'1px solid #3b434a',borderRadius:4,padding:'5px 10px',fontSize:12,cursor:'pointer'}