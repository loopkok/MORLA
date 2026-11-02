import { useEffect, useState } from 'react'
import { OperatorKeyboard as Keys } from '../lib/operatorKeyboard'

export function OperatorKeyboard({sessionId,onError,onUpdate}:{sessionId:string;onError:(message:string)=>void;onUpdate:()=>void}) {
 const [enabled,setEnabled]=useState(false)
 useEffect(()=>{
  if(!enabled)return
  const keys=new Keys(),client=crypto.randomUUID()
  let queue=Promise.resolve(),disposed=false,holdSequence=0
  const post=async(path:string,body:unknown)=>{
   const response=await fetch(`/api/v2/sessions/${sessionId}/operator/${path}`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),keepalive:true})
   const reply=await response.json()
   if(!response.ok)throw new Error(typeof reply.detail==='string'?reply.detail:JSON.stringify(reply.detail))
  }
  const hold=(held:boolean)=>post('hold',{client_id:client,held,sequence:++holdSequence})
  const actions=(items:string[])=>{for(const action of items)queue=queue.then(async()=>{
   if(disposed)return
   await post('action',{action,request_id:crypto.randomUUID()})
   if(keys.down.has('KeyS'))await hold(true)
   onUpdate()
  }).catch(e=>onError((e as Error).message))}
  const editable=(target:EventTarget|null)=>target instanceof HTMLElement&&(target.isContentEditable||!!target.closest('input:not([type=checkbox]):not([type=button]),textarea,select'))
  const known=(code:string)=>['KeyS','KeyA','KeyB','KeyN'].includes(code)
  const reset=()=>{keys.clear();void hold(false).catch(()=>{})}
  const down=(event:KeyboardEvent)=>{
   if(!known(event.code)||editable(event.target)||event.ctrlKey||event.metaKey||event.altKey||event.isComposing)return
   event.preventDefault()
   if(event.repeat)return
   actions(keys.press(event.code))
  }
  const up=(event:KeyboardEvent)=>{
   if(!keys.down.has(event.code))return
   event.preventDefault();actions(keys.release(event.code))
   if(event.code==='KeyS')void hold(false).catch(()=>{})
  }
  const visibility=()=>{if(document.hidden)reset()}
  window.addEventListener('keydown',down);window.addEventListener('keyup',up);window.addEventListener('blur',reset)
  document.addEventListener('visibilitychange',visibility)
  const timer=window.setInterval(()=>{if(keys.down.has('KeyS')&&!document.hidden)void hold(true).catch(e=>{reset();onError((e as Error).message)})},100)
  return()=>{disposed=true;clearInterval(timer);window.removeEventListener('keydown',down);window.removeEventListener('keyup',up);window.removeEventListener('blur',reset);document.removeEventListener('visibilitychange',visibility);reset()}
 },[enabled,sessionId])
 return <div className="wc-pose"><label className="wc-check"><input type="checkbox" checked={enabled} onChange={e=>setEnabled(e.target.checked)} />启用本页键盘控制</label>
 <p className="wc-note">S 开始／重定跟随零点／人工接管，按住可跟随；A 开始录制或交还控制；B 结束并保存；N 丢弃本段；A + N 停止跟随并直接回工作位。A、N 单键松开时执行。</p>
 <p className="wc-note">页面需保持焦点，编辑表单时快捷键不生效。松开 S、切换窗口或页面隐藏会解除键盘按住跟随；原有各绑定的按住使能信号仍可使用。</p></div>
}
