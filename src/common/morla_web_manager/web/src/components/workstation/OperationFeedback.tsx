import { useEffect, useRef, useState } from 'react'
export function useConfirmation(){
 const [pending,setPending]=useState<{message:string;resolve:(answer:boolean)=>void}|null>(null)
 const held=useRef(false)
 const confirm=(message:string)=>{if(held.current)return Promise.resolve(false);held.current=true;return new Promise<boolean>(resolve=>setPending({message,resolve}))}
 const answer=(value:boolean)=>{pending?.resolve(value);held.current=false;setPending(null)}
 useEffect(()=>()=>{pending?.resolve(false)},[pending])
 return {confirm,pending,answer}
}
export function ConfirmDialog({message,onAnswer}:{message:string;onAnswer:(yes:boolean)=>void}){
 const ref=useRef<HTMLDialogElement>(null)
 useEffect(()=>{const previous=document.activeElement as HTMLElement;ref.current?.showModal();return()=>{ref.current?.close();previous?.focus()}},[])
 return <dialog ref={ref} className="wc-dialog" onCancel={e=>{e.preventDefault();onAnswer(false)}} aria-labelledby="confirmation-title"><h3 id="confirmation-title">确认变更</h3><pre>{message}</pre><div className="wc-inline"><button autoFocus onClick={()=>onAnswer(false)}>取消</button><button className="wc-primary" onClick={()=>onAnswer(true)}>确认</button></div></dialog>
}
export type OperationRecord={id:string;title:string;status:string;started:number;target?:string;message?:string}
export function OperationFeedback({records}:{records:OperationRecord[]}){
 return records.length?<details className="wc-details" open={records.some(r=>r.status==='执行中')}><summary>操作记录 · {records[0].status}</summary><ol className="wc-operations">{records.map(r=><li key={r.id}><strong>{r.title}</strong><span>{r.status} · {new Date(r.started).toLocaleTimeString()} · {r.target}</span>{r.message&&<p>{r.message}</p>}<code>操作 ID：{r.id}</code></li>)}</ol></details>:null
}
export function ErrorSheet({message,onClose}:{message:string;onClose:()=>void}){
 const ref=useRef<HTMLDialogElement>(null)
 useEffect(()=>{const previous=document.activeElement as HTMLElement;ref.current?.showModal();return()=>{ref.current?.close();previous?.focus()}},[])
 return <dialog className="wc-dialog" ref={ref} onCancel={onClose} aria-labelledby="error-title"><h3 id="error-title">操作未完成</h3><p>请保留当前编辑，核对以下原因后重试对应步骤。</p><pre>{message}</pre><div className="wc-inline"><button onClick={()=>void navigator.clipboard.writeText(message).catch(()=>{})}>复制详情</button><button autoFocus onClick={onClose}>关闭</button></div></dialog>
}
