import { useCallback, useEffect, useRef, useState } from 'react'
import { request } from '../api/workstation'
// Polling and operation observers share one in-flight state read per session.
export function useWorkstationSession<T extends {session_id:string;status:string}>(session:T|null,setSession:(s:T|null)=>void){
 const [connectionError,setConnectionError]=useState('')
 const currentId=useRef(session?.session_id);currentId.current=session?.session_id
 const reads=useRef(new Map<string,Promise<T>>()),mounted=useRef(true)
 useEffect(()=>{mounted.current=true;return()=>{mounted.current=false}},[])
 const refreshSession=useCallback((id:string,afterMutation=false):Promise<T>=>{
  const pending=reads.current.get(id)
  // A read begun before a mutation cannot confirm its result. Finish it first.
  if(pending)return afterMutation?pending.catch(()=>undefined).then(()=>refreshSession(id)):pending
  const origin=currentId.current
  const result=request<T>(`/sessions/${id}/state`).then(next=>{if(mounted.current&&(currentId.current===id||currentId.current===origin)){setSession(next);setConnectionError('')}return next}).finally(()=>reads.current.delete(id))
  reads.current.set(id,result);return result
 },[setSession])
 useEffect(()=>{
  if(!session||session.status==='STOPPED')return
  let disposed=false,timer:ReturnType<typeof setTimeout>
  async function poll(){
   try {await refreshSession(session!.session_id)}
   catch(e){if(!disposed)setConnectionError(`运行状态待确认：${(e as Error).message}`)}
   if(!disposed)timer=setTimeout(poll,document.hidden?2000:500)
  }
  void poll();return()=>{disposed=true;clearTimeout(timer)}
 },[session?.session_id,session?.status,refreshSession])
 return {connectionError,refreshSession}
}
