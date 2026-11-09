export function explainError(value:unknown):string {
 if(Array.isArray(value))return value.map(row=>row&&typeof row==='object'&&'message' in row?explainError(row.message):explainError(row)).join('；')
 const message=typeof value==='string'?value:JSON.stringify(value)
 if(message==='assembly revision conflict')return '保存版本冲突：服务端已有更新，本地草稿已保留。请另存为，或打开最新版本核对。'
 if(/preview\/installed dependency hash changed|dependency hash changed/.test(message))return '插件或依赖已变化，请执行检查并保存新版本后再初始化。'
 if(message.startsWith('unresolved instance '))return '找不到引用的设备：'+message.slice('unresolved instance '.length)+'。请选择已添加的设备。'
 return message
}
export class WorkstationError extends Error {
 constructor(message:string, public status:number, public detail:unknown) { super(message) }
}
export async function request<T>(path:string, body?:unknown, signal?:AbortSignal):Promise<T> {
 const abort=new AbortController(),timeout=setTimeout(()=>abort.abort(new Error('请求超时')),body===undefined?15000:30000)
 const cancel=()=>abort.abort(signal?.reason)
 signal?.addEventListener('abort',cancel,{once:true});if(signal?.aborted)cancel()
 try {
 const response=await fetch('/api/v2'+path,{method:body===undefined?'GET':'POST',headers:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),signal:abort.signal})
 const text=await response.text()
 let result:unknown
 try { result=JSON.parse(text) } catch { throw new WorkstationError(`接口 ${path} 返回非 JSON 响应（HTTP ${response.status}）：${text.slice(0,200)||'空响应'}`,response.status,text) }
 if(!response.ok) {
  const detail=result&&typeof result==='object'&&'detail' in result?(result as {detail:unknown}).detail:result
  throw new WorkstationError(explainError(detail),response.status,detail)
 }
 return result as T
 } catch(e) {
  if(signal?.aborted)throw e
  if(!(e instanceof WorkstationError)&&body!==undefined)throw new Error(`请求结果待确认：${(e as Error).message}。请先查询当前状态，不要重复提交。`)
  throw e
 } finally {clearTimeout(timeout);signal?.removeEventListener('abort',cancel)}
}
export type OperationReply={status?:string;error?:string;request_id?:string;[key:string]:unknown}
export async function awaitOperation(sessionId:string, reply:OperationReply, requestId:string, signal?:AbortSignal):Promise<OperationReply> {
 const deadline=Date.now()+30000
 while(['RUNNING','DISPATCHED','CANCEL_REQUESTED'].includes(reply.status||'')) {
  if(Date.now()>deadline)throw new Error('操作结果待确认：请查看运行状态，不要重复提交。')
  await new Promise(resolve=>setTimeout(resolve,300))
  try { reply=await request<OperationReply>(`/sessions/${sessionId}/operations/${requestId}`,undefined,signal) }
  catch(e) { if((e as WorkstationError).status===404)throw new Error('操作回执未登记，结果待确认；请查看运行状态。');throw new Error('操作结果待确认：'+(e as Error).message+'。请查询运行状态，不要重复提交。') }
 }
 if(['FAILED','CANCELLED'].includes(reply.status||'')||reply.error)throw new Error(reply.error||`操作${reply.status==='CANCELLED'?'已取消':'失败'}`)
 return reply
}

export type ControlState={operator?:{activity:string;paused?:boolean;error?:string;transition?:{status:string;error?:string};work_position?:{status:string;error?:string}}}
export async function awaitControlTransition<T extends ControlState>(read:()=>Promise<T>,action:string):Promise<T>{
 const deadline=Date.now()+(action==='workpos'?120000:30000)
 while(true){
  let state:T
  try{state=await read()}catch(e){throw new Error('操作结果待确认：'+(e as Error).message+'。请查询运行状态，不要重复提交。')}
  const operator=state.operator
  const pending=action==='workpos'?operator?.work_position?.status==='RUNNING':operator?.transition?.status==='RUNNING'
  if(!pending){
   if(action==='workpos'){
    if(operator?.work_position?.status!=='SUCCEEDED')throw new Error(operator?.work_position?.error||operator?.error||'工作位移动未完成，请查看当前状态。')
   }else{
    if(['FAILED','CANCELLED'].includes(operator?.transition?.status||'')||operator?.transition?.error)throw new Error(operator?.transition?.error||'控制权切换未完成。')
    if(action==='teleop_start'&&(operator?.activity!=='human'||operator.paused))throw new Error(operator?.error||'遥操未开始，请查看控制权状态。')
   }
   return state
  }
  if(Date.now()>deadline)throw new Error('操作结果待确认，请查看运行状态，不要重复提交。')
  await new Promise(resolve=>setTimeout(resolve,300))
 }
}
