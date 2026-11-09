import { Children, useEffect, useRef, useState } from 'react'
import type { CSSProperties, ReactNode } from 'react'
const storageKey='morla.ui.compositionSplit'
export function ResizableComposition({children}:{children:ReactNode}){
 const [ratio,setRatio]=useState(()=>{try{const value=Number(localStorage.getItem(storageKey));return value>=.1&&value<=.9?value:.45}catch{return .45}}),[width,setWidth]=useState(0),[dragging,setDragging]=useState(false)
 const root=useRef<HTMLDivElement>(null),panes=Children.toArray(children),available=Math.max(0,width-16)
 const min=available?Math.min(.45,300/available):.2,max=available?Math.max(.45,1-360/available):.8,current=Math.max(min,Math.min(max,ratio))
 const change=(value:number)=>{const next=Math.max(min,Math.min(max,value));setRatio(next);try{localStorage.setItem(storageKey,String(next))}catch{/* UI preferences are optional. */}}
 useEffect(()=>{const element=root.current;if(!element)return;const observer=new ResizeObserver(([entry])=>setWidth(entry.contentRect.width));observer.observe(element);return()=>observer.disconnect()},[])
 useEffect(()=>{if(!dragging)return;const oldCursor=document.body.style.cursor,oldSelect=document.body.style.userSelect;document.body.style.cursor='col-resize';document.body.style.userSelect='none';return()=>{document.body.style.cursor=oldCursor;document.body.style.userSelect=oldSelect}},[dragging])
 return <div ref={root} className={'rw-composition rw-resizable '+(dragging?'is-resizing':'')} style={{'--rw-left':`${available*current}px`} as CSSProperties}>
 <div className="rw-graph-pane" id="composition-graph-pane">{panes[0]}</div>
 <div className="rw-splitter" role="separator" tabIndex={0} aria-label="调整连接画布与组件属性宽度" aria-orientation="vertical" aria-controls="composition-graph-pane composition-properties-pane" aria-valuemin={Math.round(min*100)} aria-valuemax={Math.round(max*100)} aria-valuenow={Math.round(current*100)} aria-valuetext={`连接画布 ${Math.round(current*100)}%，组件属性 ${Math.round((1-current)*100)}%`} title="左右拖动调整宽度；双击恢复默认；方向键微调"
 onPointerDown={event=>{if(event.button!==0)return;event.preventDefault();event.currentTarget.focus();event.currentTarget.setPointerCapture(event.pointerId);setDragging(true)}}
 onPointerMove={event=>{if(!event.currentTarget.hasPointerCapture(event.pointerId)||!available)return;const bounds=root.current!.getBoundingClientRect();change((event.clientX-bounds.left-8)/available)}}
 onPointerUp={event=>{if(event.currentTarget.hasPointerCapture(event.pointerId))event.currentTarget.releasePointerCapture(event.pointerId);setDragging(false)}}
 onPointerCancel={()=>setDragging(false)} onLostPointerCapture={()=>setDragging(false)} onDoubleClick={()=>change(.45)}
 onKeyDown={event=>{if(['ArrowLeft','ArrowRight','Home','End'].includes(event.key)){event.preventDefault();change(event.key==='Home'?min:event.key==='End'?max:current+(event.key==='ArrowRight'?.03:-.03))}}}/>
 <div className="rw-properties-pane" id="composition-properties-pane">{panes[1]}</div>
 </div>
}
