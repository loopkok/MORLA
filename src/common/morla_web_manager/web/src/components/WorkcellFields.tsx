import { useEffect, useState } from 'react'
import { enumLabels, identity, matrixToTransform, parameterLabels, poseToTransform, transformToMatrix, transformToPose } from '../lib/workcell'
import type { Schema } from '../lib/workcell'

export function NumberInput({ value, onChange, label, schema = {}, placeholder }: { value?: number; onChange:(v:number)=>void; label:string; schema?:Schema; placeholder?:string }) {
 const [text,setText]=useState(value==null?'':String(value))
 useEffect(()=>setText(value==null?'':String(value)),[value])
 return <input aria-label={label} type="number" step={schema.type==='integer'?1:'any'} min={schema.minimum ?? schema.exclusiveMinimum} max={schema.maximum ?? schema.exclusiveMaximum} placeholder={placeholder} value={text}
  onChange={e=>{setText(e.target.value); const v=e.target.valueAsNumber;if(Number.isFinite(v)) onChange(v)}} onBlur={()=>{if(text==='') setText(value==null?'':String(value))}} />
}
export function PoseFields({ label, value, onChange, matrix=false }: {label:string;value?:number[] | number[][];onChange:(v:number[] | number[][])=>void;matrix?:boolean}) {
 const t=matrix?matrixToTransform((value || identity()) as number[][]):(value || [0,0,0,0,0,0,1]) as number[]
 const pose=transformToPose(t)
 return <fieldset className="wc-pose"><legend>{label}</legend><div className="wc-pose-grid">{['X / m','Y / m','Z / m','Roll / °','Pitch / °','Yaw / °'].map((name,i)=><label key={name}>{name}<NumberInput label={label+' '+name} value={Number(pose[i].toFixed(8))} onChange={v=>{const next=pose.slice();next[i]=v;const transform=poseToTransform(next);onChange(matrix?transformToMatrix(transform):transform)}} /></label>)}</div></fieldset>
}
export function BasisFields({ label, value, onChange }: {label:string;value?:number[][]|number[];onChange:(v:number[][])=>void}) {
 const raw=value&&((value.length===3&&Array.isArray(value[0]))||value.length===9)?value:[[1,0,0],[0,1,0],[0,0,1]]
 const rows=(Array.isArray(raw[0])?raw:(raw as number[]).reduce<number[][]>((a,_,i)=>i%3===0?[...a,(raw as number[]).slice(i,i+3)]:a,[])) as number[][]
 const [custom,setCustom]=useState(false)
 const options=[['+X',0,1],['−X',0,-1],['+Y',1,1],['−Y',1,-1],['+Z',2,1],['−Z',2,-1]] as const
 const selection=(col:number)=>options.find(([_,axis,sign])=>rows.every((row,i)=>Math.abs(row[col]-(i===axis?sign:0))<1e-4))?.[0] || ''
 return <fieldset className="wc-pose"><legend>{label}</legend>
 <div className="wc-form-grid">{['X','Y','Z'].map((axis,col)=><label key={axis}>输入 {axis} 正方向对应<select aria-label={label+' 输入'+axis} value={selection(col)} onChange={e=>{const option=options.find(o=>o[0]===e.target.value);if(!option)return;const next=rows.map((row,i)=>row.map((v,j)=>j===col?(i===option[1]?option[2]:0):v));onChange(next)}}><option value="" disabled>使用标定矩阵</option>{options.map(([name])=><option key={name} value={name}>机器人 {name}</option>)}</select></label>)}</div>
 <p className="wc-note">将输入设备的坐标轴映射到求解器基座坐标系。保留模板标定，或按实际站位调整；三个方向必须正交。</p>
 <button type="button" className="wc-text-button" onClick={()=>setCustom(!custom)}>{custom?'收起标定矩阵':'编辑标定矩阵'}</button>
 {custom&&<div className="wc-matrix">{rows.flatMap((row,i)=>row.map((v,j)=><NumberInput key={i+'-'+j} label={`${label} ${i+1},${j+1}`} value={v} onChange={n=>onChange(rows.map((r,a)=>r.map((x,b)=>a===i&&b===j?n:x)))} />))}</div>}
 </fieldset>
}
export function ValueField({ name, value, schema={}, onChange }: {name:string;value:unknown;schema?:Schema;onChange:(v:unknown)=>void}) {
 const title=schema.title || parameterLabels[name] || name
 if(schema.enum) return <label>{title}<select aria-label={title} value={value==null&&schema.default==null?'':JSON.stringify(value ?? schema.default)} onChange={e=>{if(e.target.value)onChange(JSON.parse(e.target.value))}}><option value="" disabled>使用设备默认</option>{schema.enum.map(v=><option key={String(v)} value={JSON.stringify(v)}>{enumLabels[String(v)] || String(v)}</option>)}</select></label>
 const kind=schema.type || (Array.isArray(value)?'array':value===null?'null':typeof value)
 if(kind==='boolean') return <label className="wc-check"><input aria-label={title} type="checkbox" checked={Boolean(value)} onChange={e=>onChange(e.target.checked)} />{title}</label>
 if(kind==='number'||kind==='integer') return <label>{title}<NumberInput label={title} schema={schema} value={value==null?undefined:Number(value)} onChange={onChange} placeholder={schema.default==null?'未设置':String(schema.default)} /></label>
 if(kind==='array') {
  const values=Array.isArray(value)?value:Array.from({length:schema.minItems || 0},()=>schema.items?.type==='string'?'':0)
  return <fieldset className="wc-array"><legend>{title}</legend><div className="wc-array-grid">{values.map((v,i)=><div key={i}><ValueField name={`${title} ${i+1}`} schema={schema.items} value={v} onChange={n=>onChange(values.map((x,j)=>i===j?n:x))} />{(schema.minItems==null||schema.maxItems==null||schema.minItems!==schema.maxItems)&&(!schema.minItems || values.length>schema.minItems)&&<button type="button" className="wc-text-button" onClick={()=>onChange(values.filter((_,j)=>j!==i))}>删除</button>}</div>)}</div>{(!schema.maxItems || values.length<schema.maxItems)&&<button type="button" className="wc-text-button" onClick={()=>onChange([...values,schema.items?.type==='string'?'':0])}>添加一项</button>}</fieldset>
 }
 if(kind==='object') return <fieldset className="wc-array"><legend>{title}</legend><ObjectFields value={(value || {}) as Record<string,unknown>} properties={schema.properties} onChange={onChange} allowNew={!schema.properties} /></fieldset>
 if(kind==='null') return <label>{title}<select aria-label={title} value="" onChange={e=>onChange(e.target.value)}><option value="">未分配</option></select></label>
 return <label>{title}<input aria-label={title} value={value==null?'':String(value)} placeholder={schema.description || '未设置'} onChange={e=>onChange(e.target.value)} /></label>
}
export function ObjectFields({ value, properties={}, defaults={}, onChange, allowNew=false }: {value:Record<string,unknown>;properties?:Record<string,Schema>;defaults?:Record<string,unknown>;onChange:(v:Record<string,unknown>)=>void;allowNew?:boolean}) {
 const [newKey,setNewKey]=useState('');const [newType,setNewType]=useState('string')
 const keys=[...new Set([...Object.keys(properties),...Object.keys(value)])]
 return <><div className="wc-form-grid">{keys.map(key=>{
 const schema=properties[key] || {};const current=value[key] ?? defaults[key] ?? schema.default
 return <div key={key} className={schema.type==='array'||schema.type==='object'||Array.isArray(current)||current&&typeof current==='object'?'wc-wide':''}>
 <ValueField name={key} value={current} schema={schema} onChange={v=>onChange({...value,[key]:v})} />
 {key in value&&<button type="button" className="wc-text-button wc-reset" aria-label={'恢复默认 '+(parameterLabels[key]||key)} onClick={()=>{const next={...value};delete next[key];onChange(next)}}>恢复默认</button>}
 </div>})}</div>
 {allowNew&&<div className="wc-inline wc-new-property"><input aria-label="新增字段名称" placeholder="自定义字段名称" value={newKey} onChange={e=>setNewKey(e.target.value)} /><select aria-label="新增字段类型" value={newType} onChange={e=>setNewType(e.target.value)}><option value="string">文本</option><option value="number">数值</option><option value="boolean">开关</option><option value="array">列表</option><option value="object">属性组</option></select><button type="button" disabled={!newKey.trim()||newKey in value||['__proto__','constructor','prototype'].includes(newKey)} onClick={()=>{onChange({...value,[newKey]:newType==='number'?0:newType==='boolean'?false:newType==='array'?[]:newType==='object'?{}:''});setNewKey('')}}>添加字段</button></div>}
 </>
}
