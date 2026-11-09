import { ObjectFields } from '../WorkcellFields'
import { backendNames, visibleDeviceFields } from '../../lib/workcell'
import type { Instance } from '../../lib/workcell'

import type { WorkstationModel } from '../ComposableTab'
import { Section, Check } from './shared'
export function DeviceSelection({assembly, backend, count, descriptor, device, duplicate, instances, label, locked, patchDevice, remove, selected, setInstance, stage}:Pick<WorkstationModel,'assembly'|'backend'|'count'|'descriptor'|'device'|'duplicate'|'instances'|'label'|'locked'|'patchDevice'|'remove'|'selected'|'setInstance'|'stage'>){return <>{(stage==='devices'||stage==='geometry')&&<fieldset disabled={locked} className="wc-unframed">
   {(['input','actuator','sensor'] as const).filter(kind=>descriptor(assembly.instances[selected]||{plugin:''} as Instance)?.kind===kind).map(kind=><Section key={kind} title={{input:'输入设备',actuator:'机器人与末端执行器',sensor:'相机与传感器'}[kind]} note={{input:'选择手柄、头显、手套或标准输入接口。',actuator:'机械臂、夹爪和灵巧手分别配置，可组合使用。',sensor:'图像采集为可选项，仿真相机与机械臂共用场景。'}[kind]}>
    {instances.filter(([id,i])=>id===selected&&descriptor(i)?.kind===kind).map(([id,item])=>{const d=descriptor(item)!;const mode=device(id).backend||item.backend||'fake';const fields=d.configuration_schema?.properties||{};const props=visibleDeviceFields(d,mode,item.parameters,device(id).parameters);const basic=kind==='sensor'?(mode==='real'?['camera_backend','width','height','fps','image_topic','compressed']:['width','height','fps']):kind==='input'?['side','process_mode','protocol','deadzone']:['side','rate_hz'];const rest=Object.fromEntries(Object.entries(props).filter(([key])=>!basic.includes(key)));return <article className="wc-device" key={id}><div className="wc-device-heading"><div><h4>{item.display_name||label(item.plugin)}</h4><code>{id}</code>{!!d.joints?.length&&<span className="wc-note">{d.joints.length} 轴</span>}</div><div className="wc-inline"><button onClick={()=>duplicate(id)}>复制</button><button className="wc-text-button wc-destructive" onClick={()=>void remove(id)}>移除</button></div></div>
     {stage==='devices'&&<><fieldset className="wc-pose"><legend>通用设置</legend><div className="wc-form-grid"><label>显示名称<input aria-label="设备显示名称" value={String(item.display_name||'')} placeholder={label(item.plugin)} onChange={e=>setInstance(id,{...item,display_name:e.target.value})} /></label><label>运行方式<select aria-label={id+'运行后端'} value={mode} onChange={e=>backend(id,e.target.value)}>{Object.entries(d.backend_variants||{}).map(([key,status])=><option key={key} value={key} disabled={status.available===false}>{backendNames[key]||key}{status.available===false?'（未提供）':''}</option>)}</select></label></div></fieldset>
     <ObjectFields value={Object.fromEntries(Object.entries(item.parameters||{}).filter(([key])=>basic.includes(key)&&key in props))} properties={Object.fromEntries(basic.filter(key=>props[key]).map(key=>[key,props[key]]))} defaults={d.defaults} overrides={device(id).parameters} sourceLabel="装配显式值" onChange={value=>{const params={...item.parameters};for(const key of basic)if(key in props)delete params[key];setInstance(id,{...item,parameters:{...params,...value}})}} />
     {(Object.keys(rest).length>0||Object.keys(item.parameters||{}).some(key=>!(key in fields)))&&<details className="wc-details"><summary>设备专用参数</summary><ObjectFields value={Object.fromEntries(Object.entries(item.parameters||{}).filter(([key])=>key in rest))} properties={rest} required={d.configuration_schema?.required} overrides={device(id).parameters} sourceLabel="装配显式值" defaults={d.plugin_id==='morla.input.quest'?{pinch_range_m:[.015,.08],...d.defaults}:d.defaults} onChange={value=>setInstance(id,{...item,parameters:{...Object.fromEntries(Object.entries(item.parameters||{}).filter(([key])=>basic.includes(key)||!(key in rest))),...value}})} /></details>}
     {Object.keys(item.parameters||{}).some(key=>!(key in fields))&&<details className="wc-details"><summary>未识别或历史参数 · 保留原值</summary><p className="wc-note">这些字段未在当前插件 schema 中声明。请前往检查页查看兼容问题和已知无效字段清理。</p><pre>{JSON.stringify(Object.fromEntries(Object.entries(item.parameters||{}).filter(([key])=>!(key in fields))),null,2)}</pre></details>}
     </>}
     {stage==='geometry'&&mode==='real'&&<fieldset className="wc-pose"><legend>硬件连接</legend>
      <div className="wc-form-grid">
       <label data-field="device_id">设备标识 · 必填<input value={device(id).device_id||''} placeholder="设备序列号或资产标识" onChange={e=>patchDevice(id,{device_id:e.target.value})} /></label>
       <label>设备资源标识<input value={device(id).resource_key||''} placeholder="默认使用设备标识" onChange={e=>patchDevice(id,{resource_key:e.target.value||undefined})} /></label>
      </div>
      <ObjectFields value={Object.fromEntries(Object.entries(device(id).parameters||{}).filter(([key])=>key in visibleDeviceFields(d,mode,item.parameters,device(id).parameters,true)))}
       properties={visibleDeviceFields(d,mode,item.parameters,device(id).parameters,true)}
       sourceLabel="部署覆盖" defaults={{...d.defaults,...item.parameters}}
       onChange={parameters=>{
        const shown=visibleDeviceFields(d,mode,item.parameters,device(id).parameters,true)
        const retained=Object.fromEntries(Object.entries(device(id).parameters||{}).filter(([key])=>!(key in shown)))
        patchDevice(id,{parameters:{...retained,...parameters}})
       }} />
      <div className="wc-inline"><Check text="已核对设备身份（仅记录）" value={!!device(id).verified} onChange={verified=>patchDevice(id,{verified})} /><Check text="已完成停止测试（仅记录）" value={!!device(id).stop_verified} onChange={stop_verified=>patchDevice(id,{stop_verified})} /></div>
      {(d.resources?.sharing==='session'||!!device(id).session_id)&&<label>共享驱动标识<input value={String(device(id).session_id||'')} onChange={e=>patchDevice(id,{session_id:e.target.value||undefined})} /></label>}
     </fieldset>}
     {stage==='geometry'&&Object.keys(device(id).parameters||{}).some(key=>key in fields&&!d.web?.connection_fields?.includes(key))&&<fieldset className="wc-pose"><legend>已有部署参数覆盖</legend><p className="wc-note">这些显式值优先于设备页的装配值。恢复默认只移除部署层覆盖。</p><ObjectFields sourceLabel="部署覆盖" value={Object.fromEntries(Object.entries(device(id).parameters||{}).filter(([key])=>key in fields&&!d.web?.connection_fields?.includes(key)))} properties={Object.fromEntries(Object.keys(device(id).parameters||{}).filter(key=>key in fields&&!d.web?.connection_fields?.includes(key)).map(key=>[key,fields[key]]))} defaults={{...d.defaults,...item.parameters}} onChange={value=>{const retained=Object.fromEntries(Object.entries(device(id).parameters||{}).filter(([key])=>!(key in fields)||d.web?.connection_fields?.includes(key)));patchDevice(id,{parameters:{...retained,...value}})}} /></fieldset>}
     {stage==='geometry'&&mode!=='real'&&<p className="wc-note">当前为{backendNames[mode]}。切换为真实设备后可配置硬件连接与驱动参数。</p>}
    </article>})}
    {!count[kind]&&<p className="wc-empty">尚未添加{kind==='input'?'输入设备':kind==='actuator'?'机器人或末端':'相机'}。</p>}
   </Section>)}
  </fieldset>}</>}
