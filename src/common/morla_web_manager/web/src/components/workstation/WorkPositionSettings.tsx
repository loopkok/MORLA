import { NumberInput } from '../WorkcellFields'

import type { WorkstationModel } from '../ComposableTab'
import { Section, Check } from './shared'
export function WorkPositionSettings({actuators, assembly, changed, descriptor, label, locked, stage}:Pick<WorkstationModel,'actuators'|'assembly'|'changed'|'descriptor'|'label'|'locked'|'stage'>){return <>{stage==='control'&&<fieldset disabled={locked} className="wc-unframed"><Section title="直接回工作位" note="按关节直线插补返回配置位置，不经过途径点，也不进行避障规划。未配置设备不参与；录制期间禁止回位。">
    {actuators.filter(([,item])=>(descriptor(item)?.joints?.length||0)>0).map(([id,item])=>{const joints=descriptor(item)!.joints!,config=assembly.metadata?.operator_control||{},values=config.work_positions?.[id];return <div className="wc-pose" key={id}>
     <Check text={'设置 '+(item.display_name||label(item.plugin))+' · '+id+' 工作位'} value={!!values} onChange={enabled=>{const positions={...config.work_positions};if(enabled)positions[id]=joints.map(j=>Math.max(j.lower??-Infinity,Math.min(j.upper??Infinity,0)));else delete positions[id];changed({...assembly,metadata:{...assembly.metadata,operator_control:{...config,work_positions:positions}}})}} />
     {values&&<div className="wc-form-grid">{joints.map((joint,index)=><label key={joint.name}>{joint.name} / {joint.unit}<NumberInput label={id+'工作位'+joint.name} schema={{minimum:joint.lower,maximum:joint.upper}} value={values[index]} onChange={value=>{const next=[...values];next[index]=value;changed({...assembly,metadata:{...assembly.metadata,operator_control:{...config,work_positions:{...config.work_positions,[id]:next}}}})}} /></label>)}</div>}
    </div>})}
    <label>关节插补速度上限 / 关节单位每秒<NumberInput label="工作位插补速度" schema={{minimum:.001,maximum:1}} value={assembly.metadata?.operator_control?.work_speed??.15} onChange={value=>changed({...assembly,metadata:{...assembly.metadata,operator_control:{...assembly.metadata?.operator_control,work_speed:value}}})} /></label>
   </Section></fieldset>}</>}
