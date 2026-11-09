import type { Mount } from '../../lib/workcell'
export function MountTree({mounts,selected,onSelect}:{mounts:Mount[];selected:string;onSelect:(id:string)=>void}){
 const render=(mount:Mount,ancestors:Set<string>):React.ReactNode=>{
  const id=mount.child.split('.')[0],cycle=ancestors.has(id)
  const next=new Set([...ancestors,id])
  const children=mounts.filter(m=>m.parent.split('.')[0]===id)
  return <li key={mount.child}><button aria-pressed={id===selected} onClick={()=>onSelect(id)}>{mount.child}<small>安装于 {mount.parent}{cycle?' · 检查循环引用':''}</small></button>{!cycle&&children.length>0&&<ul>{children.map(m=>render(m,next))}</ul>}</li>
 }
 const roots=mounts.filter(m=>!mounts.some(p=>p.child.split('.')[0]===m.parent.split('.')[0]))
 // Invalid cycles remain visible so the check page can lead to their editors.
 const covered=new Set<string>();const visit=(m:Mount)=>{if(covered.has(m.child))return;covered.add(m.child);mounts.filter(c=>c.parent.split('.')[0]===m.child.split('.')[0]).forEach(visit)};roots.forEach(visit)
 return <nav className="wc-mount-tree" aria-label="安装关系树"><strong>世界坐标系</strong><ul>{roots.map(m=>render(m,new Set()))}{mounts.filter(m=>!covered.has(m.child)).map(m=>render(m,new Set()))}</ul>{!mounts.length&&<p className="wc-note">尚无安装关系。</p>}</nav>
}
