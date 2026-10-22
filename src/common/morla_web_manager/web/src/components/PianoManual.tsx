import { useEffect, useState } from 'react'
import type { PianoConfig, PianoSession, PianoSide } from '../pianoTypes'

const handNames = ['拇指弯曲', '拇指旋转1', '拇指旋转2', '食指侧摆', '食指关节1', '食指关节2', '中指关节1', '中指关节2', '无名指关节1', '无名指关节2', '小指关节1', '小指关节2']
const deg = (r: number) => r * 180 / Math.PI
type Capture = 'arm_zero' | 'hover' | 'press' | 'pose' | 'step'

export function PianoManual({ session, draft, canCommand, confirmed, command, capture, perform }: {
  session: PianoSession | null; draft: PianoConfig; canCommand: boolean; confirmed: boolean
  command: (op: string, args?: Record<string, unknown>) => Promise<unknown>
  capture: (type: Capture, side: PianoSide, shift: number, finger: 'index' | 'middle' | 'ring' | 'pinky') => Promise<unknown>
  perform: (action: () => Promise<unknown>) => Promise<void>
}) {
  const [side, setSide] = useState<PianoSide>('left')
  const [frame, setFrame] = useState('base')
  const [mm, setMm] = useState(1)
  const [degrees, setDegrees] = useState(.5)
  const [part, setPart] = useState<'arm' | 'hand'>('arm')
  const [inputs, setInputs] = useState<Record<string, string>>({})
  useEffect(() => setInputs({}), [session?.session_id])
  const [shift, setShift] = useState(1)
  const [finger, setFinger] = useState<'index' | 'middle' | 'ring' | 'pinky'>('index')
  const key = `${side}_${part}`
  const measured = session?.positions?.[key]
  const commanded = session?.command_targets?.[key]
  const age = session?.feedback_age_s?.[key]
  const fresh = !!measured && session?.connected === true && (session.mode === 'software' || (age != null && age <= .5))
  const enabled = canCommand && confirmed && fresh
  const pose = session?.poses?.[side]
  const limits = draft.hands[side][`${part}_limits_rad`] as number[][]
  function move(index: number, target: number) {
    return perform(() => command('joint', { side, part, joint_index: index, target_deg: target }))
  }
  function read(type: Capture) { return perform(() => capture(type, side, shift, finger)) }

  return <section className="piano-section piano-manual">
    <div className="piano-section-title"><h3>在线点动与实测标定</h3><span>读数为实测值；输入目标后点击发送。角度 °，距离 mm。</span></div>
    <p className="piano-muted">先勾选上方「允许当前目标用于标定试动」。每次点击执行一个有限步长，等待到位后再操作。手动调节不会自动回旧零位；完成后须保存并重新准备才能演奏。</p>
    <div className="piano-row">
      <label className="piano-field">点动 / 工作位 / 标定侧<select value={side} disabled={session?.busy} onChange={e => setSide(e.target.value as PianoSide)}><option value="left">左臂 / 左手</option><option value="right">右臂 / 右手</option></select></label>
      <span className={fresh ? 'piano-live' : 'piano-error'}>{fresh ? `实测反馈${age == null ? '（软件）' : ` · ${(age * 1000).toFixed(0)} ms`}` : '实测反馈未就绪 / 已过期'}</span>
      <button disabled={!enabled} onClick={() => void perform(() => command('work_pose', { side }))}>{side === 'left' ? '左臂' : '右臂'}沿 MORLA 四段轨迹 → 钢琴工作位</button>
    </div>
    <p className="piano-muted">工作位跟随所选侧，仅向本侧机械臂下发目标并判断到位；另一侧的位置误差不参与进段。工作位五段均使用3°到位容差，且须稳定0.2秒。全局停止与急停仍保护双臂。</p>
    {session?.work_progress && <p className="piano-notice">{session.work_progress.side === 'left' ? '左臂' : session.work_progress.side === 'right' ? '右臂' : '双臂'}工作位进度：第 {session.work_progress.stage} / {session.work_progress.total} 段。前四段为原 MORLA J 轨迹，第五段为钢琴工作位。</p>}
    <div className="piano-cartesian">
      <h4>笛卡尔有限步点动</h4>
      <p className="piano-muted">末端为 {pose?.tip_frame ?? `${side}_link7`}，参考系为 {pose?.base_frame ?? `${side}_base_link`}，不是琴键坐标系或指尖。基座轴点动与工具自身轴点动可切换。</p>
      {pose && <div className="piano-live-pose">XYZ：{pose.xyz_mm.map(v => v.toFixed(2)).join(' / ')} mm　　RPY：{pose.rpy_deg.map(v => v.toFixed(2)).join(' / ')} °</div>}
      {session?.kinematics_error && <p className="piano-error">运动学不可用：{session.kinematics_error}</p>}
      <div className="piano-row">
        <label className="piano-field">点动轴坐标系<select value={frame} onChange={e => setFrame(e.target.value)}><option value="base">本侧机械臂基座</option><option value="tool">本侧 link7 工具轴</option></select></label>
        <label className="piano-field">平移步长 mm<input type="number" min="0.1" max="5" step="0.1" value={mm} onChange={e => setMm(Number(e.target.value))} /></label>
        <label className="piano-field">旋转步长 °<input type="number" min="0.1" max="3" step="0.1" value={degrees} onChange={e => setDegrees(Number(e.target.value))} /></label>
      </div>
      <div className="piano-jog-grid">{['x', 'y', 'z', 'rx', 'ry', 'rz'].map(axis => <div key={axis}><strong>{axis.toUpperCase()}</strong>{[-1, 1].map(sign => <button key={sign} className="secondary" disabled={!enabled || !pose || !(mm > 0 && mm <= 5 && degrees > 0 && degrees <= 3)} onClick={() => void perform(() => command('cartesian', { side, frame, axis, step: sign * (axis.length === 1 ? mm : degrees) }))}>{sign < 0 ? '−' : '+'}</button>)}</div>)}</div>
    </div>
    <div className="piano-row"><h4>关节目标与实测角度</h4><select aria-label="点动部件" value={part} onChange={e => setPart(e.target.value as 'arm' | 'hand')}><option value="arm">机械臂 J1—J7</option><option value="hand">灵巧手全部12关节</option></select><button className="secondary" disabled={!fresh} onClick={() => setInputs(old => ({ ...old, ...Object.fromEntries(measured!.map((q, i) => [`${key}_${i}`, deg(q).toFixed(3)])) }))}>实测 → 输入目标</button></div>
    <p className="piano-muted">目标输入默认取当前保持目标，首次接管前取实测角；实测角与保持目标分别显示。连续调节仅修改所选关节，其他关节保留原目标。机械臂每次最多调整15°，关节角调节到位容差3°且须稳定0.2秒；笛卡尔点动、灵巧手及演奏沿用配置中的到位容差。灵巧手每次最多30°，均按限速插值执行；拇指仅可手动调节，不参与曲谱演奏。参数配置栏仍使用 rad。</p>
    <div className="piano-joint-list">{Array.from({ length: part === 'arm' ? 7 : 12 }, (_, i) => {
      const id = `${key}_${i}`, actual = measured?.[i], held = commanded?.[i], reference = held ?? actual
      const value = inputs[id] ?? (reference == null ? '' : deg(reference).toFixed(3))
      const target = value.trim() === '' ? NaN : Number(value)
      const low = deg(limits[i][0]), high = deg(limits[i][1])
      return <div className="piano-joint-row" key={id}>
        <strong>{part === 'arm' ? `J${i+1}` : handNames[i]}</strong><output>{actual == null ? '—' : `${deg(actual).toFixed(3)} °`}{held != null && <><br /><small>保持 {deg(held).toFixed(3)} °</small></>}</output>
        <label className="piano-field"><input aria-label={`${side} ${part} ${i+1} 目标角度`} type="number" step="0.1" min={low} max={high} value={value} onChange={e => setInputs(old => ({ ...old, [id]: e.target.value }))} /><small>{low.toFixed(1)} 至 {high.toFixed(1)} °</small></label>
        <button disabled={!enabled || !Number.isFinite(target) || target < low || target > high} onClick={() => void move(i, target)}>发送</button>
      </div>
    })}</div>
    <div className="piano-capture-panel">
      <h4>将当前实测位置记录到参数草稿</h4>
      <p className="piano-muted">先移动到实际对准琴键的位置，再记录。重设演奏零位会清除该侧旧换位点、黑键映射与增量，并把换位范围收为 [0,0]；移动钢琴后须重新标定换位。</p>
      <div className="piano-row"><button className="secondary" disabled={!canCommand || !fresh} onClick={() => void read('arm_zero')}>记录本侧机械臂为演奏零位</button><button className="secondary" disabled={!canCommand || !fresh} onClick={() => void read('hover')}>记录本侧手为悬停姿态</button><button disabled={!enabled} onClick={() => void perform(() => command('hand_pose', { side, target_deg: draft.hands[side].hover_rad.map(deg) }))}>本侧手移到草稿悬停姿态</button></div>
      <div className="piano-row"><label className="piano-field">按下目标手指<select value={finger} onChange={e => setFinger(e.target.value as typeof finger)}><option value="index">食指</option><option value="middle">中指</option><option value="ring">无名指</option><option value="pinky">小指</option></select></label><button className="secondary" disabled={!canCommand || !fresh} onClick={() => void read('press')}>记录该指当前角度为按下目标</button><label className="piano-field">相对零位白键步数<input type="number" min="-35" max="35" step="1" value={shift} onChange={e => setShift(Number(e.target.value))} /></label><button className="secondary" disabled={!canCommand || !fresh || !Number.isInteger(shift) || shift === 0} onClick={() => void read('pose')}>记录换位实测点</button><button className="secondary" disabled={!canCommand || !fresh || !Number.isInteger(shift) || shift === 0} onClick={() => void read('step')}>计算每白键增量</button></div>
      <p className="piano-muted">可以在同一会话连续点动并记录多个位置；记录只修改草稿。手动标定时「停止」保持实测关节，使用上方悬停按钮抬指。最后断开会话、检查并保存参数、重新连接。</p>
    </div>
  </section>
}
