import { useEffect, useRef, useState } from 'react'
import { piano } from '../api/piano'
import { recoverPianoReset } from '../api/pianoRecovery'
import type { PianoConfig, PianoConfigRecord, PianoFinger, PianoMode, PianoPlan, PianoScore, PianoSession, PianoSide } from '../pianoTypes'
import { makePianoHeartbeat } from '../api/pianoHeartbeat'
import { pushToast } from '../hooks/useToast'
import './piano.css'
import { PianoManual } from './PianoManual'

const SIDES: PianoSide[] = ['left', 'right']
// Manual calibration is a separate state; a fresh prepare is required for playback.
const FINGERS: PianoFinger[] = ['index', 'middle', 'ring', 'pinky']
const fingerNames = { index: '食指', middle: '中指', ring: '无名指', pinky: '小指' }
const captureFingerJoints = { index: ['index_bend_joint', 'index_joint1', 'index_joint2'], middle: ['mid_joint1', 'mid_joint2'], ring: ['ring_joint1', 'ring_joint2'], pinky: ['pinky_joint1', 'pinky_joint2'] }
const handJointNames = ['thumb_bend_joint', 'thumb_rota_joint1', 'thumb_rota_joint2', 'index_bend_joint', 'index_joint1', 'index_joint2', 'mid_joint1', 'mid_joint2', 'ring_joint1', 'ring_joint2', 'pinky_joint1', 'pinky_joint2']
type CaptureType = 'initial' | 'pose' | 'step' | 'arm_zero' | 'hover' | 'press'
function resetArmZero(value: PianoConfig, hand: PianoSide, position: number[]) {
  Object.assign(value.hands[hand], { initial_arm_rad: position, arm_positions_rad: {}, key_overrides: {}, arm_step_rad_per_white: Array(7).fill(0), shift_range_white: [0, 0] })
}
const stateNames: Record<string, string> = { DISCONNECTED: '未连接', STARTING: '正在连接', CONNECTED: '反馈已连接', WAITING_FEEDBACK: '等待有效反馈', PREPARING: '准备中', READY: '已到初始位', PLAYING: '演奏中', CALIBRATING: '标定试动中', CALIBRATION_IDLE: '手动标定就绪', STOPPED: '已停止', FINISHED: '演奏结束', FAULT: '故障停止', CLOSING: '正在断开', CLEANUP_FAILED: '驱动尚未退出 · 设备仍锁定' }
const timingNames: Record<string, string> = { control_hz: '控制频率 Hz', max_arm_velocity_rad_s: '臂关节最大速度 rad/s', prepare_duration_s: '初始位移动时间 s', move_duration_s: '换位基础时间 s', settle_s: '换位稳定时间 s', release_settle_s: '抬指间隔 s', lead_in_s: '演奏前预留 s', arrival_tolerance_rad: '到位误差 rad', arrival_timeout_s: '到位超时 s', arrival_dwell_s: '持续到位时间 s', feedback_timeout_s: '反馈过期 s', max_lateness_s: '调度迟到上限 s', max_tracking_error_rad: '跟踪误差上限 rad' }

const performanceDefaults = { sdk_speed_percent: 10, max_arm_velocity_rad_s: .225, move_duration_s: .45, settle_s: .2, release_settle_s: .12, min_press_s: .15 }
const performanceNames: Record<string, string> = { sdk_speed_percent: '演奏 SDK 速度 %（整数1–20）', max_arm_velocity_rad_s: '演奏臂关节最大速度 rad/s', move_duration_s: '演奏换位最短时间 s', settle_s: '演奏换位稳定时间 s', release_settle_s: '演奏抬指最短间隔 s', min_press_s: '演奏按住最短时间 s' }

function noteName(midi: number) { return `${['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B'][midi % 12]}${Math.floor(midi / 12) - 1}` }

function VectorField({ label, value, onChange, disabled = false }: { label: string; value: number[]; onChange: (v: number[]) => void; disabled?: boolean }) {
  const [text, setText] = useState(value.join(', '))
  const [invalid, setInvalid] = useState(false)
  useEffect(() => { setText(value.join(', ')); setInvalid(false) }, [value])
  function apply() {
    const parts = text.trim().replace(/^\[|\]$/g, '').split(/[,，\s]+/).filter(Boolean).map(Number)
    if (parts.length !== value.length || parts.some(n => !Number.isFinite(n))) { setInvalid(true); return }
    setInvalid(false)
    onChange(parts)
  }
  return <label className="piano-field piano-vector">{label}<input disabled={disabled} aria-invalid={invalid} value={text} onChange={e => setText(e.target.value)} onBlur={apply} onKeyDown={e => { if (e.key === 'Enter') { e.preventDefault(); apply() } }} />{invalid && <small className="piano-error">需要 {value.length} 个有限数值，用逗号分隔</small>}</label>
}

function Keyboard({ active, firstNote = 'C2' }: { active?: number; firstNote?: string }) {
  const match = /^([A-G])([#b]?)(-?\d+)$/.exec(firstNote)
  const start = match ? (Number(match[3]) + 1) * 12 + ({ C: 0, D: 2, E: 4, F: 5, G: 7, A: 9, B: 11 }[match[1]] ?? 0) + ({ '#': 1, b: -1 }[match[2]] ?? 0) : 36
  let white = 0
  const keys = Array.from({ length: 61 }, (_, i) => {
    const midi = start + i
    const black = [1, 3, 6, 8, 10].includes(midi % 12)
    const rank = white
    if (!black) white++
    return { midi, black, rank }
  })
  return <div className="piano-keyboard" aria-label={`61键示意 ${firstNote} 至 ${noteName(start + 60)}`}>{keys.map(k => <div key={k.midi} title={`${noteName(k.midi)} · 第${k.midi - start + 1}键`} className={`piano-key ${k.black ? 'black' : 'white'} ${active === k.midi ? 'active' : ''}`} style={k.black ? { left: `${(k.rank - .3) / white * 100}%`, width: `${.6 / white * 100}%` } : { left: `${k.rank / white * 100}%`, width: `${1 / white * 100}%` }}>
    {!k.black && k.midi % 12 === 0 && <span>{noteName(k.midi)}</span>}
  </div>)}</div>
}

export function PianoTab({ onActiveChange }: { onActiveChange?: (active: boolean) => void } = {}) {
  const [session, setSession] = useState<PianoSession | null>(null)
  const [record, setRecord] = useState<PianoConfigRecord | null>(null)
  const [draft, setDraft] = useState<PianoConfig | null>(null)
  const [certify, setCertify] = useState(false)
  const [mode, setMode] = useState<PianoMode>('software')
  const [scores, setScores] = useState<PianoScore[]>([])
  const [selected, setSelected] = useState('')
  const [tempo, setTempo] = useState(60)
  const [plan, setPlan] = useState<PianoPlan | null>(null)
  const [section, setSection] = useState<'performance' | 'calibration'>('performance')
  const [pending, setPending] = useState(false)
  const [error, setError] = useState('')
  const [owned, setOwned] = useState(false)
  const [calibrationConfirmed, setCalibrationConfirmed] = useState(false)
  const [side, setSide] = useState<PianoSide>('left')
  const [finger, setFinger] = useState<PianoFinger>('pinky')
  const [shift, setShift] = useState(1)
  const [expert, setExpert] = useState('')
  const [options, setOptions] = useState({ left_serial_port: '/dev/serial/by-id/usb-FTDI_USB-RS485-WE_FTAEFQWY-if00-port0', right_serial_port: '/dev/serial/by-id/usb-FTDI_USB-RS485-WE_FTAAUYUK-if00-port0', left_can: 'can_nero_left', right_can: 'can_nero_right' })
  const token = useRef<string | null>(null)
  const recoveryInProgress = useRef(false)
  const recoveryCancelled = useRef(false)
  const sessionId = useRef<string | null>(null)
  const captureJob = useRef<{ id: string; type: CaptureType; side: PianoSide; shift: number; finger: PianoFinger } | null>(null)
  const seenResult = useRef('')
  const mounted = useRef(true)
  const upload = useRef<HTMLInputElement>(null)

  const active = session?.active === true
  useEffect(() => { onActiveChange?.(active) }, [active, onActiveChange])
  const dirty = !!draft && !!record && JSON.stringify(draft) !== JSON.stringify(record.config)
  const busy = pending || session?.busy === true || session?.closing === true
  const ready = session?.state === 'READY'
  const replay = session?.replay_direct_allowed === true && (session?.state === 'FINISHED' || session?.state === 'STOPPED')
  const canCommand = active && owned && !busy && !recoveryInProgress.current && session?.connected === true && session?.state !== 'FAULT' && session?.reset_required !== true
  const canMaintain = active && owned && !busy && !recoveryInProgress.current && session?.maintenance_allowed === true
  const playBlockedReason = !active ? '请先建立会话' : !owned ? '等待当前页面确认控制权' : busy ? '正在执行操作，请等待完成或停止' : !session?.connected ? '等待有效反馈' : session?.state === 'FAULT' ? '请先恢复故障会话' : session?.reset_required ? '请先复位急停并检查反馈' : dirty ? '请先保存参数并重建会话' : !selected ? '请选择曲谱' : !plan || plan.config_revision !== record?.revision ? '请先预检当前曲谱和速度' : mode === 'hardware' && !record?.config.calibrated ? '请先认证保存配置并重建会话' : !ready && !replay ? '请先点击移动到初始位' : ready && Object.values(session?.wrist_shifts ?? {}).some(v => v !== 0) ? '请重新准备到演奏零位' : ''
  const playback = session?.playback
  const needsRecovery = session?.state === 'FAULT' || session?.motion_claimed === true
  const canReset = active && owned && !busy && !recoveryInProgress.current && session?.mode === 'hardware' && (session.maintenance_allowed === true || needsRecovery)
  const resetHint = (active ? session?.mode : mode) !== 'hardware' ? '复位急停仅用于真机模式' : recoveryInProgress.current ? '正在结束旧会话并恢复维护通道；不会自动使能或移动，可点击停止取消恢复' : !active ? '先填写设备参数并建立真机会话，再复位急停' : !owned ? '请在持有本会话的页面操作' : busy ? '请等待当前操作结束，或先停止' : needsRecovery ? '点击复位急停会安全结束旧会话、自动重建并请求复位；保留参数草稿，不会继续工作位或演奏' : !session?.maintenance_allowed ? '等待双臂维护接口；复位不要求双手反馈就绪' : '双臂维护已就绪，可独立复位；不会自动使能或演奏'

  function adopt(value: PianoConfigRecord) { setRecord(value); setDraft(value.config); setCertify(value.config.calibrated); setExpert(JSON.stringify(value.config, null, 2)) }
  function update(change: (value: PianoConfig) => void) {
    setDraft(previous => { if (!previous) return previous; const next = structuredClone(previous); change(next); setExpert(JSON.stringify(next, null, 2)); return next })
    setCertify(false); setPlan(null)
  }
  async function refreshScores() {
    const rows = await piano.scores(); if (!mounted.current) return
    setScores(rows)
    setSelected(previous => previous || rows[0]?.id || '')
  }
  async function perform(action: () => Promise<unknown>, success?: string) {
    setPending(true); setError('')
    try { await action(); if (success) pushToast(success, 'success') }
    catch (e) { const message = e instanceof Error ? e.message : String(e); setError(message); pushToast(message, 'error') }
    finally { if (mounted.current) setPending(false) }
  }

  async function resetEmergency() {
    if (!token.current || !sessionId.current || !record) throw new Error('请先建立真机会话')
    recoveryInProgress.current = true
    recoveryCancelled.current = false
    setPlan(null)
    try {
      const value = await recoverPianoReset(piano, {
        token: token.current, sessionId: sessionId.current, revision: record.revision, devices: options,
        cancelled: () => !mounted.current || recoveryCancelled.current,
        onSession: (nextToken, nextSession) => {
          token.current = nextToken
          sessionId.current = nextSession?.session_id ?? null
          if (mounted.current) { setOwned(!!nextToken); if (nextSession) setSession(nextSession) }
        },
      })
      if (mounted.current) setSession(value)
      pushToast('双臂复位已确认；请检查反馈，再单独准备', 'success')
    } finally {
      recoveryInProgress.current = false
    }
  }

  function stopPiano(emergency = false) {
    recoveryCancelled.current = true
    return piano.stop(emergency)
  }

  useEffect(() => {
    mounted.current = true
    void Promise.all([piano.config(), refreshScores()]).then(([configuration]) => { if (mounted.current) adopt(configuration) }).catch(e => setError(String(e)))
    let polling = false
    async function poll() {
      if (polling) return
      polling = true
      try {
        const value = await piano.state()
        if (mounted.current) {
          if (token.current && value.session_id !== sessionId.current) {
            if (recoveryInProgress.current) return
            token.current = null
            sessionId.current = null
            setOwned(false)
            setError('服务或会话已变化，旧控制权已失效；请重新建立会话')
          }
          setSession(value)
          if (!value.active) { token.current = null; setOwned(false); setError(old => old.startsWith('控制心跳暂时中断：') ? '' : old) }
        }
      } catch (e) { if (mounted.current) setError(`状态连接失败：${String(e)}`) }
      finally { polling = false }
    }
    void poll()
    const timer = setInterval(() => void poll(), 500)
    const renew = makePianoHeartbeat({
      getToken: () => token.current,
      send: (value, signal) => piano.heartbeat(value, signal),
      recovered: () => { if (mounted.current && !recoveryInProgress.current) { setOwned(true); setError(value => value.startsWith('控制心跳暂时中断：') ? '' : value) } },
      failed: error => { if (mounted.current && !recoveryInProgress.current) { setError(`控制心跳暂时中断：${String(error)}；连接恢复后会重新确认控制权`); setOwned(false) } },
    })
    const heartbeat = setInterval(() => void renew.beat(), 1000)
    const resume = () => { if (!document.hidden) { void renew.beat(); void poll() } }
    const close = () => { if (token.current) void piano.close(token.current).catch(() => undefined) }
    window.addEventListener('pagehide', close)
    window.addEventListener('focus', resume)
    document.addEventListener('visibilitychange', resume)
    return () => { mounted.current = false; clearInterval(timer); clearInterval(heartbeat); renew.dispose(); window.removeEventListener('pagehide', close); window.removeEventListener('focus', resume); document.removeEventListener('visibilitychange', resume); close() }
  }, [])

  useEffect(() => {
    const result = session?.result
    if (!result || result.job_id === seenResult.current) return
    seenResult.current = result.job_id
    if (!result.ok && result.error) { setError(result.error); pushToast(result.error, 'error') }
    const capture = captureJob.current
    if (capture && capture.id === result.job_id && result.ok && result.result?.positions) {
      const positions = result.result.positions
      const seeds = result.result.hand_command_seeds
      const recordHand = (value: PianoConfig, hand: PianoSide) => {
        value.hands[hand].start_reference_rad = [...positions[`${hand}_hand`]]
        value.hands[hand].hover_rad = [...(seeds?.[hand] ?? positions[`${hand}_hand`])]
      }
      update(value => {
        if (capture.type === 'initial') {
          for (const hand of SIDES) { resetArmZero(value, hand, positions[`${hand}_arm`]); recordHand(value, hand) }
        } else if (capture.type === 'arm_zero') resetArmZero(value, capture.side, positions[`${capture.side}_arm`])
        else if (capture.type === 'hover') recordHand(value, capture.side)
        else if (capture.type === 'press') value.hands[capture.side].press_rad[capture.finger] = Object.fromEntries(captureFingerJoints[capture.finger].map(name => [name, positions[`${capture.side}_hand`][handJointNames.indexOf(name)]]))
        else if (capture.type === 'pose') {
          value.hands[capture.side].arm_positions_rad[String(capture.shift)] = positions[`${capture.side}_arm`]
          const range = value.hands[capture.side].shift_range_white
          value.hands[capture.side].shift_range_white = [Math.min(range[0], capture.shift), Math.max(range[1], capture.shift)]
        }
        else value.hands[capture.side].arm_step_rad_per_white = positions[`${capture.side}_arm`].map((q, i) => (q - value.hands[capture.side].initial_arm_rad[i]) / capture.shift)
      })
      captureJob.current = null
      pushToast('实测值已填入草稿；断开会话后保存，再重新连接', 'success')
    }
  }, [session?.result])

  async function command(operation: string, extra: Record<string, unknown> = {}) {
    if (!token.current) throw new Error('本页面未持有钢琴会话')
    const sentToken = token.current
    await piano.heartbeat(sentToken)
    if (token.current !== sentToken) throw new Error('会话已变化，请重新确认当前会话')
    return piano.command(sentToken, operation, { calibration_confirmed: calibrationConfirmed, ...extra })
  }
  async function capture(type: CaptureType, hand = side, steps = shift, digit = finger) {
    if ((type === 'pose' || type === 'step') && (steps === 0 || !Number.isInteger(steps) || Math.abs(steps) > 35)) throw new Error('记录移位时需要 ±35 内非零整数步数')
    if ((type === 'hover' || type === 'initial') && !confirm('确认当前手姿正确并记录为软件悬停参考？保存本身不运动、不写设备Flash。以后明确发起试动时，贴近参考的小幅越界关节会使用最近合法边界目标，最多2.86°；原始读数保留，较大偏差仍拒绝。')) return
    const result = await command('capture')
    captureJob.current = { id: result.id, type, side: hand, shift: steps, finger: digit }
  }

  async function importFile(file: File) {
    if (file.size > 512000) throw new Error('曲谱不得超过512KB')
    const format = file.name.toLowerCase().endsWith('.csv') ? 'csv' : 'json'
    const result = await piano.importScore(file.name.replace(/\.(json|csv)$/i, ''), await file.text(), format, tempo, draft?.score_defaults.tonic ?? 'C4')
    await refreshScores(); setSelected(result.id); setPlan(null)
  }
  const presses = session?.events?.filter(e => e.event === 'press') ?? []
  const lastPress = presses[presses.length - 1]

  return <div className="piano-page">
    <div className="piano-heading"><div><p className="piano-eyebrow">MORLA · NERO + XHAND</p><h2>钢琴工作台</h2><p>曲谱、标定与演奏独立操作。切换标签页可保留空闲会话；关闭或刷新本页面会释放会话。</p></div>
      <div className={`piano-state ${session?.state === 'FAULT' ? 'fault' : ''}`}><span />{stateNames[session?.state ?? 'DISCONNECTED'] ?? session?.state}</div></div>
    {error && <div role="alert" className="piano-notice piano-error">{error}</div>}
    {session?.project_error && <div className="piano-notice piano-error">{session.project_error}</div>}
    {SIDES.map(hand => record?.config.hands[hand].arrival_check_enabled === false && <div key={hand} className="piano-notice">{hand === 'left' ? '左手' : '右手'}关节到位误差检查已关闭；动作不作为实测到位确认。反馈、故障及关节限位检查仍启用。</div>)}
    {!!session?.blockers?.length && <div className="piano-notice">暂不能建立钢琴会话：{session.blockers.join('；')}。请在机器人操作页停止相关通道。</div>}
    {active && !owned && <div className="piano-notice">当前页面尚未确认控制权，暂不可发送运动；仍可请求停止。原页面连接恢复后会重新确认控制权。</div>}
    {active && session?.reset_required && <div className="piano-notice">机械臂控制器报告急停锁存。请先点击「复位急停」，完成后点击「重新检查反馈」；工作位、点动和准备暂不可用。</div>}

    <section className="piano-section piano-session">
      <div className="piano-row"><label className="piano-field">运行环境<select value={active ? session?.mode ?? mode : mode} disabled={active || pending} onChange={e => setMode(e.target.value as PianoMode)}><option value="software">纯软件演练 · 无ROS/硬件</option><option value="ros_fake">ROS fake · MORLA驱动联调</option><option value="hardware">真机 · 独立驱动通道</option></select></label>
        <span className="piano-muted">{record?.config.calibrated ? '当前保存参数已认证' : '当前参数未认证，可先演练与标定'}</span>
        <button disabled={active || pending || dirty || !record || session?.available === false || !!session?.blockers?.length} onClick={() => void perform(async () => {
          const result = await piano.open(mode, record!.revision, options); token.current = result.token; sessionId.current = result.session.session_id ?? null; setOwned(true); setSession(await piano.state())
        })}>建立会话</button>
        <button className="secondary" disabled={!owned || !active || session?.closing || recoveryInProgress.current} onClick={() => void perform(() => piano.close(token.current!))}>断开并释放</button>
        <button className="stop" disabled={!active && !recoveryInProgress.current} onClick={() => void perform(() => stopPiano())}>停止（演奏时抬指）</button>
        <button className="emergency" disabled={!active && !recoveryInProgress.current} onClick={() => { if (confirm('向钢琴双臂请求电子急停？请检查实体反馈。')) void perform(() => stopPiano(true)) }}>急停</button>
        <button className="secondary" disabled={!canReset} title={resetHint} onClick={() => { if (confirm(needsRecovery ? '将停止并释放旧会话、自动重建会话，然后仅请求复位急停。保留未保存标定草稿，不会使能、移动或继续原动作。确认恢复？' : '请求复位双臂控制器急停？复位不会自动使能或恢复运动，完成后请重新检查反馈。')) void perform(resetEmergency) }}>复位急停</button>
      </div>
      <p className="piano-muted" role="status">{resetHint}</p>
      {active && <p className="piano-muted">空闲会话不因浏览器后台暂停而超时释放；运动及维护期间仍为6秒失联停止。返回页面立即重检连接，不会自动恢复旧动作。结束使用请点击断开并释放。</p>}
      {active && owned && session?.mode === 'hardware' && <div className="piano-row">
        <button className="secondary" disabled={busy || !session.hand_feedback_start_allowed} onClick={() => {
          if (confirm('启动双手反馈会将全部手指设为无力模式，并持续读取应答；手指可能松弛。不会移动机械臂或发送手指位置目标。停止、失联和断开也会转无力。请先确认手部可安全松弛。')) void perform(() => command('start_hand_feedback', { calibration_confirmed: true }))
        }}>启动双手反馈（无力模式）</button>
        {SIDES.map(side => session.hand_sdk?.[side] && <span className="piano-muted" key={side}>{side === 'left' ? '左手' : '右手'} SDK：{session.hand_sdk[side].feedback_valid ? '通信读数有效（不代表运动已就绪）' : session.hand_sdk[side].error || '等待初始化'}</span>)}
        <details><summary>双手 SDK 原始诊断</summary><pre>{JSON.stringify(session.hand_sdk, null, 2)}</pre><p className="piano-muted">通信应答不代表电机健康。故障字段为设备原始值；诊断查看不会发送运动。</p></details>
        {Object.entries(session.feedback_warnings || {}).map(([key, warning]) => <p className="piano-muted" key={key}>{warning}</p>)}
      </div>}
      {mode === 'hardware' && <div className="piano-grid four">{Object.entries(options).map(([key, value]) => <label key={key} className="piano-field">{{ left_serial_port: '左手串口', right_serial_port: '右手串口', left_can: '左臂 CAN', right_can: '右臂 CAN' }[key]}<input disabled={active || pending} placeholder={key.endsWith('port') ? '/dev/serial/by-id/…' : ''} value={value} onChange={e => setOptions(old => ({ ...old, [key]: e.target.value }))} /></label>)}</div>}
      <div className="piano-row piano-small"><span>控制：{session?.job?.operation ?? '空闲'}</span><span>左手移位 {session?.wrist_shifts?.left ?? 0} / 右手移位 {session?.wrist_shifts?.right ?? 0}</span><span>{session?.pressed ? `${session.pressed[0] === 'left' ? '左手' : '右手'}${fingerNames[session.pressed[1]]}按下` : session?.connected ? '无按键动作' : '实测状态尚未确认'}</span>
      {session?.state === 'WAITING_FEEDBACK' && owned && <><button className="secondary" disabled={busy} onClick={() => void perform(() => command('refresh'))}>重新检查反馈</button>{mode === 'hardware' && <button className="secondary" disabled={!canMaintain} onClick={() => { if (confirm('CAN初始化会使能两臂电机。请确认设备静止且允许上电保持。')) void perform(() => command('initialize_can', { calibration_confirmed: true })) }}>CAN初始化（含使能）</button>}</>}
      </div>
      <div className="piano-row"><button className="secondary" disabled={!canCommand || (mode === 'hardware' && !calibrationConfirmed && !record?.config.calibrated)} onClick={() => void perform(() => command('home'))}>双臂按钢琴轨迹回起始位</button><span className="piano-muted">先抬指 → 演奏零位 → J4到95° → J1到0° → J2到60° → 起始位（J2=85°，其余0°）；最后一段J4在80%时间处完成。左右臂使用各自原生关节角，返回后须重新准备才能演奏。</span></div>
      {session?.error && <p className="piano-error">{session.error}</p>}
    </section>

    <div className="piano-subtabs"><button aria-pressed={section === 'performance'} onClick={() => setSection('performance')}>曲谱与演奏</button><button aria-pressed={section === 'calibration'} onClick={() => setSection('calibration')}>参数与标定</button></div>
    {section === 'performance' ? <>
      <section className="piano-section"><div className="piano-section-title"><h3>演奏</h3><span>左手基准 {record?.config.hands.left.anchor_note ?? 'C3'}　/　右手基准 {record?.config.hands.right.anchor_note ?? 'C4'}　·　四指演奏，不用拇指</span></div>
        <Keyboard active={session?.pressed ? lastPress?.midi : undefined} firstNote={record?.config.keyboard.first_note} />
        <div className="piano-grid performance">
          <label className="piano-field">曲谱<select value={selected} disabled={busy} onChange={e => { setSelected(e.target.value); setTempo(scores.find(s => s.id === e.target.value)?.tempo_bpm ?? 60); setPlan(null) }}>{scores.map(score => <option value={score.id} key={score.id}>{score.name} · {score.events}项</option>)}</select></label>
          <label className="piano-field">速度 BPM<input type="number" min={20} max={300} value={tempo} disabled={busy} onChange={e => { setTempo(Number(e.target.value)); setPlan(null) }} /></label>
          <button className="secondary" disabled={!selected || busy || dirty} onClick={() => void perform(async () => setPlan(await piano.plan(selected, tempo)))}>预检时序 / 指法</button>
          <button disabled={!canCommand || dirty} onClick={() => void perform(() => command('prepare'))}>移动到初始位</button>
          <button disabled={!canCommand || !!playBlockedReason} title={playBlockedReason || (replay ? '先抬指并直接回演奏零位，再从头演奏' : '演奏所选曲谱')} onClick={() => void perform(() => command('play', { score_id: selected, tempo_bpm: tempo, prepare_first: replay }))}>{replay ? '重新演奏（直接回演奏零位）' : '开始演奏'}</button>
        </div>
        {record?.config.performance && <p className="piano-muted">演奏 SDK {record.config.performance.sdk_speed_percent}% · 规划上限 {record.config.performance.max_arm_velocity_rad_s} rad/s · 换位至少 {record.config.performance.move_duration_s}s + 稳定 {record.config.performance.settle_s}s。SDK已应用：左 {session?.arm_speed?.left.stream_applied_speed_percent ?? '—'}% / 右 {session?.arm_speed?.right.stream_applied_speed_percent ?? '—'}%（软件调用回执，非实测速率）。</p>}
        {playBlockedReason && <p className="piano-muted">演奏暂不可用：{playBlockedReason}</p>}
        {replay && canCommand && !playBlockedReason && <p className="piano-muted">点击“重新演奏”后先抬指，双臂从演奏结束位置直接回到保存的演奏零位，确认后从头播放。</p>}
        {session?.state === 'PREPARING' || (session?.busy && session?.job?.operation === 'play' && session?.state === 'CALIBRATING') ? <p className="piano-live" role="status">正在准备双臂演奏位置{session?.work_progress ? ` · 第${session.work_progress.stage}/${session.work_progress.total}段` : ''}，准备完成后继续本次操作。</p> : null}
        {playback && <div className="piano-playback" aria-label="演奏进度">
          <div className="piano-row"><strong>{playback.title}</strong><span>{stateNames[session?.state ?? ''] ?? session?.state}</span><span>{playback.percent.toFixed(1)}%</span></div>
          <progress max={100} value={playback.percent} aria-label="演奏完成百分比" />
          <div className="piano-row piano-small"><span>{playback.elapsed_s.toFixed(1)} / {playback.duration_s.toFixed(1)} 秒</span><span>已执行抬指 {playback.completed_notes} / {playback.total_notes} 音</span><span>动作 {playback.completed_actions} / {playback.total_actions}</span></div>
          <p className="piano-muted">{playback.completed ? '本次演奏已完成' : playback.current_action ? `${playback.current_action.side === 'left' ? '左手' : '右手'} ${playback.current_action.finger ? fingerNames[playback.current_action.finger] : '手腕'} · ${{ move: '换位', press: '按下', release: '抬起' }[playback.current_action.kind]}${playback.current_action.midi !== null ? ` ${noteName(playback.current_action.midi)}` : ''} · 白键移位 ${playback.current_action.shift}` : '等待首个动作'}{!session?.connected && !playback.completed ? ' · 反馈连接未就绪，进度暂停更新' : ''}</p>
          <p className="piano-muted">进度来自服务器执行记录；按下、抬起数量表示指令进度，不代表琴键触发已实测确认。</p>
        </div>}
        <p className="piano-muted">先保存参数 → 建立会话 → 预检曲谱 → 移动到初始位（双臂 MORLA 四段轨迹 → 工作位 → 演奏零位）→ 开始演奏。演奏结束后可直接重新演奏，先回演奏零位再从头播放；停止随时可用。</p>
      </section>
      <section className="piano-section"><div className="piano-row"><h3>曲谱库</h3><input ref={upload} type="file" accept=".json,.csv" hidden onChange={e => { const file = e.target.files?.[0]; if (file) void perform(() => importFile(file)); e.target.value = '' }} /><button className="secondary" disabled={active || pending} onClick={() => upload.current?.click()}>导入 CSV / JSON</button><button className="secondary" disabled={!selected || pending} onClick={() => void perform(async () => { const content = await piano.score(selected); const url = URL.createObjectURL(new Blob([JSON.stringify(content, null, 2)], { type: 'application/json' })); const link = document.createElement('a'); link.href = url; link.download = 'piano-score.json'; link.click(); URL.revokeObjectURL(url) })}>下载曲谱</button><button className="secondary" disabled={active || !selected || pending} onClick={() => { if (confirm('删除选中曲谱？')) void perform(async () => { await piano.removeScore(selected); setSelected(''); setPlan(null); await refreshScores() }) }}>删除</button></div>
        <p className="piano-muted">start 为绝对起始拍，duration 为持续拍数；hand=left/right；note 支持 C3、1-、1、1+、0休止。图片先整理为时序表。CSV使用当前 BPM 与已保存调号。</p>
        {plan && <><div className="piano-plan-summary"><strong>{plan.title}</strong><span>{plan.notes} 个音</span><span>{plan.duration_s.toFixed(2)} 秒</span><span>{plan.actions.filter(a => a.kind === 'move').length} 次换位</span></div><div className="piano-table-wrap"><table><thead><tr><th>秒</th><th>动作</th><th>手 / 手指</th><th>音符</th><th>移位</th><th>谱表行</th></tr></thead><tbody>{plan.actions.slice(0, 100).map((action, i) => <tr key={i}><td>{action.at_s.toFixed(3)}</td><td>{{ press: '按下', release: '抬起', move: '换位' }[action.kind]}</td><td>{action.side === 'left' ? '左' : '右'} {action.finger ? fingerNames[action.finger] : '手腕'}</td><td>{action.midi !== null ? noteName(action.midi) : '—'}</td><td>{action.shift}</td><td>{action.row}</td></tr>)}</tbody></table></div>{plan.actions.length > 100 && <p className="piano-muted">仅展示前100个动作，执行完整时序。</p>}</>}
      </section>
    </> : draft && <>
      <section className="piano-section"><label className="piano-check"><input type="checkbox" checked={calibrationConfirmed} onChange={e => setCalibrationConfirmed(e.target.checked)} />允许当前目标用于标定试动</label><p className="piano-muted">手动点动可在未认证时执行，须明确允许；原有准备和单指测试继续使用本次会话保存参数。</p></section>
      <PianoManual session={session} draft={draft} canCommand={canCommand} confirmed={calibrationConfirmed} command={command} capture={capture} perform={perform} />
      <section className="piano-section"><div className="piano-row"><h3>标定操作</h3><span className="piano-muted">所有角度为原生关节 rad，先读取再试动。</span></div>
        <div className="piano-row"><button className="secondary" disabled={!canCommand} onClick={() => void perform(() => capture('initial'))}>读取当前为双臂初始位 / 手悬停</button><button disabled={!canCommand || dirty} onClick={() => void perform(() => command('prepare'))}>按当前保存参数准备（双臂四段轨迹）</button></div>
        <div className="piano-row"><select aria-label="标定手" value={side} onChange={e => setSide(e.target.value as PianoSide)}><option value="left">左手</option><option value="right">右手</option></select><select aria-label="标定手指" value={finger} onChange={e => setFinger(e.target.value as PianoFinger)}>{FINGERS.map(f => <option value={f} key={f}>{fingerNames[f]}</option>)}</select><button disabled={!canCommand || !ready || dirty || (mode === 'hardware' && !calibrationConfirmed && !record?.config.calibrated)} onClick={() => void perform(() => command('finger', { side, finger }))}>单指按下并抬起</button><label className="piano-field">白键步数<input type="number" value={shift} min={-35} max={35} onChange={e => setShift(Number(e.target.value))} /></label><button disabled={!canCommand || !ready || dirty} onClick={() => void perform(() => command('shift', { side, shift_white: shift }))}>测试移位</button><button className="secondary" disabled={!canCommand} onClick={() => void perform(() => capture('pose'))}>记录当前为该步数实测点</button><button className="secondary" disabled={!canCommand || shift === 0} onClick={() => void perform(() => capture('step'))}>用当前位姿计算每白键增量</button></div>
        <p className="piano-muted">读取操作不会移动机器人。实测值进入草稿；断开、保存后重新连接才生效。未认证配置的真机试动需要明确勾选；整曲演奏需完成认证。</p>
      </section>
      <fieldset disabled={active || pending} className="piano-parameters">
        <section className="piano-section"><h3>钢琴工作位 · 各侧原生关节角 / 度</h3><p className="piano-muted">先执行原 MORLA 四段 J 轨迹，再低速到达以下工作位；与琴键演奏零位分开保存。临时修正：左 J5=155°，右 J2=98°、J5=−155°。</p><div className="piano-grid two">{SIDES.map(hand => <VectorField key={hand} label={`${hand === 'left' ? '左' : '右'}臂工作位 J1—J7 / °`} value={draft.calibration.work_pose_deg[hand]} onChange={v => update(value => { value.calibration.work_pose_deg[hand] = v })} />)}</div></section>
        <div className="piano-grid two">{SIDES.map(hand => <section key={hand} className="piano-section"><h3>{hand === 'left' ? '左手 · 小指起始' : '右手 · 食指起始'}</h3><label className="piano-field">基准音<input value={draft.hands[hand].anchor_note} onChange={e => update(value => { value.hands[hand].anchor_note = e.target.value })} /></label>
          <VectorField label="钢琴初始位：J1—J7 / rad" value={draft.hands[hand].initial_arm_rad} onChange={v => update(value => { resetArmZero(value, hand, v) })} />
          <VectorField label="每白键机械臂增量：7维 / rad" value={draft.hands[hand].arm_step_rad_per_white} onChange={v => update(value => { value.hands[hand].arm_step_rad_per_white = v })} />
          <VectorField label="手指悬停姿态：12维 / rad" value={draft.hands[hand].hover_rad} onChange={v => update(value => { value.hands[hand].hover_rad = v })} />
          <VectorField label="允许白键移位 [最小, 最大]" value={draft.hands[hand].shift_range_white} onChange={v => update(value => { value.hands[hand].shift_range_white = v })} />
          {FINGERS.map(f => <div key={f} className="piano-finger"><strong>{fingerNames[f]}按下目标</strong>{Object.entries(draft.hands[hand].press_rad[f]).map(([joint, angle]) => <label key={joint} className="piano-field piano-small">{joint}<input type="number" step="0.01" value={angle} onChange={e => update(value => { value.hands[hand].press_rad[f][joint] = Number(e.target.value) })} /></label>)}</div>)}
          <p className="piano-muted">已记录实测移位点：{Object.keys(draft.hands[hand].arm_positions_rad).join('、') || '无'}</p>
        </section>)}</div>
        <section className="piano-section"><h3>钢琴演奏专用速度</h3>
          <p className="piano-muted">仅演奏与单键移位测试使用；准备、工作位、回位和在线点动使用原速度。保存后重建会话并重新预检曲谱。SDK百分比不是实测速度；手指最短时值是调度约束，需要现场确认按响与回弹。</p>
          {!draft.performance ? <button className="secondary" onClick={() => update(next => { next.performance = { ...performanceDefaults } })}>启用演奏速度参数（SDK 10%）</button> : <div className="piano-grid four">{Object.entries(draft.performance).map(([key, value]) => <label className="piano-field" key={key}>{performanceNames[key] ?? key}<input type="number" step={key === 'sdk_speed_percent' ? 1 : 'any'} value={value} onChange={e => update(next => { next.performance![key] = Number(e.target.value) })} /></label>)}</div>}
        </section>
        <section className="piano-section"><h3>节奏与运动参数（基础速度）</h3><div className="piano-grid four">{Object.entries(draft.timing).map(([key, value]) => <label className="piano-field" key={key}>{timingNames[key] ?? key}<input type="number" step="any" value={value} onChange={e => update(next => { next.timing[key] = Number(e.target.value) })} /></label>)}<label className="piano-field">简谱调号<input value={draft.score_defaults.tonic} onChange={e => update(value => { value.score_defaults.tonic = e.target.value })} /></label><label className="piano-field">CSV默认 BPM<input type="number" value={draft.score_defaults.tempo_bpm} onChange={e => update(value => { value.score_defaults.tempo_bpm = Number(e.target.value) })} /></label></div></section>
        <details className="piano-section"><summary>完整配置 JSON · 关节选择、指距、黑键与实测点</summary><p className="piano-muted">仅允许对应手指的关节。硬件关节限位不能放宽。黑键须显式记录实测姿态，不自动猜测。</p><textarea aria-label="完整钢琴配置" rows={16} value={expert} onChange={e => setExpert(e.target.value)} /><button className="secondary" onClick={() => void perform(async () => { const value = await piano.validateConfig(JSON.parse(expert)); setDraft(value); setCertify(false); setPlan(null) })}>应用到参数草稿</button></details>
        <section className="piano-section"><div className="piano-row"><label className="piano-check"><input type="checkbox" checked={certify} onChange={e => setCertify(e.target.checked)} />我已实测初始位、手指角度与移位路径，认证此配置可用于真机演奏</label><button disabled={!record} onClick={() => void perform(async () => { adopt(await piano.save(draft, record!.revision, certify)); setPlan(null) }, '参数保存成功')}>保存参数{dirty ? '（有未保存修改）' : ''}</button><button className="secondary" onClick={() => void perform(async () => adopt(await piano.config()))}>重新载入</button></div></section>
      </fieldset>
      {active && <p className="piano-notice">会话参数已固定。修改参数请先断开，保存后重新连接；可以继续读取实测值到草稿。</p>}
    </>}
    <details className="piano-section"><summary>执行日志与故障信息</summary>{session?.recovery && <p className="piano-error">异常恢复已请求急停；恢复进程退出码：{session.recovery.exit_code ?? '未知'}。{session.recovery.error ?? '进程退出不代表实体已停稳，请检查设备反馈。'}</p>}<pre>{session?.logs?.join('\n') || '暂无日志'}</pre></details>
  </div>
}
