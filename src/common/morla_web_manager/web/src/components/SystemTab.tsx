import { useEffect, useState } from 'react'
import type { NormalisedState } from '../lib/mapUiState'
import type { Preset, ProfileCatalog } from '../types'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'
import { LatencyPanel } from './LatencyPanel'
import { LogConsole } from './LogConsole'
import { VideoCard } from './VideoCard'

interface Props {
  state: NormalisedState | null
  presets: Preset[]
  onAction: () => void
}

const phases: Record<string, string> = {
  idle: '待操作', accepted: '请求已接受', freezing: '正在暂停控制输入',
  executing: '正在执行', completed: '已完成', failed: '失败', cancelled: '已取消',
}
const operations: Record<string, string> = {
  prepare: '准备并使能', initialize_can: '初始化连接', enable: '使能', reset: '复位急停', ready: '厂商就绪',
  workpos: '移至工作位', position: '位置模式', home: 'HOME 归位', start: '开始遥操',
  pause: '暂停', resume: '恢复', estop: '急停', damping: '阻尼释放',
}
const armStatusLabels: Record<number, string> = {
  0: '正常', 1: '急停', 2: '无解', 3: '奇异点', 4: '目标角度超限',
  5: '关节通信异常', 6: '关节抱闸未打开', 7: '发生碰撞', 8: '拖动示教超速',
  9: '关节状态异常', 10: '其他异常', 11: '示教记录中', 12: '示教执行中',
  13: '示教暂停', 14: '主控过温', 15: '制动电阻过温',
}
const activeLaunch = (value?: string) => !!value && !['stopped', 'start_failed'].includes(value)

export function SystemTab({ state, presets, onAction }: Props) {
  const [catalog, setCatalog] = useState<ProfileCatalog | null>(null)
  const [profileId, setProfileId] = useState('')
  const [mode, setMode] = useState<'sim' | 'real'>('sim')
  const [pending, setPending] = useState(false)
  const [presetName, setPresetName] = useState('')
  const [log, setLog] = useState(false)
  const control = state?.control
  const live = !!control && !control.stale
  const busy = live && !['idle', 'completed', 'failed', 'cancelled'].includes(control.phase)
  const running = activeLaunch(state?.teleopState)
  const otherLanes = activeLaunch(state?.collectLaunch?.state) || activeLaunch(state?.inferLaunch?.state)
    || !!state?.dataCollect && ['RECORDING', 'PAUSED', 'SAVING'].includes(state.dataCollect.state)
  const locked = running || live || otherLanes || pending
  const chosen = catalog?.profiles.find((p) => p.profile_id === profileId)
  const activeId = live ? control.profile_id : state?.profileSelection?.active_profile_id
  const active = catalog?.profiles.find((p) => p.profile_id === activeId)
  const real = live && control.operating_mode === 'real'
  const driverStatusRows = Object.entries(control?.driver_status ?? {})
  const pathSettings = driverStatusRows.map(([side, status]) =>
    `${side === 'left' ? '左臂' : '右臂'}：${status.workpos_waypoints_deg?.map((q, i) => `点${i + 1}（${q.join(' ')}）`).join(' → ') ?? '等待配置'}；工作位速度 ${status.position_speed_percent ?? '配置值'}%，HOME 速度 ${status.home_speed_percent ?? '配置值'}%`
  ).join('；')
  const motionBlockers = (kind: 'home' | 'workpos') => active?.robot_id === 'nero' ? active.arms.flatMap(({ side }) => {
    const status = control?.driver_status?.[side]
    const label = side === 'left' ? '左臂' : '右臂'
    if (!status?.connected || !status.feedback_fresh || !status.controller_status_fresh) return [`${label}连接或反馈未就绪`]
    if (status.trajectory_revision !== 'nero_four_points_home85_v2') return [`${label}${kind === 'home' ? 'HOME' : '工作位轨迹'}配置尚未更新，请重启机器人 Profile`]
    if (status.controller_arm_status === 1) return [`${label}急停未解除，请先复位急停`]
    if (status.controller_arm_status === 6) return [`${label}抱闸未打开，请先准备并使能`]
    if (status.controller_arm_status !== 0) return [`${label}控制器异常（状态 ${status.controller_arm_status ?? '未知'}）`]
    if (status.motor_status_fresh == null) return [`${label}尚未加载电机使能反馈，请重启机器人 Profile`]
    if (!status.motor_status_fresh) return [`${label}电机使能反馈缺失或过期`]
    if (!status.motors_enabled) return [`${label}电机未全部使能，请先准备并使能`]
    if (!status.position_permitted) return [`${label}位置动作许可已撤销，请先准备并使能`]
    return []
  }) : []
  const homeBlockers = motionBlockers('home')
  const workposBlockers = motionBlockers('workpos')
  const resetRows = active?.arms.map(({ side }) => control?.driver_status?.[side]) ?? []
  const unknownControllerStatus = resetRows.some((status) => !status?.controller_status_fresh)
  const hasEstop = resetRows.some((status) => status?.controller_status_fresh && status.controller_arm_status === 1)
  const canReset = resetRows.length > 0 && (unknownControllerStatus || hasEstop) && resetRows.every((status) =>
    status?.connected && status.reset_unknown_status_supported && !status.enabled && !status.command_active && !status.position_active
    && (!status.controller_status_fresh || [0, 1, 6].includes(status.controller_arm_status ?? -1)))
  const hasBrakeNotReleased = driverStatusRows.some(([, status]) =>
    status.controller_status_fresh && status.controller_arm_status === 6)
  const allows = (key: string) => real && control.capabilities?.[key] === true
  const available = (key: string) => allows(key) && !busy && !pending
  const legacy = presets.filter((p) => p.package !== 'morla_data_collect')
  const selectedPreset = presetName || legacy[0]?.name || ''

  useEffect(() => {
    void api.profiles().then((res) => {
      if (!res.ok || !res.data) return
      setCatalog(res.data)
      setProfileId(res.data.requested_profile_id || res.data.profile_id)
      setMode(res.data.operating_mode)
    }).catch(() => pushToast('无法加载机器人列表，请检查 Web 服务连接', 'error'))
  }, [])
  useEffect(() => {
    if (!state?.profileSelection) return
    setProfileId(state.profileSelection.requested_profile_id || state.profileSelection.profile_id)
    setMode(state.profileSelection.operating_mode)
  }, [state?.profileSelection?.requested_profile_id, state?.profileSelection?.operating_mode])

  async function run(fn: () => Promise<{ ok: boolean; message: string }>, message: string) {
    setPending(true)
    try {
      const res = await fn()
      pushToast(res.ok ? message : res.message, res.ok ? 'success' : 'error')
      onAction()
    } catch {
      pushToast('请求未确认，请检查连接及操作状态后再试', 'error')
    } finally { setPending(false) }
  }
  function start() {
    if (mode === 'real' && !confirm(`启动 ${chosen?.title || profileId} 的真机节点？后续使能和移动需分别操作。`)) return
    void run(() => api.launchProfile(profileId, mode), '正在启动，请等待机器人控制服务上线')
  }
  function prepare() {
    if (!confirm('准备当前机器人并使能电机。系统会按当前机器人能力初始化连接、确认反馈并开启控制，不主动移至工作位。请确认设备已停止、周围无障碍且允许电机上电保持。继续？')) return
    void run(() => api.robotPrepare(), '准备请求已提交，请等待下方状态显示完成')
  }
  function moveWorkpos() {
    const path = active?.robot_id === 'nero'
      ? `按点位 1 → 2 → 3 → 4 移至工作位，每段等待所有配置手臂实测到位。${pathSettings}。`
      : '机械臂将从当前位置移动到所配置的工作位。'
    if (!confirm(`${path}遥操会暂停，请确认完整路径安全，继续？`)) return
    void run(() => api.teleopWorkpos(), '工作位请求已提交，请等待实测到位确认')
  }
  function moveHome() {
    const path = active?.robot_id === 'nero'
      ? `沿同一路径反向执行点位 4 → 3 → 2 → 1，再移至最终 HOME [0°,85°,0°,0°,0°,0°,0°]。每段等待所有配置手臂实测到位。${pathSettings}。`
      : '机械臂将沿配置的 HOME 路径收回零位。'
    if (!confirm(`${path} 遥操将暂停，请确认完整路径安全。继续？`)) return
    void run(() => api.teleopHome(), 'HOME 已提交，请等待所有阶段实测到位')
  }
  function resetEmergencyStop() {
    const reason = unknownControllerStatus ? '控制器状态未知，无法确认急停是否解除。将显式发送复位并重新请求 CAN 反馈，必须等新鲜状态回读后才算成功。' : '向处于急停的 Nero 控制器发送复位；已报告正常或抱闸未打开的一侧不重复发送复位。'
    if (!confirm(`${reason}复位可能让机械臂失电，抬起的关节可能掉落。请先支撑机械臂并确认周围安全。此次不使能、不发送运动目标。继续？`)) return
    void run(() => api.robotReset(), '急停复位请求已提交，等待控制器状态确认')
  }

  return <div style={wrap}>
    <section style={card}>
      <div style={heading}>1 · 选择并启动机器人</div>
      <div style={row}>
        <label style={field}>机器人配置
          <select aria-label="机器人配置" value={live ? control.profile_id : profileId} disabled={locked} onChange={(e) => setProfileId(e.target.value)} style={select}>
            {(catalog?.profiles ?? []).map((p) => <option key={p.profile_id} value={p.profile_id}>{p.title} · {p.profile_id}</option>)}
          </select>
        </label>
        <label style={field}>运行环境
          <select aria-label="运行环境" value={live ? control.operating_mode || mode : mode} disabled={locked} onChange={(e) => setMode(e.target.value as 'sim' | 'real')} style={select}>
            <option value="sim">仿真</option><option value="real">真实硬件</option>
          </select>
        </label>
        <button style={button('#2563eb')} disabled={!state || locked || !profileId || !chosen?.launch_modes.includes(mode)} onClick={start}>启动机器人</button>
        <button style={button('#475569')} disabled={!running || otherLanes || pending || busy || state?.teleopState === 'stopping'} onClick={() => {
          if (confirm('停止当前机器人节点？请先停止录制，并确认机械臂已停止。')) void run(() => api.stop(), '已请求停止机器人节点')
        }}>停止机器人</button>
      </div>
      <div style={hint}>
        {live ? <>当前运行：<b>{active?.title || activeId}</b> · {control.operating_mode === 'real' ? '真实硬件' : '仿真'}。切换机器人前请停止当前运行。</>
          : running ? '节点正在启动，等待控制服务上线。'
          : '选择一次即可启动；采集与策略沿用正在运行的机器人配置。'}
        {live && !running && ' 当前由外部终端启动，请在原终端停止。'}
        {otherLanes && ' 采集或策略任务在线，请先在监控页停止相关任务再切换或停止机器人。'}
        {!live && chosen && !chosen.launch_modes.includes(mode) && ' 当前组合不支持网页启动，可在高级选项中仅应用配置供命令行使用。'}
      </div>
    </section>

    <section style={card}>
      <div style={heading}>2 · 准备与移动</div>
      {real ? <>
        <div style={row}>
          <button style={button('#059669')} disabled={!available('prepare')} onClick={prepare}>准备并使能</button>
          {allows('workpos') && <button style={button('#0284c7')} disabled={!available('workpos') || workposBlockers.length > 0} title={workposBlockers.join('；')} onClick={moveWorkpos}>移至工作位</button>}
          {allows('home') && <button style={button('#7c3aed')} disabled={!available('home') || homeBlockers.length > 0} title={homeBlockers.join('；')} onClick={moveHome}>HOME · {active?.robot_id === 'nero' ? '收回姿态' : '收回零位'}</button>}
          <span style={hint}>准备完成后再移动；移动时暂停遥操。{control.capabilities?.workpos_requires_reenable ? '工作位与 HOME 可连续执行；恢复遥操前再准备并使能。' : '状态显示完成后再开始遥操。'}</span>
        </div>
        {allows('workpos') && !busy && workposBlockers.length > 0 && <div style={warning}>工作位尚未就绪：{workposBlockers.join('；')}</div>}
        {allows('workpos') && active?.robot_id === 'nero' && <div style={hint}>工作位：点位 1 → 2 → 3 → 4；HOME：点位 4 → 3 → 2 → 1 → 最终 HOME。每段等待所有配置手臂到位。</div>}
        {allows('home') && !busy && homeBlockers.length > 0 && <div style={warning}>HOME 尚未就绪：{homeBlockers.join('；')}。所有手臂电机反馈就绪后才可归位，工作位到位后无需重复使能。</div>}
        {allows('home') && active?.robot_id === 'nero' && <div style={hint}>HOME 最后一段移至 [0°,85°,0°,0°,0°,0°,0°]，实测到位后才完成。<details><summary>查看当前点位（度）与速度</summary>{pathSettings}</details></div>}
        {allows('initialize_can') && <div style={hint}>当前机器人准备流程：需要时自动执行 CAN 冷启动（含电机使能），确认反馈后启用控制。无需单独操作 CAN。</div>}
        {allows('reset') && <div style={{ ...statusPanel, marginTop: 12 }}>
          <div style={{ ...row, marginBottom: 4 }}><b>控制器与急停状态</b></div>
          <div style={row}>
            {driverStatusRows.map(([side, status]) => {
              const code = status.controller_arm_status
              const normal = status.controller_status_fresh && code === 0
              const estop = status.controller_status_fresh && code === 1
              const brakeNotReleased = status.controller_status_fresh && code === 6
              const label = !status.controller_status_fresh ? '控制器状态未知/过期'
                : estop ? '急停（状态 1）'
                : brakeNotReleased ? '关节抱闸未打开（状态 6）'
                : normal && status.motor_status_fresh && status.motors_enabled
                  ? `控制器正常，电机已使能${status.enabled ? '，遥操许可已开启' : status.position_permitted ? '，遥操输出已关闭（可继续位置动作）' : '，位置动作许可已撤销'}`
                : normal && !status.motor_status_fresh ? '控制器正常，电机使能反馈待确认'
                : normal ? '控制器正常，电机未全部使能，请准备并使能'
                : `${armStatusLabels[code ?? -1] || '控制器异常'}（状态 ${code ?? '未知'}）`
              return <span key={side} style={{ ...badge, color: estop ? '#fca5a5' : normal ? '#86efac' : '#fcd34d' }}>
                {side === 'left' ? '左臂' : side === 'right' ? '右臂' : side}：{label}
                {(status.reset_status === 'failed' || status.reset_status === 'timed_out') ? ` · ${status.reset_error || status.last_error || status.reset_status}` : ''}
              </span>
            })}
          </div>
          <div style={row}>
            <button style={button('#b45309')} disabled={!available('reset') || !canReset} onClick={resetEmergencyStop}>{unknownControllerStatus ? '复位急停 / 恢复反馈' : '复位急停'}</button>
            <span style={hint}>{unknownControllerStatus
              ? '状态未知时可在驱动连接且无运动时显式复位并请求恢复反馈；未知不代表已急停或已复位。若未返回新鲜状态，操作仍失败；检查机械臂供电、USB-CAN 与 CAN 接线。复位后仍需单独准备并使能。'
              : hasEstop
                ? '急停侧执行复位；已正常或抱闸未打开的一侧不重复复位。回读确认急停解除后，仍需单独准备并使能。'
                : hasBrakeNotReleased
                  ? '状态 6 是关节抱闸未打开，急停已解除；确认周边安全后使用“准备并使能”释放抱闸。'
                  : '控制器正常，无需复位急停。'}{resetRows.some((status) => status && !status.reset_unknown_status_supported) ? ' 请重启 Profile 加载新的状态恢复功能。' : ''}</span>
          </div>
        </div>}
        {!control.capabilities?.prepare && <div style={warning}>当前控制服务尚不支持统一准备流程，请停止机器人后重启 Web 与机器人节点。</div>}
      </> : <div style={hint}>{live ? '仿真无需硬件使能和连接初始化，使用顶部「开始遥操」。' : '启动机器人并等待控制服务上线后，可在这里准备硬件。'}</div>}
      {control && <div style={statusPanel} role="status" aria-live="polite">
        <div style={row}>
          <b style={{ color: control.stale ? '#f59e0b' : control.phase === 'failed' ? '#f87171' : '#a7f3d0' }}>
            {control.stale ? '控制服务离线' : phases[control.phase] || control.phase}
          </b>
          {control.operation && <span>{operations[control.operation] || control.operation}</span>}
          {busy && <span style={hint}>等待实测确认，请勿重复操作。</span>}
        </div>
        <div style={hint}>{control.stale ? '请检查启动日志与节点连接。' : busy ? '操作仍在执行，接受请求不代表硬件已完成。'
          : control.phase === 'failed' ? (control.operation === 'home' ? control.message || 'HOME 未完成，请查看执行详情。' : '操作未完成，请展开执行详情查看原因。')
          : control.phase === 'completed' ? '当前操作已确认完成，可继续下一步。' : '选择上方操作开始。'}</div>
        {busy && ['home', 'position', 'workpos', 'workpos_direct'].includes(control.operation ?? '') && active?.robot_id === 'nero' && <>
          <div style={hint}>{control.message}</div>
          <div style={row}>{driverStatusRows.filter(([, status]) => status.position_kind === (control.operation === 'home' ? 'home' : 'workpos')).map(([side, status]) =>
            <span key={side} style={badge}>{side === 'left' ? '左臂' : '右臂'}：第 {status.position_stage}/{status.position_stage_count} 段 · {status.position_status === 'stage_arrived' || status.position_status === 'arrived' ? '已到位' : '等待到位'}</span>
          )}</div>
        </>}
        <details><summary style={summary}>查看执行详情</summary>
          {control.message && <div style={{ ...hint, overflowWrap: 'anywhere' }}>{control.message}</div>}
          {control.components && <div style={row}>{Object.entries(control.components).map(([side, part]) => <span key={side} style={badge}>
            {side === 'left' ? '左侧' : side === 'right' ? '右侧' : side}：{phases[part.result || ''] || part.result || '等待中'}
            {part.message && <span style={hint}> · {part.message}</span>}
          </span>)}</div>}
          {control.operation_id && <div style={hint}>操作 ID：{control.operation_id}</div>}
        </details>
      </div>}
      <div style={hint}>3 · 顶部开始遥操 → 在「监控与采集」中启动采集节点并录制。状态显示完成后再执行下一步。</div>
    </section>

    <details style={card}>
      <summary style={summary}>高级选项与维护</summary>
      <div style={hint}>日常操作使用上方流程；以下为独立维护操作，仅展示当前机器人支持的能力。</div>
      <div style={row}>
        {allows('enable') && <button style={button('#475569')} disabled={!available('enable')} onClick={() => {
          if (confirm('单独使能当前机器人控制？这会使电机上电保持。')) void run(() => api.robotEnable(), '使能请求已提交')
        }}>单独使能控制</button>}
        {allows('initialize_can') && <button style={button('#475569')} disabled={!available('initialize_can')} onClick={() => {
          if (confirm('重新初始化 CAN 连接会发送电机使能，并暂停遥操。请确认设备停止、允许切换控制权。继续？')) void run(() => api.robotInitializeCan(), '连接初始化请求已提交')
        }}>重新初始化连接（含电机使能）</button>}
        {allows('damping') && <button style={button('#b45309')} disabled={!available('damping')} onClick={() => {
          if (confirm('阻尼释放会暂停遥操，允许手动拖动机械臂。确认设备安全？')) void run(() => api.robotDamping(), '阻尼释放请求已提交')
        }}>阻尼释放</button>}
        {allows('ready') && <button style={button('#475569')} disabled={!available('ready')} onClick={() => {
          if (confirm('调用厂商就绪流程，部分机器人可能使能并归零。确认设备及路径安全？')) void run(() => api.robotReady(), '厂商就绪请求已提交')
        }}>厂商就绪</button>}
        {allows('position') && !allows('workpos') && <button style={button('#475569')} disabled={!available('position')} onClick={() => {
          if (confirm('执行当前机器人位置模式操作，可能发生运动。继续？')) void run(() => api.robotPosition(), '位置模式请求已提交')
        }}>位置模式</button>}
      </div>
      <details><summary style={summary}>配置详情 / 仅应用配置</summary>
        <div style={hint}>{chosen?.robot_id} · 状态 {chosen?.state_dim} 维 / 动作 {chosen?.action_dim} 维
          {chosen?.hands.map((hand) => ` · ${hand.side} ${hand.type}`)}</div>
        <div style={hint}>Profile：{profileId} · 运动学版本：{chosen?.kinematics_hash?.slice(0, 12) || '未解析'}。仅应用配置不会启动节点。</div>
        <button style={button('#475569')} disabled={!profileId || locked} onClick={() => void run(() => api.selectProfile(profileId, mode), '配置已应用，节点尚未启动')}>仅应用配置（供外部启动）</button>
        {control?.operation_id && <div style={hint}>操作 ID：{control.operation_id}</div>}
      </details>
      <details><summary style={summary}>旧版启动预设（兼容）</summary>
        <div style={hint}>用于已有的定制 launch；不要与上方机器人启动混用。数采节点在「监控与采集」中单独管理。</div>
        <div style={row}>
          <select aria-label="兼容启动预设" style={select} value={selectedPreset} disabled={locked} onChange={(e) => setPresetName(e.target.value)}>
            {legacy.map((preset) => <option key={preset.name} value={preset.name}>{preset.name}</option>)}
          </select>
          <label style={hint}><input type="checkbox" checked={log} disabled={locked} onChange={(e) => setLog(e.target.checked)} />记录诊断日志</label>
          <button style={button('#475569')} disabled={!state || locked || !selectedPreset} onClick={() => {
            if (confirm('启动兼容预设？硬件行为由此预设定义，请确认配置正确及设备安全。')) void run(() => api.start(selectedPreset, log), '兼容预设启动请求已提交')
          }}>启动兼容预设</button>
        </div>
      </details>
    </details>

    <details style={card}><summary style={summary}>相机与画面回传</summary><VideoCard state={state} /></details>
    <details style={card}><summary style={summary}>诊断与启动日志</summary><LatencyPanel state={state} /><LogConsole state={state} /></details>
  </div>
}

const wrap: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: 16 }
const card: React.CSSProperties = { background: '#151e2c', border: '1px solid #2c394d', borderRadius: 12, padding: 20 }
const heading: React.CSSProperties = { fontSize: 16, fontWeight: 700, marginBottom: 16 }
const row: React.CSSProperties = { display: 'flex', gap: 12, flexWrap: 'wrap', alignItems: 'center', marginBottom: 12 }
const field: React.CSSProperties = { display: 'flex', gap: 6, flexDirection: 'column', fontSize: 12, color: '#94a3b8' }
const select: React.CSSProperties = { background: '#0f172a', color: '#e2e8f0', border: '1px solid #475569', borderRadius: 6, padding: '9px 12px', maxWidth: '100%', minWidth: 170 }
const hint: React.CSSProperties = { fontSize: 13, color: '#94a3b8', lineHeight: 1.7 }
const warning: React.CSSProperties = { ...hint, color: '#fbbf24' }
const summary: React.CSSProperties = { fontSize: 14, fontWeight: 600, cursor: 'pointer', padding: '8px 0', color: '#cbd5e1' }
const badge: React.CSSProperties = { fontSize: 12, background: '#1e293b', borderRadius: 6, padding: '6px 10px', overflowWrap: 'anywhere' }
const statusPanel: React.CSSProperties = { background: '#0d1522', borderRadius: 8, padding: 14, margin: '14px 0' }
const button = (color: string): React.CSSProperties => ({ background: color, color: '#fff', border: '1px solid #ffffff18', borderRadius: 7, padding: '10px 16px', fontSize: 13, fontWeight: 600, cursor: 'pointer' })
