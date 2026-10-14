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

export function SystemTab({ state, presets, onAction }: Props) {
  const [selected, setSelected] = useState('')
  const [profileCatalog, setProfileCatalog] = useState<ProfileCatalog | null>(null)
  const [profileId, setProfileId] = useState('')
  const [operatingMode, setOperatingMode] = useState<'sim' | 'real'>('sim')
  // 记录遥操诊断日志（VR/滤波/命令/计数 → /tmp/teleop_teleop_{left,right}.jsonl）。
  // 仅对 morla_bringup / astral_arm_teleop 预设生效（其余预设后端不注入）。
  const [log, setLog] = useState(false)
  // 数采预设走独立泳道（监控 tab 卡片上的节点启停），不占遥操主泳道。
  const teleopPresets = presets.filter((p) => p.package !== 'morla_data_collect')
  const eff = selected || teleopPresets[0]?.name || ''
  const teleopState = state?.teleopState ?? 'stopped'
  const canStart = teleopState === 'stopped' || teleopState === 'start_failed'
  const canStop = teleopState === 'running' || teleopState === 'paused' || teleopState === 'starting'
  const canRestart = teleopState === 'running' || teleopState === 'paused'
  const control = state?.control
  const chosenProfile = profileCatalog?.profiles.find((p) => p.profile_id === profileId)
  const controlAllows = (capability: string) =>
    !control || control.stale || control.capabilities?.[capability] !== false

  useEffect(() => {
    void api.profiles().then((res) => {
      if (!res.ok || !res.data) return
      setProfileCatalog(res.data)
      setProfileId(res.data.requested_profile_id || res.data.profile_id)
      setOperatingMode(res.data.operating_mode || 'sim')
    })
  }, [])

  useEffect(() => {
    if (!state?.profileSelection) return
    setProfileId(state.profileSelection.requested_profile_id || state.profileSelection.profile_id)
    setOperatingMode(state.profileSelection.operating_mode)
  }, [state?.profileSelection?.requested_profile_id, state?.profileSelection?.operating_mode])

  async function run(fn: () => Promise<{ ok: boolean; message: string }>, okMsg?: string) {
    const res = await fn()
    if (res.ok) pushToast(okMsg ?? res.message, 'success')
    else pushToast(res.message, 'error')
    onAction()
  }

  function stop() {
    if (!confirm('确认停止 launch？（SIGINT，30s 超时 SIGKILL）')) return
    void run(() => api.stop(), '已发送停止信号')
  }
  function restart() {
    if (!confirm('确认重启当前预设？（停止后重新启动）')) return
    void run(() => api.restart(), '重启中')
  }

  return (
    <div style={wrapStyle}>
      <div style={cardStyle}>
        <div style={titleStyle}>机器人 Profile 与运行模式</div>
        <div style={rowStyle}>
          <select value={profileId} onChange={(e) => setProfileId(e.target.value)} style={selectStyle}>
            {(profileCatalog?.profiles ?? []).map((p) => (
              <option key={p.profile_id} value={p.profile_id}>
                {p.title} · {p.profile_id} · state/action {p.state_dim}/{p.action_dim}
              </option>
            ))}
          </select>
          <select value={operatingMode} onChange={(e) => setOperatingMode(e.target.value as 'sim' | 'real')} style={selectStyle}>
            <option value="sim">仿真</option>
            <option value="real">真实硬件</option>
          </select>
          <button
            style={btn('#3b82f6')}
            disabled={!profileId}
            onClick={() => void run(() => api.selectProfile(profileId, operatingMode), `已选择 ${profileId} (${operatingMode})`)}
          >应用 Profile</button>
          <button
            style={btn('#22c55e')}
            disabled={!profileId || !chosenProfile?.launch_modes.includes(operatingMode)}
            title={!chosenProfile?.launch_modes.includes(operatingMode) ? '此 Profile/模式组合暂不支持网页直接启动；仍可用于匹配 CLI 数采/推理配置' : ''}
            onClick={() => {
              if (operatingMode === 'real' && !confirm(`启动真实硬件 Profile ${profileId}？启动不会自动使能或回零。`)) return
              void run(() => api.launchProfile(profileId, operatingMode), `启动 ${profileId} (${operatingMode})`)
            }}
          >启动 Profile</button>
        </div>
        {chosenProfile && (
          <div style={hintStyle}>
            当前选择 <b>{chosenProfile.robot_id}</b> · state/action {chosenProfile.state_dim}/{chosenProfile.action_dim} 维
            {chosenProfile.hands.map((h) => ` · ${h.side} ${h.type} ${h.dim} 维`)}。
            {chosenProfile.coordinate_contract && (
              <>
                <br />坐标契约 v{chosenProfile.coordinate_contract.version} · 输入 {chosenProfile.coordinate_contract.input_basis_id}
                {' · '}机器人参考系 {chosenProfile.coordinate_contract.robot_body_frame}
                {' · '}运动学 {chosenProfile.kinematics_hash?.slice(0, 12) || '未解析'}
                {Object.entries(chosenProfile.coordinate_contract.arms ?? {}).map(([side, arm]) => (
                  <span key={side}>
                    {' · '}{side}: {arm.solver_base_frame} → {arm.solver_tip_frame} → {arm.tool?.frame}
                    {' ('}{arm.tool?.calibration_status || '未标定'}{')'}
                  </span>
                ))}
              </>
            )}
            {' '}
            数据采集和策略泳道会沿用活动 coordinator 的 Profile；CLI 节点在线或任一泳道运行时，后端拒绝切换配置。
            {state?.profileSelection?.active_profile_id && ` 活动 Profile：${state.profileSelection.active_profile_id}。`}
          </div>
        )}
      </div>

      <div style={cardStyle}>
        <div style={titleStyle}>预设管理</div>
        <div style={rowStyle}>
          <select
            value={eff}
            onChange={(e) => setSelected(e.target.value)}
            disabled={!canStart}
            style={selectStyle}
          >
            {teleopPresets.map((p) => (
              <option key={p.name} value={p.name}>{p.name}</option>
            ))}
          </select>
          <label style={logStyle} title="启动遥操时把诊断数据（VR/滤波/命令/计数）写到 /tmp/teleop_teleop_{left,right}.jsonl（仅遥操预设）">
            <input type="checkbox" checked={log} onChange={(e) => setLog(e.target.checked)} />
            <span>记录遥操日志</span>
          </label>
          <button style={btn('#22c55e')} disabled={!canStart || !eff} onClick={() => run(() => api.start(eff, log), `启动 ${eff}`)}>
            启动
          </button>
          <button style={btn('#ef4444')} disabled={!canStop} onClick={stop}>
            停止
          </button>
          <button style={btn('#3b82f6')} disabled={!canRestart} onClick={restart}>
            重启
          </button>
          <button
            style={workposBtn}
            title="启动不再自动归位：让双臂沿 init_waypoints → init_pose 慢速走到初始工作位"
            disabled={!canRestart || !controlAllows('workpos')}
            onClick={() => {
              if (!confirm('确认移到工作位？双臂将从当前位姿经 init_waypoints 走到 init_pose（期间会 disarm，需重新开始遥操再操作）')) return
              void run(() => api.teleopWorkpos(), '工作位已下发')
            }}
          >
            工作位
          </button>
          <button
            style={homeBtn}
            title="先确保电机使能，再让双臂沿 init_pose → init_waypoints → 零位 慢速收回"
            disabled={!canRestart || !controlAllows('home')}
            onClick={() => {
              if (!confirm('确认 HOME？双臂将从当前位姿经 init_pose → init_waypoints 收回零位（期间会 disarm，需重新启动/一键就绪再遥操）')) return
              void run(() => api.teleopHome(), 'HOME 已下发')
            }}
          >
            HOME
          </button>
        </div>
        <div style={hintStyle}>
          非侵入：仅通过子进程管理 <code>ros2 launch</code>（启动/停止/重启），不直接控制硬件。
          CLI 启动的遥操无预设记录，不支持重启（请用停止+启动）。
        </div>
      </div>

      {control && (
        <div style={cardStyle}>
          <div style={titleStyle}>统一控制状态</div>
          <div style={{ ...rowStyle, color: control.stale ? '#f59e0b' : '#d1d5db' }}>
            <b>{control.profile_id}</b>
            <span>{control.stale ? '协调器状态已过期' : control.phase}</span>
            {control.operation && <span>操作：{control.operation}</span>}
            {control.operation_id && <code>{control.operation_id.slice(0, 12)}</code>}
          </div>
          {control.message && <div style={hintStyle}>{control.message}</div>}
          {control.components && Object.keys(control.components).length > 0 && (
            <div style={rowStyle}>
              {Object.entries(control.components).map(([side, component]) => (
                <span key={side} style={componentBadgeStyle}>
                  {side}: {component.result ?? 'pending'}
                  {component.message ? ` · ${component.message}` : ''}
                </span>
              ))}
            </div>
          )}
          <div style={hintStyle}>操作请求被接受后，最终完成或失败会在此处按实测状态更新。</div>
        </div>
      )}

      <div style={cardStyle}>
        <div style={titleStyle}>机器人模式（driver 服务）</div>
        <div style={rowStyle}>
          <button style={btn('#22c55e')} disabled={!controlAllows('ready')} onClick={() => run(() => api.robotReady(), '一键就绪 OK')}>
            一键就绪
          </button>
          <button style={btn('#3b82f6')} disabled={!controlAllows('home')} onClick={() => run(() => api.robotHome(), '全关节归零')}>
            归零
          </button>
          <button style={btn('#3b82f6')} disabled={!controlAllows('position')} onClick={() => run(() => api.robotPosition(), '位置保持')}>
            位置保持
          </button>
          <button style={btn('#f59e0b')} disabled={!controlAllows('damping')} onClick={() => { if (confirm('阻尼释放会先自动 disarm 遥操，之后可手动拖拽臂，确认？')) run(() => api.robotDamping(), '阻尼释放') }}>
            阻尼释放
          </button>
          <button style={estopBtn} disabled={!controlAllows('estop')} onClick={() => { if (confirm('确认急停断电？')) run(() => api.robotEstop(), '已断电') }}>
            ⛔ 急停(断电)
          </button>
        </div>
        <div style={hintStyle}>
          调用 <code>astral_robot_control</code> driver 已有的 Trigger 服务（~/ready · ~/home · ~/position · ~/damping · ~/estop）。
          每个按钮都会<b>先自动 disarm 遥操</b>（把臂节点带出 armed/homing 轨迹），再切驱动模式——避免陈旧命令流覆盖本次目标（如阻尼后归零/一键就绪无效）。
          典型流程：遥操中 → <b>阻尼释放</b>（自动 disarm，手动拖回 home）→ <b>位置保持</b>或<b>归零</b>。
          之后要用遥操需重新点<b>开始遥操</b>。急停=真断电（disable），臂失去保持力；恢复需重新<b>一键就绪</b>。
        </div>
      </div>

      <LatencyPanel state={state} />

      <VideoCard state={state} />

      <LogConsole state={state} />
    </div>
  )
}

const wrapStyle: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: '14px' }
const cardStyle: React.CSSProperties = {
  background: '#1f2937',
  border: '1px solid #374151',
  borderRadius: '10px',
  padding: '14px',
  display: 'flex',
  flexDirection: 'column',
  gap: '10px',
}
const titleStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 700, fontSize: '14px' }
const componentBadgeStyle: React.CSSProperties = {
  color: '#d1d5db', background: '#111827', borderRadius: '6px', padding: '5px 9px', fontSize: '12px',
}
const rowStyle: React.CSSProperties = { display: 'flex', gap: '8px', flexWrap: 'wrap', alignItems: 'center' }
const selectStyle: React.CSSProperties = {
  background: '#374151',
  color: '#e5e7eb',
  border: '1px solid #4b5563',
  borderRadius: '6px',
  padding: '6px 10px',
  fontSize: '13px',
  minWidth: '180px',
}
const logStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: '5px',
  color: '#9ca3af',
  fontSize: '12px',
  cursor: 'pointer',
}
const btn = (color: string): React.CSSProperties => ({
  background: color,
  color: 'white',
  border: 'none',
  borderRadius: '6px',
  padding: '7px 16px',
  fontSize: '13px',
  fontWeight: 600,
  cursor: 'pointer',
})
const estopBtn: React.CSSProperties = {
  background: '#dc2626',
  color: 'white',
  border: '2px solid #fca5a5',
  borderRadius: '6px',
  padding: '7px 16px',
  fontSize: '13px',
  fontWeight: 800,
  cursor: 'pointer',
}
const workposBtn: React.CSSProperties = {
  background: '#0ea5e9',
  color: 'white',
  border: 'none',
  borderRadius: '6px',
  padding: '7px 16px',
  fontSize: '13px',
  fontWeight: 600,
  cursor: 'pointer',
  marginLeft: '4px',
}
const homeBtn: React.CSSProperties = {
  background: '#7c3aed',
  color: 'white',
  border: 'none',
  borderRadius: '6px',
  padding: '7px 16px',
  fontSize: '13px',
  fontWeight: 600,
  cursor: 'pointer',
  marginLeft: '4px',
}
const hintStyle: React.CSSProperties = { color: '#6b7280', fontSize: '12px', lineHeight: 1.5 }
