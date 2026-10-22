import type { NormalisedState } from '../lib/mapUiState'
import { api } from '../api/client'
import type { TeleopState } from '../types'
import { StatusBadge } from './StatusBadge'
import { pushToast } from '../hooks/useToast'

interface Props {
  state: NormalisedState | null
  onAction: () => void
}

export function ControlBar({ state, onAction }: Props) {
  const teleopState = (state?.teleopState ?? 'stopped') as TeleopState
  const control = state?.control
  const pianoActive = state?.piano?.active === true
  const live = !!control && !control.stale && !pianoActive
  const busy = live && !['idle', 'completed', 'failed', 'cancelled'].includes(control.phase)
  const needsPrepare = live && control.capabilities?.workpos_requires_reenable === true
    && ['workpos', 'position', 'home'].includes(control.operation || '') && control.phase === 'completed'
  const alive = live && !busy
  const startBlockers = live ? control.start_readiness?.blockers ?? [] : []
  const startBlocked = live && control.start_readiness?.ready === false
  const canPause = alive && teleopState === 'running'
  const canResume = alive && !needsPrepare && teleopState === 'paused'
  const canEstop = pianoActive || (live && control.operating_mode === 'real' && control.capabilities?.estop === true)

  async function run(fn: () => Promise<{ ok: boolean; message: string }>, okMsg?: string) {
    const res = await fn()
    if (res.ok) {
      pushToast(okMsg ?? res.message, 'success')
    } else {
      pushToast(res.message, 'error')
    }
    onAction()
  }

  function estop() {
    if (!confirm('执行当前机器人的急停？设备可能失去保持力，请确认安全。')) return
    void run(() => api.robotEstop(), '急停请求已提交，请检查设备与控制状态')
  }

  return (
    <div style={barStyle}>
      <StatusBadge state={teleopState} />
      {pianoActive && <span style={presetStyle}>钢琴会话占用机器人；急停仍可用</span>}
      {state?.preset && <span style={presetStyle}>运行配置: {state.preset}</span>}
      {state && state.uptimeS > 0 && (
        <span style={uptimeStyle}>运行 {formatUptime(state.uptimeS)}</span>
      )}

      <div style={spacer} />

      {/* Emergency stop remains available while another operation is busy. */}
      <button
        style={estopBtn}
        disabled={!canEstop}
        onClick={estop}
        title="按当前机器人能力请求急停；状态以设备实际反馈为准"
      >
        ⛔ 急停
      </button>
      <button style={btn('#f59e0b')} disabled={!alive || needsPrepare || startBlocked || teleopState !== 'running'} title={startBlockers.join('；') || (needsPrepare ? '位置动作完成，请先准备并使能恢复遥操' : '接管当前机器人的遥操控制')} onClick={() => run(() => api.teleopStart())}>
        开始遥操
      </button>
      {startBlocked && !busy && <details style={{ width: '100%', color: '#fbbf24', fontSize: '13px' }}>
        <summary>遥操尚未就绪：{startBlockers[0]}{startBlockers.length > 1 ? `（共 ${startBlockers.length} 项，展开查看）` : ''}</summary>
        {startBlockers.length > 1 && <ul>{startBlockers.map((reason) => <li key={reason}>{reason}</li>)}</ul>}
      </details>}
      <button style={btn('#3b82f6')} disabled={!canPause} onClick={() => run(() => api.pause())}>
        暂停
      </button>
      <button style={btn('#22c55e')} disabled={!canResume} onClick={() => run(() => api.resume())}>
        恢复
      </button>
    </div>
  )
}

function formatUptime(s: number): string {
  if (s < 60) return `${s.toFixed(0)}s`
  const m = Math.floor(s / 60)
  const sec = Math.floor(s % 60)
  if (m < 60) return `${m}m${sec}s`
  const h = Math.floor(m / 60)
  return `${h}h${m % 60}m`
}

const barStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: '12px',
  padding: '12px 16px',
  background: '#1f2937',
  borderBottom: '1px solid #374151',
  flexWrap: 'wrap',
}
const spacer: React.CSSProperties = { flex: 1 }
const presetStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '13px' }
const uptimeStyle: React.CSSProperties = { color: '#6b7280', fontSize: '13px' }
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
  padding: '7px 18px',
  fontSize: '14px',
  fontWeight: 800,
  cursor: 'pointer',
  letterSpacing: '0.5px',
}
