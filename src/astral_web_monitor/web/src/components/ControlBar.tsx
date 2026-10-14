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
  const canPause = teleopState === 'running'
  const canResume = teleopState === 'paused'
  // E-STOP + 开始遥操 are unconditional on the backend (arm node is the
  // authority). Enable whenever the monitor is alive.
  const alive = state != null

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
    if (!confirm('确认急停？将下发 driver ~/estop → 断电（真急停，臂失去保持力）。')) return
    void run(() => api.robotEstop(), '已断电 (e_stop)')
  }
  function damping() {
    if (!confirm('确认阻尼释放？会先自动 disarm 遥操，切换运动模式=阻尼，可手动拖拽臂回 home（电机仍上电）。')) return
    void run(() => api.robotDamping(), '阻尼释放 (可手动拖拽)')
  }

  return (
    <div style={barStyle}>
      <StatusBadge state={teleopState} />
      {state?.preset && <span style={presetStyle}>预设: {state.preset}</span>}
      {state && state.uptimeS > 0 && (
        <span style={uptimeStyle}>运行 {formatUptime(state.uptimeS)}</span>
      )}

      <div style={spacer} />

      {/* E-STOP: real hardware power-off via driver ~/estop. */}
      <button
        style={estopBtn}
        disabled={!alive}
        onClick={estop}
        title="真急停：调 driver ~/estop → SDK disable() 断电，臂失去保持力"
      >
        ⛔ 急停
      </button>
      <button
        style={btn('#f59e0b')}
        disabled={!alive}
        onClick={damping}
        title="阻尼释放：调 driver ~/damping → motion_mode=0，可手动拖拽臂"
      >
        阻尼释放
      </button>
      <button style={btn('#f59e0b')} disabled={!alive} onClick={() => run(() => api.teleopStart())}>
        开始遥操
      </button>
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
