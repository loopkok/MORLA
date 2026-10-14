import type { TeleopState } from '../types'

const COLORS: Record<TeleopState, string> = {
  stopped: '#6b7280',
  starting: '#f59e0b',
  running: '#22c55e',
  stopping: '#f59e0b',
  paused: '#3b82f6',
  start_failed: '#ef4444',
}

const LABELS: Record<TeleopState, string> = {
  stopped: '已停止',
  starting: '启动中',
  running: '运行中',
  stopping: '停止中',
  paused: '已暂停',
  start_failed: '启动失败',
}

export function StatusBadge({ state }: { state: TeleopState }) {
  const color = COLORS[state] ?? '#6b7280'
  const label = LABELS[state] ?? state
  return (
    <span style={badgeStyle(color)}>
      <span style={dotStyle(color)} />
      {label}
    </span>
  )
}

function badgeStyle(color: string): React.CSSProperties {
  return {
    display: 'inline-flex',
    alignItems: 'center',
    gap: '6px',
    padding: '4px 10px',
    borderRadius: '9999px',
    background: color + '22',
    color,
    fontSize: '13px',
    fontWeight: 600,
  }
}

function dotStyle(color: string): React.CSSProperties {
  return { width: '8px', height: '8px', borderRadius: '50%', background: color }
}
