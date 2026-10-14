import type { NormalisedState } from '../lib/mapUiState'

interface Props {
  state: NormalisedState | null
}

interface Row {
  key: string
  label: string
  unit: string
  hint: string
}

const ROWS: Row[] = [
  { key: 'mocap_left', label: 'Mocap 左臂', unit: 'ms', hint: 'quest3 左腕数据包到达 → 发布的 stamp 时延' },
  { key: 'ik_left', label: 'IK 左臂', unit: 'ms', hint: '左臂 IK 求解耗时' },
  { key: 'e2e_left', label: 'E2E 左臂', unit: 'ms', hint: 'VR 时间戳 → 左臂 joint_commands 的端到端时延' },
  { key: 'mocap_right', label: 'Mocap 右臂', unit: 'ms', hint: 'quest3 右腕数据包到达 → 发布的 stamp 时延' },
  { key: 'ik_right', label: 'IK 右臂', unit: 'ms', hint: '右臂 IK 求解耗时' },
  { key: 'e2e_right', label: 'E2E 右臂', unit: 'ms', hint: 'VR 时间戳 → 右臂 joint_commands 的端到端时延' },
]

export function LatencyPanel({ state }: Props) {
  const stages = state?.latency?.stages ?? {}

  return (
    <div style={cardStyle}>
      <div style={headerStyle}>
        <span style={titleStyle}>管线延迟（实时）</span>
        <span style={subStyle}>启动后各环节延迟，每 {Math.round(1000 / 30)}ms 刷新</span>
      </div>
      <div style={gridStyle}>
        {ROWS.map((r) => {
          const s = stages[r.key]
          const value = s ? s.valueMs : 0
          const stale = !s || s.stale
          const failed = s && !s.ok
          const color = stale ? '#6b7280' : failed ? '#ef4444' : valueColor(value)
          return (
            <div key={r.key} style={cellStyle}>
              <div style={labelStyle}>{r.label}</div>
              <div style={{ ...valueStyle, color }}>
                {stale ? '—' : value.toFixed(1)}
                <span style={unitStyle}> {r.unit}</span>
              </div>
              {r.key.startsWith('mocap_') && s?.hz !== undefined && (
                <div style={hzStyle}>
                  {s.hz.toFixed(1)} Hz{s.expectedHz ? ` / 期望 ${s.expectedHz}` : ''}
                </div>
              )}
              {r.key.startsWith('ik_') && failed && (
                <div style={failStyle}>FAILED</div>
              )}
              <div style={hintStyle}>{r.hint}</div>
            </div>
          )
        })}
      </div>
    </div>
  )
}

function valueColor(ms: number): string {
  if (ms < 20) return '#22c55e'
  if (ms < 50) return '#eab308'
  if (ms < 100) return '#f59e0b'
  return '#ef4444'
}

const cardStyle: React.CSSProperties = {
  background: '#1f2937',
  border: '1px solid #374151',
  borderRadius: '10px',
  padding: '14px',
  display: 'flex',
  flexDirection: 'column',
  gap: '10px',
}
const headerStyle: React.CSSProperties = { display: 'flex', justifyContent: 'space-between', alignItems: 'baseline' }
const titleStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 700, fontSize: '14px' }
const subStyle: React.CSSProperties = { color: '#6b7280', fontSize: '11px' }
const gridStyle: React.CSSProperties = {
  display: 'grid',
  gridTemplateColumns: 'repeat(auto-fill, minmax(180px, 1fr))',
  gap: '10px',
}
const cellStyle: React.CSSProperties = {
  background: '#111827',
  border: '1px solid #374151',
  borderRadius: '8px',
  padding: '10px',
  display: 'flex',
  flexDirection: 'column',
  gap: '4px',
}
const labelStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '12px', fontWeight: 600 }
const valueStyle: React.CSSProperties = { fontSize: '22px', fontWeight: 700, fontVariantNumeric: 'tabular-nums' }
const unitStyle: React.CSSProperties = { fontSize: '12px', fontWeight: 400, color: '#9ca3af' }
const hzStyle: React.CSSProperties = { color: '#60a5fa', fontSize: '11px' }
const failStyle: React.CSSProperties = { color: '#ef4444', fontSize: '11px', fontWeight: 700 }
const hintStyle: React.CSSProperties = { color: '#4b5563', fontSize: '10px', lineHeight: 1.4 }
