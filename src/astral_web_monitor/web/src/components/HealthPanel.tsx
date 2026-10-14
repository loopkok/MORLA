import type { NormalisedState } from '../lib/mapUiState'

const STATUS_COLOR: Record<string, string> = {
  ok: '#22c55e',
  slow: '#f59e0b',
  stale: '#ef4444',
  down: '#6b7280',
}
const STATUS_LABEL: Record<string, string> = {
  ok: '正常',
  slow: '偏慢',
  stale: '陈旧/断流',
  down: '离线',
}

const ENTITY_LABEL: Record<string, string> = {
  left_arm: '左臂',
  right_arm: '右臂',
  left_gripper: '左夹爪',
  right_hand: '右灵巧手',
  head: '头部',
  full_body: '全身',
}

export function HealthPanel({ state }: { state: NormalisedState | null }) {
  const health = state?.health
  const overall = health?.overall ?? 'down'
  const entities = health?.entities ?? {}

  return (
    <div style={wrapStyle}>
      <div style={overallStyle(STATUS_COLOR[overall] ?? '#6b7280')}>
        <span style={dotStyle(STATUS_COLOR[overall] ?? '#6b7280')} />
        总体: {STATUS_LABEL[overall] ?? overall}
      </div>

      <div style={gridStyle}>
        {Object.entries(entities).map(([k, v]) => {
          const color = STATUS_COLOR[v.status] ?? '#6b7280'
          return (
            <div key={k} style={cardStyle}>
              <div style={cardHeaderStyle}>
                <span style={dotStyle(color)} />
                <span style={nameStyle}>{ENTITY_LABEL[k] ?? k}</span>
                <span style={statusBadge(color)}>{STATUS_LABEL[v.status] ?? v.status}</span>
              </div>
              <div style={metricRowStyle}>
                <Metric label="状态流" value={`${v.stateHz.toFixed(1)} Hz`} />
                <Metric label="指令流" value={`${v.cmdHz.toFixed(1)} Hz`} />
                <Metric
                  label="期望"
                  value={v.expectedHz > 0 ? `${v.expectedHz.toFixed(0)} Hz` : '—'}
                />
                <Metric
                  label="数据龄期"
                  value={state?.joints[k]?.ageS != null && state.joints[k].ageS >= 0
                    ? `${state.joints[k].ageS!.toFixed(2)}s`
                    : '—'}
                  warn={state?.joints[k]?.stale}
                />
              </div>
            </div>
          )
        })}
        {Object.keys(entities).length === 0 && (
          <div style={emptyStyle}>（无健康数据，等待 ROS 节点就绪...）</div>
        )}
      </div>

      <div style={hintStyle}>
        非侵入：仅读取已有 joint_states/joint_commands 话题估算频率与新鲜度，不下发任何控制命令。
        期望频率可在 <code>ASTRAL_WEB_MONITOR_EXPECTED_HZ</code> 环境变量配置。
      </div>
    </div>
  )
}

function Metric({ label, value, warn }: { label: string; value: string; warn?: boolean }) {
  return (
    <div style={metricStyle}>
      <span style={metricLabelStyle}>{label}</span>
      <span style={metricValueStyle(warn)}>{value}</span>
    </div>
  )
}

const wrapStyle: React.CSSProperties = {
  display: 'flex',
  flexDirection: 'column',
  gap: '14px',
}
function overallStyle(color: string): React.CSSProperties {
  return {
    display: 'inline-flex',
    alignItems: 'center',
    gap: '8px',
    padding: '8px 14px',
    borderRadius: '9999px',
    background: color + '22',
    color,
    fontWeight: 700,
    fontSize: '14px',
    alignSelf: 'flex-start',
  }
}
const gridStyle: React.CSSProperties = {
  display: 'grid',
  gridTemplateColumns: 'repeat(auto-fit, minmax(260px, 1fr))',
  gap: '12px',
}
const cardStyle: React.CSSProperties = {
  background: '#1f2937',
  border: '1px solid #374151',
  borderRadius: '10px',
  padding: '12px',
  display: 'flex',
  flexDirection: 'column',
  gap: '10px',
}
const cardHeaderStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: '8px',
}
const nameStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 600, fontSize: '13px' }
function statusBadge(color: string): React.CSSProperties {
  return {
    marginLeft: 'auto',
    fontSize: '11px',
    padding: '2px 8px',
    borderRadius: '9999px',
    background: color + '22',
    color,
  }
}
const metricRowStyle: React.CSSProperties = {
  display: 'grid',
  gridTemplateColumns: 'repeat(2, 1fr)',
  gap: '6px 12px',
}
const metricStyle: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: '2px' }
const metricLabelStyle: React.CSSProperties = { color: '#6b7280', fontSize: '11px' }
function metricValueStyle(warn?: boolean): React.CSSProperties {
  return {
    color: warn ? '#f59e0b' : '#d1d5db',
    fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
    fontSize: '13px',
  }
}
function dotStyle(color: string): React.CSSProperties {
  return { width: '9px', height: '9px', borderRadius: '50%', background: color }
}
const emptyStyle: React.CSSProperties = { color: '#6b7280', fontStyle: 'italic', fontSize: '13px' }
const hintStyle: React.CSSProperties = { color: '#6b7280', fontSize: '12px', lineHeight: 1.5 }
