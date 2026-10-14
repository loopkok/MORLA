import type { NormalisedJoint } from '../lib/mapUiState'

interface Props {
  title: string
  joint: NormalisedJoint | undefined
  rateHz: number | undefined
  unit?: string
}

export function JointPanel({ title, joint, rateHz, unit = 'rad' }: Props) {
  const values = joint?.values ?? []
  const stale = joint?.stale ?? true
  const age = joint?.ageS ?? -1

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>
        <span style={titleStyle}>{title}</span>
        <span style={rateStyle}>
          {rateHz !== undefined ? `${rateHz.toFixed(1)} Hz` : '—'}
        </span>
        {stale ? (
          <span style={staleBadge}>陈旧 {age >= 0 ? `${age.toFixed(1)}s` : ''}</span>
        ) : (
          <span style={okBadge}>实时</span>
        )}
      </div>
      <div style={valuesStyle}>
        {values.length === 0 ? (
          <span style={emptyStyle}>无数据</span>
        ) : (
          values.map((v, i) => (
            <div key={i} style={rowStyle}>
              <span style={idxStyle}>j{i}</span>
              <div style={barWrap}>
                <div
                  style={{
                    ...barFill,
                    width: `${Math.min(100, Math.abs(v) / 3.14 * 50 + 50)}%`,
                    background: stale ? '#6b7280' : '#3b82f6',
                  }}
                />
              </div>
              <span style={valStyle}>{v.toFixed(3)} {unit}</span>
            </div>
          ))
        )}
      </div>
    </div>
  )
}

const panelStyle: React.CSSProperties = {
  background: '#1f2937',
  border: '1px solid #374151',
  borderRadius: '10px',
  padding: '14px',
  display: 'flex',
  flexDirection: 'column',
  gap: '10px',
  minWidth: '260px',
}
const headerStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: '10px',
  borderBottom: '1px solid #374151',
  paddingBottom: '8px',
}
const titleStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 700, fontSize: '14px' }
const rateStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '12px', marginLeft: 'auto' }
const staleBadge: React.CSSProperties = {
  background: '#f59e0b22', color: '#f59e0b', fontSize: '11px',
  padding: '2px 8px', borderRadius: '9999px',
}
const okBadge: React.CSSProperties = {
  background: '#22c55e22', color: '#22c55e', fontSize: '11px',
  padding: '2px 8px', borderRadius: '9999px',
}
const valuesStyle: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: '4px' }
const emptyStyle: React.CSSProperties = { color: '#6b7280', fontSize: '13px', fontStyle: 'italic' }
const rowStyle: React.CSSProperties = {
  display: 'flex', alignItems: 'center', gap: '8px', fontSize: '12px',
}
const idxStyle: React.CSSProperties = { color: '#6b7280', width: '24px' }
const barWrap: React.CSSProperties = {
  flex: 1, height: '6px', background: '#111827', borderRadius: '3px', overflow: 'hidden',
}
const barFill: React.CSSProperties = { height: '100%', borderRadius: '3px', transition: 'width 0.1s' }
const valStyle: React.CSSProperties = { color: '#d1d5db', width: '90px', textAlign: 'right' }
