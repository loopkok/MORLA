import { useSyncExternalStore } from 'react'
import { getSamples, subscribeHistory, type Sample } from '../hooks/historyStore'

// SVG line chart for a single numeric series over the sample ring buffer.
// Hand-rolled (no chart lib) to keep deps minimal, mirroring rob_station's
// FrameDebug SVG polylines.

interface Props {
  title: string
  series: { label: string; color: string; values: number[] }[]
  yMin?: number
  yMax?: number
  unit?: string
  height?: number
}

export function ChartPanel({ title, series, yMin, yMax, unit = '', height = 120 }: Props) {
  const samples = useSyncExternalStore(subscribeHistory, getSamples, getSamples)
  const W = 600
  const H = height
  const padL = 36
  const padR = 8
  const padT = 10
  const padB = 18
  const plotW = W - padL - padR
  const plotH = H - padT - padB

  const allVals = series.flatMap((s) => s.values)
  const lo = yMin ?? (allVals.length ? Math.min(...allVals) : 0)
  const hi = yMax ?? (allVals.length ? Math.max(...allVals) : 1)
  const span = hi - lo || 1
  const n = samples.length

  function toPath(values: number[]): string {
    if (n < 2) return ''
    return values
      .map((v, i) => {
        const x = padL + (i / (n - 1)) * plotW
        const y = padT + plotH - ((v - lo) / span) * plotH
        return `${i === 0 ? 'M' : 'L'}${x.toFixed(1)},${y.toFixed(1)}`
      })
      .join(' ')
  }

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>
        <span style={titleStyle}>{title}</span>
        <div style={legendStyle}>
          {series.map((s) => (
            <span key={s.label} style={legendItem}>
              <span style={dotStyle(s.color)} />
              {s.label}
            </span>
          ))}
        </div>
      </div>
      <svg width="100%" viewBox={`0 0 ${W} ${H}`} preserveAspectRatio="none" style={svgStyle}>
        {/* y-axis labels */}
        <text x={4} y={padT + 8} style={axisTextStyle}>{hi.toFixed(0)}{unit}</text>
        <text x={4} y={padT + plotH} style={axisTextStyle}>{lo.toFixed(0)}{unit}</text>
        {/* grid baseline */}
        <line x1={padL} y1={padT + plotH} x2={W - padR} y2={padT + plotH} stroke="#374151" strokeWidth={1} />
        <line x1={padL} y1={padT} x2={padL} y2={padT + plotH} stroke="#374151" strokeWidth={1} />
        {series.map((s) => (
          <path key={s.label} d={toPath(s.values)} fill="none" stroke={s.color} strokeWidth={1.5} />
        ))}
      </svg>
      <div style={footStyle}>{n < 2 ? '（等待数据...）' : `${n} 样本 · 最新 ${samples[n - 1].ts.toFixed(1)}`}</div>
    </div>
  )
}

export function samplesToSeries(
  samples: Sample[],
  pick: (s: Sample) => number,
): number[] {
  return samples.map(pick)
}

const panelStyle: React.CSSProperties = {
  background: '#1f2937',
  border: '1px solid #374151',
  borderRadius: '10px',
  padding: '12px',
  display: 'flex',
  flexDirection: 'column',
  gap: '8px',
}
const headerStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: '12px',
  flexWrap: 'wrap',
}
const titleStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 700, fontSize: '14px' }
const legendStyle: React.CSSProperties = { display: 'flex', gap: '12px', marginLeft: 'auto', flexWrap: 'wrap' }
const legendItem: React.CSSProperties = { display: 'inline-flex', alignItems: 'center', gap: '5px', color: '#9ca3af', fontSize: '12px' }
function dotStyle(color: string): React.CSSProperties {
  return { width: '8px', height: '8px', borderRadius: '50%', background: color }
}
const svgStyle: React.CSSProperties = { display: 'block', width: '100%' }
const axisTextStyle: React.CSSProperties = { fill: '#6b7280', fontSize: 10, fontFamily: 'ui-monospace, monospace' }
const footStyle: React.CSSProperties = { color: '#6b7280', fontSize: '11px' }
