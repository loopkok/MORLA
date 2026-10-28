import { useEffect, useState } from 'react'
import { useRealtime, useWsConnected } from './hooks/useRealtime'
import { usePresets } from './hooks/usePresets'
import { ControlBar } from './components/ControlBar'
import { Tabs } from './components/Tabs'
import { MonitorTab } from './components/MonitorTab'
import { HealthPanel } from './components/HealthPanel'
import { SystemTab } from './components/SystemTab'
import { DataProcessTab } from './components/DataProcessTab'
import { PianoTab } from './components/PianoTab'
import { ComposableTab } from './components/ComposableTab'
import { ToastHost } from './components/ToastHost'
import { ErrorBoundary } from './components/ErrorBoundary'
import { api } from './api/client'
import { pushSample } from './hooks/historyStore'

const TABS = [
  { id: 'compose', label: '组合配置' },
  { id: 'system', label: '机器人操作' },
  { id: 'monitor', label: '监控与采集' },
  { id: 'health', label: '健康' },
  { id: 'data', label: '数据处理' },
  { id: 'piano', label: '钢琴工作台' },
]

export default function App() {
  const state = useRealtime()
  const wsConnected = useWsConnected()
  const { presets } = usePresets()
  const [active, setActive] = useState('compose')
  const [pianoVisited, setPianoVisited] = useState(false)
  const [pianoActive, setPianoActive] = useState(false)
  useEffect(() => { if (active === 'piano') setPianoVisited(true) }, [active])

  // Feed the chart ring buffer on every telemetry frame.
  useEffect(() => {
    if (!state) return
    pushSample({
      ts: state.ts,
      ratesHz: state.ratesHz,
      joints: Object.fromEntries(
        Object.entries(state.joints).map(([k, v]) => [k, v.values]),
      ),
    })
  }, [state])

  return (
    <ErrorBoundary>
      <div style={appStyle}>
        <ToastHost />

        <header style={headerStyle}>
        <h1 style={titleStyle}>MORLA Robot Manager</h1>
        <span style={subStyle}>统一机器人操作、数据与策略监控</span>
        <div style={spacer} />
        <StatusPill ok={wsConnected} okLabel="WS" badLabel="WS 断" colorOk="#22c55e" />
        <StatusPill ok={!!state} okLabel="ROS" badLabel="ROS 断" colorOk="#3b82f6" />
        <StatusPill
          ok={state?.health?.overall === 'ok'}
          okLabel="数据正常"
          badLabel={state?.health?.overall ? `数据${state.health.overall === 'stale' ? '陈旧' : '偏慢'}` : '无数据'}
          colorOk="#22c55e"
          warn={state?.health?.overall === 'slow' || state?.health?.overall === 'stale'}
        />
      </header>

      {active !== 'compose' && <ControlBar state={state} onAction={() => void api.health()} />}

      <Tabs tabs={TABS} active={active} onChange={setActive} />
      {active !== 'piano' && pianoActive && <p role="status" style={{ color: '#fbbf24', padding: '0 16px' }}>钢琴会话仍保留，原机器人控制仍受互斥保护。<button onClick={() => setActive('piano')}>返回钢琴工作台停止或断开</button></p>}

      <main style={mainStyle}>
        {active === 'compose' && <ComposableTab />}
        {active === 'monitor' && <MonitorTab state={state} />}
        {active === 'health' && <HealthPanel state={state} />}
        {active === 'data' && <DataProcessTab state={state} connected={wsConnected} />}
        {(active === 'piano' || pianoVisited) && <div hidden={active !== 'piano'}><PianoTab onActiveChange={setPianoActive} /></div>}
        {active === 'system' && <SystemTab state={state} presets={presets} onAction={() => void api.health()} />}
      </main>
      </div>
    </ErrorBoundary>
  )
}

function StatusPill({
  ok,
  okLabel,
  badLabel,
  colorOk,
  warn,
}: {
  ok: boolean
  okLabel: string
  badLabel: string
  colorOk: string
  warn?: boolean
}) {
  const color = ok && !warn ? colorOk : warn ? '#f59e0b' : '#ef4444'
  return (
    <span style={pillStyle(color)}>
      <span style={dotStyle(color)} />
      {ok ? okLabel : badLabel}
    </span>
  )
}

const appStyle: React.CSSProperties = {
  minHeight: '100vh',
  background: '#0b0f17',
  color: '#e5e7eb',
  fontFamily: 'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif',
  display: 'flex',
  flexDirection: 'column',
}
const headerStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: '12px',
  padding: '14px 20px',
  borderBottom: '1px solid #1f2937',
  flexWrap: 'wrap',
}
const titleStyle: React.CSSProperties = { margin: 0, fontSize: '20px', fontWeight: 700 }
const subStyle: React.CSSProperties = { color: '#6b7280', fontSize: '13px' }
const spacer: React.CSSProperties = { flex: 1 }
const mainStyle: React.CSSProperties = {
  flex: 1,
  padding: '20px',
  display: 'flex',
  flexDirection: 'column',
  gap: '16px',
  maxWidth: '1400px',
  width: '100%',
  margin: '0 auto',
}
function pillStyle(color: string): React.CSSProperties {
  return {
    display: 'inline-flex',
    alignItems: 'center',
    gap: '6px',
    padding: '3px 10px',
    borderRadius: '9999px',
    background: color + '22',
    color,
    fontSize: '12px',
    fontWeight: 600,
  border: `1px solid ${color}44`,
  }
}
function dotStyle(color: string): React.CSSProperties {
  return { width: '7px', height: '7px', borderRadius: '50%', background: color }
}
