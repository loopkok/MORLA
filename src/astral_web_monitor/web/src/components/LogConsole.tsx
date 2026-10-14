import { useEffect, useRef } from 'react'
import type { NormalisedState } from '../lib/mapUiState'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'

interface LaunchLogPanelProps {
  title: string
  lines: string[]
  /** Download uses the complete backend ring buffer; the live panel is only its tail. */
  loadAll: () => Promise<string[]>
  downloadPrefix: string
  height?: string
}

export function LaunchLogPanel({
  title,
  lines,
  loadAll,
  downloadPrefix,
  height,
}: LaunchLogPanelProps) {
  const ref = useRef<HTMLPreElement>(null)

  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight
  }, [lines.length])

  async function copy() {
    const text = lines.join('\n')
    if (!text) {
      pushToast('暂无日志', 'info')
      return
    }
    try {
      await navigator.clipboard.writeText(text)
      pushToast(`已复制 ${lines.length} 行`, 'success')
    } catch {
      pushToast('复制失败', 'error')
    }
  }

  async function download() {
    let all = lines
    try {
      all = await loadAll()
    } catch {
      pushToast('读取完整日志失败', 'error')
      return
    }
    const text = all.join('\n')
    if (!text.trim()) {
      pushToast('暂无日志', 'info')
      return
    }
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)
    const blob = new Blob([text], { type: 'text/plain;charset=utf-8' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `${downloadPrefix}-${stamp}.txt`
    a.click()
    URL.revokeObjectURL(url)
    pushToast(`已下载 ${all.length} 行`, 'success')
  }

  return (
    <div style={{ ...wrapStyle, height: height ?? wrapStyle.height }}>
      <div style={headerStyle}>
        <span>{title}</span>
        <span style={btnRow}>
          <button type="button" style={hdrBtn} onClick={() => void copy()}>复制</button>
          <button type="button" style={hdrBtn} onClick={() => void download()}>下载</button>
        </span>
      </div>
      <pre ref={ref} style={preStyle}>
        {lines.length === 0 ? '（等待启动...）' : lines.join('\n')}
      </pre>
    </div>
  )
}

export function LogConsole({ state }: { state: NormalisedState | null }) {
  const lines = state?.logTail ?? []
  return (
    <LaunchLogPanel
      title="Launch 日志"
      lines={lines}
      downloadPrefix="launch-logs"
      loadAll={async () => {
        const res = await api.logs()
        if (!res.ok || !res.data) return lines
        const parts: string[] = []
        if (res.data.teleop?.length) parts.push('# teleop launch', ...res.data.teleop)
        if (res.data.collect?.length) parts.push('', '# data collect', ...res.data.collect)
        if (res.data.infer?.length) parts.push('', '# policy inference', ...res.data.infer)
        return parts
      }}
    />
  )
}

const wrapStyle: React.CSSProperties = {
  background: '#111827',
  border: '1px solid #374151',
  borderRadius: '10px',
  display: 'flex',
  flexDirection: 'column',
  overflow: 'hidden',
  height: '70vh',
}
const headerStyle: React.CSSProperties = {
  padding: '8px 14px',
  color: '#9ca3af',
  fontSize: '13px',
  fontWeight: 600,
  borderBottom: '1px solid #374151',
  background: '#1f2937',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'space-between',
  gap: '8px',
}
const btnRow: React.CSSProperties = { display: 'flex', gap: '6px' }
const hdrBtn: React.CSSProperties = {
  background: '#374151',
  color: '#e5e7eb',
  border: '1px solid #4b5563',
  borderRadius: '6px',
  padding: '4px 10px',
  fontSize: '12px',
  fontWeight: 600,
  cursor: 'pointer',
}
const preStyle: React.CSSProperties = {
  flex: 1,
  margin: 0,
  padding: '10px 14px',
  color: '#d1d5db',
  fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
  fontSize: '12px',
  lineHeight: '1.5',
  overflow: 'auto',
  whiteSpace: 'pre-wrap',
  wordBreak: 'break-all',
}
