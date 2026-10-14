import { useSyncExternalStore } from 'react'
import { dismissToast, getToasts, subscribeToasts, type Toast, type ToastKind } from '../hooks/useToast'

const COLORS: Record<ToastKind, string> = {
  info: '#3b82f6',
  success: '#22c55e',
  warn: '#f59e0b',
  error: '#ef4444',
}

export function ToastHost() {
  const toasts = useSyncExternalStore(subscribeToasts, getToasts, getToasts)
  return (
    <div style={hostStyle}>
      {toasts.map((t: Toast) => (
        <div key={t.id} style={toastStyle(COLORS[t.kind])} onClick={() => dismissToast(t.id)}>
          <span style={dotStyle(COLORS[t.kind])} />
          <span>{t.message}</span>
        </div>
      ))}
    </div>
  )
}

const hostStyle: React.CSSProperties = {
  position: 'fixed',
  top: '16px',
  right: '16px',
  display: 'flex',
  flexDirection: 'column',
  gap: '8px',
  zIndex: 1000,
  maxWidth: '420px',
}

function toastStyle(color: string): React.CSSProperties {
  return {
    display: 'flex',
    alignItems: 'center',
    gap: '10px',
    padding: '10px 14px',
    background: '#1f2937',
    border: `1px solid ${color}55`,
    borderLeft: `3px solid ${color}`,
    borderRadius: '8px',
    color: '#e5e7eb',
    fontSize: '13px',
    boxShadow: '0 6px 20px rgba(0,0,0,0.35)',
    cursor: 'pointer',
  }
}

function dotStyle(color: string): React.CSSProperties {
  return { width: '8px', height: '8px', borderRadius: '50%', background: color, flexShrink: 0 }
}
