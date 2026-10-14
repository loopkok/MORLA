// Module-level singleton WebSocket hook (one connection per app).
// Uses useSyncExternalStore so 30Hz updates don't trigger deep re-renders.
import { useSyncExternalStore } from 'react'
import type { UiState } from '../types'
import { mapUiState, type NormalisedState } from '../lib/mapUiState'

let ws: WebSocket | null = null
let state: NormalisedState | null = null
let connected = false
const listeners = new Set<() => void>()
const connListeners = new Set<() => void>()

function emit() {
  listeners.forEach((l) => l())
}
function emitConn() {
  connListeners.forEach((l) => l())
}

function setConnected(v: boolean) {
  if (connected !== v) {
    connected = v
    emitConn()
  }
}

function connect() {
  if (ws) return
  const proto = location.protocol === 'https:' ? 'wss' : 'ws'
  ws = new WebSocket(`${proto}://${location.host}/ws/telemetry`)
  ws.onopen = () => setConnected(true)
  ws.onmessage = (e) => {
    try {
      const msg = JSON.parse(e.data) as UiState
      if (msg.type === 'ui_state') {
        state = mapUiState(msg)
        emit()
      }
    } catch {
      /* ignore malformed frames */
    }
  }
  ws.onclose = () => {
    setConnected(false)
    ws = null
    setTimeout(connect, 3000)
  }
  ws.onerror = () => {
    ws?.close()
  }
}

function subscribe(cb: () => void) {
  if (!ws) connect()
  listeners.add(cb)
  return () => listeners.delete(cb)
}

function getSnapshot(): NormalisedState | null {
  return state
}

function subscribeConn(cb: () => void) {
  if (!ws) connect()
  connListeners.add(cb)
  return () => connListeners.delete(cb)
}
function getConnSnapshot(): boolean {
  return connected
}

export function useRealtime(): NormalisedState | null {
  return useSyncExternalStore(subscribe, getSnapshot, getSnapshot)
}

export function useWsConnected(): boolean {
  return useSyncExternalStore(subscribeConn, getConnSnapshot, getConnSnapshot)
}
