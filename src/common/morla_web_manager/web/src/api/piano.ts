import type { ApiEnvelope } from '../types'
import type { PianoConfig, PianoConfigRecord, PianoMode, PianoPlan, PianoScore, PianoSession } from '../pianoTypes'

const base = import.meta.env.VITE_API_BASE ?? ''
async function request<T>(path: string, body?: unknown, method = 'GET', keepalive = false, signal?: AbortSignal): Promise<T> {
  const response = await fetch(`${base}/api/v1/piano/${path}`, {
    method, headers: { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body), keepalive, signal,
  })
  const data = await response.json() as ApiEnvelope<T> & { detail?: unknown }
  if (!response.ok || data.ok === false) throw new Error(typeof data.detail === 'string' ? data.detail : data.message || `HTTP ${response.status}`)
  return data.data
}
export const piano = {
  state: () => request<PianoSession>('state'),
  config: () => request<PianoConfigRecord>('config'),
  validateConfig: (config: unknown) => request<PianoConfig>('config/validate', { config }, 'POST'),
  save: (config: PianoConfig, revision: string, certify: boolean) => request<PianoConfigRecord>('config', { config, revision, certify }, 'POST'),
  scores: () => request<PianoScore[]>('scores'),
  score: (id: string) => request<Record<string, unknown>>(`scores/${encodeURIComponent(id)}`),
  removeScore: (id: string) => request(`scores/${encodeURIComponent(id)}`, undefined, 'DELETE'),
  importScore: (name: string, content: string, format: string, tempo_bpm: number, tonic: string) =>
    request<{ id: string }>('scores', { name, content, format, tempo_bpm, tonic }, 'POST'),
  plan: (score_id: string, tempo_bpm: number) => request<PianoPlan>('plan', { score_id, tempo_bpm }, 'POST'),
  open: (mode: PianoMode, revision: string, options: Record<string, string>) => request<{ token: string; session: PianoSession }>('session', { mode, revision, options }, 'POST'),
  command: (token: string, operation: string, arguments_: Record<string, unknown> = {}) =>
    request<{ id: string }>('command', { token, operation, ...arguments_ }, 'POST'),
  heartbeat: (token: string, signal?: AbortSignal) => request('heartbeat', { token }, 'POST', false, signal),
  close: (token: string) => request('close', { token }, 'POST', true),
  stop: (emergency = false) => request('stop', { emergency }, 'POST'),
}
