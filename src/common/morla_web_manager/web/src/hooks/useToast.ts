// Lightweight toast notification store (module-level singleton).
// No provider/context needed — call pushToast() from anywhere, render
// <ToastHost/> once at the app root.

export type ToastKind = 'info' | 'success' | 'warn' | 'error'

export interface Toast {
  id: number
  kind: ToastKind
  message: string
}

let nextId = 1
const toasts: Toast[] = []
const listeners = new Set<() => void>()

function emit() {
  listeners.forEach((l) => l())
}

export function pushToast(message: string, kind: ToastKind = 'info', ttlMs = 3500): void {
  const t: Toast = { id: nextId++, kind, message }
  toasts.push(t)
  emit()
  if (ttlMs > 0) {
    setTimeout(() => dismissToast(t.id), ttlMs)
  }
}

export function dismissToast(id: number): void {
  const i = toasts.findIndex((t) => t.id === id)
  if (i >= 0) {
    toasts.splice(i, 1)
    emit()
  }
}

export function getToasts(): Toast[] {
  return toasts
}

export function subscribeToasts(cb: () => void): () => void {
  listeners.add(cb)
  return () => listeners.delete(cb)
}
