/** Renew only the current owner; a stalled request must not block all retries. */
export function makePianoHeartbeat(options: {
  getToken: () => string | null
  send: (token: string, signal: AbortSignal) => Promise<unknown>
  recovered: () => void
  failed: (error: unknown) => void
  timeoutMs?: number
}) {
  let controller: AbortController | null = null
  let disposed = false
  async function beat() {
    const token = options.getToken()
    if (disposed || !token || controller) return
    const current = new AbortController()
    controller = current
    const timeout = setTimeout(() => current.abort(), options.timeoutMs ?? 4000)
    try {
      await options.send(token, current.signal)
      if (!disposed && token === options.getToken()) options.recovered()
    } catch (error) {
      if (!disposed && token === options.getToken()) options.failed(error)
    } finally {
      clearTimeout(timeout)
      if (controller === current) controller = null
    }
  }
  return { beat, dispose: () => { disposed = true; controller?.abort() } }
}
