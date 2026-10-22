import type { PianoSession } from '../pianoTypes'

interface RecoveryClient {
  state: () => Promise<PianoSession>
  close: (token: string) => Promise<unknown>
  open: (mode: 'hardware', revision: string, options: Record<string, string>) => Promise<{ token: string; session: PianoSession }>
  command: (token: string, operation: string, args: Record<string, unknown>) => Promise<{ id: string }>
}

interface RecoveryOptions {
  token: string
  sessionId: string
  revision: string
  devices: Record<string, string>
  onSession: (token: string | null, state?: PianoSession) => void
  cancelled: () => boolean
  sleep?: (ms: number) => Promise<void>
  now?: () => number
  timeoutMs?: number
}

/** Recreate a faulted controller session before reset, preserving the page's draft.
 * This function can only close/open/reset. It never enables or resumes motion.
 */
export async function recoverPianoReset(client: RecoveryClient, options: RecoveryOptions): Promise<PianoSession> {
  const sleep = options.sleep ?? (ms => new Promise(resolve => setTimeout(resolve, ms)))
  const now = options.now ?? Date.now
  const timeout = options.timeoutMs ?? 25000
  function checkCancelled() {
    if (options.cancelled()) throw new Error('急停复位恢复已取消；不会自动继续')
  }
  async function waitFor(read: (state: PianoSession) => boolean, message: string) {
    const deadline = now() + timeout
    while (now() < deadline) {
      checkCancelled()
      const state = await client.state()
      checkCancelled()
      if (read(state)) return state
      await sleep(200)
    }
    throw new Error(message)
  }

  checkCancelled()
  let state = await client.state()
  checkCancelled()
  if (!state.active || state.session_id !== options.sessionId || state.mode !== 'hardware') throw new Error('原真机会话已变化，请检查当前页面控制权')
  if (state.busy || state.closing) throw new Error('已有操作正在执行，请先停止并等待空闲')
  let token = options.token
  let id = options.sessionId

  if (state.state === 'FAULT' || state.motion_claimed === true) {
    await client.close(token)
    options.onSession(null)
    state = await waitFor(current => {
      if (current.active && current.session_id !== id) throw new Error('其他页面已建立会话，已取消恢复')
      return !current.active && !(current.blockers?.length)
    }, '旧会话或 ROS 发布者尚未释放，请查看占用提示后重试')
    checkCancelled()
    const opened = await client.open('hardware', options.revision, { ...options.devices })
    token = opened.token
    id = opened.session.session_id ?? ''
    if (options.cancelled()) {
      await client.close(token)
      throw new Error('急停复位恢复已取消；新会话已请求关闭')
    }
    options.onSession(token, opened.session)
    if (!id) throw new Error('新会话缺少标识，请断开后重试')
    state = await waitFor(current => {
      if (current.session_id !== id || !current.active) throw new Error('新会话已退出或被替换')
      if (current.state === 'FAULT') throw new Error(current.error || '新会话故障，未发送复位')
      return current.maintenance_allowed === true && !current.busy
    }, '新会话维护接口尚未就绪，未发送复位')
  }

  checkCancelled()
  if (state.maintenance_allowed !== true) throw new Error('当前状态不允许复位，请检查会话和驱动状态')
  const job = await client.command(token, 'reset', { calibration_confirmed: true })
  return waitFor(current => {
    if (current.session_id !== id || !current.active) throw new Error('复位期间会话已退出或被替换')
    const result = current.result
    if (result?.job_id !== job.id) return false
    if (!result.ok) throw new Error(result.error || '复位请求失败，请查看驱动日志')
    return true
  }, '复位请求未返回结果，请检查控制器状态；未自动恢复运动')
}
