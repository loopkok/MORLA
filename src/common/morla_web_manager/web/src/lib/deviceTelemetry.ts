// Rates come from device frame counters, never browser polling or configured Hz.
export type DeviceFeedback = {
  ready?: boolean; fresh?: boolean; fault?: boolean; reason?: string; stop_error?: string
  sample_ns?: number; age_ns?: number; source?: string; measured?: boolean; synthetic?: boolean
  values?: number[]; device_status?: Record<string, unknown>; [key: string]: unknown
}
export type RateSample = { sequence: number; stamp: number; source: string }
export function feedbackRateSample(feedback?: DeviceFeedback): RateSample | undefined {
  const sequence = feedback?.device_status?.sequence, stamp = feedback?.sample_ns
  if (feedback?.source !== 'hardware' || feedback.measured !== true || feedback.synthetic === true || feedback.fresh !== true || feedback.fault === true) return
  if (typeof sequence !== 'number' || !Number.isSafeInteger(sequence) || sequence < 0 || typeof stamp !== 'number' || !Number.isFinite(stamp) || stamp <= 0) return
  return { sequence, stamp, source: feedback.source }
}
export function measuredFeedbackHz(previous?: RateSample, current?: RateSample): number | undefined {
  if (!previous || !current || previous.source !== current.source) return
  const frames = current.sequence - previous.sequence, elapsed = current.stamp - previous.stamp
  if (frames <= 0 || elapsed <= 0) return
  return frames * 1e9 / elapsed
}
export function feedbackState(feedback?: DeviceFeedback, unavailable = false): { label: string; tone: string } {
  if (unavailable) return { label: '状态连接异常', tone: 'warning' }
  if (!feedback) return { label: '等待反馈', tone: 'idle' }
  if (feedback.fault) return { label: '设备故障', tone: 'error' }
  if (feedback.fresh === false) return { label: '反馈过期', tone: 'warning' }
  if (feedback.ready && feedback.fresh) return { label: '反馈在线', tone: 'online' }
  return { label: '尚未就绪', tone: 'idle' }
}
export function solverDurationMs(report: Record<string, unknown>): number | undefined {
  const start = report.sample_ns, end = report.completed_ns
  return typeof start === 'number' && typeof end === 'number' && Number.isFinite(start) && Number.isFinite(end) && end >= start ? (end - start) / 1e6 : undefined
}
