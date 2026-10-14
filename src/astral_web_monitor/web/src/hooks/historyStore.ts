// Ring-buffered time-series of telemetry samples for the chart panel.
// Keeps the last N samples (default 200 ≈ 6.7s @ 30Hz push). Each sample
// records the command-rate map and a snapshot of selected joint values.

export interface Sample {
  ts: number
  ratesHz: Record<string, number>
  joints: Record<string, number[]> // entity -> values
}

const MAX = 200
const samples: Sample[] = []
const listeners = new Set<() => void>()

function emit() {
  listeners.forEach((l) => l())
}

export function pushSample(s: Sample): void {
  samples.push(s)
  if (samples.length > MAX) samples.shift()
  emit()
}

export function getSamples(): Sample[] {
  return samples
}

export function subscribeHistory(cb: () => void): () => void {
  listeners.add(cb)
  return () => listeners.delete(cb)
}

export function clearHistory(): void {
  samples.length = 0
  emit()
}
