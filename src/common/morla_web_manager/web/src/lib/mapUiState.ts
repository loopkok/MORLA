// Normalise the raw ui_state frame into a camelCase view model.
import type { CollectLaunchInfo, ControlOperationState, DataCollectState, DataProcessState, InferLaunchInfo, InferState, ProfileStreamValue, UiState } from '../types'

export interface NormalisedJoint {
  values: number[]
  stale: boolean
  ageS: number
}

export interface NormalisedState {
  ts: number
  teleopState: string
  preset: string
  uptimeS: number
  pid: number | null
  joints: Record<string, NormalisedJoint>
  ratesHz: Record<string, number>
  stateRatesHz: Record<string, number>
  health: {
    overall: string
    entities: Record<string, {
      stale: boolean
      stateHz: number
      cmdHz: number
      expectedHz: number
      slow: boolean
      status: string
    }>
  }
  latency: {
    stages: Record<string, {
      valueMs: number
      stale: boolean
      ok: boolean
      hz?: number
      expectedHz?: number
    }>
  }
  videoGate: { pushEnabled: boolean; configured: string[]; active: string[] } | null
  dataCollect: DataCollectState | null
  dataProcess: DataProcessState | null
  infer: InferState | null
  control: ControlOperationState | null
  profileId: string | null
  robotStreams: Record<string, ProfileStreamValue>
  profileSelection: UiState['profile_selection'] | null
  collectLaunch: CollectLaunchInfo | null
  inferLaunch: InferLaunchInfo | null
  logTail: string[]
}

export function mapUiState(msg: UiState, now: number = Date.now() / 1000): NormalisedState {
  const joints: Record<string, NormalisedJoint> = {}
  for (const [k, v] of Object.entries(msg.joints)) {
    joints[k] = {
      values: v.values,
      stale: v.stale,
      ageS: v.ts ? Math.max(0, now - v.ts) : -1,
    }
  }
  const rawHealth = msg.health ?? { overall: 'ok', entities: {} }
  const entities: NormalisedState['health']['entities'] = {}
  for (const [k, v] of Object.entries(rawHealth.entities)) {
    entities[k] = {
      stale: v.stale,
      stateHz: v.state_hz,
      cmdHz: v.cmd_hz,
      expectedHz: v.expected_hz,
      slow: v.slow,
      status: v.status,
    }
  }
  const rawLatency = msg.latency ?? { stages: {} }
  const latencyStages: NormalisedState['latency']['stages'] = {}
  for (const [k, v] of Object.entries(rawLatency.stages)) {
    latencyStages[k] = {
      valueMs: v.value_ms,
      stale: v.stale,
      ok: v.ok,
      hz: v.hz,
      expectedHz: v.expected_hz,
    }
  }
  return {
    ts: msg.ts,
    teleopState: msg.teleop.state,
    preset: msg.teleop.preset,
    uptimeS: msg.teleop.uptime_s,
    pid: msg.teleop.pid,
    joints,
    ratesHz: msg.rates_hz,
    stateRatesHz: msg.state_rates_hz ?? {},
    health: { overall: rawHealth.overall, entities },
    latency: { stages: latencyStages },
    videoGate: msg.video_gate
      ? {
          pushEnabled: msg.video_gate.push_enabled,
          configured: msg.video_gate.configured ?? [],
          active: msg.video_gate.active ?? [],
        }
      : null,
    dataCollect: msg.data_collect ?? null,
    dataProcess: msg.data_process ?? null,
    infer: msg.infer ?? null,
    control: msg.control ?? null,
    profileId: msg.profile_id ?? null,
    robotStreams: msg.robot_streams ?? {},
    profileSelection: msg.profile_selection ?? null,
    collectLaunch: msg.collect_launch ?? null,
    inferLaunch: msg.infer_launch ?? null,
    logTail: msg.log_tail,
  }
}
