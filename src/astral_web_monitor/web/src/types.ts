// Shared types for the ui_state WebSocket frame and REST envelopes.

export interface JointSlot {
  values: number[]
  ts: number
  stale: boolean
}

export type HealthStatus = 'ok' | 'slow' | 'stale' | 'down'

export interface EntityHealth {
  stale: boolean
  state_hz: number
  cmd_hz: number
  expected_hz: number
  slow: boolean
  status: HealthStatus
}

export interface HealthSummary {
  overall: HealthStatus
  entities: Record<string, EntityHealth>
}

export interface LatencyStage {
  value_ms: number
  stale: boolean
  ok: boolean
  hz?: number
  expected_hz?: number
}

export interface LatencySummary {
  stages: Record<string, LatencyStage>
}

export interface VideoGateState {
  push_enabled: boolean
  configured: string[]
  active: string[]
}

// 数采节点泳道（独立于遥操预设的 LaunchManager）状态。
export interface CollectLaunchInfo {
  state: TeleopState
  preset: string
  uptime_s: number
  pid: number | null
  log_tail?: string[]
}

// astral_data_collect 的 /data_collect/state latched JSON 镜像。
// stale 由 monitor 后端加注：latched 消息在节点死后仍残留，靠龄期判活。
export interface DataCollectState {
  state: 'IDLE' | 'RECORDING' | 'PAUSED' | 'SAVING'
  session: string
  episode_index: number
  elapsed_s: number
  task_next: string
  samples_per_s: Record<string, number>
  dropped: Record<string, number>
  // 录制期参考相机实率低于 dataset_fps 一半时的告警文案（无告警为 null）
  low_fps_warning?: string | null
  // 录制启动 ~2s 数值/图像全 0（源未就绪空录）的告警文案（无告警为 null）
  empty_warning?: string | null
  // 状态不合法被忽略的控制指令计数（如 SAVING 期按 start；键盘/VR 无
  // disabled 视觉，靠它留痕）
  ignored?: Record<string, number>
  // monitor 后端加注：/data_collect/state 发布者数量（>1 = 有残留/双开节点）
  node_count?: number
  schema?: Record<string, unknown>
  // 本次采集数据（实时下拉窗口）——由采集节点 1Hz 发布：
  // folder=完整目录路径；episode_counts/camera_counts=当前段已写样本/帧数（精确）；
  // episode_bytes/session_bytes=磁盘占用（含 HDF5 chunk 预分配）
  folder?: string
  episode_counts?: Record<string, number>
  camera_counts?: Record<string, number>
  episode_bytes?: number
  session_bytes?: number
  stale?: boolean
}

export interface UiState {
  type: 'ui_state'
  ts: number
  teleop: {
    state: TeleopState
    preset: string
    uptime_s: number
    pid: number | null
  }
  joints: Record<string, JointSlot>
  rates_hz: Record<string, number>
  state_rates_hz?: Record<string, number>
  health?: HealthSummary
  latency?: LatencySummary
  video_gate?: VideoGateState | null
  data_collect?: DataCollectState | null
  data_process?: DataProcessState
  infer?: InferState | null
  collect_launch?: CollectLaunchInfo
  infer_launch?: InferLaunchInfo
  log_tail: string[]
}

export type TeleopState =
  | 'stopped'
  | 'starting'
  | 'running'
  | 'stopping'
  | 'paused'
  | 'start_failed'

export interface Preset {
  name: string
  package: string
  launch: string
  args: Record<string, string>
  description: string
}

export interface ApiEnvelope<T = unknown> {
  ok: boolean
  message: string
  data: T
}

export interface HealthData {
  ros_ok: boolean
  launch_state: TeleopState
  launch_pid: number | null
  uptime_s: number
  ws_clients: number
  health: HealthSummary
}

export interface VideoCameraInfo {
  label: string
  device: string
  source: string
  preset: string
  exists: boolean
  sysfs_name: string
}

export interface VideoStatusData {
  configured: VideoCameraInfo[]
  gate: VideoGateState | null
  online: boolean
}

// astral_policy_inference 的 /policy_inference/state latched JSON 镜像。
// stale 由 monitor 后端加注（latched 消息在节点死后残留，靠龄期判活）。
export interface DequeStats {
  avg?: number
  p50?: number
  p95?: number
  max?: number
}

export interface InferState {
  state: string // IDLE | POLICY | PLAYBACK | HUMAN (+ _PAUSED)
  activity?: string
  paused?: boolean
  prompt?: string
  state_dim?: number
  engine?: Record<string, number> // plans/pops/last_plan_ms/remaining
  playback?: { idx: number; frames: number; remaining: number } | null
  cam_frames?: Record<string, number>
  latency_ms?: { loop?: DequeStats; obs_age?: DequeStats }
  exec_events?: string[]
  error?: string | null
  // monitor 后端加注：/policy_inference/state 发布者数量（>1 = web/CLI 双开推理节点）
  node_count?: number
  stale?: boolean
}

export interface InferLaunchInfo {
  state: TeleopState
  preset: string
  uptime_s: number
  pid: number | null
  log_tail?: string[]
}

// 推理节点泳道启动配置（→ /api/v1/infer/launch/start，后端转 launch 参数）
export interface InferLaunchConfig {
  backend_type: string
  model: string
  host: string
  port: number
  camera_image_size: number
  engine_mode: string
  log: boolean
}

export interface LaunchLogsData {
  teleop: string[]
  collect: string[]
  infer: string[]
}

export type DataProcessAction = 'full' | 'raw-check' | 'quantify' | 'align' | 'validate' | 'repair-preview' | 'repair' | 'verify' | 'export' | 'preview' | 'upgrade-v3'
export interface DataProcessRequest {
  action: DataProcessAction
  source: string
  target: 'openpi' | 'act' | 'both'
  image_size: number
  camera_map: string
  episodes: string
  quantify: boolean
  repair: boolean
  reference: string
}
export interface DataProcessStep {
  name: string
  status: string
  returncode?: number
  started_at?: number
  finished_at?: number
}
export interface DataArtifact { path: string; name: string; kind: 'preview' | 'report' }
export interface DataProcessJob {
  id: string
  status: string
  action: DataProcessAction
  source: string
  directory: string
  started_at: number
  finished_at: number | null
  steps: DataProcessStep[]
  artifacts: DataArtifact[]
  error: string
  deep_check: boolean
  log_tail?: string[]
  validation?: { stage: string; total: number; pass: number; warn: number; fail: number }[]
}
export interface DataProcessState {
  busy: boolean
  current: DataProcessJob | null
  history: { id: string; status: string; action: DataProcessAction; source: string; started_at: number }[]
}
export interface DataSession {
  path: string
  name: string
  episodes: number
  kind: 'session' | 'dataset'
  cameras: string[]
  task: string
}
export interface DataSessionsResponse {
  data_root: string
  python_available: boolean
  deep_check_available: boolean
  sessions: DataSession[]
}
