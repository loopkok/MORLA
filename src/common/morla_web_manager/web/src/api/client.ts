// REST client mirroring rob_station's api/client.js convention.
import type { ApiEnvelope, DataProcessJob, DataProcessRequest, DataProcessState, DataSessionsResponse, HealthData, InferLaunchConfig, LaunchLogsData, Preset, ProfileCatalog, ReplayBrowserData, VideoStatusData } from '../types'

const base = import.meta.env.VITE_API_BASE ?? ''

async function post<T>(path: string, body?: unknown): Promise<ApiEnvelope<T>> {
  const res = await fetch(base + path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  // HTTP errors (409/503) arrive as {detail}; normalise into the envelope shape
  // so callers can always read .ok / .message.
  if (!res.ok) {
    let detail = `HTTP ${res.status}`
    try {
      const j = await res.json()
      detail = j.detail ?? detail
    } catch {
      /* keep status */
    }
    return { ok: false, message: detail, data: null as unknown as T }
  }
  return res.json()
}

async function get<T>(path: string): Promise<ApiEnvelope<T>> {
  const res = await fetch(base + path)
  return res.json()
}

export const api = {
  health: () => get<HealthData>('/api/v1/health'),
  presets: () => get<Preset[]>('/api/v1/presets'),
  profiles: () => get<ProfileCatalog>('/api/v1/profiles'),
  selectProfile: (profile_id: string, operating_mode: 'sim' | 'real') =>
    post<unknown>('/api/v1/profile/select', { profile_id, operating_mode }),
  launchProfile: (profile_id: string, operating_mode: 'sim' | 'real') =>
    post<unknown>('/api/v1/profile/launch', { profile_id, operating_mode }),
  state: () => get<unknown>('/api/v1/state'),
  start: (preset: string, log = false) => post<unknown>('/api/v1/start', { preset, log }),
  stop: () => post<unknown>('/api/v1/stop'),
  pause: () => post<unknown>('/api/v1/pause'),
  resume: () => post<unknown>('/api/v1/resume'),
  teleopStart: () => post<unknown>('/api/v1/teleop/start'),
  // HOME / park-to-zero：双臂沿 init_pose → init_waypoints → 零位 收回（先使能电机）
  teleopHome: () => post<unknown>('/api/v1/teleop/home'),
  // 工作位 / go-to-init：双臂沿 init_waypoints → init_pose 走到初始工作位
  // （启动自动归位已关闭，回工作位靠此按钮手动触发；途经点路径）
  teleopWorkpos: () => post<unknown>('/api/v1/teleop/workpos'),
  // 段间回位 / go-to-init direct：双臂**直接**（不经 init_waypoints）回到
  // init_pose——与左 X 同功能，数采段间快速回工作位（直达路径）
  teleopWorkposDirect: () => post<unknown>('/api/v1/teleop/workpos/direct'),
  restart: () => post<unknown>('/api/v1/restart'),
  // Robot hardware mode (driver services — real hardware actions)
  robotPrepare: () => post<unknown>('/api/v1/control/prepare'),
  robotReady: () => post<unknown>('/api/v1/robot/ready'),
  robotEnable: () => post<unknown>('/api/v1/control/enable'),
  robotReset: () => post<unknown>('/api/v1/control/reset'),
  robotInitializeCan: () => post<unknown>('/api/v1/control/initialize_can'),
  robotHome: () => post<unknown>('/api/v1/robot/home'),
  robotEstop: () => post<unknown>('/api/v1/robot/estop'),
  robotDamping: () => post<unknown>('/api/v1/robot/damping'),
  robotPosition: () => post<unknown>('/api/v1/robot/position'),
  // Video return gate (quest3_video_streamer — streamer owns the cameras)
  videoStatus: () => get<VideoStatusData>('/api/v1/video/status'),
  videoPush: (enabled: boolean) => post<unknown>('/api/v1/video/push', { enabled }),
  videoCameras: (cameras: string[]) => post<unknown>('/api/v1/video/cameras', { cameras }),
  // MJPEG live preview URL for <img src> (streamed, not fetched)
  videoFeedUrl: (label: string) => `${base}/api/v1/video/feed/${encodeURIComponent(label)}`,
  // Data collection recorder (morla_data_collect; works even when the
  // recorder was started from CLI — the control surface is pure topics)
  collectControl: (cmd: string) => post<unknown>('/api/v1/collect/control', { cmd }),
  collectTask: (text: string) => post<unknown>('/api/v1/collect/task', { text }),
  collectSession: (text: string) => post<unknown>('/api/v1/collect/session', { text }),
  // 数采节点泳道（独立于遥操预设生命周期）
  collectLaunchStart: () => post<unknown>('/api/v1/collect/launch/start'),
  collectLaunchStop: () => post<unknown>('/api/v1/collect/launch/stop'),
  collectLaunchRestart: () => post<unknown>('/api/v1/collect/launch/restart'),
  // 推理节点泳道（morla_policy_inference，配置化）+ 控制面（与 policy_keyboard 等价）
  inferLaunchStart: (cfg: InferLaunchConfig) => post<unknown>('/api/v1/infer/launch/start', cfg),
  inferLaunchStop: () => post<unknown>('/api/v1/infer/launch/stop'),
  inferLaunchRestart: () => post<unknown>('/api/v1/infer/launch/restart'),
  inferCmd: (cmd: string) => post<unknown>('/api/v1/infer/cmd', { cmd }),
  inferTask: (text: string) => post<unknown>('/api/v1/infer/task', { text }),
  replaySources: async (path = '', signal?: AbortSignal): Promise<ApiEnvelope<ReplayBrowserData>> => {
    const res = await fetch(`${base}/api/v1/infer/replay/sources?path=${encodeURIComponent(path)}`, { signal })
    const body = await res.json()
    if (!res.ok) return { ok: false, message: body.detail ?? `HTTP ${res.status}`, data: null as unknown as ReplayBrowserData }
    return body
  },
  logs: () => get<LaunchLogsData>('/api/v1/logs'),
  dataSessions: () => get<DataSessionsResponse>('/api/v1/data/sessions'),
  dataJobs: () => get<DataProcessState>('/api/v1/data/jobs'),
  dataJob: (id: string) => get<DataProcessJob>(`/api/v1/data/jobs/${encodeURIComponent(id)}`),
  dataStart: (request: DataProcessRequest) => post<DataProcessJob>('/api/v1/data/jobs', request),
  dataCancel: () => post<unknown>('/api/v1/data/jobs/cancel'),
  dataFileUrl: (id: string, path: string) => `${base}/api/v1/data/jobs/${encodeURIComponent(id)}/files/${path.split('/').map(encodeURIComponent).join('/')}`,
}
