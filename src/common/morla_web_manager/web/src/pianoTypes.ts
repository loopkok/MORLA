export type PianoSide = 'left' | 'right'
export type PianoFinger = 'index' | 'middle' | 'ring' | 'pinky'
export type PianoMode = 'software' | 'ros_fake' | 'hardware'
export interface PianoHand {
  zone_label: string
  anchor_note: string
  finger_white_offsets: Record<PianoFinger, number>
  initial_arm_rad: number[]
  hover_rad: number[]
  start_reference_rad?: number[]
  arrival_check_enabled?: boolean
  arm_step_rad_per_white: number[]
  arm_positions_rad: Record<string, number[]>
  shift_range_white: number[]
  press_rad: Record<PianoFinger, Record<string, number>>
  [key: string]: unknown
}
export interface PianoConfig {
  version: number
  calibrated: boolean
  keyboard: { first_note: string; key_count: number }
  timing: Record<string, number>
  performance?: Record<string, number>
  score_defaults: { tempo_bpm: number; tonic: string }
  hands: Record<PianoSide, PianoHand>
  calibration: { work_pose_deg: Record<PianoSide, number[]> }
  [key: string]: unknown
}
export interface PianoConfigRecord { config: PianoConfig; revision: string }
export interface PianoScore { id: string; name: string; tempo_bpm: number; events: number }
export interface PianoAction { at_s: number; kind: string; side: PianoSide; finger: PianoFinger | null; midi: number | null; shift: number; row: number; duration_s: number }
export interface PianoPlan { title: string; duration_s: number; notes: number; actions: PianoAction[]; config_revision: string }
export interface PianoSession {
  session_id?: string | null
  state: string
  active: boolean
  available?: boolean
  project_error?: string | null
  blockers?: string[]
  mode: PianoMode | null
  connected?: boolean
  browser_heartbeat_age_s?: number | null
  browser_heartbeat_timeout_s?: number | null
  motion_claimed?: boolean
  replay_direct_allowed?: boolean
  maintenance_allowed?: boolean
  hand_feedback_start_allowed?: boolean
  feedback_warnings?: Record<string, string>
  arm_speed?: Record<PianoSide, { stream_applied_speed_percent?: number | null }>
  hand_sdk?: Record<PianoSide, { feedback_active: boolean; feedback_valid: boolean; error_code: number; error: string; passive: boolean }>
  reset_required?: boolean
  busy: boolean
  closing: boolean
  error: string | null
  job: { id: string; operation: string } | null
  result: { job_id: string; ok: boolean; error?: string; result?: { positions?: Record<string, number[]>; hand_command_seeds?: Record<PianoSide, number[]> } } | null
  wrist_shifts?: Record<PianoSide, number>
  pressed?: [PianoSide, PianoFinger] | null
  playback?: {
    title: string; duration_s: number; elapsed_s: number; percent: number
    total_actions: number; completed_actions: number
    total_notes: number; started_notes: number; completed_notes: number; completed: boolean
    current_action: PianoAction | null
  } | null
  events?: { event: string; side?: PianoSide; midi?: number; scheduled_s?: number; actual_s?: number }[]
  positions?: Record<string, number[]>
  command_targets?: Record<string, number[]>
  feedback_age_s?: Record<string, number | null>
  poses?: Partial<Record<PianoSide, { xyz_mm: number[]; rpy_deg: number[]; base_frame: string; tip_frame: string }>>
  kinematics_error?: string | null
  work_progress?: { stage: number; total: number; mode: string; side?: PianoSide | 'both' } | null
  logs: string[]
  recovery?: { requested: boolean; exit_code?: number; error?: string } | null
}
