// 策略推理模块：astral_policy_inference 的 web 控制面。
// 节点泳道（配置化启动/重启/停止）+ 控制面（cmd/task 纯话题桥，与 policy_keyboard
// 完全等价，CLI 启动的推理节点同样可控）+ 实时状态（/policy_inference/state 镜像）。
import { useState } from 'react'
import type { InferLaunchConfig, InferLaunchInfo, InferState } from '../types'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'
import { LaunchLogPanel } from './LogConsole'

interface Props {
  infer: InferState | null
  launch: InferLaunchInfo | null
}

const MODE_META: Record<string, { label: string; color: string }> = {
  IDLE: { label: '空闲', color: '#6b7280' },
  POLICY: { label: '● 策略控制', color: '#22c55e' },
  PLAYBACK: { label: '● 回放', color: '#3b82f6' },
  HUMAN: { label: '● 人接管', color: '#f59e0b' },
}

function modeMeta(state: string | undefined) {
  if (!state) return { label: '—', color: '#6b7280', paused: false }
  const paused = state.endsWith('_PAUSED')
  const base = paused ? state.slice(0, -7) : state
  const m = MODE_META[base] ?? { label: state, color: '#6b7280' }
  return { ...m, label: paused ? `${m.label}（暂停）` : m.label, paused }
}

// 引擎统计值的安全渲染：嵌套对象（如 engine.server_timing: {prep_ms, pre_ms,
// infer_ms, ...}）不能直接当 React child（React 会抛 "Objects are not valid as
// a React child" → 整棵树卸载白屏）——对象压平为标量摘要，标量原样。
function fmtStat(v: unknown): string {
  if (typeof v === 'number') return String(Number.isFinite(v) ? Number(v.toFixed(2)) : v)
  if (typeof v === 'string' || typeof v === 'boolean') return String(v)
  if (v && typeof v === 'object' && !Array.isArray(v)) {
    const inner = Object.entries(v as Record<string, unknown>)
      .filter(([, x]) => typeof x === 'number' || typeof x === 'string' || typeof x === 'boolean')
      .map(([k, x]) => `${k}:${fmtStat(x)}`)
    return inner.length ? `{${inner.join(' ')}}` : JSON.stringify(v)
  }
  return String(v)
}

export function InferenceCard({ infer, launch }: Props) {
  const [cfg, setCfg] = useState<InferLaunchConfig>({
    backend_type: 'remote',
    model: 'pi05',
    host: '127.0.0.1',
    port: 8001,
    camera_image_size: 224,
    engine_mode: 'queue_async',
    log: false,
  })
  const [task, setTask] = useState('')
  const [pbSource, setPbSource] = useState('')

  const online = infer != null && !infer.stale
  const meta = modeMeta(infer?.state)
  const launchState = launch?.state ?? 'stopped'
  const nodeRunning = launchState === 'running' || launchState === 'starting'
  const err = online ? infer?.error : null
  const engine = online ? infer?.engine ?? {} : {}
  const loop = online ? infer?.latency_ms?.loop?.avg : undefined
  const obsAge = online ? infer?.latency_ms?.obs_age?.avg : undefined
  const execEvents = online ? infer?.exec_events ?? [] : []

  async function launchOp(fn: () => Promise<{ ok: boolean; message: string }>) {
    const res = await fn()
    pushToast(res.message, res.ok ? 'success' : 'error')
  }

  function startNode() {
    if (!cfg.host.trim()) {
      pushToast('请先填写 GPU 主机 IP', 'error')
      return
    }
    void launchOp(() => api.inferLaunchStart(cfg))
  }

  async function sendCmd(cmd: string) {
    const res = await api.inferCmd(cmd)
    pushToast(res.message, res.ok ? 'success' : 'error')
  }

  async function sendPlayback() {
    const src = pbSource.trim()
    await sendCmd(src ? `playback:${src}` : 'playback')
  }

  async function sendTask() {
    const text = task.trim()
    if (!text) return
    const res = await api.inferTask(text)
    if (res.ok) {
      pushToast(`已设置任务: ${text}`, 'success')
      setTask('')
    } else {
      pushToast(res.message, 'error')
    }
  }

  return (
    <div style={cardStyle}>
      <div style={headStyle}>
        <span style={titleStyle}>策略推理</span>
        <span style={{ ...badgeStyle, background: meta.color }}>{meta.label}</span>
        {online && infer?.prompt && (
          <span style={dimStyle}>任务：{infer.prompt}</span>
        )}
        {online && infer?.state_dim != null && (
          <span style={dimStyle}>state {infer.state_dim} 维</span>
        )}
        {infer != null && infer.stale && <span style={warnStyle}>状态超时，节点可能已退出</span>}
        <div style={spacer} />
        {/* 节点进程控制：配置化泳道，与遥操/数采泳道解耦 */}
        <span style={nodeChipStyle(launchState)}>
          节点: {launchState === 'running' ? '运行中' : launchState === 'starting' ? '启动中' : '已停止'}
          {launch?.preset ? ` (${launch.preset})` : ''}
        </span>
        {!nodeRunning ? (
          <button style={btn('#7c3aed')} onClick={startNode}>
            启动节点
          </button>
        ) : (
          <>
            <button
              style={btn('#7c3aed')}
              title="改了 host/端口/图像尺寸/日志开关后重启生效"
              onClick={() => void launchOp(api.inferLaunchRestart)}
            >
              重启节点
            </button>
            <button style={btn('#4b5563')} onClick={() => void launchOp(api.inferLaunchStop)}>
              停止节点
            </button>
          </>
        )}
      </div>

      {!online && (
        <div style={offlineStyle}>
          推理节点离线——点右上「启动节点」（配置化泳道），或命令行
          <code style={codeStyle}>
            ros2 launch astral_policy_inference policy_inference.launch.py backend_type:=remote
            host:=&lt;GPU主机IP&gt; port:=8001 camera_image_size:=224 model:=pi05
          </code>
          。CLI 启动的节点，本卡片命令按钮同样可用（纯话题控制面）。
        </div>
      )}

      {online && err && (
        <div style={errorStyle}>
          <b>error：</b>{String(err)}
        </div>
      )}
      {online && (infer?.node_count ?? 1) > 1 && (
        <div style={errorStyle}>
          检测到 {infer!.node_count} 个推理节点同时在线——web 泳道与 CLI 双开会双写
          joint_commands。请停止其中一个（`ros2 node list` 确认 /policy_node 数量）。
        </div>
      )}
      {online && execEvents.length > 0 && (
        <div style={dimStyle}>
          安全层事件: {execEvents.slice(-4).join(' · ')}
        </div>
      )}

      {/* 启动配置 */}
      <div style={configStyle}>
        <div style={configTitleStyle}>启动配置</div>
        <div style={configRowStyle}>
          <label style={fieldStyle}>
            <span style={labelStyle}>GPU 主机 IP</span>
            <input style={inputStyle} value={cfg.host}
              onChange={(e) => setCfg({ ...cfg, host: e.target.value })} />
          </label>
          <label style={fieldSmallStyle}>
            <span style={labelStyle}>端口</span>
            <input style={inputStyle} type="number" value={cfg.port}
              onChange={(e) => setCfg({ ...cfg, port: Number(e.target.value) || 8001 })} />
          </label>
          <label style={fieldSmallStyle}>
            <span style={labelStyle}>图像尺寸</span>
            <input style={inputStyle} type="number" value={cfg.camera_image_size}
              onChange={(e) => setCfg({ ...cfg, camera_image_size: Number(e.target.value) || 224 })} />
          </label>
          <label style={fieldStyle}>
            <span style={labelStyle}>引擎模式</span>
            <select style={inputStyle} value={cfg.engine_mode}
              onChange={(e) => setCfg({ ...cfg, engine_mode: e.target.value })}>
              <option value="queue_async">queue_async（默认 30Hz）</option>
              <option value="queue_sync">queue_sync</option>
              <option value="rtc">rtc</option>
            </select>
          </label>
          <label style={fieldStyle}>
            <span style={labelStyle}>模型族</span>
            <select style={inputStyle} value={cfg.model}
              onChange={(e) => setCfg({ ...cfg, model: e.target.value })}>
              <option value="act">ACT</option>
              <option value="pi05">pi0.5</option>
            </select>
          </label>
          <label style={fieldSmallStyle}>
            <span style={labelStyle}>记录完整诊断日志</span>
            <input type="checkbox" checked={cfg.log}
              onChange={(e) => setCfg({ ...cfg, log: e.target.checked })} />
            <span style={dimStyle}>{cfg.log ? 'state+joint → /tmp' : '关'}</span>
          </label>
        </div>
      </div>

      {/* 任务 + 命令按钮（与 policy_keyboard 等价） */}
      <div style={rowStyle}>
        <div style={taskWrap}>
          <input
            style={inputStyle}
            value={task}
            placeholder="任务文本（如：把方块放进盒子）"
            onChange={(e) => setTask(e.target.value)}
            onKeyDown={(e) => e.key === 'Enter' && void sendTask()}
          />
          <button style={btn('#4b5563')} disabled={!task.trim()} onClick={() => void sendTask()}>
            设定任务
          </button>
        </div>
        <div style={taskWrap}>
          <input
            style={inputStyle}
            value={pbSource}
            placeholder="回放源（可选：<路径>[:<段号>]）"
            onChange={(e) => setPbSource(e.target.value)}
          />
        </div>
      </div>
      <div style={rowStyle}>
        <button disabled={!online} style={btn('#22c55e')} onClick={() => void sendCmd('policy')}>开始策略 (s)</button>
        <button disabled={!online} style={btn('#3b82f6')} onClick={() => void sendPlayback()}>回放 (y)</button>
        <button disabled={!online} style={btn('#f59e0b')} onClick={() => void sendCmd('pause')}>暂停 (空格)</button>
        <button disabled={!online} style={btn('#22c55e')} onClick={() => void sendCmd('resume')}>恢复 (n)</button>
        <button disabled={!online} style={btn('#f59e0b')} onClick={() => void sendCmd('takeover')}>接管 HUMAN (h)</button>
        <button disabled={!online} style={btn('#3b82f6')} onClick={() => void sendCmd('release')}>释放 (g)</button>
        <button disabled={!online} style={dangerBtn} onClick={() => void sendCmd('stop')}>停止 (x)</button>
      </div>

      {/* 实时状态 */}
      {online && (
        <div style={statsStyle}>
          <span style={chipStyle}>模式: <b>{meta.label}</b></span>
          {Object.entries(engine).map(([k, v]) => (
            <span key={k} style={chipStyle}>{k}: <b>{fmtStat(v)}</b></span>
          ))}
          {loop != null && <span style={chipStyle}>loop: <b>{loop}ms</b></span>}
          {obsAge != null && <span style={chipStyle}>obs_age: <b>{obsAge}ms</b></span>}
          {infer?.playback && (
            <span style={chipStyle}>回放: <b>{infer.playback.idx}/{infer.playback.frames}</b></span>
          )}
          {infer?.cam_frames && Object.entries(infer.cam_frames).map(([k, v]) => (
            <span key={k} style={chipStyle}>{k}: <b>{fmtStat(v)}</b>帧</span>
          ))}
        </div>
      )}

      <LaunchLogPanel
        title="推理 Launch 日志"
        lines={launch?.log_tail ?? []}
        downloadPrefix="policy-launch-logs"
        height="260px"
        loadAll={async () => {
          const res = await api.logs()
          return res.ok && res.data ? res.data.infer ?? [] : launch?.log_tail ?? []
        }}
      />
    </div>
  )
}

const cardStyle: React.CSSProperties = {
  background: '#1f2937',
  borderRadius: '8px',
  padding: '12px 14px',
  display: 'flex',
  flexDirection: 'column',
  gap: '10px',
}
const headStyle: React.CSSProperties = { display: 'flex', alignItems: 'center', gap: '10px', flexWrap: 'wrap' }
const spacer: React.CSSProperties = { flex: 1 }
const titleStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 700, fontSize: '14px' }
const badgeStyle: React.CSSProperties = {
  color: 'white', borderRadius: '6px', padding: '2px 10px', fontSize: '12px', fontWeight: 700,
}
const dimStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '12px' }
const warnStyle: React.CSSProperties = { color: '#f59e0b', fontSize: '12px' }
const errorStyle: React.CSSProperties = {
  color: '#fecaca', background: '#7f1d1d', borderRadius: '6px',
  padding: '6px 10px', fontSize: '12px', lineHeight: 1.6,
}
const offlineStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '13px', lineHeight: 1.7 }
const codeStyle: React.CSSProperties = {
  background: '#111827', borderRadius: '4px', padding: '1px 6px', marginLeft: '6px', fontSize: '12px',
}
const configStyle: React.CSSProperties = {
  background: '#111827', borderRadius: '6px', padding: '8px 10px',
  display: 'flex', flexDirection: 'column', gap: '8px',
}
const configTitleStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 600, fontSize: '12px' }
const configRowStyle: React.CSSProperties = { display: 'flex', gap: '10px', flexWrap: 'wrap', alignItems: 'flex-end' }
const fieldStyle: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: '4px', flex: 1, minWidth: '150px' }
const fieldSmallStyle: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: '4px', minWidth: '110px' }
const labelStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '11px' }
const inputStyle: React.CSSProperties = {
  flex: 1, background: '#1f2937', border: '1px solid #374151', borderRadius: '6px',
  color: '#e5e7eb', padding: '6px 10px', fontSize: '13px', outline: 'none',
}
const nodeChipStyle = (launchState: string): React.CSSProperties => ({
  background: launchState === 'running' ? '#14532d' : '#374151',
  color: launchState === 'running' ? '#86efac' : '#9ca3af',
  borderRadius: '6px', padding: '3px 10px', fontSize: '12px',
})
const rowStyle: React.CSSProperties = { display: 'flex', alignItems: 'center', gap: '8px', flexWrap: 'wrap' }
const taskWrap: React.CSSProperties = { display: 'flex', gap: '8px', flex: 1, minWidth: '260px' }
const statsStyle: React.CSSProperties = {
  display: 'flex', flexWrap: 'wrap', gap: '8px', fontSize: '12px', color: '#9ca3af',
}
const chipStyle: React.CSSProperties = { background: '#374151', padding: '3px 10px', borderRadius: '6px' }
const btn = (color: string): React.CSSProperties => ({
  background: color, color: 'white', border: 'none', borderRadius: '6px',
  padding: '7px 14px', fontSize: '13px', fontWeight: 600, cursor: 'pointer',
})
const dangerBtn: React.CSSProperties = {
  background: 'transparent', color: '#f87171', border: '1px solid #f87171',
  borderRadius: '6px', padding: '7px 14px', fontSize: '13px', fontWeight: 600, cursor: 'pointer',
}
