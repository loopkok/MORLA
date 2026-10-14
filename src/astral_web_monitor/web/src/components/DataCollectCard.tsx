// 数据采集卡片：astral_data_collect 的 web 控制面。
// 后端桥 /data_collect/{control,task} 话题 + 镜像 latched /data_collect/state。
// 采集节点即使由 CLI 启动也能被控制（纯话题接口），offline/stale 仅作提示。
import { useState } from 'react'
import type { CollectLaunchInfo, DataCollectState } from '../types'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'

interface Props {
  dc: DataCollectState | null
  launch: CollectLaunchInfo | null
  // 遥操栈状态（running/paused 时「段间回位」才可用，对齐后端 workpos 409 守卫）
  teleopState: string
}

const STATE_META: Record<string, { label: string; color: string }> = {
  IDLE: { label: '空闲', color: '#6b7280' },
  RECORDING: { label: '● 录制中', color: '#dc2626' },
  PAUSED: { label: '已暂停', color: '#f59e0b' },
  SAVING: { label: '保存中', color: '#3b82f6' },
}

export function DataCollectCard({ dc, launch, teleopState }: Props) {
  const [task, setTask] = useState('')
  const [sessionDir, setSessionDir] = useState('')
  // 「本次采集数据」实时下拉面板的展开态（数据随 WS 帧 1Hz 刷新）
  const [showData, setShowData] = useState(false)
  const online = dc != null && !dc.stale
  const st = dc?.state ?? 'IDLE'
  const meta = STATE_META[st] ?? STATE_META.IDLE
  const dropped = Object.entries(dc?.dropped ?? {}).filter(([, v]) => v > 0)
  const stateDim = Number(dc?.schema?.state_dim ?? 0) || null
  // 节点进程状态（web 泳道；CLI 启动的节点 launch 为 stopped 但 online=true）
  const launchState = launch?.state ?? 'stopped'
  const nodeRunning = launchState === 'running' || launchState === 'starting'
  // 段间回位（与 VR 左手柄 X 同功能）：遥操 RUNNING/PAUSED 才可用（对齐后端 409 守卫）；
  // 且录制中/保存中不可用（与 X 闸门录制保护一致——防手臂回位毁段）
  const workposEnabled =
    (teleopState === 'running' || teleopState === 'paused') &&
    !(online && (st === 'RECORDING' || st === 'PAUSED' || st === 'SAVING'))

  async function send(cmd: string, okMsg: string) {
    const res = await api.collectControl(cmd)
    pushToast(res.ok ? okMsg : res.message, res.ok ? 'success' : 'error')
  }

  async function launchOp(fn: () => Promise<{ ok: boolean; message: string }>) {
    const res = await fn()
    pushToast(res.message, res.ok ? 'success' : 'error')
  }

  async function sendTask() {
    const text = task.trim()
    if (!text) return
    const res = await api.collectTask(text)
    if (res.ok) {
      pushToast(`下一段任务: ${text}`, 'success')
      setTask('')
    } else {
      pushToast(res.message, 'error')
    }
  }

  async function sendSession() {
    const text = sessionDir.trim()
    if (!text) return
    const res = await api.collectSession(text)
    if (res.ok) {
      pushToast(`已切换录制目录: ${text}`, 'success')
      setSessionDir('')
    } else {
      pushToast(res.message, 'error')
    }
  }

  function discard() {
    if (!confirm('确认丢弃当前段？录制数据将被删除（段号会被下一段复用）。')) return
    void send('discard', '已丢弃当前段')
  }

  function workpos() {
    if (!confirm('确认段间回位？双臂将直接从当前位姿回到 init_pose 工作位（不经途径点，期间会 disarm 停止跟随 VR，需重新「开始遥操」再操作）。')) return
    void launchOp(api.teleopWorkposDirect)
  }

  // 「本次采集数据」下拉：字节格式化 + 当前段样本/帧汇总
  function fmtMB(b: number | undefined): string {
    if (b == null || b <= 0) return '0 MB'
    return `${(b / (1024 * 1024)).toFixed(1)} MB`
  }
  const epSamples = Object.values(dc?.episode_counts ?? {}).reduce((a, v) => a + v, 0)
  const epFrames = Object.values(dc?.camera_counts ?? {}).reduce((a, v) => a + v, 0)

  return (
    <div style={cardStyle}>
      <div style={headStyle}>
        <span style={titleStyle}>数据采集</span>
        <span style={{ ...badgeStyle, background: meta.color }}>{meta.label}</span>
        {dc != null && (
          <span style={dimStyle}>
            {dc.session} · 段 #{String(dc.episode_index).padStart(6, '0')} ·{' '}
            {dc.elapsed_s.toFixed(1)}s{stateDim ? ` · state ${stateDim} 维` : ''}
          </span>
        )}
        {dc != null && dc.stale && <span style={warnStyle}>状态超时，节点可能已退出</span>}
        <div style={spacer} />
        {/* 节点进程控制：独立泳道，与遥操预设生命周期解耦 */}
        <span style={nodeChipStyle(launchState)}>
          节点: {launchState === 'running' ? '运行中' : launchState === 'starting' ? '启动中' : '已停止'}
          {launch?.preset ? ` (${launch.preset})` : ''}
        </span>
        {!nodeRunning ? (
          <button style={btn('#4b5563')} onClick={() => void launchOp(api.collectLaunchStart)}>
            启动节点
          </button>
        ) : (
          <>
            <button
              style={btn('#4b5563')}
              title="改了 data_collect.yaml 的 schema 配置后重启生效"
              onClick={() => void launchOp(api.collectLaunchRestart)}
            >
              重启节点
            </button>
            <button
              style={dangerBtn}
              disabled={st === 'RECORDING' || st === 'PAUSED'}
              title={st === 'RECORDING' || st === 'PAUSED' ? '录制中不能停节点（先停止保存或丢弃）' : ''}
              onClick={() => void launchOp(api.collectLaunchStop)}
            >
              停止节点
            </button>
          </>
        )}
      </div>

      {!online && (
        <div style={offlineStyle}>
          采集节点离线——点右上「启动节点」（独立泳道，不占用遥操预设），或命令行
          <code style={codeStyle}>ros2 launch astral_data_collect data_collect.launch.py</code>
          。schema 硬件配置在 <code style={codeStyle}>config/data_collect.yaml</code>，
          改动后点「重启节点」生效。
        </div>
      )}

      {online && dc?.low_fps_warning && (
        <div style={offlineStyle}>
          {dc.low_fps_warning}
        </div>
      )}
      {online && dc?.empty_warning && (
        <div style={offlineStyle}>
          {dc.empty_warning}
        </div>
      )}

      {online && (dc?.node_count ?? 1) > 1 && (
        <div style={offlineStyle}>
          检测到 {dc!.node_count} 个采集节点同时在线——同一条指令会被各录一份
          （段数据重复）。请停止残留节点：<code style={codeStyle}>ros2 node list</code>
          {' '}确认 /data_collect 数量，多余进程 kill 后刷新。
        </div>
      )}

      <div style={rowStyle}>
        <button
          style={btn('#22c55e')}
          disabled={!online || st !== 'IDLE'}
          title="开始新段（s）"
          onClick={() => void send('start', '开始录制')}
        >
          开始录制
        </button>
        <button
          style={btn('#3b82f6')}
          disabled={!online || (st !== 'RECORDING' && st !== 'PAUSED')}
          title="结束并保存当前段（q）"
          onClick={() => void send('stop', '已保存当前段')}
        >
          停止保存
        </button>
        <button
          style={btn('#3b82f6')}
          disabled={!online || (st !== 'RECORDING' && st !== 'PAUSED')}
          title="保存当前段并立即开新段（n）"
          onClick={() => void send('next', '已保存，开始新段')}
        >
          下一段
        </button>
        {st === 'PAUSED' ? (
          <button
            style={btn('#f59e0b')}
            disabled={!online}
            onClick={() => void send('resume', '已继续录制')}
          >
            继续
          </button>
        ) : (
          <button
            style={btn('#f59e0b')}
            disabled={!online || st !== 'RECORDING'}
            title="暂停写盘（p），数据不入缓冲"
            onClick={() => void send('pause', '已暂停')}
          >
            暂停
          </button>
        )}
        <button
          style={dangerBtn}
          disabled={!online || (st !== 'RECORDING' && st !== 'PAUSED')}
          title="丢弃当前段（d），文件删除"
          onClick={discard}
        >
          丢弃
        </button>
        <button
          style={btn('#14b8a6')}
          disabled={!workposEnabled}
          title={workposEnabled ? '停止跟随 VR 并直达回到工作位（与左手柄 X 同功能，不经途径点）' : '遥操需 RUNNING/PAUSED 且非录制中才可用'}
          onClick={workpos}
        >
          段间回位
        </button>

        <div style={taskWrap}>
          <input
            style={inputStyle}
            value={task}
            placeholder="下一段任务文本（如：把方块放进盒子）"
            disabled={!online}
            onChange={(e) => setTask(e.target.value)}
            onKeyDown={(e) => e.key === 'Enter' && void sendTask()}
          />
          <button style={btn('#4b5563')} disabled={!online || !task.trim()} onClick={() => void sendTask()}>
            设定任务
          </button>
        </div>
        <div style={taskWrap}>
          <input
            style={inputStyle}
            value={sessionDir}
            placeholder={`录到目录（当前: ${dc?.session ?? '?'}；如 pick_place_0907，仅空闲可换）`}
            disabled={!online}
            onChange={(e) => setSessionDir(e.target.value)}
            onKeyDown={(e) => e.key === 'Enter' && void sendSession()}
          />
          <button
            style={btn('#4b5563')}
            disabled={!online || !sessionDir.trim()}
            onClick={() => void sendSession()}
          >
            设定目录
          </button>
        </div>
      </div>

      {/* 本次采集数据：实时下拉窗口（数据随 WS 帧刷新，实际内容 1Hz 由采集节点发布） */}
      <div style={rowStyle}>
        <button
          style={dataToggleStyle}
          onClick={() => setShowData((v) => !v)}
          title="实时查看本次采集的数据量 / 所在文件夹（录制中每秒刷新）"
        >
          {showData ? '▾' : '▸'} 本次采集数据
          {dc && (dc.session_bytes ?? 0) > 0 && (
            <span style={dataBadgeStyle}>{fmtMB(dc.session_bytes)}</span>
          )}
        </button>
      </div>
      {showData && (
        <div style={dataPanelStyle}>
          <div style={dimStyle}>
            文件夹：<code style={codeStyle}>{dc?.folder ?? '—'}</code>
          </div>
          <div style={dimStyle}>
            本次 session：磁盘 {fmtMB(dc?.session_bytes)}（累计
            {dc?.episode_index != null && dc.episode_index >= 0 ? ` · 已到段 #${String(dc.episode_index).padStart(6, '0')}` : ''}）
          </div>
          <div style={dimStyle}>
            当前段：{st === 'RECORDING' || st === 'PAUSED' ? (
              <>#{String(dc?.episode_index ?? 0).padStart(6, '0')} · 时长 {dc?.elapsed_s.toFixed(1)}s ·
                样本 {epSamples} · 帧 {epFrames} · 磁盘 {fmtMB(dc?.episode_bytes)}</>
            ) : (
              '未录制'
            )}
          </div>
          {Object.entries(dc?.episode_counts ?? {}).length > 0 && (
            <div style={statsStyle}>
              每流样本：{Object.entries(dc!.episode_counts!).map(([k, v]) => `${k}: ${v}`).join('  ')}
            </div>
          )}
          {Object.entries(dc?.camera_counts ?? {}).length > 0 && (
            <div style={statsStyle}>
              每相机帧：{Object.entries(dc!.camera_counts!).map(([k, v]) => `${k}: ${v}`).join('  ')}
            </div>
          )}
          <div style={dimStyle}>
            样本/帧数为精确数据量；磁盘占用含 HDF5 预分配（"占了多少磁盘"口径）。
          </div>
        </div>
      )}

      {online && (
        <div
          style={{
            ...dimStyle,
            lineHeight: 1.7,
            borderLeft: st === 'SAVING' ? '3px solid #f59e0b' : '3px solid #4b5563',
            paddingLeft: 8,
          }}
        >
          <b style={{ color: st === 'SAVING' ? '#f59e0b' : '#e5e7eb' }}>
            {meta.label.replace(/^● /, '')}
          </b>
          {'：'}
          {st === 'IDLE' &&
            '可开始新段（web 按钮 / A 键 / s）。保存后段号自动接续；丢弃的段若为最大号会被下一段复用。'}
          {st === 'RECORDING' &&
            '可停止保存 / 下一段 / 暂停 / 丢弃。VR：B=停止保存，摇杆按下=丢弃当前段（删文件不可逆，误触即丢）；键位在非法状态被静默忽略。'}
          {st === 'PAUSED' &&
            '缓冲已落盘、期间不录数据：可「继续」录制，或停止保存 / 丢弃当前段。'}
          {st === 'SAVING' &&
            '正在收尾写盘，请稍候——此期间按的开始/停止会被忽略并计入「忽略指令」，别急着操作。'}
        </div>
      )}

      {dc != null && (
        <div style={statsStyle}>
          {dc.task_next && (
            <span style={chipStyle}>
              下段任务: <b>{dc.task_next}</b>
            </span>
          )}
          {Object.entries(dc.samples_per_s).map(([k, v]) => (
            <span key={k} style={chipStyle}>
              {k}: <b>{v.toFixed(0)}</b>/s
            </span>
          ))}
          {dropped.length > 0 && (
            <span style={dropChipStyle}>
              掉帧: {dropped.map(([k, v]) => `${k}×${v}`).join(' ')}
            </span>
          )}
          {dc.ignored && Object.entries(dc.ignored).some(([, v]) => v > 0) && (
            <span style={chipStyle}>
              忽略指令:{' '}
              <b>
                {Object.entries(dc.ignored)
                  .map(([k, v]) => `${k}×${v}`)
                  .join(' ')}
              </b>
            </span>
          )}
        </div>
      )}
      {online && (
        <div style={hintWrapStyle}>
          <div style={hintTitleStyle}>操作键位与门控（web / VR / 键盘三端等价，可混用）</div>
          <div>
            VR 右手柄：<b style={hintKeyStyle}>A</b>=开始（仅空闲）｜
            <b style={hintKeyStyle}>B</b>=停止保存（仅录制中）｜
            <b style={hintKeyStyle}>摇杆按下</b>=丢弃（仅录制中，删文件不可逆）
          </div>
          <div>
            VR 左手柄：<b style={hintKeyStyle}>X</b>=段间回位（停止跟随 VR，**直接**回到工作位、
            不经途径点；录制中/保存中无效）｜web「段间回位」按钮同功能（仅遥操运行中）；系统 tab
            「工作位」=途经点路径
          </div>
          <div>
            键盘：<b style={hintKeyStyle}>s</b>=开始 <b style={hintKeyStyle}>q</b>=停止保存{' '}
            <b style={hintKeyStyle}>d</b>=丢弃 <b style={hintKeyStyle}>n</b>=保存并开新段{' '}
            <b style={hintKeyStyle}>p</b>=暂停/继续 <b style={hintKeyStyle}>t</b>=任务文本
          </div>
          <div>
            门控：键位在非法状态被静默忽略（如空闲时按摇杆、保存中按 A），无副作用；需要确认弹窗的
            破坏性操作（丢弃）请用本卡片按钮。
          </div>
        </div>
      )}
    </div>
  )
}

const hintWrapStyle: React.CSSProperties = {
  background: '#111827',
  borderRadius: '6px',
  padding: '8px 10px',
  display: 'flex',
  flexDirection: 'column',
  gap: '4px',
  fontSize: '12px',
  color: '#9ca3af',
  lineHeight: 1.7,
}
const hintTitleStyle: React.CSSProperties = {
  color: '#e5e7eb',
  fontWeight: 600,
  fontSize: '12px',
}
const hintKeyStyle: React.CSSProperties = {
  color: '#e5e7eb',
  background: '#374151',
  borderRadius: '4px',
  padding: '0 4px',
  marginRight: '2px',
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
const nodeChipStyle = (launchState: string): React.CSSProperties => ({
  background: launchState === 'running' ? '#14532d' : '#374151',
  color: launchState === 'running' ? '#86efac' : '#9ca3af',
  borderRadius: '6px',
  padding: '3px 10px',
  fontSize: '12px',
})
const titleStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 700, fontSize: '14px' }
const badgeStyle: React.CSSProperties = {
  color: 'white',
  borderRadius: '6px',
  padding: '2px 10px',
  fontSize: '12px',
  fontWeight: 700,
}
const dimStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '12px' }
const warnStyle: React.CSSProperties = { color: '#f59e0b', fontSize: '12px' }
const offlineStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '13px', lineHeight: 1.7 }
const codeStyle: React.CSSProperties = {
  background: '#111827',
  borderRadius: '4px',
  padding: '1px 6px',
  marginLeft: '6px',
  fontSize: '12px',
}
const rowStyle: React.CSSProperties = { display: 'flex', alignItems: 'center', gap: '8px', flexWrap: 'wrap' }
const dataToggleStyle: React.CSSProperties = {
  background: 'transparent',
  color: '#14b8a6',
  border: '1px solid #14b8a655',
  borderRadius: '6px',
  padding: '5px 12px',
  fontSize: '12px',
  fontWeight: 600,
  cursor: 'pointer',
  display: 'flex',
  alignItems: 'center',
  gap: '6px',
}
const dataBadgeStyle: React.CSSProperties = {
  background: '#14b8a633',
  color: '#5eead4',
  borderRadius: '9999px',
  padding: '1px 8px',
  fontSize: '11px',
}
const dataPanelStyle: React.CSSProperties = {
  background: '#111827',
  borderRadius: '6px',
  padding: '8px 10px',
  display: 'flex',
  flexDirection: 'column',
  gap: '4px',
  fontSize: '12px',
  lineHeight: 1.7,
}
const taskWrap: React.CSSProperties = { display: 'flex', gap: '8px', flex: 1, minWidth: '260px' }
const inputStyle: React.CSSProperties = {
  flex: 1,
  background: '#111827',
  border: '1px solid #374151',
  borderRadius: '6px',
  color: '#e5e7eb',
  padding: '7px 10px',
  fontSize: '13px',
  outline: 'none',
}
const statsStyle: React.CSSProperties = {
  display: 'flex',
  flexWrap: 'wrap',
  gap: '8px',
  fontSize: '12px',
  color: '#9ca3af',
}
const chipStyle: React.CSSProperties = { background: '#374151', padding: '3px 10px', borderRadius: '6px' }
const dropChipStyle: React.CSSProperties = {
  background: '#7f1d1d',
  color: '#fecaca',
  padding: '3px 10px',
  borderRadius: '6px',
  fontWeight: 700,
}
const btn = (color: string): React.CSSProperties => ({
  background: color,
  color: 'white',
  border: 'none',
  borderRadius: '6px',
  padding: '7px 14px',
  fontSize: '13px',
  fontWeight: 600,
  cursor: 'pointer',
})
const dangerBtn: React.CSSProperties = {
  background: 'transparent',
  color: '#f87171',
  border: '1px solid #f87171',
  borderRadius: '6px',
  padding: '7px 14px',
  fontSize: '13px',
  fontWeight: 600,
  cursor: 'pointer',
}
