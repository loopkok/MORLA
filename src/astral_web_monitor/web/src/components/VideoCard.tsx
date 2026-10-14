// 视频回传控制卡：quest3_video_streamer 运行时门控（非侵入 —— streamer 拥有相机，
// 这里只调它的 SetBool 总开关、发 latched active_cameras 子集）。
import { useCallback, useEffect, useMemo, useState } from 'react'
import type { NormalisedState } from '../lib/mapUiState'
import type { VideoCameraInfo } from '../types'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'

interface Props {
  state: NormalisedState | null
}

export function VideoCard({ state }: Props) {
  const [cams, setCams] = useState<VideoCameraInfo[]>([])
  const [online, setOnline] = useState(false)
  const [checked, setChecked] = useState<string[]>([])
  const [showPreview, setShowPreview] = useState(true)

  const gate = state?.videoGate ?? null
  const pushEnabled = gate?.pushEnabled ?? false

  const refresh = useCallback(async () => {
    const res = await api.videoStatus()
    if (!res.ok || !res.data) return
    setCams(res.data.configured ?? [])
    setOnline(res.data.online)
  }, [])

  useEffect(() => {
    void refresh()
    const t = setInterval(() => void refresh(), 5000)
    return () => clearInterval(t)
  }, [refresh])

  // Sync the checked set from the live gate (WS) whenever it changes.
  const activeKey = (gate?.active ?? []).join(',')
  useEffect(() => {
    if (gate) setChecked(gate.active)
  }, [activeKey]) // eslint-disable-line react-hooks/exhaustive-deps

  // Offline fallback: show all existing configured cameras as checked.
  useEffect(() => {
    if (!gate && cams.length > 0) setChecked(cams.filter((c) => c.exists).map((c) => c.label))
  }, [gate, cams])

  const allLabels = useMemo(() => cams.map((c) => c.label), [cams])

  // 路数下拉：active == 前 K 路配置 → K；全选 → N；否则“自定义”。
  const countValue = useMemo(() => {
    if (checked.length === allLabels.length && allLabels.every((l) => checked.includes(l))) {
      return String(allLabels.length)
    }
    for (let k = 0; k <= allLabels.length; k++) {
      const prefix = allLabels.slice(0, k)
      if (prefix.length === checked.length && prefix.every((l) => checked.includes(l))) return String(k)
    }
    return 'custom'
  }, [checked, allLabels])

  async function applyCameras(next: string[]) {
    setChecked(next)
    const res = await api.videoCameras(next)
    if (res.ok) pushToast(res.message, 'success')
    else pushToast(res.message, 'error')
    setTimeout(() => void refresh(), 300)
  }

  function onCountSelect(v: string) {
    if (v === 'custom') return
    const k = parseInt(v, 10)
    void applyCameras(allLabels.slice(0, k))
  }

  function onToggle(label: string, on: boolean) {
    const next = on ? [...checked, label] : checked.filter((l) => l !== label)
    // Keep configured order stable.
    void applyCameras(allLabels.filter((l) => next.includes(l)))
  }

  async function onPushToggle(on: boolean) {
    const res = await api.videoPush(on)
    if (res.ok) pushToast(on ? '视频推送已开启' : '视频推送已关闭（黑帧静音）', 'success')
    else pushToast(res.message, 'error')
    setTimeout(() => void refresh(), 300)
  }

  // Live previews: checked + online cameras only (MJPEG, ~10fps JPEG relay).
  const previewCams = useMemo(
    () => (online && showPreview ? cams.filter((c) => checked.includes(c.label) && c.exists) : []),
    [online, showPreview, cams, checked],
  )

  return (
    <div style={cardStyle}>
      <div style={titleStyle}>
        视频回传（quest3_video_streamer）
        <span style={{ ...dotStyle, background: online ? '#22c55e' : '#6b7280' }} />
        <span style={statusText}>{online ? (pushEnabled ? '推送中' : '已静音') : '离线'}</span>
      </div>

      <div style={rowStyle}>
        <label style={checkLabel}>
          <input
            type="checkbox"
            checked={pushEnabled}
            disabled={!online}
            onChange={(e) => void onPushToggle(e.target.checked)}
          />
          推送画面（总开关）
        </label>
        <label style={checkLabel}>
          路数
          <select
            value={countValue}
            disabled={cams.length === 0}
            onChange={(e) => onCountSelect(e.target.value)}
            style={selectStyle}
          >
            {allLabels.map((_, i) => (
              <option key={i} value={String(i + 1)}>{i + 1} 路</option>
            ))}
            <option value="0">0 路（全关）</option>
            <option value="custom">自定义</option>
          </select>
        </label>
        {cams.map((c) => (
          <label key={c.label} style={{ ...checkLabel, opacity: c.exists ? 1 : 0.45 }}>
            <input
              type="checkbox"
              checked={checked.includes(c.label)}
              disabled={!c.exists}
              onChange={(e) => onToggle(c.label, e.target.checked)}
            />
            {c.label}
            <span style={devText}>
              {c.device}{c.sysfs_name ? ` · ${c.sysfs_name}` : ''}{c.exists ? '' : '（未接）'}
            </span>
          </label>
        ))}
        {cams.length === 0 && <span style={hintStyle}>主机未扫描到可采集的视频设备（/dev/video*）</span>}
        <label style={checkLabel}>
          <input
            type="checkbox"
            checked={showPreview}
            onChange={(e) => setShowPreview(e.target.checked)}
          />
          实时画面
        </label>
      </div>

      {previewCams.length > 0 && (
        <div style={previewRowStyle}>
          {previewCams.map((c) => (
            <figure key={c.label} style={previewItemStyle}>
              <img
                src={api.videoFeedUrl(c.label)}
                alt={c.label}
                style={previewImgStyle}
              />
              <figcaption style={previewCapStyle}>{c.label}</figcaption>
            </figure>
          ))}
        </div>
      )}

      <div style={hintStyle}>
        设备列表来自自动扫描（按物理设备去重的 /dev/video* 采集节点），不写死配置。
        勾选通过 latched <code>~/active_cameras</code> 下发：<b>离线时也可预选</b>，streamer 启动后即生效；
        在线时取消勾选 = 该轨改发 2fps 黑帧（几乎不占带宽，Quest 面板变黑），重新勾选即时恢复，无需重连。
        总开关（<code>~/set_push_enabled</code> 服务）需 streamer 在线。
        实时画面 = streamer 抽帧 JPEG（10fps/640宽/q65，独立线程编码，不影响 Quest 链路）经本站 MJPEG 转发，
        延迟约 0.2~0.4s，仅供监控；取消勾选的路预览同步停止。
        推流前提：<code>adb reverse tcp:8765 tcp:8765</code>（full_teleop 已自动执行）+ Quest 端开启 video feed。
        启动后才插入的相机需重启栈才会进入 track 集合。
      </div>
    </div>
  )
}

const cardStyle: React.CSSProperties = {
  background: '#1f2937',
  border: '1px solid #374151',
  borderRadius: '10px',
  padding: '14px',
  display: 'flex',
  flexDirection: 'column',
  gap: '10px',
}
const titleStyle: React.CSSProperties = {
  color: '#e5e7eb',
  fontWeight: 700,
  fontSize: '14px',
  display: 'flex',
  alignItems: 'center',
  gap: '8px',
}
const dotStyle: React.CSSProperties = {
  width: '9px',
  height: '9px',
  borderRadius: '50%',
  display: 'inline-block',
}
const statusText: React.CSSProperties = { color: '#9ca3af', fontSize: '12px', fontWeight: 400 }
const rowStyle: React.CSSProperties = { display: 'flex', gap: '14px', flexWrap: 'wrap', alignItems: 'center' }
const checkLabel: React.CSSProperties = {
  color: '#e5e7eb',
  fontSize: '13px',
  display: 'flex',
  alignItems: 'center',
  gap: '6px',
  cursor: 'pointer',
}
const devText: React.CSSProperties = { color: '#6b7280', fontSize: '11px' }
const selectStyle: React.CSSProperties = {
  background: '#374151',
  color: '#e5e7eb',
  border: '1px solid #4b5563',
  borderRadius: '6px',
  padding: '5px 10px',
  fontSize: '13px',
  marginLeft: '4px',
}
const hintStyle: React.CSSProperties = { color: '#6b7280', fontSize: '12px', lineHeight: 1.5 }
const previewRowStyle: React.CSSProperties = {
  display: 'flex',
  gap: '10px',
  flexWrap: 'wrap',
}
const previewItemStyle: React.CSSProperties = {
  margin: 0,
  display: 'flex',
  flexDirection: 'column',
  gap: '4px',
}
const previewImgStyle: React.CSSProperties = {
  width: '320px',
  aspectRatio: '16 / 9',
  objectFit: 'cover',
  borderRadius: '8px',
  border: '1px solid #374151',
  background: '#111827',
}
const previewCapStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '11px' }
