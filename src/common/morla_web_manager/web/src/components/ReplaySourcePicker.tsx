import { useEffect, useRef, useState } from 'react'
import type { CSSProperties } from 'react'
import { api } from '../api/client'
import type { ReplayBrowserData } from '../types'

interface Props {
  source: string
  onSelect: (source: string) => void
  onClose: () => void
}

const labels: Record<string, string> = {
  folder: '文件夹', raw: '原始录制段', aligned: '对齐轨迹', hdf5: 'HDF5 轨迹',
  lerobot: 'LeRobot v2.1', recording: '尚未保存', unsupported: '暂不支持的数据集',
}

export function ReplaySourcePicker({ source, onSelect, onClose }: Props) {
  const dialog = useRef<HTMLDialogElement>(null)
  const [path, setPath] = useState(source.replace(/:\d+$/, ''))
  const [data, setData] = useState<ReplayBrowserData | null>(null)
  const [busy, setBusy] = useState(true)
  const [error, setError] = useState('')
  const [filter, setFilter] = useState('')
  const [episode, setEpisode] = useState(source.match(/:(\d+)$/)?.[1] ?? '0')
  const [revision, setRevision] = useState(0)

  useEffect(() => {
    if (dialog.current && !dialog.current.open) dialog.current.showModal()
  }, [])

  useEffect(() => {
    const controller = new AbortController()
    setBusy(true)
    setError('')
    setFilter('')
    api.replaySources(path, controller.signal).then(res => {
      if (controller.signal.aborted) return
      if (res.ok) setData(res.data)
      else { setData(null); setError(res.message) }
    }).catch(err => {
      if (!controller.signal.aborted) {
        setData(null)
        setError(`读取目录失败：${String(err)}`)
      }
    }).finally(() => {
      if (!controller.signal.aborted) setBusy(false)
    })
    return () => controller.abort()
  }, [path, revision])

  const validEpisode = /^\d+$/.test(episode) && Number.isSafeInteger(Number(episode))
  function select(selectedPath: string, kind: string) {
    if (kind === 'lerobot' && !validEpisode) return
    onSelect(kind === 'lerobot' ? `${selectedPath}:${Number(episode)}` : selectedPath)
  }
  const entries = data?.entries.filter(entry => entry.name.toLocaleLowerCase().includes(filter.toLocaleLowerCase())) ?? []

  return (
    <dialog ref={dialog} className="replay-source-picker" style={dialogStyle}
      aria-labelledby="replay-picker-title" onCancel={onClose}>
      <style>{'.replay-source-picker::backdrop { background: rgba(0,0,0,.65); } .replay-source-picker button:disabled { opacity: .45; cursor: not-allowed; }'}</style>
      <div style={row}>
        <strong id="replay-picker-title" style={{ fontSize: 17 }}>选择回放源</strong>
        <button style={{ ...button, marginLeft: 'auto' }} onClick={onClose} aria-label="关闭回放源选择">关闭</button>
      </div>
      <div style={muted}>浏览机器人主机上的数据目录，选择文件夹或轨迹文件。</div>
      <div style={row}>
        <button style={button} disabled={busy || path === ''} onClick={() => setPath('')}>数据根目录</button>
        {data?.roots.map(root => (
          <button key={root.path} style={{ ...button, fontSize: 11, overflowWrap: 'anywhere' }}
            disabled={busy} onClick={() => setPath(root.path)}>{root.name}</button>
        ))}
      </div>
      <div style={row}>
        <button style={button} disabled={busy || !data?.parent}
          onClick={() => data?.parent && setPath(data.parent)}>上一级</button>
        <span style={{ ...muted, overflowWrap: 'anywhere', flex: 1 }}>{data?.path ?? path}</span>
        <button style={button} disabled={busy} onClick={() => setRevision(value => value + 1)}>刷新</button>
      </div>
      <input aria-label="筛选回放源" style={input} value={filter} disabled={busy}
        placeholder="按名称筛选当前文件夹" onChange={event => setFilter(event.target.value)} />
      {error && <div role="alert" style={{ color: '#fca5a5' }}>{error}</div>}
      <div style={list} aria-busy={busy}>
        {busy ? <div style={muted}>正在读取目录…</div> : entries.map(entry => (
          <div key={entry.path} style={{ ...row, padding: '9px 0', borderBottom: '1px solid #374151' }}>
            <button style={{ ...entryButton, cursor: entry.is_dir || entry.selectable ? 'pointer' : 'default' }}
              disabled={!entry.is_dir && !entry.selectable}
              onClick={() => entry.is_dir ? setPath(entry.path) : select(entry.path, entry.kind)}>
              {entry.is_dir ? '📁' : '📄'} {entry.name}
            </button>
            <span style={muted}>{labels[entry.kind] ?? entry.kind}</span>
            {entry.selectable && (
              <button style={button} onClick={() => entry.kind === 'lerobot'
                ? setPath(entry.path) : select(entry.path, entry.kind)}>
                {entry.kind === 'lerobot' ? '进入选段' : '选择'}
              </button>
            )}
          </div>
        ))}
        {!busy && !error && entries.length === 0 && <div style={muted}>没有匹配的文件夹或回放文件。</div>}
      </div>
      <div style={row}>
        {data?.kind === 'lerobot' && <label style={row}>
          数据集段号
          <input aria-label="回放段号" type="number" min="0" step="1" value={episode}
            style={{ ...input, width: 90 }} onChange={event => setEpisode(event.target.value)} />
        </label>}
        <button style={{ ...button, marginLeft: 'auto', background: '#2563eb' }}
          disabled={busy || !data?.selectable || (data.kind === 'lerobot' && !validEpisode)}
          onClick={() => data && select(data.path, data.kind)}>
          {data?.kind === 'lerobot' ? '使用此数据集' : '使用当前录制段'}
        </button>
      </div>
      <div style={muted}>选择后点击推理卡的“回放 (y)”开始；原始数据需先停止录制并保存。</div>
    </dialog>
  )
}

const dialogStyle: CSSProperties = {
  width: 'min(820px, calc(100vw - 48px))', maxHeight: '85vh', overflow: 'auto',
  background: '#1f2937', color: '#e5e7eb', border: '1px solid #4b5563', borderRadius: 12,
  padding: 20, boxSizing: 'border-box',
}
const row: CSSProperties = { display: 'flex', alignItems: 'center', gap: 10, flexWrap: 'wrap', marginBottom: 8 }
const muted: CSSProperties = { color: '#9ca3af', fontSize: 12, marginBottom: 4 }
const button: CSSProperties = { border: '1px solid #4b5563', borderRadius: 6, background: '#374151', color: '#fff', padding: '6px 10px', cursor: 'pointer' }
const input: CSSProperties = { border: '1px solid #4b5563', borderRadius: 6, background: '#111827', color: '#e5e7eb', padding: '7px 10px', boxSizing: 'border-box', width: '100%' }
const list: CSSProperties = { maxHeight: '42vh', minHeight: 160, overflow: 'auto', margin: '10px 0' }
const entryButton: CSSProperties = { flex: '1 1 200px', textAlign: 'left', overflowWrap: 'anywhere', background: 'none', color: '#e5e7eb', border: 0, padding: '4px 0', fontSize: 13 }
