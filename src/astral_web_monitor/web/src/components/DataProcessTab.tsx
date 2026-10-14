import { useEffect, useRef, useState } from 'react'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'
import type { NormalisedState } from '../lib/mapUiState'
import type { DataArtifact, DataProcessAction, DataProcessJob, DataProcessState, DataSessionsResponse } from '../types'

const actions: { value: DataProcessAction; label: string }[] = [
  { value: 'full', label: '完整流程：检查 → 诊断 → 对齐 → 可选清洗 → 导出' },
  { value: 'raw-check', label: '单步：原始 raw 校验' },
  { value: 'quantify', label: '单步：指令与状态诊断' },
  { value: 'align', label: '单步：时间对齐 + 校验 + 预览' },
  { value: 'validate', label: '单步：已有 aligned 校验' },
  { value: 'repair-preview', label: '单步：清洗预览（不删除帧）' },
  { value: 'repair', label: '单步：保守清洗 + 校验 + 来源验证' },
  { value: 'verify', label: '单步：清洗数据来源验证' },
  { value: 'preview', label: '单步：图像与状态曲线预览' },
  { value: 'export', label: '单步：已有 aligned 导出' },
  { value: 'upgrade-v3', label: '单步：LeRobot v2.1 升级 v3' },
]
const labels: Record<string, string> = { pending: '等待', running: '执行中', stopping: '正在停止', succeeded: '完成', failed: '失败', cancelled: '已停止', skipped: '未执行', interrupted: '服务重启中断' }
const colors: Record<string, string> = { running: '#38bdf8', succeeded: '#4ade80', failed: '#f87171', cancelled: '#fbbf24', interrupted: '#fbbf24' }

export function DataProcessTab({ state, connected }: { state: NormalisedState | null; connected: boolean }) {
  const [catalog, setCatalog] = useState<DataSessionsResponse | null>(null)
  const [fallback, setFallback] = useState<DataProcessState | null>(null)
  const [source, setSource] = useState('')
  const [action, setAction] = useState<DataProcessAction>('full')
  const [target, setTarget] = useState<'openpi' | 'act' | 'both'>('openpi')
  const [imageSize, setImageSize] = useState(224)
  const [cameraMap, setCameraMap] = useState('')
  const [episodes, setEpisodes] = useState('')
  const [quantify, setQuantify] = useState(true)
  const [repair, setRepair] = useState(false)
  const [reference, setReference] = useState('')
  const [submitting, setSubmitting] = useState(false)
  const [selectedId, setSelectedId] = useState('')
  const [historical, setHistorical] = useState<DataProcessJob | null>(null)
  const [preview, setPreview] = useState('')
  const [report, setReport] = useState('')
  const logRef = useRef<HTMLPreElement>(null)
  const processing = connected ? state?.dataProcess ?? fallback : fallback
  const job = selectedId && selectedId !== processing?.current?.id ? historical : processing?.current
  const busy = Boolean(processing?.busy || submitting)
  const logs = (job?.log_tail ?? []).join('\n')
  const selected = catalog?.sessions.find(s => s.path === source)
  const exports = action === 'full' || action === 'export'

  async function refresh() {
    try {
      const [sessions, jobs] = await Promise.all([api.dataSessions(), api.dataJobs()])
      if (sessions.ok) {
        setCatalog(sessions.data)
        setSource(previous => previous || sessions.data.sessions.find(s => s.kind === 'session')?.path || '')
      } else pushToast(sessions.message, 'error')
      if (jobs.ok) setFallback(jobs.data)
    } catch (error) { pushToast(String(error), 'error') }
  }
  useEffect(() => { void refresh() }, [])
  useEffect(() => { if (logRef.current) logRef.current.scrollTop = logRef.current.scrollHeight }, [logs])
  useEffect(() => { setPreview(''); setReport('') }, [job?.id])
  useEffect(() => {
    if (!selectedId) return
    let active = true
    void api.dataJob(selectedId).then(response => {
      if (active && response.ok) setHistorical(response.data)
    }).catch(error => pushToast(String(error), 'error'))
    return () => { active = false }
  }, [selectedId])

  async function start() {
    setSubmitting(true)
    try {
      const response = await api.dataStart({ action, source: source.trim(), target, image_size: imageSize,
        camera_map: cameraMap.replace(/\s/g, ''), episodes: episodes.replace(/\s/g, ''), quantify, repair, reference: reference.trim() })
      if (response.ok) {
        setSelectedId(''); setHistorical(null)
        setFallback(previous => ({ busy: true, current: response.data, history: previous?.history ?? [] }))
        pushToast(response.message, 'success')
      } else pushToast(response.message, 'error')
    } catch (error) { pushToast(String(error), 'error') }
    finally { setSubmitting(false) }
  }
  async function cancel() {
    if (!confirm('确认停止数据处理任务？已完成的结果会保留，原始 raw 不变。')) return
    try {
      const response = await api.dataCancel()
      pushToast(response.message, response.ok ? 'info' : 'error')
    } catch (error) { pushToast(String(error), 'error') }
  }
  async function showArtifact(artifact: DataArtifact) {
    if (!job) return
    const url = api.dataFileUrl(job.id, artifact.path)
    if (artifact.kind === 'preview') { setPreview(url); setReport(''); return }
    try {
      const response = await fetch(url)
      if (!response.ok) throw new Error(`读取报告失败：${response.status}`)
      const text = await response.text()
      setReport(artifact.path.endsWith('.json') ? JSON.stringify(JSON.parse(text), null, 2) : text)
      setPreview('')
    } catch (error) { pushToast(String(error), 'error') }
  }

  return <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
    <section style={card}>
      <div style={heading}>数据处理</div>
      <div style={hint}>每次任务建立独立工作副本，原始 raw 保留。遇到失败就停止，不自动移动失败段；清洗默认保护操作意图。</div>
      <div style={row}>
        <label style={label}>已有数据<select aria-label="已有数据" style={input} value={selected ? source : ''} disabled={busy} onChange={e => setSource(e.target.value)}>
          <option value="">选择 session，或填写下方路径</option>
          {catalog?.sessions.map(s => <option key={s.path} value={s.path}>{s.name} · {s.kind === 'session' ? `${s.episodes} 段` : 'LeRobot 数据集'}</option>)}
        </select></label>
        <button style={button()} onClick={() => void refresh()}>刷新列表</button>
        {state?.dataCollect?.session && <button style={button()} disabled={busy} onClick={() => setSource(`raw/${state.dataCollect!.session}`)}>使用当前采集 session</button>}
      </div>
      <label style={label}>输入目录（服务器路径）<input aria-label="输入目录" style={input} value={source} disabled={busy} onChange={e => setSource(e.target.value)} placeholder="raw/pick_place 或数据根目录内的绝对路径" /></label>
      <div style={hint}>数据根目录：{catalog?.data_root ?? '加载中'}{selected?.cameras.length ? ` · 配置相机：${selected.cameras.join(', ')}` : ''}</div>
      <div style={row}>
        <label style={{ ...label, flex: 2 }}>处理方式<select aria-label="处理方式" style={input} value={action} disabled={busy} onChange={e => setAction(e.target.value as DataProcessAction)}>
          {actions.map(a => <option key={a.value} value={a.value}>{a.label}</option>)}
        </select></label>
        {exports && <label style={label}>导出格式<select aria-label="导出格式" style={input} value={target} disabled={busy} onChange={e => setTarget(e.target.value as typeof target)}>
          <option value="openpi">OpenPI · LeRobot v2.1</option><option value="act">ACT · LeRobot v3</option><option value="both">OpenPI + ACT</option>
        </select></label>}
      </div>
      {action === 'full' && <div style={row}>
        <label><input type="checkbox" checked={quantify} disabled={busy} onChange={e => setQuantify(e.target.checked)} /> 生成指令与状态诊断</label>
        <label><input type="checkbox" checked={repair} disabled={busy} onChange={e => setRepair(e.target.checked)} /> 执行保守清洗</label>
      </div>}
      {action === 'verify' && <label style={label}>清洗前的源 session<input aria-label="清洗前的源目录" style={input} value={reference} disabled={busy} onChange={e => setReference(e.target.value)} /></label>}
      <details><summary style={{ cursor: 'pointer', color: '#93c5fd' }}>段号、相机映射与图像尺寸</summary><div style={{ ...row, marginTop: 10 }}>
        <label style={label}>段号（空 = 全部）<input aria-label="段号" style={input} value={episodes} disabled={busy} onChange={e => setEpisodes(e.target.value)} placeholder="0,24,49,74,99" /></label>
        <label style={label}>历史相机映射<input aria-label="相机映射" style={input} value={cameraMap} disabled={busy} onChange={e => setCameraMap(e.target.value)} placeholder="video8=base,video0=left_wrist" /></label>
        <label style={label}>导出图像边长<input aria-label="图像尺寸" style={input} type="number" min={0} max={2048} value={imageSize} disabled={busy} onChange={e => setImageSize(Number(e.target.value))} /></label>
      </div><p style={hint}>映射只应用于工作副本；0 表示原生分辨率。单步清洗、预览和导出需要已有 aligned。</p></details>
      {exports && target !== 'openpi' && <div style={hint}>{catalog?.deep_check_available ? 'ACT 将执行结构检查、实际数据加载和预处理验证。' : '当前未配置 ACT 检查环境，仅执行结构检查。'}</div>}
      {catalog && !catalog.python_available && <div style={{ color: '#f87171' }}>数据处理环境不存在，请配置 ASTRAL_DATA_PYTHON。</div>}
      <div style={row}>
        <button style={button('#2563eb')} disabled={busy || !source.trim() || !catalog?.python_available || (action === 'verify' && !reference.trim())} onClick={() => void start()}>开始处理</button>
        <button style={button('#b91c1c')} disabled={!processing?.busy || processing.current?.status === 'stopping'} onClick={() => void cancel()}>停止任务</button>
        <span style={hint}>任务和结果保存在 web_jobs 下；训练单独启动。</span>
      </div>
    </section>
    <section style={card}>
      <div style={row}><div style={heading}>任务与结果</div><select aria-label="任务历史" style={input} value={selectedId} onChange={e => setSelectedId(e.target.value)}>
        <option value="">当前 / 最近任务</option>{processing?.history.map(item => <option key={item.id} value={item.id}>{item.id} · {labels[item.status] ?? item.status}</option>)}
      </select></div>
      {!job ? <div style={hint}>尚无处理任务。选择数据后开始处理。</div> : <>
        <div style={row}><strong style={{ color: colors[job.status] ?? '#9ca3af' }}>{labels[job.status] ?? job.status}</strong><span>{job.id}</span><span style={hint}>{Math.max(0, Math.round((job.finished_at ?? Date.now()/1000)-job.started_at))} 秒</span></div>
        <div style={pathStyle}>输入：{job.source}<br />结果目录：{job.directory}</div>
        <div style={{ display: 'grid', gap: 6 }}>{job.steps.map((step, i) => <div key={i} style={{ ...row, padding: '6px 10px', background: '#111827', borderRadius: 5 }}>
          <span style={{ color: '#6b7280', minWidth: 22 }}>{i+1}</span><span style={{ flex: 1 }}>{step.name}</span><span style={{ color: colors[step.status] ?? '#9ca3af' }}>{labels[step.status] ?? step.status}</span>
        </div>)}</div>
        {job.error && <div role="alert" style={{ color: '#fca5a5', background: '#450a0a', padding: 10, borderRadius: 6 }}>{job.error}</div>}
        {job.validation?.map(v => <div key={v.stage} style={row}><b>{v.stage}</b><span>共 {v.total} 段</span><span style={{ color: '#4ade80' }}>通过 {v.pass}</span><span style={{ color: '#fbbf24' }}>告警 {v.warn}</span><span style={{ color: '#f87171' }}>失败 {v.fail}</span></div>)}
        <div style={row}>
          <button style={button()} disabled={job.status === 'running' || job.action === 'upgrade-v3'} onClick={() => { setSource(`${job.directory}/${job.artifacts.some(a => a.path.startsWith('cleaned/')) ? 'cleaned' : 'working'}`); setSelectedId(''); setAction('validate') }}>用工作结果继续处理</button>
          <a style={link} href={api.dataFileUrl(job.id, 'job.log')} target="_blank" rel="noreferrer">完整日志</a>
          <button style={button()} onClick={() => {
            if (!navigator.clipboard) { pushToast(job.directory, 'info', 10000); return }
            void navigator.clipboard.writeText(job.directory).then(() => pushToast('结果路径已复制', 'success')).catch(() => pushToast(job.directory, 'info', 10000))
          }}>复制结果路径</button>
        </div>
        <pre ref={logRef} style={logStyle}>{logs || '等待任务日志…'}</pre>
        <div style={row}>{job.artifacts.filter(a => a.path !== 'job.log').map(a => <button key={a.path} style={button(a.kind === 'preview' ? '#075985' : '#374151')} onClick={() => void showArtifact(a)}>{a.kind === 'preview' ? '图像预览 · ' : ''}{a.name}</button>)}</div>
        {preview && <iframe title="数据图像与轨迹预览" src={preview} sandbox="" style={{ width: '100%', height: 650, border: '1px solid #374151', borderRadius: 8 }} />}
        {report && <pre style={{ ...logStyle, maxHeight: 500 }}>{report}</pre>}
      </>}
    </section>
  </div>
}
const card: React.CSSProperties = { background: '#1f2937', border: '1px solid #374151', borderRadius: 10, padding: 16, display: 'flex', flexDirection: 'column', gap: 12 }
const heading: React.CSSProperties = { fontSize: 16, fontWeight: 700 }
const row: React.CSSProperties = { display: 'flex', flexWrap: 'wrap', gap: 12, alignItems: 'center', fontSize: 13 }
const label: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: 5, flex: 1, minWidth: 200, fontSize: 13, color: '#d1d5db' }
const input: React.CSSProperties = { background: '#111827', border: '1px solid #4b5563', borderRadius: 6, padding: '8px 10px', color: '#e5e7eb', fontSize: 13, minWidth: 160, maxWidth: '100%' }
const hint: React.CSSProperties = { color: '#9ca3af', fontSize: 12, lineHeight: 1.6 }
const pathStyle: React.CSSProperties = { ...hint, overflowWrap: 'anywhere' }
const logStyle: React.CSSProperties = { margin: 0, padding: 12, background: '#0b0f17', color: '#cbd5e1', fontSize: 12, lineHeight: 1.6, maxHeight: 260, overflow: 'auto', whiteSpace: 'pre-wrap', overflowWrap: 'anywhere', borderRadius: 6 }
const link: React.CSSProperties = { color: '#93c5fd', fontSize: 13 }
function button(color = '#374151'): React.CSSProperties { return { background: color, color: '#e5e7eb', border: '1px solid #4b5563', borderRadius: 6, padding: '7px 12px', fontSize: 13, cursor: 'pointer' } }
