import { useEffect, useRef, useState } from 'react'

type Port = { direction: string; semantic: string; dim: number; unit: string; axis_names?: string[] }
type Component = { plugin_id: string; plugin_version: string; kind: string; ports: Record<string, Port>; joints?: { name: string; unit: string; lower?: number; upper?: number }[]; controller_types?: { inputs?: string[]; outputs?: string[] }; backend_variants?: Record<string, { available?: boolean }>; configuration_schema?: { properties?: { side?: { enum?: string[] }; device_id?: { type?: string } } } }
type Instance = { plugin: string; version: string; backend?: string; role?: string; parameters?: Record<string, unknown> }
type Binding = { id: string; source: string; target: string; deadman: string; controller: string; parameters: Record<string, unknown> }
type Assembly = { assembly_version: number; assembly_id: string; revision: number; instances: Record<string, Instance>; mounts: unknown[]; bindings: Binding[]; control_groups: Record<string, { members: string[]; fault_policy: string }>; capture: { state: string[]; action: string[]; images: Record<string, string>; auxiliary: string[]; action_source: string; dataset_fps: number } }
type Resolved = { assembly_hash: string; status?: Record<string, boolean | null>; feature_contract: { state_dim: number; action_dim: number; state_features: { name: string; unit: string }[]; action_features: { name: string; unit: string }[]; cameras: Record<string, unknown> }; diagnostics: unknown[] }
type Session = { assembly_id?: string; revision?: number; session_id: string; status: string; components?: Record<string, unknown>; inputs?: Record<string, unknown>; recording?: boolean }
type Endpoint = { ref: string; instance: string; port: Port }
type BoardKind = 'input' | 'robot' | 'sensor'

const initial = (): Assembly => ({ assembly_version: 1, assembly_id: 'my_workcell', revision: 1, instances: {}, mounts: [], bindings: [], control_groups: {}, capture: { state: [], action: [], images: {}, auxiliary: [], action_source: 'command', dataset_fps: 30 } })
const controllers: Record<string, [string[], string[]]> = {
  'morla.pinch_filtered': [['hand_landmarks'], ['scalar', 'joint_targets']],
  'morla.trigger_filtered': [['scalar'], ['scalar', 'joint_targets']],
  'morla.pinch_scalar': [['hand_landmarks'], ['scalar', 'joint_targets']],
  'morla.cartesian_delta_ik': [['pose6d'], ['joint_targets']],
  'morla.gamepad_cartesian_jog': [['axes_buttons'], ['joint_targets']],
  'morla.joint_jog': [['axes_buttons', 'joint_targets'], ['joint_targets']],
  'morla.scalar_map': [['scalar'], ['scalar', 'joint_targets']],
  'morla.xhand.retarget': [['hand_landmarks'], ['joint_targets']],
  'morla.wuji.retarget': [['hand_landmarks'], ['joint_targets']],
  'morla.identity': [['joint_targets', 'scalar', 'numeric'], ['joint_targets', 'scalar', 'numeric']],
}
const labels: Record<string, string> = {
  'morla.input.vader5': '飞智 Vader 5 手柄', 'morla.input.quest': 'Quest 3（手柄/手部）',
  'morla.input.gamepad': '通用游戏手柄', 'morla.input.xbox': 'Xbox 手柄', 'morla.input.glove': '数据手套',
  'morla.input.mobile': '手机体感输入', 'morla.input.standard': '标准输入', 'morla.fake.input': '虚拟输入',
  'morla.dobot.cr5': 'Dobot CR5 机械臂', 'morla.astral.arm': 'Astral 机械臂', 'morla.nero.arm': 'Nero 机械臂',
  'morla.astral.waist': 'Astral 腰部', 'morla.astral.head': 'Astral 头部', 'morla.fake.arm': '虚拟机械臂',
  'morla.ros2_control.mock': 'ROS2 Control 模拟臂', 'morla.astral.gripper': 'Astral 夹爪', 'morla.jodell.epg50_060': 'Jodell EPG50 夹爪',
  'morla.wuji.hand': 'Wuji 灵巧手', 'morla.xhand.hand': 'XHand 灵巧手', 'morla.loophand.compact': 'Loophand 灵巧手',
  'morla.camera': '相机', 'morla.camera.ros_image': 'ROS 图像相机', 'morla.camera.uvc': 'USB 相机',
  'morla.fake.camera': '虚拟相机', 'morla.fake.sensor': '虚拟传感器',
}
const baseNames: Record<string, string> = {
  'morla.input.vader5': 'pad', 'morla.input.quest': 'quest',
  'morla.input.gamepad': 'gamepad', 'morla.input.xbox': 'xbox', 'morla.input.glove': 'glove',
  'morla.input.mobile': 'phone', 'morla.input.standard': 'input', 'morla.fake.input': 'fake_input',
  'morla.dobot.cr5': 'arm', 'morla.astral.arm': 'arm', 'morla.nero.arm': 'arm', 'morla.fake.arm': 'arm',
  'morla.ros2_control.mock': 'mock_arm', 'morla.astral.waist': 'waist', 'morla.astral.head': 'head',
  'morla.astral.gripper': 'gripper', 'morla.jodell.epg50_060': 'epg', 'morla.wuji.hand': 'hand',
  'morla.xhand.hand': 'hand', 'morla.loophand.compact': 'hand',
  'morla.camera': 'camera', 'morla.camera.ros_image': 'image', 'morla.camera.uvc': 'usb_camera',
  'morla.fake.camera': 'camera', 'morla.fake.sensor': 'sensor',
}
const backendNames: Record<string, string> = { fake: 'fake · 虚拟', sim: 'sim · 仿真', real: 'real · 实机' }
const sessionNames: Record<string, string> = {
  ACTIVE: '运行中 · 已授权运动', READY_DISARMED: '就绪 · 未授权运动', STOPPED: '已停止',
  FAULT: '故障', ESTOP: '急停', CONNECTING: '连接中', PREPARING: '准备中',
}
const pluginLabel = (id: string) => labels[id] || id
const backendLabel = (id: string) => backendNames[id] || id
const sessionLabel = (status: string) => sessionNames[status] || status
const sessionTone = (status: string) => status === 'ACTIVE' ? '#22c55e' : status === 'READY_DISARMED' ? '#f59e0b' : ['FAULT', 'ESTOP'].includes(status) ? '#ef4444' : status === 'STOPPED' ? '#6b7280' : '#60a5fa'

async function request<T>(path: string, body?: unknown): Promise<T> {
  const response = await fetch('/api/v2' + path, { method: body === undefined ? 'GET' : 'POST', headers: { 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body) })
  const result = await response.json()
  if (!response.ok) {
    const error = new Error(typeof result.detail === 'string' ? result.detail : JSON.stringify(result.detail)) as Error & { status?: number }
    error.status = response.status
    throw error
  }
  return result
}
function SimPreview({ sessionId, cameraId }: { sessionId: string; cameraId: string }) {
  const image = useRef<HTMLImageElement | null>(null)
  const drag = useRef<{ action: 'orbit' | 'pan'; x: number; y: number } | null>(null)
  const queued = useRef<{ action: string; dx: number; dy: number } | null>(null)
  const timer = useRef<number | null>(null)
  // Pointer moves arrive faster than the renderer and the preview stream; keep
  // one request per ~30 ms and let the server queue the remainder.
  const flush = () => {
    timer.current = null
    const item = queued.current
    queued.current = null
    if (item) void request(`/sessions/${sessionId}/cameras/${cameraId}/view`, item).catch(() => {})
  }
  const queue = (action: string, dx: number, dy: number) => {
    if (!dx && !dy) return
    const item = queued.current
    if (item && item.action === action) { item.dx += dx; item.dy += dy }
    else { if (timer.current !== null) flush(); queued.current = { action, dx, dy } }
    if (timer.current === null) timer.current = window.setTimeout(flush, 33)
  }
  useEffect(() => {
    const node = image.current
    if (!node) return
    // React registers wheel handlers as passive; zoom needs preventDefault so
    // the page does not scroll while the view zooms.
    const wheel = (event: WheelEvent) => { event.preventDefault(); queue('zoom', 0, event.deltaY) }
    node.addEventListener('wheel', wheel, { passive: false })
    return () => node.removeEventListener('wheel', wheel)
  }, [sessionId, cameraId])
  const down = (event: React.PointerEvent<HTMLImageElement>) => {
    const pan = event.button === 2 || event.button === 1 || event.shiftKey
    if (event.button !== 0 && !pan) return
    event.preventDefault()
    drag.current = { action: pan ? 'pan' : 'orbit', x: event.clientX, y: event.clientY }
    event.currentTarget.setPointerCapture(event.pointerId)
  }
  const move = (event: React.PointerEvent<HTMLImageElement>) => {
    if (!drag.current) return
    queue(drag.current.action, event.clientX - drag.current.x, event.clientY - drag.current.y)
    drag.current.x = event.clientX; drag.current.y = event.clientY
  }
  const up = (event: React.PointerEvent<HTMLImageElement>) => {
    if (!drag.current) return
    drag.current = null
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
  }
  return <figure style={{ margin: 0, ...subCard }}>
    <figcaption style={{ ...mutedSmall, display: 'flex', alignItems: 'center', gap: 8 }}>
      <span>MuJoCo · {cameraId}</span>
      <span style={{ flex: 1 }} />
      <button type="button" style={{ ...ghostBtn, padding: '3px 10px', fontSize: 12 }} onClick={() => void request(`/sessions/${sessionId}/cameras/${cameraId}/view`, { action: 'reset' }).catch(() => {})}>复位视角</button>
    </figcaption>
    <img ref={image} alt={`${cameraId} 仿真画面`} src={`/api/v2/sessions/${sessionId}/cameras/${cameraId}/stream`} draggable={false}
      onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up}
      onContextMenu={event => event.preventDefault()}
      style={{ maxWidth: '100%', width: 640, borderRadius: 8, touchAction: 'none', cursor: 'grab', userSelect: 'none' }} />
    <p style={{ ...mutedSmall, margin: 0 }}>左键拖动旋转 · 右键或 Shift 拖动平移 · 滚轮缩放</p>
  </figure>
}
function JsonField({ label, value, apply }: { label: string; value: unknown; apply: (next: Record<string, unknown>) => void }) {
  const serialized = JSON.stringify(value, null, 2)
  const [draft, setDraft] = useState(serialized)
  const [error, setError] = useState('')
  useEffect(() => { setDraft(serialized) }, [serialized])
  return <div style={jsonField}>
    <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
      <span style={fieldLabel}>{label}</span>
      <span style={{ flex: 1 }} />
      <button style={ghostBtn} onClick={() => { try { const parsed = JSON.parse(draft); if (!parsed || Array.isArray(parsed) || typeof parsed !== 'object') throw new Error('请输入 JSON 对象'); apply(parsed); setError('已应用') } catch (e) { setError((e as Error).message) } }}>应用{label}</button>
    </div>
    <textarea aria-label={label} rows={4} style={editorStyle} value={draft} onChange={e => { setDraft(e.target.value); setError('') }} />
    <span role="status" style={{ fontSize: 12, color: error === '已应用' ? '#86efac' : '#fca5a5' }}>{error}</span>
  </div>
}
function Board({ title, subtitle, accent, count, children }: { title: string; subtitle: string; accent: string; count: number; children: React.ReactNode }) {
  return <div style={{ ...card, borderColor: accent + '3d' }}>
    <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
      <span style={{ width: 8, height: 8, borderRadius: '50%', background: accent, boxShadow: `0 0 0 4px ${accent}22` }} />
      <h3 style={boardTitle}>{title}</h3>
      <span style={{ ...countBadge, color: accent, background: accent + '18', borderColor: accent + '44' }}>{count}</span>
    </div>
    <p style={boardSub}>{subtitle}</p>
    {children}
  </div>
}
export function ComposableTab() {
  const [catalog, setCatalog] = useState<Component[]>([])
  const [assembly, setAssembly] = useState<Assembly>(initial)
  const [deployment, setDeployment] = useState<Record<string, unknown>>({})
  const [resolved, setResolved] = useState<Resolved | null>(null)
  const [preflight, setPreflight] = useState<unknown>(null)
  const [session, setSession] = useState<Session | null>(null)
  const [savedRevision, setSavedRevision] = useState(0)
  const [savedHash, setSavedHash] = useState('')
  const [saved, setSaved] = useState<{ assembly_id: string; revision: number }[]>([])
  const [message, setMessage] = useState('选择组件，建立控制绑定，再预览和保存版本。')
  const [busy, setBusy] = useState(false)
  const [pick, setPick] = useState<Record<BoardKind, string>>({ input: 'morla.input.vader5', robot: 'morla.nero.arm', sensor: 'morla.camera' })
  const [pairLink, setPairLink] = useState('')
  const [editor, setEditor] = useState('')
  useEffect(() => { request<{ components: Component[] }>('/components').then(x => setCatalog(x.components)).catch(e => setMessage(e.message)); request<typeof saved>('/assemblies').then(setSaved).catch(() => {}) }, [])
  useEffect(() => { if (!session) return; const timer = window.setInterval(() => { request<Session>(`/sessions/${session.session_id}/state`).then(setSession).catch((e: Error & { status?: number }) => { if (e.status === 404) { setSession(null); setMessage('会话已结束或服务已重启，请重新启动会话。') } else setMessage(e.message) }) }, 500); return () => clearInterval(timer) }, [session?.session_id])
  useEffect(() => { void request<Session | null>('/active-session').then(async active => {
    if (!active || active.status === 'STOPPED' || !active.assembly_id || !active.revision) return
    const item = await request<{ assembly: Assembly; deployment?: Record<string, unknown>; resolved: Resolved }>(`/assemblies/${active.assembly_id}/revisions/${active.revision}`)
    setAssembly({ ...initial(), ...item.assembly }); setDeployment(item.deployment || {}); setResolved(item.resolved)
    setSavedRevision(active.revision); setSavedHash(item.resolved.assembly_hash); setSession(active)
    setMessage('已连接当前运行会话。')
  }).catch(e => setMessage(e.message)) }, [])
  const descriptor = (instance: Instance) => catalog.find(x => x.plugin_id === instance.plugin)
  const endpoints = Object.entries(assembly.instances).flatMap(([id, item]) => Object.entries(descriptor(item)?.ports || {}).map(([name, port]) => ({ ref: id + '.' + name, instance: id, port })))
  const inputEndpoints = endpoints.filter(x => descriptor(assembly.instances[x.instance])?.kind === 'input' && x.port.direction === 'output')
  const targets = endpoints.filter(x => descriptor(assembly.instances[x.instance])?.kind === 'actuator' && x.port.direction === 'input')
  const observations = endpoints.filter(x => x.port.direction === 'output' && x.port.semantic !== 'image')
  const images = endpoints.filter(x => x.port.direction === 'output' && x.port.semantic === 'image')
  const compatible = (source: string, target: string) => { const sp = endpoints.find(x => x.ref === source)?.port, tp = endpoints.find(x => x.ref === target)?.port; const s = sp?.semantic, t = tp?.semantic; return Object.entries({ ...controllers, ...Object.fromEntries(catalog.filter(c => c.kind === 'controller').map(c => [c.plugin_id, [c.controller_types?.inputs || [], c.controller_types?.outputs || []]])) }).filter(([id, [ins, outs]]) => !!s && !!t && ins.includes(s) && outs.includes(t) && (!['morla.scalar_map', 'morla.trigger_filtered', 'morla.pinch_scalar', 'morla.pinch_filtered'].includes(id) || tp?.dim === 1) && (id !== 'morla.identity' || (sp?.dim === tp?.dim && sp?.unit === tp?.unit))).map(([id]) => id) }
  const mutate = (next: Assembly) => { setAssembly(next); setResolved(null); setPreflight(null); setPairLink('') }
  const run = async (fn: () => Promise<void>) => { setBusy(true); try { await fn() } catch (e) { setMessage((e as Error).message) } finally { setBusy(false) } }
  const members = (kind: BoardKind) => Object.entries(assembly.instances).filter(([, item]) => {
    const k = descriptor(item)?.kind
    if (kind === 'input') return k === 'input'
    if (kind === 'robot') return k === 'actuator'
    return k !== 'input' && k !== 'actuator' && k !== 'controller'
  })
  const catalogFor = (kind: BoardKind) => catalog.filter(c => kind === 'input' ? c.kind === 'input' : kind === 'robot' ? c.kind === 'actuator' : c.kind !== 'input' && c.kind !== 'actuator' && c.kind !== 'controller')
  const pickFor = (kind: BoardKind) => { const list = catalogFor(kind); return list.some(x => x.plugin_id === pick[kind]) ? pick[kind] : list[0]?.plugin_id || '' }
  function uniqueId(d: Component, next: Assembly) {
    const raw = (baseNames[d.plugin_id] || d.plugin_id.split('.').pop() || 'component').toLowerCase().replace(/[^a-z0-9_-]/g, '_')
    const base = /^[a-z]/.test(raw) ? raw : 'c_' + raw
    if (!next.instances[base]) return base
    let ordinal = 2
    while (next.instances[base + '_' + ordinal]) ordinal++
    return base + '_' + ordinal
  }
  function defaultBackend(d: Component) {
    const variants = Object.keys(d.backend_variants || {})
    if (!variants.length) return 'fake'
    if (variants.includes('fake')) return 'fake'
    return variants.find(id => d.backend_variants?.[id]?.available !== false) || variants[0]
  }
  function addComponent(pluginId: string) {
    const d = catalog.find(x => x.plugin_id === pluginId)
    if (!d) { setMessage('组件目录尚未加载，请稍后重试。'); return }
    const next = structuredClone(assembly)
    const id = uniqueId(d, next)
    next.instances[id] = { plugin: pluginId, version: d.plugin_version, backend: defaultBackend(d) }
    if (d.kind === 'actuator') { next.capture.state.push(id + '.position'); next.capture.action.push(id + (d.ports.closure ? '.closure' : '.position_target')) }
    if (d.ports.rgb) next.capture.images[id] = id + '.rgb'
    mutate(next)
    setMessage(`已添加 ${id}（${pluginLabel(pluginId)}）。选择运行后端，再建立输入与控制绑定。`)
  }
  function remove(id: string) {
    const next = structuredClone(assembly); delete next.instances[id]
    next.capture.state = next.capture.state.filter(x => !x.startsWith(id + '.')); next.capture.action = next.capture.action.filter(x => !x.startsWith(id + '.')); next.capture.auxiliary = (next.capture.auxiliary || []).filter(x => !x.startsWith(id + '.'))
    next.capture.images = Object.fromEntries(Object.entries(next.capture.images).filter(([, v]) => !v.startsWith(id + '.'))); next.bindings = next.bindings.filter(x => ![x.source, x.target, x.deadman].some(p => p.startsWith(id + '.')))
    next.mounts = next.mounts.filter(m => !JSON.stringify(m).includes('"' + id + '.'))
    next.control_groups = Object.fromEntries(Object.entries(next.control_groups).map(([key, group]) => [key, { ...group, members: group.members.filter(member => member !== id) }]).filter(([, group]) => typeof group !== 'string' && group.members.length)) as Assembly['control_groups']
    mutate(next)
  }
  const inputs = members('input')
  function addBinding() {
    const source = inputEndpoints.find(x => x.ref.endsWith('.axes')) || inputEndpoints[0], target = targets.find(x => x.ref.endsWith('.joint_target')) || targets[0]
    if (!source || !target) { setMessage('先添加一个输入设备和一个执行器。'); return }
    const deadman = inputEndpoints.find(x => x.instance === source.instance && x.ref.endsWith('.hold_to_control')) || inputEndpoints.find(x => x.instance === source.instance && x.port.semantic === 'scalar')
    const choices = compatible(source.ref, target.ref)
    const controller = choices.includes('morla.joint_jog') ? 'morla.joint_jog' : choices[0] || ''
    let ordinal = assembly.bindings.length + 1; while (assembly.bindings.some(x => x.id === 'binding_' + ordinal)) ordinal++
    mutate({ ...assembly, bindings: [...assembly.bindings, { id: 'binding_' + ordinal, source: source.ref, target: target.ref, deadman: deadman?.ref || '', controller, parameters: controller === 'morla.scalar_map' ? { input_range: [0, 1], output_range: [descriptor(assembly.instances[target.instance])?.joints?.[0]?.lower ?? 0, descriptor(assembly.instances[target.instance])?.joints?.[0]?.upper ?? 1] } : { axes_map: Array.from({ length: target.port.dim }, (_, i) => source.port.axis_names?.[i] ?? null), speed: .25 } }] })
  }
  function updateBinding(index: number, key: 'source' | 'target' | 'deadman' | 'controller', value: string) {
    const next = structuredClone(assembly), binding = next.bindings[index]; binding[key] = value
    if (key === 'source') binding.deadman = inputEndpoints.find(x => x.instance === value.split('.')[0] && x.ref.endsWith('.hold_to_control'))?.ref || inputEndpoints.find(x => x.instance === value.split('.')[0] && x.port.semantic === 'scalar')?.ref || ''
    if (key === 'source' || key === 'target') { const choices = compatible(binding.source, binding.target); if (!choices.includes(binding.controller)) binding.controller = choices[0] || '' }
    mutate(next)
  }
  function toggleCapture(kind: 'state' | 'action', ref: string, checked: boolean) { const next = structuredClone(assembly); next.capture[kind] = next.capture[kind].filter(x => x !== ref && !(kind === 'action' && checked && x.split('.')[0] === ref.split('.')[0])); if (checked) next.capture[kind].push(ref); mutate(next) }
  function cameraKey(ref: string, old: string, nextKey: string) { if (!/^[a-z][a-z0-9_-]*$/.test(nextKey) || (nextKey !== old && nextKey in assembly.capture.images)) { setMessage('相机数据键必须唯一，使用小写字母、数字、下划线。'); return } const next = structuredClone(assembly); delete next.capture.images[old]; next.capture.images[nextKey] = ref; mutate(next) }
  const option = (item: Endpoint) => <option key={item.ref} value={item.ref}>{item.ref} · {item.port.dim}维 {item.port.unit}</option>
  const operate = (operation: string) => {
    if (operation === 'estop' && !window.confirm('确认向当前会话发送急停？')) return
    if (operation === 'stop' && !window.confirm('确认停止当前会话？停止后输入与采集进程会被回收。')) return
    void run(async () => { const result = await request<unknown>(`/sessions/${session!.session_id}/operations`, { operation, request_id: crypto.randomUUID() }); setMessage(JSON.stringify(result)); setSession(await request<Session>(`/sessions/${session!.session_id}/state`)) })
  }
  const instanceCard = (id: string, item: Instance, accent: string, extra?: React.ReactNode) => {
    const d = descriptor(item)
    const variants = Object.entries(d?.backend_variants || { fake: {} })
    const sideEnum = d?.configuration_schema?.properties?.side?.enum
    return <div key={id} style={{ ...instanceCardStyle, borderLeft: `3px solid ${accent}` }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, flexWrap: 'wrap' }}>
        <strong style={{ fontSize: 14 }}>{id}</strong>
        <span title={item.plugin} style={{ ...tagStyle, color: accent, borderColor: accent + '55', background: accent + '14' }}>{pluginLabel(item.plugin)}</span>
        {!!d?.joints?.length && <span style={mutedSmall}>{d.joints.length} 自由度</span>}
        <span style={{ flex: 1 }} />
        <button style={iconBtn} aria-label={'移除' + id} title={'移除 ' + id} onClick={() => remove(id)}>✕</button>
      </div>
      <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', alignItems: 'end' }}>
        <label style={miniField}>运行后端
          <select aria-label={id + '运行后端'} style={input} value={item.backend || 'fake'} onChange={e => mutate({ ...assembly, instances: { ...assembly.instances, [id]: { ...item, backend: e.target.value } } })}>
            {variants.map(([backend, status]) => <option key={backend} value={backend}>{backendLabel(backend)}{status.available === false ? '（不可用）' : ''}</option>)}
          </select>
        </label>
        {sideEnum && <label style={miniField}>{d?.kind === 'input' ? '使用手' : '安装侧'}
          <select aria-label={id + (d?.kind === 'input' ? '使用手' : '安装侧')} style={input} value={String(item.parameters?.side || '')} onChange={e => { const parameters = { ...item.parameters }; if (e.target.value) parameters.side = e.target.value; else delete parameters.side; mutate({ ...assembly, instances: { ...assembly.instances, [id]: { ...item, parameters } } }) }}>
            <option value="">默认</option>
            {sideEnum.map(value => <option key={value} value={value}>{(value === 'both' ? '双手' : value === 'left' ? (d?.kind === 'input' ? '左手' : '左侧') : (d?.kind === 'input' ? '右手' : '右侧'))}</option>)}
          </select>
        </label>}
        {d?.kind === 'input' && item.backend === 'real' && d?.configuration_schema?.properties?.device_id &&
          <label style={miniField}>设备标识
            <input aria-label={id + '设备标识'} style={{ ...input, width: 150 }} placeholder="real 必填" value={String(item.parameters?.device_id || '')} onChange={e => { const parameters = { ...item.parameters }; if (e.target.value) parameters.device_id = e.target.value; else delete parameters.device_id; mutate({ ...assembly, instances: { ...assembly.instances, [id]: { ...item, parameters } } }) }} />
          </label>}
        {extra}
      </div>
    </div>
  }
  const addPicker = (kind: BoardKind, accent: string) => <div style={{ display: 'flex', gap: 8 }}>
    <select aria-label={kind + '组件'} style={{ ...input, flex: 1 }} value={pickFor(kind)} onChange={e => setPick(p => ({ ...p, [kind]: e.target.value }))}>
      {catalogFor(kind).map(c => <option key={c.plugin_id} value={c.plugin_id}>{pluginLabel(c.plugin_id)}{c.joints?.length ? ` · ${c.joints.length} DOF` : ''}</option>)}
    </select>
    <button style={{ ...primaryBtn, background: accent, borderColor: accent }} disabled={busy || !pickFor(kind)} onClick={() => addComponent(pickFor(kind))}>＋ 添加</button>
  </div>
  const emptyBoard = (text: string) => <p style={emptyStyle}>{text}</p>
  return <section style={{ display: 'grid', gap: 16 }}>
    <div style={{ ...card, background: 'linear-gradient(135deg,#101b2e,#0d1524)' }}>
      <div style={{ minWidth: 260 }}>
        <h2 style={{ margin: 0, fontSize: 20 }}>组合机器人系统</h2>
        <p style={{ ...muted, margin: '6px 0 0' }}>按「输入 → 机器人本体 / 执行器 → 传感器」搭好系统，建立绑定后校验、保存并启动。仿真可离线验证；真机需独立部署绑定和现场验收。</p>
      </div>
      <div style={{ display: 'flex', gap: 12, flexWrap: 'wrap', alignItems: 'end' }}>
        <label style={field}>配置名称 <input style={input} value={assembly.assembly_id} onChange={e => { setSavedRevision(0); mutate({ ...assembly, assembly_id: e.target.value, revision: 1 }) }} /></label>
        <label style={field}>打开已保存配置 <select aria-label="打开已保存配置" style={input} defaultValue="" onChange={e => void run(async () => { const item = await request<{ assembly: Assembly; deployment?: Record<string, unknown>; resolved: Resolved }>(`/assemblies/${e.target.value}/revisions/${saved.find(x => x.assembly_id === e.target.value)!.revision}`); setAssembly(item.assembly); setDeployment(item.deployment || {}); setResolved(item.resolved); setSavedRevision(item.assembly.revision); setSavedHash(item.resolved.assembly_hash); setMessage('已打开保存版本，可启动或编辑下一版本。') })}><option value="" disabled>选择配置</option>{saved.map(x => <option key={x.assembly_id} value={x.assembly_id}>{x.assembly_id} · v{x.revision}</option>)}</select></label>
      </div>
    </div>
    <p role="status" data-testid="compose-message" style={messageStyle}>{message}</p>

    <div style={boardGrid}>
      <Board title="输入设备" subtitle="手柄、手套、手机或 Quest" accent="#38bdf8" count={members('input').length}>
        {addPicker('input', '#38bdf8')}
        {members('input').length ? members('input').map(([id, item]) => instanceCard(id, item, '#38bdf8',
          <label style={miniField}>采集进程
            <select aria-label={id + '进程模式'} style={input} value={String(item.parameters?.process_mode || 'auto')} onChange={e => mutate({ ...assembly, instances: { ...assembly.instances, [id]: { ...item, parameters: { ...item.parameters, process_mode: e.target.value } } } })}>
              <option value="auto">自动托管</option><option value="managed">框架启动</option><option value="external">已有外部进程</option>
            </select>
          </label>)) : emptyBoard('还没有输入设备。选择一个组件并点「添加」。')}
      </Board>
      <Board title="机器人本体 / 执行器" subtitle="机械臂、腰部、头部、夹爪与灵巧手" accent="#34d399" count={members('robot').length}>
        {addPicker('robot', '#34d399')}
        {members('robot').length ? members('robot').map(([id, item]) => instanceCard(id, item, '#34d399',
          <label style={miniField}>角色（可选）
            <input aria-label={id + '角色'} style={{ ...input, width: 120 }} value={item.role || ''} onChange={e => mutate({ ...assembly, instances: { ...assembly.instances, [id]: { ...item, role: e.target.value } } })} />
          </label>)) : emptyBoard('还没有执行器。先添加机械臂或夹爪。')}
      </Board>
      <Board title="传感器 / 相机" subtitle="相机与数值传感器" accent="#a78bfa" count={members('sensor').length}>
        {addPicker('sensor', '#a78bfa')}
        {members('sensor').length ? members('sensor').map(([id, item]) => instanceCard(id, item, '#a78bfa')) : emptyBoard('还没有传感器。需要图像时在这里添加相机。')}
      </Board>
    </div>

    <div style={card}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 10, flexWrap: 'wrap' }}>
        <h3 style={sectionTitle}>输入与控制绑定</h3>
        <span style={mutedSmall}>{assembly.bindings.length} 条</span>
        <span style={{ flex: 1 }} />
        <button style={btn} onClick={addBinding}>＋ 添加绑定</button>
      </div>
      {assembly.bindings.length ? assembly.bindings.map((b, index) => <div key={b.id} style={bindingCard}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <span style={bindingTag}>{b.id}</span>
          <span style={mutedSmall}>{b.source} → {b.target}</span>
          <span style={{ flex: 1 }} />
          <button style={ghostBtn} onClick={() => mutate({ ...assembly, bindings: assembly.bindings.filter((_, i) => i !== index) })}>移除绑定</button>
        </div>
        <div style={bindingGrid}>
          <label style={miniField}>输入端口<select aria-label={b.id + '输入端口'} style={input} value={b.source} onChange={e => updateBinding(index, 'source', e.target.value)}>{inputEndpoints.map(option)}</select></label>
          <label style={miniField}>控制目标<select aria-label={b.id + '控制目标'} style={input} value={b.target} onChange={e => updateBinding(index, 'target', e.target.value)}>{targets.map(option)}</select></label>
          <label style={miniField}>按住控制<select aria-label={b.id + '按住控制'} style={input} value={b.deadman} onChange={e => updateBinding(index, 'deadman', e.target.value)}><option value="">选择同源按住控制端口</option>{inputEndpoints.filter(x => x.instance === b.source.split('.')[0] && x.port.semantic === 'scalar' && x.port.dim === 1).map(option)}</select></label>
          <label style={miniField}>映射方式<select aria-label={b.id + '映射方式'} style={input} value={b.controller} onChange={e => updateBinding(index, 'controller', e.target.value)}><option value="">无兼容映射，请调整端口</option>{compatible(b.source, b.target).map(id => <option key={id} value={id}>{id}</option>)}</select></label>
        </div>
        <details style={bindingParams}><summary style={summaryInline}>映射参数</summary><JsonField label={b.id + '映射参数'} value={b.parameters} apply={parameters => { const next = structuredClone(assembly); next.bindings[index].parameters = parameters; mutate(next) }} /></details>
      </div>) : <p style={emptyStyle}>还没有绑定。添加输入与执行器后点「添加绑定」，可选择端口、按住控制与映射方式。</p>}
    </div>

    <details style={card}>
      <summary style={{ cursor: 'pointer', fontWeight: 700 }}>数据采集投影 <span style={mutedSmall}>· 观测 {assembly.capture.state.length} · 动作 {assembly.capture.action.length} · 相机 {Object.keys(assembly.capture.images).length}</span></summary>
      <div style={twoCol}>
        <div style={subCard}>
          <div style={{ display: 'flex', gap: 12, flexWrap: 'wrap', alignItems: 'end' }}>
            <label style={miniField}>动作来源<select aria-label="动作来源" style={input} value={assembly.capture.action_source} onChange={e => mutate({ ...assembly, capture: { ...assembly.capture, action_source: e.target.value } })}>{['command', 'next_state', 'dispatched', 'authorized', 'requested', 'feedback'].map(x => <option key={x}>{x}</option>)}</select></label>
            <label style={miniField}>采样频率<input aria-label="采样频率" style={input} type="number" min={1} value={assembly.capture.dataset_fps} onChange={e => mutate({ ...assembly, capture: { ...assembly.capture, dataset_fps: Number(e.target.value) } })} /></label>
          </div>
          <div>
            <div style={fieldLabel}>观测端口</div>
            <div style={checkboxList}>{observations.map(x => <label key={x.ref} style={choice}><input type="checkbox" aria-label={'观测' + x.ref} checked={assembly.capture.state.includes(x.ref)} onChange={e => toggleCapture('state', x.ref, e.target.checked)} />{x.ref} · {x.port.dim}维 {x.port.unit}</label>)}{!observations.length && <span style={mutedSmall}>添加执行器后出现可采集的观测端口。</span>}</div>
          </div>
          <div>
            <div style={fieldLabel}>动作端口 · 每个执行器选择一个目标</div>
            <div style={checkboxList}>{targets.map(x => <label key={x.ref} style={choice}><input type="checkbox" aria-label={'动作' + x.ref} checked={assembly.capture.action.includes(x.ref)} onChange={e => toggleCapture('action', x.ref, e.target.checked)} />{x.ref} · {x.port.dim}维 {x.port.unit}</label>)}{!targets.length && <span style={mutedSmall}>添加执行器后出现可采集的动作端口。</span>}</div>
          </div>
        </div>
        <div style={subCard}>
          <h4 style={h4}>相机与数据键</h4>
          {images.map(image => { const entry = Object.entries(assembly.capture.images).find(([, ref]) => ref === image.ref); return <div key={image.ref} style={{ display: 'flex', gap: 10, alignItems: 'center', flexWrap: 'wrap' }}>
            <label style={choice}><input aria-label={'采集相机' + image.ref} type="checkbox" checked={!!entry} onChange={e => { const next = structuredClone(assembly); if (e.target.checked) next.capture.images[image.instance] = image.ref; else if (entry) delete next.capture.images[entry[0]]; mutate(next) }} />{image.ref}</label>
            {entry && <input key={entry[0]} aria-label={image.ref + '数据键'} style={{ ...input, width: 160 }} defaultValue={entry[0]} onBlur={e => cameraKey(image.ref, entry[0], e.target.value)} />}
          </div> })}
          {!images.length && <p style={emptyStyle}>未选择相机，可进行纯数值采集。</p>}
        </div>
      </div>
    </details>

    <details style={card}>
      <summary style={{ cursor: 'pointer', fontWeight: 700 }}>高级：部署、安装关系与完整装配 JSON</summary>
      <p style={mutedSmall}>部署 JSON 独立于装配保存，填入设备标识、连接参数与已核验的部署信息。选择 real 不代表已经安装依赖或通过实体部署验收。</p>
      <div style={twoCol}>
        <JsonField label="部署配置" value={deployment} apply={next => { setDeployment(next); setResolved(null); setPreflight(null) }} />
        <JsonField label="安装关系" value={{ mounts: assembly.mounts, control_groups: assembly.control_groups }} apply={next => { if (!Array.isArray(next.mounts) || !next.control_groups || typeof next.control_groups !== 'object' || Array.isArray(next.control_groups)) { setMessage('安装关系需要 mounts 数组与 control_groups 对象'); return } mutate({ ...assembly, mounts: next.mounts, control_groups: next.control_groups as Assembly['control_groups'] }) }} />
      </div>
      <details>
        <summary style={summaryInline}>完整装配配置</summary>
        <div style={{ display: 'flex', gap: 8, margin: '10px 0' }}><button style={ghostBtn} onClick={() => setEditor(JSON.stringify(assembly, null, 2))}>导出到编辑区</button></div>
        <textarea aria-label="完整装配配置" rows={16} style={editorStyle} value={editor} onChange={e => setEditor(e.target.value)} />
        <div style={{ marginTop: 10 }}><button style={btn} onClick={() => { try { const parsed = JSON.parse(editor); const object = (value: unknown) => !!value && typeof value === 'object' && !Array.isArray(value); const capture = parsed?.capture; if (!object(parsed) || typeof parsed.assembly_id !== 'string' || !Number.isInteger(parsed.revision) || !object(parsed.instances) || !object(capture) || !Array.isArray(capture.state) || !Array.isArray(capture.action) || !object(capture.images) || !Array.isArray(parsed.bindings) || !Array.isArray(parsed.mounts) || !object(parsed.control_groups)) throw new Error('装配需要合法的 instances、capture.state/action/images、bindings、mounts 与 control_groups'); if (Object.values(parsed.instances).some(item => !object(item) || typeof (item as Instance).plugin !== 'string') || parsed.bindings.some((b: Binding) => !b || ['id', 'source', 'target', 'deadman', 'controller'].some(key => typeof (b as unknown as Record<string, unknown>)[key] !== 'string') || !object(b.parameters))) throw new Error('实例或控制绑定结构不完整'); parsed.capture.auxiliary ||= []; mutate(parsed) } catch (e) { setMessage((e as Error).message) } }}>载入编辑配置</button></div>
      </details>
    </details>

    <div style={{ ...card, display: 'flex', flexWrap: 'wrap', gap: 10, alignItems: 'center' }}>
      <div style={{ minWidth: 200, flex: 1 }}>
        <div style={{ fontWeight: 700 }}>发布并启动</div>
        <div style={mutedSmall}>{resolved ? (savedRevision === assembly.revision && resolved.assembly_hash === savedHash ? '校验通过 · 版本已保存，可启动会话' : '校验通过 · 有修改尚未保存') : '校验 → 保存下一版本 → 启动已保存版本'}</div>
      </div>
      <button style={btn} disabled={busy} onClick={() => void run(async () => { setResolved(await request<Resolved>('/assemblies/compile', { assembly, deployment })); setMessage('配置校验通过，请检查数据契约后保存。') })}>校验并预览</button>
      <button style={ghostBtn} disabled={busy} onClick={() => void run(async () => { setPreflight(await request('/deployments/preflight', { assembly, deployment })); setMessage('部署预检返回；依赖可用与现场部署核验分别显示。') })}>部署预检</button>
      <button style={btn} disabled={busy || !resolved} onClick={() => void run(async () => { const next = { ...assembly, revision: savedRevision + 1 }; const item = await request<{ assembly: Assembly; resolved: Resolved }>('/assemblies', { assembly: next, deployment, expected_revision: savedRevision }); setAssembly(item.assembly); setResolved(item.resolved); setSavedRevision(next.revision); setSavedHash(item.resolved.assembly_hash); setSaved(await request<typeof saved>('/assemblies')); setMessage('已保存不可变版本 v' + next.revision) })}>保存下一版本</button>
      <button style={primaryBtn} disabled={busy || !resolved || savedRevision !== assembly.revision || resolved.assembly_hash !== savedHash || !!session && session.status !== 'STOPPED'} onClick={() => void run(async () => { const result = await request<Session>('/sessions', { assembly_id: assembly.assembly_id, revision: savedRevision, assembly_hash: resolved!.assembly_hash, request_id: crypto.randomUUID() }); setSession(result); setMessage('已启动保存版本；请按实际会话状态准备设备。') })}>启动已保存版本</button>
    </div>

    {resolved && <div style={card}>
      <h3 style={sectionTitle}>冻结数据契约</h3>
      <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
        <span style={statChip}>观测 {resolved.feature_contract.state_dim} 维</span>
        <span style={statChip}>动作 {resolved.feature_contract.action_dim} 维</span>
        <span style={statChip}>相机 {Object.keys(resolved.feature_contract.cameras || {}).length} 路</span>
        <span style={statChip}>后端可用 {String(resolved.status?.backend_available ?? '未知')}</span>
        <span style={statChip}>部署核验 {String(resolved.status?.deployment_verified ?? '未知')}</span>
        <span style={statChip}>模型兼容 {resolved.status?.model_compatible == null ? '尚未检查' : String(resolved.status.model_compatible)}</span>
      </div>
      <p data-testid="contract-dimensions" style={mutedSmall}>观测 {resolved.feature_contract.state_dim} 维 · 动作 {resolved.feature_contract.action_dim} 维 · 相机 {Object.keys(resolved.feature_contract.cameras || {}).length} 路</p>
      <details><summary style={summaryInline}>查看键名、单位与校验结果</summary><pre style={preStyle}>{JSON.stringify({ ...resolved.feature_contract, diagnostics: resolved.diagnostics }, null, 2)}</pre></details>
    </div>}

    {preflight !== null && <div style={card}>
      <h3 style={sectionTitle}>部署预检结果</h3>
      <p style={mutedSmall}>未进行硬件探测的预检结果不能替代设备现场验收。</p>
      <details><summary style={summaryInline}>查看预检详情</summary><pre style={preStyle}>{JSON.stringify(preflight, null, 2)}</pre></details>
    </div>}

    {session && session.status !== 'STOPPED' && <div style={previewGrid}>{Object.entries(assembly.instances).filter(([, item]) => item.plugin.startsWith('morla.camera') && item.backend === 'sim').map(([id]) => <SimPreview key={id} sessionId={session.session_id} cameraId={id} />)}</div>}

    {session && <div style={{ ...card, borderColor: sessionTone(session.status) + '55' }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 10, flexWrap: 'wrap' }}>
        <span style={{ width: 9, height: 9, borderRadius: '50%', background: sessionTone(session.status), boxShadow: `0 0 0 4px ${sessionTone(session.status)}22` }} />
        <h3 data-testid="session-status" style={{ ...sectionTitle, margin: 0 }}>运行会话 · {sessionLabel(session.status)}</h3>
        <span style={monoBadge}>{session.status}</span>
        <span style={{ fontFamily: 'monospace', fontSize: 11, color: '#64748b' }}>{session.session_id.slice(0, 8)}</span>
        <span style={{ flex: 1 }} />
        {session.recording && <span style={{ ...tagStyle, color: '#fca5a5', borderColor: '#7f1d1d', background: '#3f1216' }}>● 采集中</span>}
      </div>
      <p style={mutedSmall}>按「连接输入 → 准备 → 激活运动许可」操作；未激活前不会输出运动。撤销许可只停止输出并保持当前位姿（电机仍使能），急停或停止会话才会断电。</p>
      <div style={opGroup}>
        <span style={fieldLabel}>会话</span>
        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
          {(['connect_inputs', 'prepare', 'activate', 'disarm', 'reset', 'estop', 'stop'] as const).map(op => <button style={op === 'activate' ? successBtn : op === 'estop' || op === 'stop' ? dangerBtn : btn} disabled={busy && !['estop', 'disarm', 'stop'].includes(op)} key={op} onClick={() => operate(op)}>{({ connect_inputs: '连接输入设备', prepare: '准备', activate: '激活运动许可', disarm: '撤销许可', reset: '复位故障', estop: '急停', stop: '停止会话' })[op]}</button>)}
        </div>
      </div>
      <div style={opGroup}>
        <span style={fieldLabel}>数据采集</span>
        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
          <button style={btn} disabled={busy} onClick={() => void run(async () => { setMessage(JSON.stringify(await request(`/sessions/${session.session_id}/recordings`, { task: 'demonstration' }))) })}>开始采集</button>
          <button style={btn} disabled={busy} onClick={() => void run(async () => { setMessage(JSON.stringify(await request(`/sessions/${session.session_id}/recordings/stop`, {}))) })}>结束保存</button>
          {inputs.filter(([, i]) => i.plugin === 'morla.input.mobile').map(([id]) => <button key={id} style={btn} onClick={() => void run(async () => { const p = await request<{ token: string }>(`/sessions/${session.session_id}/inputs/${id}/pair`, {}); setPairLink(`${location.origin}/mobile#session=${session.session_id}&input=${id}&token=${encodeURIComponent(p.token)}`) })}>配对手机输入 {id}</button>)}
        </div>
      </div>
      {pairLink && <p style={mutedSmall}><a href={pairLink} target="_blank" rel="noreferrer">打开手机控制页面</a><br /><small style={{ wordBreak: 'break-all' }}>{pairLink}</small></p>}
      <details><summary style={summaryInline}>组件与输入健康</summary><pre style={preStyle}>{JSON.stringify(session, null, 2)}</pre></details>
    </div>}
  </section>
}
const card: React.CSSProperties = { background: '#111a2a', border: '1px solid #233046', borderRadius: 12, padding: 16, display: 'grid', gap: 12 }
const subCard: React.CSSProperties = { background: '#0d1524', border: '1px solid #1f2b3d', borderRadius: 10, padding: 12, display: 'grid', gap: 10, alignContent: 'start' }
const boardGrid: React.CSSProperties = { display: 'grid', gridTemplateColumns: 'repeat(auto-fit,minmax(300px,1fr))', gap: 14, alignItems: 'start' }
const twoCol: React.CSSProperties = { display: 'grid', gridTemplateColumns: 'repeat(auto-fit,minmax(320px,1fr))', gap: 12, alignItems: 'start' }
const previewGrid: React.CSSProperties = { display: 'grid', gridTemplateColumns: 'repeat(auto-fit,minmax(320px,1fr))', gap: 12 }
const boardTitle: React.CSSProperties = { margin: 0, fontSize: 15 }
const h4: React.CSSProperties = { margin: 0, fontSize: 13, color: '#cbd5e1' }
const sectionTitle: React.CSSProperties = { margin: 0, fontSize: 16 }
const boardSub: React.CSSProperties = { margin: 0, color: '#7f8ea6', fontSize: 12 }
const countBadge: React.CSSProperties = { border: '1px solid', borderRadius: 999, padding: '0 8px', fontSize: 11, fontWeight: 700, lineHeight: '20px' }
const fieldLabel: React.CSSProperties = { fontSize: 12, color: '#8fa1ba', fontWeight: 600 }
const field: React.CSSProperties = { display: 'grid', gap: 5, fontSize: 12, color: '#8fa1ba', minWidth: 200 }
const miniField: React.CSSProperties = { display: 'grid', gap: 4, fontSize: 11, color: '#8fa1ba' }
const muted: React.CSSProperties = { color: '#94a3b8', fontSize: 13, lineHeight: 1.55 }
const mutedSmall: React.CSSProperties = { color: '#7f8ea6', fontSize: 12, lineHeight: 1.5 }
const emptyStyle: React.CSSProperties = { margin: 0, padding: '14px 12px', border: '1px dashed #2a3a52', borderRadius: 8, color: '#64748b', fontSize: 12, textAlign: 'center' }
const input: React.CSSProperties = { background: '#0b1220', color: '#e5e7eb', border: '1px solid #2a3a52', borderRadius: 8, padding: '7px 10px', fontSize: 13, minWidth: 0 }
const btn: React.CSSProperties = { background: '#1b283c', color: '#dbeafe', border: '1px solid #31445f', borderRadius: 8, padding: '8px 14px', fontSize: 13, fontWeight: 600, cursor: 'pointer' }
const primaryBtn: React.CSSProperties = { ...btn, background: '#2563eb', borderColor: '#3b82f6', color: '#fff' }
const successBtn: React.CSSProperties = { ...btn, background: '#15803d', borderColor: '#22c55e', color: '#fff' }
const ghostBtn: React.CSSProperties = { ...btn, background: 'transparent', color: '#a8b6cb' }
const dangerBtn: React.CSSProperties = { ...btn, background: '#3b1218', borderColor: '#7f1d1d', color: '#fca5a5' }
const iconBtn: React.CSSProperties = { ...ghostBtn, padding: '2px 8px', fontSize: 12, lineHeight: '20px' }
const tagStyle: React.CSSProperties = { border: '1px solid', borderRadius: 999, padding: '1px 8px', fontSize: 11, fontWeight: 600 }
const monoBadge: React.CSSProperties = { fontFamily: 'monospace', fontSize: 11, color: '#93a4bb', background: '#0b1220', border: '1px solid #2a3a52', borderRadius: 6, padding: '1px 7px' }
const statChip: React.CSSProperties = { background: '#0b1220', border: '1px solid #2a3a52', borderRadius: 8, padding: '5px 10px', fontSize: 12, color: '#cbd5e1' }
const messageStyle: React.CSSProperties = { whiteSpace: 'pre-wrap', color: '#bfdbfe', background: '#12233d', border: '1px solid #1e3a5f', borderRadius: 10, padding: '10px 14px', margin: 0, fontSize: 13 }
const instanceCardStyle: React.CSSProperties = { background: '#0d1524', border: '1px solid #1f2b3d', borderRadius: 8, padding: '10px 12px', display: 'grid', gap: 8 }
const bindingCard: React.CSSProperties = { background: '#0d1524', border: '1px solid #1f2b3d', borderRadius: 10, padding: 12, display: 'grid', gap: 10 }
const bindingTag: React.CSSProperties = { fontFamily: 'monospace', fontSize: 12, color: '#93c5fd', background: '#12233d', border: '1px solid #1e3a5f', borderRadius: 6, padding: '1px 8px' }
const bindingGrid: React.CSSProperties = { display: 'grid', gridTemplateColumns: 'repeat(auto-fit,minmax(200px,1fr))', gap: 10 }
const bindingParams: React.CSSProperties = { borderTop: '1px dashed #243348', paddingTop: 8 }
const summaryInline: React.CSSProperties = { cursor: 'pointer', color: '#a8b6cb', fontSize: 12 }
const choice: React.CSSProperties = { display: 'inline-flex', gap: 6, alignItems: 'flex-start', marginBottom: 6, color: '#cbd5e1', fontSize: 12, wordBreak: 'break-all' }
const checkboxList: React.CSSProperties = { display: 'grid', gap: 2, marginTop: 6 }
const opGroup: React.CSSProperties = { display: 'grid', gap: 6 }
const jsonField: React.CSSProperties = { display: 'grid', gap: 6 }
const editorStyle: React.CSSProperties = { display: 'block', width: '100%', fontFamily: 'monospace', fontSize: 12, background: '#0b1220', color: '#e5e7eb', border: '1px solid #2a3a52', borderRadius: 8, padding: 10 }
const preStyle: React.CSSProperties = { whiteSpace: 'pre-wrap', overflowWrap: 'anywhere', background: '#0b1220', border: '1px solid #1f2b3d', borderRadius: 8, padding: 10, fontSize: 12 }
