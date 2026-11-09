import { useEffect, useRef, useState } from 'react'
import type { WorkstationModel } from '../ComposableTab'
import type { Instance } from '../../lib/workcell'
import { presentIkReport } from '../../lib/workcell'
import { deviceCategories, deviceCategory } from '../../lib/workstationPresentation'
import { feedbackRateSample, feedbackState, measuredFeedbackHz, solverDurationMs } from '../../lib/deviceTelemetry'
import type { RateSample } from '../../lib/deviceTelemetry'
import { metric, reasons } from './shared'

function DeviceStatus({ m, id, instance }: { m: WorkstationModel; id: string; instance: Instance }) {
  const feedback = m.session?.components?.[id]
  const previous = useRef<RateSample | undefined>(undefined)
  const [hz, setHz] = useState<number>()
  useEffect(() => {
    const current = feedbackRateSample(feedback)
    setHz(measuredFeedbackHz(previous.current, current))
    previous.current = current
  }, [feedback])
  const state = feedbackState(feedback, !!m.connectionError)
  const descriptor = m.descriptor(instance)
  const backend = m.device(id).backend || instance.backend || 'fake'
  const configuredHz = m.device(id).parameters?.rate_hz ?? instance.parameters?.rate_hz ?? descriptor?.defaults?.rate_hz
  const chains = m.assembly.bindings.filter(binding => binding.target.split('.')[0] === id)
  const reports = chains.flatMap(binding => Object.entries(m.session?.controllers?.[binding.id] || {}).filter(([, raw]) => Object.keys(raw).length).map(([plugin, raw]) => ({ binding, plugin, report: presentIkReport(raw) })))
  const source = feedback?.source === 'hardware' && feedback.measured === true && feedback.synthetic !== true ? '硬件实测' : feedback?.source === 'simulation' ? 'MuJoCo 仿真' : feedback?.synthetic || feedback?.source === 'fake' ? '测试数据' : '等待数据源'
  const hasFeedback = !!feedback && !m.connectionError
  const sourceLabel = ({ real: '实机', sim: '仿真', fake: '测试' } as Record<string, string>)[backend] || backend
  return <details className="rw-device-status">
    <summary><span className="rw-device-identity"><strong>{instance.display_name || m.label(instance.plugin)}</strong><small>{id} · {sourceLabel}</small></span><span className={'rw-device-state ' + state.tone}>{state.label}</span><span className="rw-detail-chevron" aria-hidden="true">⌄</span></summary>
    <div className="rw-device-status-body">
      <dl className="rw-telemetry-grid"><div><dt>反馈来源</dt><dd>{source}</dd></div><div><dt>实测反馈频率</dt><dd title="按硬件帧序号增量与设备采样时间差计算；不使用页面刷新率。">{hasFeedback && hz !== undefined ? hz.toFixed(1) + ' Hz' : '暂无数据'}</dd></div><div><dt>反馈龄期</dt><dd>{hasFeedback && typeof feedback.age_ns === 'number' && feedback.age_ns >= 0 ? metric(feedback.age_ns, 1e-6, 1) + ' ms' : '暂无数据'}</dd></div><div><dt>配置控制频率</dt><dd>{typeof configuredHz === 'number' ? configuredHz + ' Hz' : '未指定'}</dd></div></dl>
      {(feedback?.reason || feedback?.stop_error) && <p className={'rw-device-message ' + (feedback.fault ? 'error' : '')}>{[feedback.reason, feedback.stop_error].filter(Boolean).join(' · ')}</p>}
      <h4>实时反馈{m.connectionError ? ' · 上次缓存' : ''}</h4>
      {feedback?.values?.length ? <div className="rw-joint-values">{feedback.values.map((value, index) => <div key={index}><span>{descriptor?.joints?.[index]?.name || descriptor?.ports.position?.axis_names?.[index] || `关节 ${index + 1}`}</span><strong>{Number.isFinite(value) ? value.toFixed(4) : '无效'} <small>{descriptor?.joints?.[index]?.unit || descriptor?.ports.position?.unit || ''}</small></strong></div>)}</div> : <p className="wc-note">暂无位置反馈</p>}
      {!!Object.keys(feedback?.device_status || {}).length && <details className="rw-raw-status"><summary>设备状态与错误码</summary><pre>{JSON.stringify(feedback!.device_status, null, 2)}</pre></details>}
      <h4>遥操链路 <small>{chains.length} 条映射</small></h4>
      <dl className="rw-telemetry-grid"><div><dt>端到端延迟 · E2E</dt><dd>暂无数据</dd></div><div><dt>控制器报告</dt><dd>{reports.length ? reports.length + ' 份' : '等待报告'}</dd></div></dl>
      {!chains.length && <p className="wc-note">尚未连接控制映射</p>}
      {chains.map(binding => <p className="rw-chain-route" key={binding.id}>{binding.source} → {binding.target}<small>{m.label(binding.controller)}</small></p>)}
      {reports.map(({ binding, plugin, report }) => <div className="rw-chain-report" key={binding.id + plugin}><strong>{binding.id}</strong><span className={report.error ? 'error' : ''}>{typeof report.error === 'string' ? report.error : reasons[String(report.reason || '')] || String(report.reason || (report.success ? '求解成功' : '等待目标'))}</span><dl className="rw-telemetry-grid"><div><dt>求解耗时</dt><dd>{metric(solverDurationMs(report))} ms</dd></div><div><dt>报告龄期</dt><dd>{metric(typeof report.age_ms === 'number' ? report.age_ms : undefined, 1, 0)} ms</dd></div><div><dt>候选位置误差</dt><dd>{metric(typeof report.position_error_m === 'number' ? report.position_error_m : undefined, 1000)} mm</dd></div><div><dt>候选姿态误差</dt><dd>{metric(typeof report.angle_error_rad === 'number' ? report.angle_error_rad : undefined, 180 / Math.PI)} °</dd></div></dl><details className="rw-raw-status"><summary>完整控制器报告</summary><pre>{JSON.stringify(report, null, 2)}</pre></details></div>)}
    </div>
  </details>
}

export function RobotStatus({ m, onDiagnostics }: { m: WorkstationModel; onDiagnostics: () => void }) {
  const groups = deviceCategories.map(([category, title]) => ({ category, title, devices: m.actuators.filter(([, instance]) => { const descriptor = m.descriptor(instance); return descriptor && deviceCategory(descriptor) === category }) })).filter(group => group.devices.length)
  return <section className="rw-card rw-robot-status"><header><h3>机器人状态 <small>{m.actuators.length} 台设备</small></h3><button onClick={onDiagnostics}>运行日志 ↗</button></header><div className="rw-card-body">{groups.length ? groups.map(group => <section className="rw-robot-group" key={group.category}><h4>{group.title}<small>{group.devices.length}</small></h4>{group.devices.map(([id, instance]) => <DeviceStatus key={(m.session?.session_id || 'draft') + id} m={m} id={id} instance={instance} />)}</section>) : <p className="wc-note">尚未添加机器人。在系统编排中添加机械臂、末端或整机后，将在这里显示状态。</p>}</div></section>
}
