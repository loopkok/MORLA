import { useSyncExternalStore } from 'react'
import type { NormalisedState } from '../lib/mapUiState'
import { JointPanel } from './JointPanel'
import { ChartPanel } from './ChartPanel'
import { DataCollectCard } from './DataCollectCard'
import { InferenceCard } from './InferenceCard'
import { getSamples, subscribeHistory } from '../hooks/historyStore'

interface Props {
  state: NormalisedState | null
}

export function MonitorTab({ state }: Props) {
  const samples = useSyncExternalStore(subscribeHistory, getSamples, getSamples)
  const j = state?.joints ?? {}
  const rates = state?.ratesHz ?? {}

  // Build chart series from the sample ring buffer.
  const rateSeries = [
    { label: '左臂', color: '#3b82f6', values: samples.map((s) => s.ratesHz.left_arm_cmd ?? 0) },
    { label: '右臂', color: '#22c55e', values: samples.map((s) => s.ratesHz.right_arm_cmd ?? 0) },
    { label: '左夹爪', color: '#f59e0b', values: samples.map((s) => s.ratesHz.left_gripper_cmd ?? 0) },
    { label: '右灵巧手', color: '#a855f7', values: samples.map((s) => s.ratesHz.right_hand_cmd ?? 0) },
    { label: '头部', color: '#14b8a6', values: samples.map((s) => s.ratesHz.head_cmd ?? 0) },
  ]

  // Left arm joint-0 angle over time (representative trace).
  const leftArmJ0 = samples.map((s) => s.joints.left_arm?.[0] ?? 0)
  const rightArmJ0 = samples.map((s) => s.joints.right_arm?.[0] ?? 0)
  // Head yaw/pitch over time (right-controller thumbstick teleop).
  const headYaw = samples.map((s) => s.joints.head?.[0] ?? 0)
  const headPitch = samples.map((s) => s.joints.head?.[1] ?? 0)

  return (
    <div style={wrapStyle}>
      <DataCollectCard
        dc={state?.dataCollect ?? null}
        launch={state?.collectLaunch ?? null}
        teleopState={state?.teleopState ?? 'stopped'}
      />
      <InferenceCard infer={state?.infer ?? null} launch={state?.inferLaunch ?? null} />
      <div style={gridStyle}>
        <JointPanel title="左臂 (7-DoF)" joint={j.left_arm} rateHz={rates.left_arm_cmd} />
        <JointPanel title="右臂 (7-DoF)" joint={j.right_arm} rateHz={rates.right_arm_cmd} />
        <JointPanel title="左夹爪" joint={j.left_gripper} rateHz={rates.left_gripper_cmd} />
        <JointPanel title="右灵巧手 (20-DoF)" joint={j.right_hand} rateHz={rates.right_hand_cmd} />
        <JointPanel title="头部 (yaw/pitch)" joint={j.head} rateHz={rates.head_cmd} />
      </div>

      {state && (
        <div style={ratesStyle}>
          <strong>指令频率:</strong>{' '}
          {Object.entries(state.ratesHz).map(([k, v]) => (
            <span key={k} style={rateChip}>
              {k}: <b>{v.toFixed(1)}</b> Hz
            </span>
          ))}
        </div>
      )}

      <div style={chartGridStyle}>
        <ChartPanel
          title="指令频率 (Hz)"
          series={rateSeries}
          yMin={0}
          yMax={60}
          unit="Hz"
        />
        <ChartPanel
          title="臂关节0 角度 (rad)"
          series={[
            { label: '左臂 j0', color: '#3b82f6', values: leftArmJ0 },
            { label: '右臂 j0', color: '#22c55e', values: rightArmJ0 },
          ]}
          unit="rad"
        />
        <ChartPanel
          title="头部 yaw/pitch (rad)"
          series={[
            { label: 'yaw', color: '#14b8a6', values: headYaw },
            { label: 'pitch', color: '#f97316', values: headPitch },
          ]}
          unit="rad"
        />
      </div>
    </div>
  )
}

const wrapStyle: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: '16px' }
const gridStyle: React.CSSProperties = {
  display: 'grid',
  gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))',
  gap: '14px',
}
const ratesStyle: React.CSSProperties = {
  display: 'flex',
  flexWrap: 'wrap',
  gap: '12px',
  alignItems: 'center',
  fontSize: '13px',
  color: '#9ca3af',
  padding: '10px 14px',
  background: '#1f2937',
  borderRadius: '8px',
}
const rateChip: React.CSSProperties = {
  background: '#374151',
  padding: '3px 10px',
  borderRadius: '6px',
  color: '#d1d5db',
}
const chartGridStyle: React.CSSProperties = {
  display: 'grid',
  gridTemplateColumns: 'repeat(auto-fit, minmax(320px, 1fr))',
  gap: '14px',
}
