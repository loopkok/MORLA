interface Tab {
  id: string
  label: string
}

interface Props {
  tabs: Tab[]
  active: string
  onChange: (id: string) => void
}

export function Tabs({ tabs, active, onChange }: Props) {
  return (
    <div style={barStyle} role="tablist">
      {tabs.map((t) => {
        const on = t.id === active
        return (
          <button
            key={t.id}
            role="tab"
            aria-selected={on}
            style={tabStyle(on)}
            onClick={() => onChange(t.id)}
          >
            {t.label}
          </button>
        )
      })}
    </div>
  )
}

const barStyle: React.CSSProperties = {
  display: 'flex',
  gap: '4px',
  overflowX: 'auto',
  padding: '0 20px',
  borderBottom: '1px solid #1f2937',
}
function tabStyle(on: boolean): React.CSSProperties {
  return {
    background: on ? '#1f2937' : 'transparent',
    color: on ? '#e5e7eb' : '#9ca3af',
    border: 'none',
    borderBottom: on ? '2px solid #d9b770' : '2px solid transparent',
    padding: '10px 16px',
    cursor: 'pointer',
    whiteSpace: 'nowrap',
    flexShrink: 0,
    fontSize: '14px',
    fontWeight: on ? 600 : 400,
  }
}
