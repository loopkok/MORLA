// 顶层错误边界：单个组件渲染异常不再让整棵树卸载白屏。
// 显示错误卡 + 「重试渲染」按钮；WS 数据流在模块层继续跑，重试即可恢复。
// （背景：推理节点 engine.server_timing 嵌套对象曾触发 "Objects are not valid
// as a React child" → 整树白屏且刷新仍崩——数据形状防御不能只靠单点修复。）
import { Component, type ErrorInfo, type ReactNode } from 'react'

interface State {
  error: Error | null
}

export class ErrorBoundary extends Component<{ children: ReactNode }, State> {
  state: State = { error: null }

  static getDerivedStateFromError(error: Error): State {
    return { error }
  }

  componentDidCatch(error: Error, info: ErrorInfo): void {
    // 可观测：错误进 console，便于从浏览器控制台定位
    console.error('[ErrorBoundary] render error:', error, info.componentStack)
  }

  render() {
    if (this.state.error) {
      return (
        <div style={wrapStyle}>
          <div style={cardStyle}>
            <div style={titleStyle}>界面渲染出错（已隔离——数据采集/推理不受影响）</div>
            <pre style={errStyle}>{String(this.state.error.message || this.state.error)}</pre>
            <div style={rowStyle}>
              <button style={btnStyle} onClick={() => this.setState({ error: null })}>
                重试渲染
              </button>
              <span style={hintStyle}>
                若重试后仍报错，多为遥测数据结构与前端类型不一致，请查看浏览器控制台
              </span>
            </div>
          </div>
        </div>
      )
    }
    return this.props.children
  }
}

const wrapStyle: React.CSSProperties = { minHeight: '100vh', background: '#0b0f17', padding: 24, boxSizing: 'border-box' }
const cardStyle: React.CSSProperties = {
  background: '#7f1d1d',
  borderRadius: '8px',
  padding: '16px 18px',
  color: '#fecaca',
  maxWidth: '720px',
  margin: '40px auto',
}
const titleStyle: React.CSSProperties = { fontWeight: 700, fontSize: '15px', marginBottom: 8 }
const errStyle: React.CSSProperties = {
  background: '#111827',
  color: '#fca5a5',
  padding: '10px 12px',
  borderRadius: '6px',
  fontSize: '12px',
  whiteSpace: 'pre-wrap',
  wordBreak: 'break-word',
}
const rowStyle: React.CSSProperties = { display: 'flex', alignItems: 'center', gap: 12, marginTop: 10, flexWrap: 'wrap' }
const btnStyle: React.CSSProperties = {
  background: '#f87171',
  color: '#111827',
  border: 'none',
  borderRadius: '6px',
  padding: '8px 16px',
  fontWeight: 700,
  cursor: 'pointer',
}
const hintStyle: React.CSSProperties = { color: '#fecaca', fontSize: '12px' }
