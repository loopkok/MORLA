// A/N resolve on release so A+N never also records or discards.
export type OperatorAction = 'teleop_start' | 'record_or_release' | 'record_stop' | 'record_discard' | 'workpos'
export class OperatorKeyboard {
  down = new Set<string>()
  chord = false
  blocked = new Set<string>()
  press(code: string): OperatorAction[] {
    if (!['KeyS','KeyA','KeyB','KeyN'].includes(code) || this.down.has(code)) return []
    this.down.add(code)
    if (this.down.has('KeyA') && this.down.has('KeyN')) {
      this.chord = true
      this.blocked.add('KeyA'); this.blocked.add('KeyN')
      return ['workpos']
    }
    return code === 'KeyS' ? ['teleop_start'] : code === 'KeyB' ? ['record_stop'] : []
  }
  release(code: string): OperatorAction[] {
    if (!this.down.delete(code)) return []
    const blocked = this.blocked.delete(code)
    if (!this.down.has('KeyA') && !this.down.has('KeyN')) this.chord = false
    if (blocked) return []
    return code === 'KeyA' ? ['record_or_release'] : code === 'KeyN' ? ['record_discard'] : []
  }
  clear() { this.down.clear(); this.blocked.clear(); this.chord = false }
}
