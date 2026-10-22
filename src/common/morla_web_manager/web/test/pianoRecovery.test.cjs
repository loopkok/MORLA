const { test } = require('node:test')
const assert = require('node:assert/strict')
const fs = require('node:fs')
const path = require('node:path')
const Module = require('node:module')
const ts = require('typescript')
const file = path.resolve(__dirname, '../src/api/pianoRecovery.ts')
const compiled = ts.transpileModule(fs.readFileSync(file, 'utf8'), { compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 } }).outputText
const subject = new Module(file, module)
subject._compile(compiled, file)
const { recoverPianoReset } = subject.exports

function setup(initial = {}) {
  const fixture = { time: 0, cancelled: false, phase: 'old', blocks: 0, calls: [], adopted: [], afterOpenCancel: false, resultOk: true }
  function state() {
    if (fixture.phase === 'old') return { active: true, mode: 'hardware', state: 'FAULT', session_id: 'old', busy: false, motion_claimed: false, ...initial }
    if (fixture.phase === 'closed') return { active: false, session_id: 'old', blockers: fixture.blocks-- > 0 ? ['waiting for DDS'] : [] }
    if (fixture.phase === 'new') return { active: true, mode: 'hardware', state: 'CONNECTED', session_id: 'new', busy: false, maintenance_allowed: true }
    if (fixture.phase === 'other') return { active: true, mode: 'hardware', state: 'CONNECTED', session_id: 'other' }
    return { active: true, mode: 'hardware', state: 'WAITING_FEEDBACK', session_id: fixture.resetId, busy: false,
             result: { job_id: 'reset-job', ok: fixture.resultOk, error: fixture.resultOk ? undefined : 'controller rejected reset' } }
  }
  fixture.client = {
    state: async () => state(),
    close: async token => { fixture.calls.push(['close', token]); fixture.phase = 'closed' },
    open: async (...args) => {
      fixture.calls.push(['open', ...args]); fixture.phase = 'new'
      if (fixture.afterOpenCancel) fixture.cancelled = true
      return { token: 'new-token', session: state() }
    },
    command: async (...args) => {
      fixture.calls.push(['command', ...args]); fixture.resetId = fixture.phase === 'old' ? 'old' : 'new'; fixture.phase = 'reset'
      return { id: 'reset-job' }
    },
  }
  fixture.options = {
    token: 'old-token', sessionId: 'old', revision: 'saved-revision', devices: { left_can: 'can_nero_left', right_can: 'can_nero_right' },
    onSession: (token, state) => fixture.adopted.push([token, state?.session_id]),
    cancelled: () => fixture.cancelled,
    sleep: async ms => { fixture.time += ms }, now: () => fixture.time, timeoutMs: 1000,
  }
  return fixture
}

test('fault -> release -> DDS departure -> new session -> reset, with no motion', async () => {
  const f = setup(); f.blocks = 2
  const result = await recoverPianoReset(f.client, f.options)
  assert.equal(result.state, 'WAITING_FEEDBACK')
  assert.deepEqual(f.calls, [
    ['close', 'old-token'], ['open', 'hardware', 'saved-revision', f.options.devices],
    ['command', 'new-token', 'reset', { calibration_confirmed: true }],
  ])
  assert.deepEqual(f.adopted.map(v => v[0]), [null, 'new-token'])
  assert.equal(f.time, 400)
})

test('idle maintenance session resets directly', async () => {
  const f = setup({ state: 'CONNECTED', maintenance_allowed: true })
  await recoverPianoReset(f.client, f.options)
  assert.deepEqual(f.calls.map(v => v[0]), ['command'])
  assert.equal(f.calls[0][1], 'old-token')
})

test('claimed READY session is recreated before reset', async () => {
  const f = setup({ state: 'READY', motion_claimed: true })
  await recoverPianoReset(f.client, f.options)
  assert.deepEqual(f.calls.map(v => v[0]), ['close', 'open', 'command'])
})

for (const initial of [{ busy: true }, { session_id: 'other' }, { mode: 'software' }, { active: false }]) {
  test(`reject unsafe or replaced initial session ${JSON.stringify(initial)}`, async () => {
    const f = setup(initial)
    await assert.rejects(recoverPianoReset(f.client, f.options))
    assert.deepEqual(f.calls, [])
  })
}

test('persistent external writers prevent reopen and reset', async () => {
  const f = setup(); f.blocks = 100
  await assert.rejects(recoverPianoReset(f.client, f.options), /尚未释放/)
  assert.deepEqual(f.calls.map(v => v[0]), ['close'])
})

test('another page acquiring the robot is never closed or reset', async () => {
  const f = setup()
  f.client.close = async token => { f.calls.push(['close', token]); f.phase = 'other' }
  await assert.rejects(recoverPianoReset(f.client, f.options), /其他页面/)
  assert.deepEqual(f.calls, [['close', 'old-token']])
})

test('stop/unmount after release prevents a new session', async () => {
  const f = setup()
  f.options.onSession = () => { f.cancelled = true }
  await assert.rejects(recoverPianoReset(f.client, f.options), /取消/)
  assert.deepEqual(f.calls.map(v => v[0]), ['close'])
})

test('unmount during open closes the newly returned session without reset', async () => {
  const f = setup(); f.afterOpenCancel = true
  await assert.rejects(recoverPianoReset(f.client, f.options), /取消/)
  assert.deepEqual(f.calls.map(v => v[0]), ['close', 'open', 'close'])
  assert.equal(f.calls[2][1], 'new-token')
})

test('reset rejection is reported and never followed by enable or motion', async () => {
  const f = setup(); f.resultOk = false
  await assert.rejects(recoverPianoReset(f.client, f.options), /controller rejected/)
  assert.deepEqual(f.calls.map(v => v[0]), ['close', 'open', 'command'])
})
