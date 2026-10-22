const { test } = require('node:test')
const assert = require('node:assert/strict')
const fs = require('node:fs')
const path = require('node:path')
const Module = require('node:module')
const ts = require('typescript')
const file = path.resolve(__dirname, '../src/api/pianoHeartbeat.ts')
const compiled = ts.transpileModule(fs.readFileSync(file, 'utf8'), { compilerOptions: { module: ts.ModuleKind.CommonJS } }).outputText
const subject = new Module(file, module)
subject._compile(compiled, file)
const { makePianoHeartbeat } = subject.exports

function fixture(send) {
  const value = { token: 'owner', recovered: 0, failed: 0, calls: 0 }
  value.client = makePianoHeartbeat({ getToken: () => value.token,
    send: async (...args) => { value.calls++; return send(...args) },
    recovered: () => value.recovered++, failed: () => value.failed++, timeoutMs: 20 })
  return value
}

test('no token never sends or acquires ownership', async () => {
  const f=fixture(async () => {});f.token=null
  await f.client.beat()
  assert.equal(f.calls,0);assert.equal(f.recovered,0)
})

test('overlapping timer and foreground events use one in-flight renewal', async () => {
  let finish
  const f=fixture(() => new Promise(resolve => { finish=resolve }))
  const pending=f.client.beat()
  await f.client.beat()
  assert.equal(f.calls,1)
  finish();await pending
  assert.equal(f.recovered,1);f.client.dispose()
})

test('a transient network failure can recover current ownership on retry', async () => {
  let broken=true
  const f=fixture(async () => { if (broken) throw new Error('network') })
  await f.client.beat();broken=false;await f.client.beat()
  assert.equal(f.failed,1);assert.equal(f.recovered,1);assert.equal(f.calls,2)
})

test('a stale response cannot grant ownership after the token changes', async () => {
  let finish
  const f=fixture(() => new Promise(resolve => { finish=resolve }))
  const pending=f.client.beat();f.token='different';finish();await pending
  assert.equal(f.recovered,0);assert.equal(f.failed,0)
})

test('a stalled fetch is aborted and a later beat can retry', async () => {
  let stalled=true
  const f=fixture((_token,signal) => stalled ? new Promise((_resolve,reject) => {
    signal.addEventListener('abort',() => reject(new Error('aborted')),{once:true})
  }) : Promise.resolve())
  await f.client.beat();assert.equal(f.failed,1)
  stalled=false;await f.client.beat()
  assert.equal(f.calls,2);assert.equal(f.recovered,1)
})

test('disposing cancels renewal without callbacks or subsequent requests', async () => {
  const f=fixture((_token,signal) => new Promise((_resolve,reject) => {
    signal.addEventListener('abort',() => reject(new Error('aborted')),{once:true})
  }))
  const pending=f.client.beat();f.client.dispose();await pending;await f.client.beat()
  assert.equal(f.calls,1);assert.equal(f.failed,0);assert.equal(f.recovered,0)
})
