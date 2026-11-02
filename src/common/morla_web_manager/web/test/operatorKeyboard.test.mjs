import test from 'node:test'
import assert from 'node:assert/strict'
import { readFile } from 'node:fs/promises'
import ts from 'typescript'
const source=await readFile(new URL('../src/lib/operatorKeyboard.ts',import.meta.url),'utf8')
const js=ts.transpileModule(source,{compilerOptions:{module:ts.ModuleKind.ES2022,target:ts.ScriptTarget.ES2020}}).outputText
const {OperatorKeyboard}=await import('data:text/javascript;base64,'+Buffer.from(js).toString('base64'))
test('S starts once per press, B saves, old keys have no action',()=>{
 const k=new OperatorKeyboard()
 assert.deepEqual(k.press('KeyS'),['teleop_start']);assert.deepEqual(k.press('KeyS'),[])
 assert.deepEqual(k.release('KeyS'),[]);assert.deepEqual(k.press('KeyS'),['teleop_start'])
 assert.deepEqual(k.press('KeyB'),['record_stop'])
 for(const key of ['KeyQ','KeyD','KeyP','KeyT','KeyH','KeyG','KeyX','Space'])assert.deepEqual(k.press(key),[])
})
test('single A and N execute only on release',()=>{
 const k=new OperatorKeyboard()
 assert.deepEqual(k.press('KeyA'),[]);assert.deepEqual(k.release('KeyA'),['record_or_release'])
 assert.deepEqual(k.press('KeyN'),[]);assert.deepEqual(k.release('KeyN'),['record_discard'])
})
test('A+N in either order only moves to work position, both release orders work',()=>{
 for(const order of [['KeyA','KeyN'],['KeyN','KeyA']])for(const releases of [order,[...order].reverse()]){
  const k=new OperatorKeyboard()
  assert.deepEqual(k.press(order[0]),[]);assert.deepEqual(k.press(order[1]),['workpos'])
  assert.deepEqual(k.press(order[1]),[])
  for(const key of releases)assert.deepEqual(k.release(key),[])
 }
})
test('focus loss cancels pending actions and held state',()=>{
 const k=new OperatorKeyboard();k.press('KeyS');k.press('KeyA');k.clear()
 assert.equal(k.down.size,0);assert.deepEqual(k.release('KeyA'),[])
 assert.deepEqual(k.press('KeyA'),[]);assert.deepEqual(k.release('KeyA'),['record_or_release'])
})
