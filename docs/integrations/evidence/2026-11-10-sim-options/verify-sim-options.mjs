/** Browser check for the workstation simulation options panel.
 *
 * Serves web/dist through stub-api.py (below) with a canned MMK2-like assembly,
 * then drives real Chromium. Three scenarios:
 *   no-session  – panel body renders, display option changes and holds
 *   active      – change posts to /sessions/{id}/simulation and shows the reply
 *   stopped     – regression: the select must not snap back after 结束工作站
 *
 * Usage:
 *   NODE_PATH=<dir with playwright> node verify-sim-options.mjs <base-url> <scenario>
 * Requires ACTIVE=1 on the stub for the active/stopped scenarios.
 */
import { createRequire } from 'node:module'
const require = createRequire(import.meta.url)
const { chromium } = require('playwright')

const base = process.argv[2] || 'http://127.0.0.1:18082'
const scenario = process.argv[3] || 'no-session'
const executablePath = process.env.BROWSER_EXECUTABLE || undefined
let failures = 0
const check = (ok, label, detail = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'} ${label}${detail ? ' — ' + detail : ''}`)
  if (!ok) failures += 1
}

const browser = await chromium.launch({ executablePath, headless: true })
const page = await browser.newPage({ viewport: { width: 1440, height: 1200 } })
try {
  await page.goto(base)
  await page.getByRole('heading', { name: '遥操控制', exact: true }).waitFor()

  if (scenario === 'no-session') {
    // Open the saved assembly through the composition page.
    await page.getByRole('button', { name: '系统编排', exact: true }).click()
    await page.getByRole('button', { name: '导入 / 打开' }).click()
    await page.locator('.rw-config-grid select').first().selectOption('demo_sim')
    await page.getByRole('button', { name: '确认', exact: true }).click()
    await page.waitForTimeout(800)
    await page.getByRole('button', { name: '遥操控制', exact: true }).click()
    await page.waitForTimeout(300)
    await page.locator('details.rw-advanced summary').first().click()
    const text = await page.locator('details.rw-advanced').first().innerText()
    check(text.includes('尚未建立运行会话'), 'panel explains why controls/pairing are absent')
    const display = page.locator('details.rw-advanced').first().getByLabel('仿真显示方式', { exact: true })
    await display.selectOption('local')
    await page.waitForTimeout(400)
    check(await display.inputValue() === 'local', 'display option holds without a session', await display.inputValue())
    // The check tab must not leave the panel empty when we come back.
    await page.getByRole('button', { name: '运行前检查', exact: true }).click()
    await page.waitForTimeout(600)
    await page.getByRole('button', { name: '运行概览', exact: true }).click()
    await page.waitForTimeout(600)
    const details = page.locator('details.rw-advanced').first()
    if (!(await details.evaluate(el => el.open))) await page.locator('details.rw-advanced summary').first().click()
    const after = await details.innerText()
    check(after.includes('运行设置') && after.includes('仿真显示'), 'panel still renders after the check tab')
  } else {
    await page.locator('details.rw-advanced summary').first().click()
    const panel = page.locator('details.rw-advanced').first()
    const display = panel.getByLabel('仿真显示方式', { exact: true })
    if (scenario === 'active') {
      check((await panel.innerText()).includes('运行控制'), 'live session shows the run controls')
      await display.selectOption('local')
      await page.waitForTimeout(900)
      check(await display.inputValue() === 'local', 'active session applies the posted option', await display.inputValue())
    } else {
      await page.getByRole('button', { name: '结束工作站', exact: true }).click()
      await page.getByRole('button', { name: '确认', exact: true }).click().catch(() => {})
      await page.waitForTimeout(1500)
      check(await page.locator('[data-testid=session-status]').innerText() === '已停止', 'workstation stopped')
      await display.selectOption('local')
      await page.waitForTimeout(1000)
      check(await display.inputValue() === 'local', 'display option no longer snaps back after stop', await display.inputValue())
    }
  }
} finally {
  await browser.close()
}
console.log(failures ? `${failures} check(s) failed` : 'all checks passed')
process.exit(failures ? 1 : 0)
