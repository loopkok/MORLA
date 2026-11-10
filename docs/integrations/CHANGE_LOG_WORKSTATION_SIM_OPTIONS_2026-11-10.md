# 工作站「仿真显示」选项无法切换的两处缺陷（2026-11-10）

## 现象

操作员报告，在新工作站 UI 的「完整运行控制、工作位、配对与诊断」折叠面板里：

1. 运行前检查之前，点击该面板标题“点不开”，面板里看不到运行控制、工作位、配对内容。
2. 运行前检查之后，在面板里想改「仿真显示」为「浏览器画面 / 本机 MuJoCo 窗口」，选不动，列表会弹回原值。

## 根因

### 1. 面板内容被运行阶段门控（`RunConsole.tsx`）

`RunConsole` 整个函数体包在 `{stage==='run'&&<>…</>}` 里。`stage` 由
`ReferenceWorkspace` 根据当前页/子页同步（`useEffect` 里
`m.setStage(page==='teleop'?tab:'run')`），而 `locate()`、`PreflightReview`
（检查页）等路径也会改写 `stage`。只要 `stage` 与 `tab` 短暂或持续不一致，
面板 `<details>` 能展开，但内部渲染为空——表现就是“点不开”。

另外，面板标题写着“运行控制、工作位、配对与诊断”，但这三部分都依赖
`session`；启动工作站之前它们本来就不存在，而面板没有任何说明，看起来也像
“点不开”。

### 2. `sim` 的读取源与写入源不一致（`ComposableTab.tsx`）

```ts
const sim = session?.simulation || assembly.metadata?.simulation || {…}
function simOptions(patch){
  if(active) void run(… POST /sessions/{id}/simulation …)
  else changed({…assembly, metadata:{…metadata, simulation:{…sim, …patch}}})
}
```

`active = !!session && session.status !== 'STOPPED'`。工作站**结束后**，
`session` 仍保留在状态里（`status='STOPPED'`），此时：

- 下拉框读的是 `session.simulation`（旧值）；
- 选择后 `simOptions` 走 `changed()` 分支，只改草稿装配的
  `metadata.simulation`；
- 下一次渲染 `sim` 仍然先取 `session.simulation`，于是选择被弹回原值。

真实浏览器复现（`ACTIVE=1` 打桩会话 + 点击「结束工作站」）：

```
display after stop web    ← 停用后下拉仍读会话快照
display after choosing local web   ← 选“本机窗口”被弹回
```

顺带发现：这条路径把会话运行态字段写进了装配元数据。本机保存的
`mmk2_quest_whole_sim` revision 2/3 里已经带上了
`{"viewer":{"status":"closed"},"render_error":null}`，属于同一缺陷的产物。

## 修改

- `web/src/components/workstation/RunConsole.tsx`
  - 去掉 `stage==='run'` 门控，面板内容只要挂载就渲染（该组件只挂在“运行概览”页，
    门控本就是多余的）。
  - 没有会话时显示说明：「尚未建立运行会话。点击「启动工作站」后，这里提供输入连接、
    设备准备、开始遥操、手机配对与运行诊断。」
  - 工作站已结束时，仿真设置下方补充「当前工作站已结束，这里的修改写入工作空间配置，
    重新启动工作站后生效。」
- `web/src/components/ComposableTab.tsx`
  - `sim` 只在 `active` 时读会话值，否则读草稿装配：
    `(active ? session?.simulation : undefined) || assembly.metadata?.simulation`。
    非活动分支写回草稿时只保留 `mode`/`display`。
  - 新增 `storedSimulation()`：保存与另存为时剥掉 `simulation` 里的
    `viewer`/`render_error` 等运行态字段，避免再次写入历史版本。

## 验证

- `npm run build` 通过（tsc + vite），`npm run test:workcell` **47 passed**。
- 真实 Chromium（Playwright，隔离打桩 API，不启动正式 Web 服务）：
  - 无会话：打开折叠面板有内容，包含新的空状态说明；「仿真显示」可改并保持。
  - 点击「运行前检查」后回到运行概览：面板仍有内容，「仿真显示」仍可改。
  - 有活动会话：改「仿真显示」会 `POST /sessions/{id}/simulation`，值随状态刷新。
  - **结束后**：改「仿真显示」不再弹回（修复前必然弹回）。
- 未验证：真实 MMK2 场景下的 `set_options`（本机窗口需要图形环境），真机方向与
  实机验收不在本次范围；未改动 watchdog/断链相关逻辑。

## 遗留

- 已保存的 `mmk2_quest_whole_sim` revision 2/3 元数据里仍有
  `viewer`/`render_error`。它们不影响编译与启动，下次在该工作空间里保存新版本时会被
  自动清理；如需立即清理，重新保存一次即可。
- 修改仿真显示后需要「保存工作空间」才会持久化（草稿会被检查失效保护拦住，属预期）。
