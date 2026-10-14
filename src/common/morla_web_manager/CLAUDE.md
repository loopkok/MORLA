# CLAUDE.md — morla_web_manager 工程

非侵入式 Web 监控/控制台：浏览器启停遥操栈、监控关节与健康、机器人模式、视频回传、VLA 数采控制。FastAPI + React，端口 8080。本文件面向新会话/新接手者：先读这里，再进 `README.md`（API/话题契约全表）和 `MORLA/CHANGELOG.md`（按天演进史，含每个问题的症状→根因→修法）。

## 当前状态（2026-09-03）

- 四 Tab 布局：数据处理 + **监控**（数采卡片 + 关节面板 + 实时折线）、**健康**（逐实体巡检）、**系统**（预设/机器人模式/管线延迟/视频回传/日志）；
- **双 launch 泳道**：遥操作主泳道 + 数采独立泳道（各自 `LaunchManager`，互不阻塞，可并存）；
- **管线延迟面板**（系统页）：mocap stamp 龄期 / IK 求解耗时 / VR→指令端到端，左右臂各一列；
- 数采卡片完整控制面：泳道启停 + 录制控制（纯话题桥接，CLI 启动的采集节点同样可控）；
- 视频回传：门控（总开关/选路）+ MJPEG 实时预览（`~/preview/{label}` 转发）；
- 仿真与真机共用本监控，driver 服务在 sim 预设下 503 属预期（driver 未启动）。

## 文件地图

```text
morla_web_manager/
├── config.py               ← 唯一 ROS 契约处：所有话题/服务名、阈值、端口、环境变量
├── monitor_node.py         ← rclpy 节点（后台 spin 线程）：只读订阅+快照+控制发布+
│                              driver 服务客户端+视频门控+数采话题桥+延迟采样
├── launch_manager.py       ← subprocess ros2 launch 生命周期 + 孤儿检测 + 状态机
├── rate_counter.py         ← EWMA Hz 估计（RateCounter/RateRegistry）
├── data_process_manager.py ← 独立数据任务：调用统一 CLI，工作副本、日志、取消与历史
├── web_server.py           ← FastAPI：REST /api/v1/* + WS /ws/telemetry + SPA 托管
├── schemas.py              ← Pydantic 请求模型（信封 ApiEnvelope）
├── config/presets.yaml     ← 启动预设（加预设只改这里，不改代码）
├── launch/web_manager.launch.py
├── test/                   ← pytest（无 ROS 依赖的纯逻辑：presets/log_config）
├── scripts/smoke_test.sh   ← curl 冒烟（需 monitor 运行中）
└── web/src/
    ├── hooks/useRealtime.ts   ← 单例 WebSocket + useSyncExternalStore（30Hz 不穿透渲染）
    ├── hooks/historyStore.ts  ← 图表环形缓冲（200 样本）
    ├── hooks/useToast.ts      ← Toast store
    ├── lib/mapUiState.ts      ← snake_case→camelCase 归一化（前后端唯一翻译层）
    ├── types.ts               ← 与 ui_state 帧对齐的 TS 类型
    ├── api/client.ts          ← REST 封装（错误归一化为 ApiEnvelope）
    └── components/            ← App/ControlBar/Tabs/MonitorTab/HealthPanel/SystemTab/
                                 DataCollectCard/VideoCard/LatencyPanel/ChartPanel/
                                 JointPanel/LogConsole/ToastHost/StatusBadge
```

## 架构不变量（改动前先确认不破坏这些）

1. **非侵入性是最高原则**。只读订阅已有话题；只发布已有控制话题（`/teleop/armed`、`/teleop/disarm`、`/teleop/start`）；硬件模式只调 driver 已暴露的 Trigger 服务；启停走 subprocess `ros2 launch`。**不修改任何功能包、不新增硬件写入路径**。新功能先问：现有接口能否做到？
2. **ROS 契约只在 `config.py`**。话题名/服务名/阈值/端口集中于此（env 可覆盖）。别在节点或 web_server 里硬编码话题名。
3. **监控订阅 QoS 全部 BEST_EFFORT + depth=1**。只留最新帧，不在 DDS 队列堆旧数据（与 astral_arm_teleop 同一 Jetson 教训）。发布侧例外：armed/disarm/video 用 RELIABLE+TRANSIENT_LOCAL（latched），start 用 RELIABLE+VOLATILE。
4. **`/teleop/start` 必须保持 VOLATILE（非 latched）**。latched 会让晚启动的臂节点收到旧信号自动开始——这是刻意设计，别"修复"它。
5. **暂停语义 = 臂的软 disarm**。`/teleop/disarm` 只作用于 `astral_arm_teleop_node`；夹爪/灵巧手/头继续运行。"恢复"被 VR 看门狗 fault 拒绝（臂节点是权威），必须重发 `/teleop/start` 重标定。
6. **`ui_state` 是唯一实时数据通道**。WS 30Hz 单帧携带 joints/rates/state_rates/health/latency/video_gate/data_collect/collect_launch/log_tail。另含 data_process（离线处理状态）。加字段必须四处同步：`web_server._build_ui_state` → `types.ts` → `mapUiState.ts` → 组件。REST `/api/v1/state` 与 WS 共用 `_build_ui_state`。
7. **双泳道孤儿检测对称豁免**。遥操主泳道启动前 `_find_orphan()` 拒绝残留 launch，但豁免 `morla_web_manager`/`morla_data_collect` 自身；数采泳道 `start(check_orphan=False)`（纯订阅者不抢 joint_commands）。破坏任一侧都会出现"互相挡启动"。
8. **前端构建产物要重建**。改 `web/src` 后必须 `npm run build`（部署走 `web/dist`），否则页面不变；开发模式用 `npm run dev`（:5173 代理 /api /ws → :8080）。

## 已解决的坑（症状 → 根因 → 防护/参数）

| 症状 | 根因 | 防护/参数 |
|---|---|---|
| POST /start 409 Conflict | `_find_orphan` 把监控自己的 launch 当残留 | 排除含 `morla_web_manager`/`morla_data_collect` 的进程行 |
| 数采泳道挡住遥操预设启动 | 数采 launch 被当孤儿 | 数采 `check_orphan=False` + 对称豁免 |
| 8080 端口占用 (Errno 98) | 旧实例/Cursor IDE 占用 | `fuser -k 8080/tcp` 或 `MORLA_WEB_MANAGER_PORT=8090` |
| `ModuleNotFoundError: fastapi` | conda python 抢占 / 未安装 | 显式 `/usr/bin/python3`；`pip3 install fastapi uvicorn`（系统 python） |
| `bad marshal data` | miniconda py3.13 的 `.pyc` 混入 py3.10 | 清全部 `__pycache__`/build/install；`PATH=/usr/bin:$PATH colcon build` |
| 前端改了没生效 | `web/dist` 未重建 | `cd web && npm run build` + 重启 monitor |
| 停止栈后机器人按钮 503 | driver 随栈退出，属预期 | 先启动栈再点；运行中仍 503 查 driver 是否旧代码/`with_arm_driver:=false` |
| 数采"双开同录" | 残留节点 + `/data_collect/control` 谁订阅谁开录 | 数采侧 domain 单例锁；web 按 `/data_collect/state` 发布者计数>1 显示红条 |
| streamer 死后视频卡误报在线 | gate_state latched 消息活过进程死亡 | `count_publishers` 活性检查 |
| WS 只收一帧后不动 | 日志环形缓冲全量塞进 ui_state 帧撑爆 | `log_tail_for_push` 只推尾 800 行（全量走 REST /api/v1/logs） |

## 关键契约速查

- **状态机**（launch_manager.py）：`stopped → starting →(暖机2s)→ running ⇄ paused；stopping → stopped；启动异常 → start_failed`。暂停/恢复仅改状态标签，实际 disarm/arm 由 MonitorNode 发布。
- **延迟面板三项**（monitor_node._latency_summary）：`mocap_{side}`=腕姿 stamp 龄期+到达 Hz；`ik_{side}`=解析 `ik_solver_*/ik_status` 的 `dt=X.XXms`；`e2e_{side}`=`{side}_arm/joint_commands` stamp 龄期。
- **机器人模式五按钮** = driver Trigger 服务 `~/ready ~/home ~/position ~/damping ~/estop`（estop=真断电，恢复需重新一键就绪；damping=阻尼可拖拽）。调用方 `asyncio.to_thread`，后台 spin 线程完成 future。
- **REST 信封**：`{ok, message, data}`；冲突 409 + `{"detail": "..."}`；ROS 未就绪 503。

## 测试

```bash
cd /home/robot/loopkok/sdk/MORLA/src/common/morla_web_manager
# 必须 /usr/bin/python3（conda 无 pytest）；-p no:anyio 绕开系统 anyio 插件与旧 pytest 的兼容崩溃
/usr/bin/python3 -m pytest test/ -v -p no:anyio
cd web && npm run build             # tsc 类型检查 + 产物构建（前端唯一"测试"）
# 冒烟（monitor 运行中、driver dry_run 可选）:
./scripts/smoke_test.sh
```

测试风格：pytest 函数式，无 ROS 依赖的纯逻辑才进 test/（需要 ROS 图的验证走 smoke_test.sh 或手动冒烟）。新增能力必须在 CHANGELOG 附冒烟记录（贴具体返回/数字）。

## 修改流程约定（从历史 commit 沉淀）

1. **先想非侵入性**：任何新控制功能先确认对端是否已暴露话题/服务；没有则优先在对端包加接口（作为对端的能力），monitor 只做调用方。
2. **ui_state 加字段的固定四步**：`_build_ui_state` → `types.ts` → `mapUiState.ts` → 组件；漏 mapUiState 一步前端就静默丢数据。
3. **改预设只改 `config/presets.yaml`**；数采泳道按 `package: morla_data_collect` 识别条目（该条目不进遥操下拉）。
4. **每次实质改动更新** `MORLA/CHANGELOG.md`（当日日期——改动——动机——冒烟结果）+ 本包 README 对应段落；README 的契约表（话题/REST/WS 帧示例）保持与代码一致。
5. **前端样式沿用现有内联 style 对象风格**（无 CSS 框架），暗色 `#0b0f17`/卡片 `#1f2937`；新增面板照 LatencyPanel 的卡片+网格模式写。
6. 用户重视**操作安全**（破坏性操作必须 confirm 弹窗：急停/停止/丢弃录制）与**状态可见性**（每个按钮的状态机约束用 disabled 表达，不要靠报错兜底）。

## 遗留事项 / 下一步候选

- **延迟面板的 ik_{side} 在多数预设下为 stale**：`/ik_solver_{side}/ik_status` 来自 standalone ik_solver 节点，而主 pipeline 的 IK 在 `astral_arm_teleop_node` 内解算（只打 `[Latency]` 日志不上话题）——面板只能显示 stamp 可推导的三项；要细分（vr_rx/loop/apply 等）需 arm_teleop/sim 侧先把指标发布成话题；
- 延迟指标只取最新值，无 p95/max 统计（EWMA 只用于 Hz）；历史曲线未覆盖延迟项；
- 前端无组件级自动化测试（仅 tsc）；WS 无鉴权（当前局域网设计如此）；
- Jetson 部署注意：非 symlink 构建时 install 里的 yaml 是构建期拷贝，改 yaml 必须重新 colcon build。

## 离线数据处理约定（2026-10-10）

数据算法留在 morla_data_collect/pipeline；本包只负责后台进程编排、Web 任务与结果读取。统一 data_pipeline 子命令，禁止复制算法到 Web。任务必须先 prepare 真实工作副本，不能对原始 raw 执行 align/validate --apply/repair；Web 默认不隔离失败段。单任务串行，失败立即停止。配置集中在 config.py（ASTRAL_DATA_*）。完整流程可选诊断/保守清洗，输出 OpenPI/ACT。前端 DataProcessTab 与 ui_state.data_process、types/mapUiState 对齐。报告路径限制在任务根目录，不能暴露任意文件；取消按进程组处理，历史持久化。
