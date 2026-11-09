# 重构 Web UI 测试数据全流程验证 · 2026-11-09

测试通过 Web UI 操作完成，不以脚本直接调用 API 代替选型、配置、保存和运行。文件读取仅用于核对落盘 HDF5、元数据和诊断回执。

## 环境与范围

- UI：本地隔离服务 `http://127.0.0.1:18089/`，加载项目实际 `web/dist` 与公共 v2 控制面。
- 新配置：`webui_flow_fake_20261109`，从空工作空间手工搭建，最终保存 v3。Quest 输入、FR3 机械臂、Astral 夹爪、测试相机全部使用 fake 后端。
- 两条映射：`quest.right_pose → FR3 末端位姿控制器 → arm.position_target`；`quest.right_scalar → 扳机控制器 → gripper.closure`。夹爪挂载到 `arm.flange`，重合安装位姿，核对输入轴/TCP。
- 记录配置：7 维机器人观测、7 维动作、1 路图像、2 路映射输入，10Hz；诊断反馈采样上限 10Hz。测试相机为明确标记的 8×8 JPEG 测试图，不代表真实相机画质。
- Quest 数据由隔离服务注入；仅对全部实例均为 fake 的会话生效。后续改为小幅周期位姿和扳机变化，以验证反馈实际改变。
- 策略验证使用 `127.0.0.1:8001` 本地 echo 桩，校验真实 manifest/normalization/相机契约，返回当前状态作为绝对动作。它不是 ACT/OpenPI 训练 checkpoint，不能作为模型精度验收。测试后 echo 服务已停止。

## 界面操作与结果

1. 遥操页未配置时保留页面，点击“配置工作空间”进入系统编排；填写新标识。
2. 按设备分类逐个添加 Quest、FR3、夹爪、测试相机，均确认测试后端。Quest 选择右侧输入。
3. 在图形连接区选择输入/目标/控制器，建立两条连接；控制链表单核对方向、TCP 和跟随方式。在连接与标定中设置夹爪挂载到机械臂末端。
4. 验证连接、保存 v1；采集页配置任务、10Hz、两条观测/动作、图像与当前映射输入；诊断页设日志采样上限，回编排保存 v2。
5. 遥操页执行运行前检查：4 个组件依赖可用，观测 7 维、动作 7 维、图像 1 路。保存并进入运行生成 v3。
6. 启动工作站，确认 READY_DISARMED、输入有效、机器人反馈在线、测试相机 JPEG 已加载；单独点击开始遥操后进入 ACTIVE。验证暂停、恢复及动态关节/夹爪反馈。
7. 在采集页命名任务、开始录制、结束并保存，核对完整性和实际 HDF5 数据；录制开始/结束提示与状态一致。
8. 诊断页确认日志配置运行时只读、实时 200 行日志、筛选、暂停与恢复；停止后显示 closed，下载 `run_ended` 筛选日志成功，文件已留作证据。
9. 策略页启动本地 echo 策略，确认运行状态与已接受的策略指令；人工接管后显示人工遥操，交还后恢复策略控制，最后停止策略。第二段录制覆盖上述过程，落盘 dispatched 事件同时包含 human_mobile 和 policy。
10. 验证刷新恢复活动会话与 v3 配置、重新打开保存版本、运行时保存禁止。最后通过 UI 停止工作站，所有测试运行日志 clean_shutdown=true，当前无活动运动/录制/策略；预览服务保留。

## 落盘核对

- 基础流程：18.55 秒，169 帧图像；机器人观测/Quest 两路输入各 169 条，两路动作各 498 条，complete，丢样 0。
- 动态 + 推理/HITL：70.09 秒，616 帧图像；机器人观测/Quest 两路输入各 616 条，两路动作各 957 条，complete，丢样 0。记录包含 human_mobile 1593 次、policy 160 次 dispatched 事件。
- 最终正常动态录制：60.78 秒，551 帧图像；机器人观测各 551 条，Quest 输入各 550 条，两路动作各 1642 条，complete，丢样 0。
- 所有完整片段的流值均为有限值，时间戳严格递增；机械臂和夹爪反馈变化得到验证。机器人来源均标为 fake/synthetic，measured=false。
- 两段只有暂停反馈、没有动作指令的片段均被正确标为 failed，保留失败数据用于排查；修复后的界面明确显示 `required streams are empty: arm__position_target, gripper__closure`。没有放宽完整性标准。

具体文件位置、逐流计数与检查结果见 [results.json](evidence/workstation-flow-2026-11-09/results.json)。原始 HDF5 保留在 `/tmp/morla-ux-preview-store/recordings/`，未把大文件加入仓库。

## 流程发现并修复的问题

- 暂停后 activity 仍保留 human，卡片曾误显示人工遥操；现优先显示 paused 与控制权切换状态。
- 新操作曾残留上一条错误/成功提示；新操作清空旧消息，录制与策略启动校验回读状态并给出对应成功反馈。
- 未激活设备时策略曾持续取得预测但无法下发；Web 和服务均要求先显式激活。运行时拒绝策略指令时关闭预测器、丢弃剩余动作并释放权限；错误呈现在 UI。新增后端回归覆盖未激活拒绝、下发失败清理。
- 冷进程加载图像解码依赖曾耗尽首轮预测时限；现于获取策略权限前完成依赖加载，冷启动复测通过。策略重启时清理上次已解决的 operator.error。
- 推理包漏声明同步 WebSocket 客户端依赖，补充 websockets>=12；本地隔离环境安装 15.0.1。
- 同步停止回执漏传 meta 中的完整性 failure，异步包装还会用空的线程错误覆盖原因；现在两层均保留具体原因，同步/异步负例回归通过。

## 负例与边界

- 配置未保存/没有组件时不能启动；运行时配置保存禁止。
- READY_DISARMED 下策略启动按钮禁止，后端同样返回 409，不会连接预测器或取得权限。
- 错误 normalization hash 被契约验证拒绝；错误详情保留，原人工控制权不变。
- 没有动作流的录制明确失败；测试/仿真反馈不冒充硬件实测频率。
- 隔离 v2 服务未提供 ROS launch/v1 日志，界面明确提示需完整服务。真实 ROS launch、真实训练、真实模型推理、真机和 MuJoCo 不在本次测试数据验收范围。

## 自动检查

- 前端 47 项回归通过，TypeScript 检查与 Vite 生产构建通过，dist 已更新。
- 后端 61 项通过：composable_data、composable policy、WebSocket client、composable API、workstation operator。仅有现有 Starlette/httpx 弃用提示。
- git diff --check 通过。

## 证据与复用

- [可导入的全 fake 配置](evidence/workstation-flow-2026-11-09/workspace-fake.json)
- [测试专用策略清单](evidence/workstation-flow-2026-11-09/policy-manifest-test-only.json)（仅搭配对应 echo 测试服务）
- [编排](evidence/workstation-flow-2026-11-09/01-composition.png)、[运行前检查](evidence/workstation-flow-2026-11-09/02-preflight.png)、[录制](evidence/workstation-flow-2026-11-09/03-recording.png)、[日志](evidence/workstation-flow-2026-11-09/04-diagnostics.png)
- [策略交还](evidence/workstation-flow-2026-11-09/05-policy-handoff.png)、[契约拒绝](evidence/workstation-flow-2026-11-09/06-contract-rejection.png)、[空动作拦截](evidence/workstation-flow-2026-11-09/07-empty-action-rejection.png)
- [最终录制保存](evidence/workstation-flow-2026-11-09/08-recording-saved.png)、[最终停止状态](evidence/workstation-flow-2026-11-09/09-final-stopped.png)、[实际下载的关闭日志](evidence/workstation-flow-2026-11-09/closed-run-log.jsonl)、[前端测试输出](evidence/workstation-flow-2026-11-09/frontend-tests.txt)
