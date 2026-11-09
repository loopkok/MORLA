# 参考图工作站 UI 实现记录

日期：2026-11-09。基线：8878a7a。依据用户提供的四张界面图重新实现前端视觉与布局；本轮改动尚未提交。

## 实现

- 浅色工作站外壳：顶栏、工作空间、分组侧栏、卡片、按钮与状态层次。
- 遥操控制：主画面、控制会话、输入/设备反馈、录制入口；设备选型、连接标定、控制链和运行前检查在本页可配置。
- 系统编排：真实设备与控制器节点、SVG 连接、节点拖动、缩放、自动布局、端口选择、兼容控制器选择、建立连接、组件属性与设备库。所有连接写入现有 assembly.bindings，不维护第二份运行配置。
- 主画面选择依据执行器实际后端（含 deployment 覆盖）。sim 选用已添加的 sim 相机并保留 MuJoCo 视角交互；real/fake 选用对应非 sim 相机流。仿真中没有仿真相机时显示配置提示，不替换为真机图像。未初始化和流失败时分别给出提示与重试。
- 数据采集：实际相机预览、任务名、开始/结束录制、通道及诊断设置；训练与推理页接入既有 policy/start、policy/stop、人工接管和交还。
- 原工作位、手机配对、完整运行诊断与日志置于完整控制面板；原独立控制、数据处理、监视、健康和钢琴入口保留。
- 复用既有配置、检查、保存版本、初始化、请求追踪、后端门控和确认逻辑。后端未修改。

## 验证

- 42 项 node:test 回归全部通过（含新增的画面后端选择与空仿真相机场景）。见[测试输出](evidence/workstation-reference-2026-11-09/tests.txt)。
- tsc -b 和 Vite 生产构建通过，web/dist 已更新。
- 隔离预览加载旧 CR5 fake 配置，复制执行器、选择 quest.right_pose → arm_copy.position_target，兼容控制器为 CR5，建立连接后实际控制链数从 1 增至 2。
- 检查通过后保存至隔离存储 v3，界面返回运行概览，未触发设备连接或运动。
- 390px 下策略页及编排页 document.scrollWidth 均为 390；画布在自身区域滚动。
- 图示里的在线状态、延迟、触觉、历史 episode、训练曲线和模型仓库示例数据未复制到实际产品；现有 v2 没有提供的功能保留原工具入口或明确说明。
- 本轮没有执行实机、ROS 或 MuJoCo 渲染环境验收，实际流需对应运行环境和已配置相机。

## 截图

[遥操控制](evidence/workstation-reference-2026-11-09/teleop.png) · [系统编排](evidence/workstation-reference-2026-11-09/composition.png) · [数据采集](evidence/workstation-reference-2026-11-09/capture.png) · [训练与推理](evidence/workstation-reference-2026-11-09/policy.png) · [窄屏](evidence/workstation-reference-2026-11-09/mobile.png)。前两张为最终构建完整页面，其余为本轮对应页面验证时的截图。
