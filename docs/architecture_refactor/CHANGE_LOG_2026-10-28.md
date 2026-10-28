# 架构重构详细提交记录：2026-10-28

## 版本与范围

基线：9449e59ee0850d1844247e3fa215e28258c167d9；目标：origin/main。本次提交保存当前完整实现工作区，不宣称 COMPOSABLE_ARCHITECTURE_PLAN.md 的全部验收已完成。不修改外部 MORLA-piano、morla-xr 或设备 SDK 仓库。

## 主要改动

1. **契约与组件**：morla_core/contracts、composition、plugins 和 schemas 定义组件端口、独立状态动作布局、装配图/部署、锁定插件版本、来源身份和哈希。限制配置大小、深度、重复字段及异常数值；静态发现阶段不执行插件代码。
2. **调度与执行**：新增 morla_runtime、morla_interfaces、system.launch.py，统一生命周期、操作状态、资源所有权、租约、命令序号与期限、故障和急停。接入 Nero/Astral/XHand/Wuji 原生桥，共享设备会话；旧 Python/C++ 驱动增加互斥保护。真实能力不足时拒绝，不回退为 fake。
3. **输入与遥操**：新增 normalized/ingress/mobile/gamepad/pose 接口与 ROS 输入订阅，新增 morla_teleop controller、IK 与手部重定向桥。同源 deadman、端口独立时效、重连及锚点重建保护。ROS 输入驱动自动启动及实物输入验收尚有收尾项。
4. **数据**：Schema v3、通用 codec、有界异步 recorder、数据来源/有效性及时间戳、独立 state/action、任意相机键；更新在线采集、对齐、修复、验证、LeRobot/ACT 导出、回放与处理脚本。PyAV 帧率通过分子/分母处理，避免新版本 AVRational 无 int 转换错误。
5. **学习与策略**：新增 morla_learning TrainingSpec/ModelManifest、版本锁定后端桥、投影与容量检查；新增 RuntimePolicy、ManifestRemotePredictor、RuntimeReplay。Web 提供策略启停和单步回放；真实后端 loader/训练/推理、serve training-spec 接线及新增路径最终独立审查仍待闭环。
6. **Web**：新增 v2 API、持久化 revision、编译/预检、请求边界、输入配对、录制、动态组件与绑定表单。组合页面隐藏旧全局 ControlBar，避免展示旧运行状态及误用旧急停。前端 dist 随源码重新构建。
7. **验证与文档**：新增计划、执行/验收规格、progress、OPERATIONS、SDK、12 个 Profile fixture、单场景 builder、独立审查和性能证据、示例插件、分层脚本与 CI。CI 文件已编写，不能等同远端 CI 已运行。

## 提交前实际复验

在 macOS 本机分别使用已有 conda Python 3.13 与独立 Python 3.12 依赖目录，无 ROS 环境：

- MORLA_PYTHON=/opt/anaconda3/bin/python bash scripts/test_composable.sh --layer pure：103 passed、1 skipped，退出码 0。
- Python 3.12 + /tmp/morla-composable-deps 执行 --layer fake：23 passed、1 warning，退出码 0。
- conda Python 执行 --layer sim：7 passed，退出码 0。
- Node 执行 tsc -b、vite build：通过；61 modules，产物 index-Je7NMGOM.js。
- git diff --check：通过。

原始复验输出保存在 evidence/submission-20261028/。不同层和历史报告存在测试重叠，禁止把全部数字直接相加作为唯一测试数。历史浏览器、驱动和独立审查范围见 progress.md 及 reviews/，本次没有重新连接实机或重新运行浏览器测试。

## 审查与剩余门禁

已保留 integration_review.md 和 runtime_review.md 的问题、反例与修复记录，包括 resolved 哈希绕过、失败停止被掩盖、命令回显冒充测量、复位/陈旧 ACK 及保存 revision 一致性等。它们不能替代最后新增策略/Web/输入路径的完整最终审查。

未验收：ROS Humble colcon/IDL/C++ 构建及 ROS 集成、实体输入/机器人/相机、真实 loader/训练/模型、8 小时长稳、全部 VALIDATION 编号和独立开发者扩展考试。progress.md 为交接入口，本提交不授予 real 硬件已验收状态。

## 提交与回退

主内容使用一个完整源码提交；成功推送后另作回执文档提交，记录实际 SHA 和远端核对，避免预先写成功或自引用。提交明细以 git show --stat 和 git log 为准。

回退应先停止会话、确认物理 writer 已退出，再使用 git revert 撤销相应提交；保留原始数据、部署配置与标定，禁止删除活动锁来绕过互斥。禁止将 v3 原始数据原地降级。
