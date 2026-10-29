# 可组合架构实施记录

更新时间：2026-10-29。以下重构实施记录原基线为9449e59，重构代码已提交，实际提交与推送见[2026-10-28回执](PUSH_LOG_2026-10-28.md)。本文后续原始测试数字保留阶段含义，不代表当前全部代码或目标主机已验收。

## 当前主机测试交接

后续CR5/LoopHand/EPG/Vader5已补软件接线与专项测试，当前本机pure172、fake23、SDK5通过；Vader5 SDK同级迁移后SDK5与输入专项11再次通过。完整Linux ROS编译、DDS、真机、真实训练后端和长稳仍不能据此标为通过。

主机执行入口见[编译与真机测试手册](../testing/host_build_and_hardware_acceptance.md)、[逐项报告模板](../testing/host_acceptance_report_template.md)。手册列出目标主机步骤、当前Nero资产哈希问题、LoopHand real运动禁止及Astral夹爪反馈限制；原有旧链路实测与新runtime验收分别留证。

本文件记录已实现范围和验收边界，不把实现代码等同于真实设备验收。

## 已落地的软件

- R0–R2：12 个旧 Profile 快照、独立 v3 契约、静态插件目录、受限配置解析、装配/部署编译、冻结哈希、组件 SDK 和外部插件符合性样板。
- R3–R4：生命周期、资源文件锁、租约/epoch/序号/时效网关、异步操作、部分失败停止；既有设备原生桥、共享 Astral 会话、ROS/UVC 相机、单 MuJoCo 场景及可选离屏渲染。real 能力需逐设备验收。
- R5：统一输入帧、Xbox/mobile/pose 接口、同源 deadman、重连释放要求、joint jog、Cartesian IK、既有手部重定向和选中插件加载。
- R6：独立 state/action 布局、任意相机键、通用传感器 codec、有界异步采集、来源与有效性记录、对齐/修复/导出兼容路径。
- R7：TrainingSpec/ModelManifest、后端版本与 checkpoint 绑定、ACT/OpenPI 桥、特征/几何/归一化握手、有界异步策略、过期动作拒绝、冻结契约回放。Web 提供 policy start/stop 与显式 replay step API。真实 loader、训练和模型推理尚未验收。
- R8：持久化 revision、预检、动态组件表单、绑定/部署编辑、运行监控、手机配对、采集和急停。真实浏览器端到端已执行。
- R9–R10：分层测试脚本、CI 配置、独立审查与反例、扩展 SDK、10,000 次编译和 gateway 测量。远端 CI 未执行，完整发布门禁尚未满足。

## 可复现证据

- `scripts/test_composable.sh --layer pure|fake|sim|web|learning`：源码环境需先安装 dependencies/composable-test.txt，可通过 MORLA_PYTHON 选择解释器。
- 本轮 conda Python 3.13 纯逻辑：103 passed、1 skipped；跳过项不能计入通过。日志 `/tmp/morla-pure-final.log`。
- 本轮 fake 数据/策略/Web 联测：23 passed（PyAV 19 有理帧率兼容已修复）；场景测试 7 passed。日志 `/tmp/morla-fake-final.log`、`/tmp/morla-sim-final.log`。
- Runtime、驱动离线及 MuJoCo 证据：`evidence/runtime-20261028/manifest.json`。
- 前端浏览器测试：`src/common/morla_web_manager/web/test/composable_browser.mjs`；临时截图和结果 `/tmp/morla-composable-browser/`。TypeScript/构建及原有 18 项 Node 测试通过。
- 独立审查：`reviews/integration_review.md`、`reviews/runtime_review.md`；修复后回归和原漏洞反例分开，旧漏洞脚本不作为验收通过证据。
- 性能：`performance.json`。仅代表本机短时软件测试，无实机实时性承诺。

## 尚未完成的验收

以下项目必须保留 BLOCKED_ENV / NOT_RUN，不能把整个计划标为 COMPLETE：

1. Linux ROS Humble 的 colcon/IDL/C++ 全量构建、真实 ROS topics/QoS 和 ros2_control fixture 运行。
2. 各机器人、末端、相机、Quest、Xbox、手机实物的 T4 验收；固件、停止 ACK、总线断连与部署标定。Astral 夹爪仅命令回显时禁止冒充真实测量。
3. 锁定 LeRobot/OpenPI 版本的实际 loader、少步训练、checkpoint 加载和真实预测；本机缺目标后端/GPU/模型。
4. 8 小时稳定性、同硬件新旧性能对照、真实输入/相机压力及部署恢复演练。
5. VALIDATION.md 每个编号逐项形成可核验结论；当前软件回归不是该完整清单的替代。
6. R10 独立开发者接入考试与现场发布/回退验收。

现有实现保留旧入口及数据；运行 composable 时通过全局/物理资源锁阻止旧 writer 争用。不得绕过未满足的 real 前提，也不得以 fake 成功代替 real 验收。
