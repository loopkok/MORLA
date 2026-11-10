# MMK2 SDK 驱动实施与发布日志 — 2026-11-10

用户要求将本地更新补齐 changelog 与日志后提交、推送。目标仓库为 `https://github.com/loopkok/MORLA`，沿用当前 `main` 分支，不创建额外工作树。

## 提交前状态与范围

- 基线：`7a5addcff9de891f63a2c51786f1c8e0099f6cda`。本次 fetch 成功，提交前 HEAD 与 origin/main 一致（领先 0、落后 0）。
- 新增独立 morla_mmk2_driver：SDK 协议、七组执行器、共享连接、三路 RGB-D、严格反馈/目标边界、只读门禁、受限微分 IK、ROS 入口与 launch、静态插件、Web/Quest 模板、许可与来源。
- 公共层扩展底盘原生速度语义及 ROS 反馈；Web 继续通过 descriptor 发现组件和字段。增加 SDK fake Web 验证宿主、录制审计脚本与测试入口。
- 根 CHANGELOG、设备接入指南、MMK2 SDK 说明和测试证据同步入库。厂商生成文件保持原样并记录 SHA-256；不纳入 build、egg-info、缓存或原始大体积录制。

## 验证与限制

开发阶段联合回归 309 passed、1 skipped；公共 Web/采集/策略 37 passed；原 MMK2 MuJoCo 21 passed，合计 367 个通过案例。驱动 18 项包含在联合回归中，不重复累加。

提交前再次执行 `MORLA_PYTHON=/opt/anaconda3/bin/python bash scripts/test_mmk2_driver.sh`，输出另存到证据目录的 publish-check.log；执行 `git diff --check`。本轮不因发布而重复已通过且没有新改动的全部回归。

实际浏览器两段录制均完整，19/19 维、七组变化和三路 JPEG 已审计；原始故障过程保留，不把最终 STOPPED 描述成全程无故障。录制结束采用“暂停遥操→结束并保存→结束工作站”；直接保存曾出现授权撤销，根因未确认，新增原始故障诊断，详见 MMK2_SDK_DRIVER.md。

控制器示例 IP 为 172.25.14.1，用户明确实机自行测试，本轮未连接。默认只读；SDK 没有真实使能/失能/急停实现，保持/零速度请求不保证物理停止。本机无 ROS 2/rclpy，Linux 构建、ROS/DDS、真实 MMK2/Quest、映射、断网与长稳均未验收。

## 发布回执

本次先提交功能、文档和证据，再推送 main。成功后的功能提交 SHA 与远端核验记录将补充在本节；日志回执使用后续文档提交发布，最终 SHA 在回复中提供，避免日志自引用。
