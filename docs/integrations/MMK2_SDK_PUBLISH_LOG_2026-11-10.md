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

- 提交前专项复验：18 passed、1 warning（Starlette/httpx 依赖弃用提示），7.12 s；暂存区 `git diff --cached --check` 通过。
- 最初本地功能提交：`3d854d4396a42850b6024381b0b41ac8e5128ee7`，56 个文件，6498 行新增、7 行删除。
- 命令行 `git push origin main` 因 HTTPS 凭据不可用失败：`could not read Username for https://github.com: Device not configured`。未修改认证配置，改用用户已连接且具有 push 权限的 GitHub 接口。
- 两张 PNG 以原始字节创建 blob；其余文本按 UTF-8 创建。远端生成的完整文件树为 `3ba83a8c21f643186e55d2e8e2209bee93c90fa9`，与本地功能提交树 SHA 完全一致，包含许可证、SDK 原始协议、可执行脚本权限和全部证据。
- 远端功能提交：[95fa56d792f1c16c1da6970a103800b4419dab1f](https://github.com/loopkok/MORLA/commit/95fa56d792f1c16c1da6970a103800b4419dab1f)，父提交为原基线 7a5addc。通过非强制、带 expected_sha 的 main 引用更新发布，接口返回成功。
- 随后 `git fetch origin` 成功，origin/main 与上述功能 SHA、文件树一致。GitHub 接口生成的提交身份/时间与本地不同，因此 SHA 不同；本地 main 已按相同文件树对齐到远端功能提交，原始本地提交保留在 reflog 中。
- 本回执作为后续文档提交发布。最终提交号和远端/工作区一致性在回复中提供，避免日志自引用。
