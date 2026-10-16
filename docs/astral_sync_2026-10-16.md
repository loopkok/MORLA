# Astral WS 更新核对与 MORLA 同步（2026-10-16）

## 源码边界

- Astral 源：`/home/robot/loopkok/sdk/astral_ws`，`main` 为 `40b87dac762a40fef22d54a251083657c0c1fa5e`；远端 `main` 同一提交，源工作区干净。
- MORLA 基线：`ee43910b29ff210fc9aed91f44086f23b0118106`。上次同步至 Astral `48642270361bf525d91b5b73f7bea893d9d918fe`。本次只审查后续 `b717e12`、`bb90d1e`、`40b87da` 三个提交（11 个变更文件）。
- 源目录不修改；不连接机器人或 Quest 真机。

## 逐项处理

| Astral 更新 | MORLA 现状与处理 |
|---|---|
| `b717e12` Web 默认监听 `0.0.0.0` | MORLA 的 `morla_web_manager` 已支持 `web_host:=0.0.0.0` 和 `MORLA_WEB_MANAGER_HOST`，局域网使用方式已具备。保留默认 `127.0.0.1`：Web API 可发控制请求，当前未实现身份认证；自动向局域网开放会改变控制面访问范围。需要局域网访问时显式设置监听地址。 |
| `bb90d1e` Quest 新版 APK 严格包校验 | MORLA `quest3_mocap.yaml` 与 `PacketGuard` 原已默认 `allow_legacy_packets=false`，支持单调时钟自动校时、session/sequence/age 校验；旧 APK 可显式开启兼容。同步启动时的保护模式日志，显示 max age 和 legacy 开关，旧包警告保留。 |
| `40b87da` Astral HITL 肘先验修复 | 在 `AstralTeleopArmNode._control_psi_ref()` 检查统一 `PolicyLease.activity`；HUMAN 接管期间不提供绝对人体肘角给软/硬 IK，仍从 `q_cmd` warm-start。普通 Quest 遥操及释放接管后继续按 `use_human_elbow` 使用肘先验。新增回归覆盖 HUMAN、空闲及关闭先验。仅 Astral 机械臂适用，不改 Nero 独立 IK。 |
| `40b87da` 源延迟审计文档 | 这是源码与配置审计，没有运行时参数变更。MORLA 保留其适用结论：主链路应检查最新帧 QoS、150 Hz 控制预算、同步诊断/日志负载、Quest TCP 队列及 IOBT 150 ms 肘平滑。Astral SDK 原生 `poll_receive(10 ms)` 等待持有 GIL 的离线证据需在 MORLA 使用的 SDK 版本复核；未把源项目的隔离测量宣称为 MORLA 真机延迟，也不在本轮擅改驱动或滤波。 |

## 验证与未覆盖范围

- 定向回归：HITL 肘先验、Quest PacketGuard、Web 配置共 **9 passed**。
- `scripts/build.sh --mode symlink`：**31/31** ROS 包构建完成。
- `scripts/test_ros.sh`：**366 passed**、2 deselected；三个独立 Astral IK/映射回归均 PASS（有一项第三方 `hppfcl` 弃用警告）。
- `scripts/test_policy.sh`：**156 passed**，包含 HITL ownership/writer 边界。
- 未启动真实 Astral、Nero、XHand 或 Quest。
- 现场验收仍需比较策略 HUMAN 接管前后的肘角连续性，并测量真实输入、控制命令和关节反馈的 p95/p99 延迟；静态测试不能证明物理响应。

## 工作区隔离

执行时 MORLA 工作区已有另一项仿真 viewer 默认值工作的未提交修改，涉及 `CHANGELOG.md`、`morla_bringup`、Web server/test 和 Nero 仿真 README。本次没有改动这些文件；本日志独立记录同步范围。
