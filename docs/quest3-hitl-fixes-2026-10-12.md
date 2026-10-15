# Quest 3 人在环修复记录（2026-10-12）

本次修复覆盖从 Quest 输入、策略/人工控制交接，到数采数据标签和 LeRobot 导出的完整路径。

## 控制权交接

- `/policy_inference/state` 增加 epoch、递增序号、发送时间、切换 ID 和 pending 标志。所有遥操 writer 使用同一租约规则，拒绝过期、乱序和非法状态。
- 策略和回放切换等待当前 schema 中所有 writer 确认 ownership transition。接管使用 `~/prepare_reanchor`：只建立锚点并返回一次性 token；所有准备成功并提交 HUMAN 后，臂 writer 才能凭匹配 token 开始发送。缺 writer ACK、服务失败、输入过期都会中止切换。
- pending 期间控制循环不发送策略/回放动作，仍先处理 `stop`。首轮策略规划移到后台，首个动作只在规划和新鲜观测都就绪后发出。所有权刚提交的周期不额外发送第一帧。
- 人工接管失败时不恢复可能陈旧的旧策略块，而是暂停并要求显式重规划。策略引擎拆除移出控制周期。
- 臂在策略租约过期或无效时强制 disarm，旧 VR 锚点不会自动恢复。头部 writer 在 HUMAN 后等待新鲜摇杆样本，关节反馈也必须新鲜。
- 夹爪用策略实际输出的闭合比初始化人工控制，等待新鲜 trigger/手部输入跟上后才接管；交接前缓存输入不会触发夹爪运动。Wuji 手部 retarget/HOME 在策略拥有话题时停止，人工恢复时以实测关节状态为起点并限速。
- 左 grip 在 HUMAN 状态仍走人工 `/teleop/start` 重锚；右 A 交还路径保持由常驻 `controller_start_gate` 处理。

## Quest 网络输入

Quest 输入后续于 2026-10-15 更新为 `@q3m <session> <sequence> <monotonic_us>|` 包头，ROS UDP/TCP 和 standalone 共用同一 `PacketGuard`。PC 接收端发送带 nonce 的 `@q3sync` challenge，Quest 回传单调时钟收发时刻；接收端用四时间戳公式估计时钟偏移并对源年龄限幅（默认 0.5 秒），无需 Quest/PC 系统 UTC 对时。重复/乱序和已退役 session 仍会被拒绝，ROS 头戳回到接收主机 wall-clock 时间域。

`allow_legacy_packets` 仅接受不带任何 envelope 的原始旧载荷，不提供源时效、序号及 session 保护；旧版 `@q3` UTC-ms envelope 不作为 legacy 接受。MORLA 常规配置继续设为 `false`。Quest 端相应 Unity 源码已提交到本机 `astral-tracking` `a6e262c`；该提交记录尚未使用 Unity 重建或安装 APK，因此设备侧握手和时效仍待现场验证。

另同步 Astral WS 2026-10-14 HOME 行为：HOME 到回程最后途经点后，保持其它关节、单独把 J4 置零，再到全零位；该过渡只进入 HOME park 路径，不改变工作位/段间回位。Astral 双臂工作位 J1 途经点由 ±1.6 调整为 ±1.2 rad。

## 数据标签与导出

采集节点把策略所有权状态写成 episode 事件。对齐把 `intervention`、`intervention_valid`、`control_owner` 对应到帧；清洗按选中的来源帧同步标签；v2.1/v3 导出保留三列。合成 hold 帧标为非人工。历史数据没有所有权事件时保持 unknown/invalid，不从轨迹猜测操作者身份。

真实 LeRobot 加载验证发现 `shape=[1]` 的新增标量标签应以 scalar Arrow 列写入；旧实现写成 `list<bool>`，现代 v3 reader 会拒绝。现已修正为 scalar bool/int64，并加入列类型回归。

## 验证

- policy inference 套件：143 项通过，其中 node flow 26 项、HITL 边界 3 项、writer 7 项；新增 HDF5 真机回放中的接管、帧冻结与交还续播回归。
- 数据管线回归：117 项通过、1 项跳过；包含 LeRobot Python 环境下的离线导出/转换测试，以及 ROS Humble 系统 Python 下的采集节点测试。
- 臂与遥操兼容回归：70 项通过；包含重锚、HOME/初始化、控制闸门和生命周期日志。
- Quest 包检查：3 项通过。
- `astral_data/raw/pick_place_merged` 抽样 episode 0、50、99：旧 meta 把相机标为 `video8/video0`，而相机 HDF5 已改名为 `base/left_wrist`；只在 `/tmp` 副本应用 `video8=base,video0=left_wrist` 映射，raw HDF5 以硬链接只读使用。输出 3 段共 611 帧；对齐、清洗、校验、v2.1 导出、v3 升级及现代 LeRobot `LeRobotDataset`/ACT CPU 预处理均通过。
- 7 个相关 ROS Python 包 `colcon build --symlink-install` 成功。
- 三段历史数据均没有 `control_owner` 事件，因此 611 帧被保守标为 owner unknown、intervention invalid；校验报告的 W7 是旧 armed/disarm 快照冲突警告，不代表标签被推断。

策略回归命令：

```bash
cd /home/robot/loopkok/sdk/astral_ws
ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 \
  src/common/morla_policy_inference/scripts/run_policy_inference_tests.sh
```

离线回归在 OpenPI Python 环境执行，导出深度检查使用现代 LeRobot Python 环境。源目录和输出路径见 `/tmp/astral-hitl-raw-real-v2-20261012/report.json`。

本次未启动真机或 Web UI，也未在 Windows 构建 Unity APK；设备端时钟同步和新 APK 安装后仍需现场确认。
