# Quest 最新状态快照与 depth=1 修复

日期：2026-11-03。基线：`884719f`，承接 BUG.md 的现场定位，不修改头显项目。代码和本记录随本轮提交归档，实际提交与推送结果另记回执；现场验收仍待执行。

## 问题与实施选择

原事件流把 pose、controller_state 和 landmarks 分别发布到同一按手话题。订阅 KEEP_LAST depth=1 时，后面的按键或关键点可能覆盖位姿。单改接收回调无法恢复 DDS 已经覆盖的消息。

本次保留 BEST_EFFORT/KEEP_LAST/depth=1，改电脑接收器的公共输出和框架解析器，不增加运动数据积压，不延长 250 ms 时效，不改 IK。一个话题内每条消息都带每只手最新状态；任一字段到达立即更新并发布，没有配对等待、拼帧定时器或独立控制线程。TCP 读取边界不视为采样帧边界。

新增 `morla_input/quest_snapshot.py` 负责无 ROS 的协议验证、快照发布缓存、按键操作编号和有界确认。`quest_events.py` 的公开 QuestEventSource 仅接受 v3，复用内部字段解码器生成既有 InputFrame 端口。内部 v2 解码器只是字段处理实现和回归测试对象，不是可部署的旧协议兼容入口。

## 数据协议与时效

原话题 `/quest3/{left,right}_teleop_events` 保持，发布/订阅均为 BEST_EFFORT + KEEP_LAST depth=1。v3 外层包含 version/kind=snapshot、epoch、side、source、generation、sequence、sent_ns、basis、frame_id、fields、commands、command_sequence、commands_dropped。

fields 按当前模式包含：

- controller：pose，及已收到的 controller_state（trigger、grip、axes、buttons）。
- hand：pose，及已收到的原始 landmarks。

有效 pose 建立来源/坐标身份；其前到达的状态不会凭空建立运动基准。每个字段有独立 sequence、sent_ns、frame_id、values。外层时间是当前字段中的最新时间；它不刷新旧字段时间。位姿从首次出现到被替换只处理一次，过期则从端口移除。新按键不能续命旧位姿，新关键点也不能续命旧腕位。

InputFrame.port_metadata 增加可选 sample_sequence/source_time_ns，原 received_ns/frame_id 保留。BindingEngine 与重锚控制器用一致的样本身份去重，因此携带同一位姿的后续快照不重复算 IK；即使两个不同样本的时间恰好相同，序号仍可区分。其他输入没有这些可选元数据时保留原来行为。

公共 Quest 从快照读取 axes，不再额外订阅 Joy 来更新相同端口。旧 ROS Pose/Joy/Landmarks 仍正常发布，旧 Profile 可继续使用；公开端口名称和组件 descriptor 不变，保存配置无需因协议变化重新搭建。

## 操作按键、确认与边界

接收器在原始 controller_state 到达时识别上升沿，按现有优先级每次产生至多一个操作：右 grip、右 A/B/摇杆按下、左 X。初始已按住只采纳电平；连接、模式改变或超过 250 ms 的按钮数据空档后，不凭空补出一次按下。

每手 journal 最多 32 条、250 ms 有效，包含独立 id、发生时间和动作。后续快照重复携带短时记录，所以“按下消息丢掉、只收到松开快照”仍能识别操作。接收端按 epoch/side/generation/id 去重，并给公共工作站传稳定操作身份；同一快照内多次同名操作也有不同身份。过期记录不执行，溢出显式累计 commands_dropped；没有承诺无限丢包下的可靠交付。

ACK 在 `<原话题>/ack` 上使用 BEST_EFFORT/depth=1，不等待往返，失败计数不会断开输入或阻止控制。确认表示接收端已处置记录，不是业务成功、重锚完成或物理动作完成；业务错误仍由既有操作回执/日志体现，不能自动重试破坏性操作。

ACK 记录按消费者区分且有界。为避免一个订阅者把另一个的记录提前清掉，收到 ACK 后不全局删除 journal，而是让短时记录按 TTL/容量自然退出。这样多个观察者互不抢事件；客户端重连仍先建立命令基线，拒绝补执行旧操作。若整个 250 ms 窗口内没有有效快照，操作可能丢失；此时应由操作者重新发起，不在网络恢复后延迟运动。

## 重连与手柄/裸手

接收器 epoch 变化清空双方发布缓存；每侧 pose 的来源或坐标系变化增加 generation，清除该侧旧字段、按钮基线和 journal。旧代次消息不能恢复控制器状态。

左右最新字段独立保存，单手选择不屏蔽另一控制器的操作身份。接收器在 side 过滤时仅额外发布另一侧的公共快照身份，标准运动话题仍遵守原过滤。公共 Runtime 仍可能以整个输入实例退役许可，因此模式切换后需要重新开始，不声称另一臂完全不中断。

消费者连接/重连时对两只手分别建立操作基线，即使另一手稍后才发来快照，也不会回放断开前的 X/A 等操作。手柄切到裸手后旧 grip/trigger 不会授权新手部输入，裸手捏合仍不自动发起全局开始遥操。

## 验证

软件回归：336 passed、3 skipped、1 deselected，18.58 s。覆盖核心合同、公共 Runtime/遥操/学习、CR5/LoopHand/EPG、Quest 字段/快照、Vader、采集、策略、Web/HITL 和机器人模型。3 项跳过分别因缺少 hypothesis 和两项 Jodell SDK；排除项为此前已复现的手机输入基线失败 test_mobile_input_and_recording。

新增 19 项快照测试，以及公共映射样本身份回归，覆盖丢弃全部 pose-only 消息仍能取到位姿、只收到最后快照也能建立模式、独立时间/序号、单快照统一时钟基准、短按/松开丢帧、操作顺序与同名编号、失去 ACK 不重复执行、过期操作、重连双方基线、手柄/裸手代次、腕/关键点独立时效、单手运动与另一侧按键、损坏报文、journal 容量和 ACK 身份、实际 RosInputManager 调用路径、ACK 发布故障隔离，以及订阅仅使用 depth=1 快照话题。

## 提交归档检查

提交前通过 SSH 命令级 URL 重写 fetch origin/main，确认远端仍为 `884719f460d9abda00295c62cf52e9007153bff4`，与本地 HEAD 一致，没有需要合并的新增提交。保持 main 分支正常快进推送，不重写历史、不修改持久 remote 配置。

功能实现完成后的最终回归结果为上面的 336/3/1；提交准备只更新归档文字，没有再改动功能代码，因此不重复运行相同套件。提交前再次执行修改/新增 Python 的 AST 语法检查、`bash -n scripts/test_composable.sh` 和 `git diff --check`，并核对暂存区包含新协议模块、新测试和本日志，未纳入临时 ROS 替身、基准工具、运行数据或依赖环境。详细推送回执将独立提交。

另在临时 ROS 消息替身中运行 4 项实际接收器发布方法测试，包含 array.array 按键序列可 JSON 序列化、快照携带 pose、按侧序号和另一控制器身份过滤。这只是消息边界测试，本机无 rclpy，不代表 ROS executor、DDS 丢包复现或实体头显验收已通过。

4000 个暖机后的本机软件样本（发布缓存→JSON 编解码→公共解析器，无 ROS/IK）测得 P50 约 0.117 ms，P95 约 0.133 ms，普通无操作快照平均约 576 字节。这不是端到端延迟、Linux 性能保证或与旧链路的严格基准；journal/裸手关键点会增大消息。未重新构建前端，因为页面源码和配置接口没有改变。

## 部署和现场验收

先在页面停止工作站、结束旧 Web 和托管/外部 Quest 接收器。更新同一工作区的 morla_input、morla_teleop、morla_web_manager 后重新构建并 source：

```bash
cd /实际路径/MORLA
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to morla_input morla_teleop morla_web_manager
source install/setup.bash
bash tools/start_morla_web.sh
```

公共受管输入无需更改配置；external 模式必须自行重启新版接收器并开启 publish_teleop_events。头显 APK 和现有 Quest.right_pose 绑定不用因此更改。不要运行旧接收器配新解析器，旧 v1/v2 会被明确拒绝。

Linux ROS 环境中运行 `scripts/test_composable.sh --layer pure`（包含真实 ROS 消息依赖的测试），并运行 fake 层及 `test_workstation_operator.py`。现场重新执行 BUG.md 的合成 90 Hz pose+state 覆盖实验，保持 BEST_EFFORT/depth=1，确认不再按消息类型偏置丢位姿，再用实体 Quest 检查重锚、连续移动、短按、暂停、断流恢复、双手及裸手。查看日志中各端口 received_ns/source_time_ns/sample_sequence，不能仅看外层帧率；确认长期运行没有堆积和反复重锚。

本轮未启动实体输入、机器人、真实模型或完整 Linux ROS/DDS 服务，尚不把 BUG 的现场验收关闭。
