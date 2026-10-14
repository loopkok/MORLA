# Quest 3 人工介入代码检查（2026-10-12）

本次检查 Quest 输入、策略仲裁、臂重锚、夹爪、回位按键和采集/导出。没有修改控制逻辑，没有启动真机或 Web。结论来自当前源码及离线复现；不代表真实网络或机械执行已验证。

## 已有流程

左控制器 grip 上升沿：策略/回放活跃时发送 takeover，否则发送 teleop/start + armed。policy_node 冻结引擎，异步调用配置的臂 reanchor，全部成功后进入 HUMAN 并释放夹爪门控。HUMAN 时策略不发布动作。右控制器 A 交还；策略重新建立引擎，回放重设 offset；被接管前为暂停时，交还保留暂停状态。臂重锚同时重设 FK 原点、VR 零点、IK/安全滤波状态和肘部参考。腕流超时后臂 disarm，需要再次启动。

以上设计应保留。主要问题在切换边界、旁路控制及数据来源标记。

## 优先处理

### 1. 接管等待绕过停止队列；新接管当帧仍运行策略分支

位置：`src/common/morla_policy_inference/morla_policy_inference/node.py`，`_tick`（1210）、`_drain_cmds`。

`_tick` 先轮询 pending takeover，未完成即 return，因此 `_drain_cmds` 中“pending 时仍允许 stop”的逻辑根本没有执行机会。普通 `/policy_inference/cmd=stop` 可等待整个 3 秒接管超时；这里不是说独立硬件急停接口被阻塞。

从无 pending 的 tick 开始，drain 创建 pending 后没有再次检查，仍调用 `_policy_tick` / `_playback_tick`。冻结引擎不能阻止 `_resend_last` 或回放写指令，破坏接管期间不发布的约定。

修改方向：控制周期优先处理停止；处理命令后再次检查 pending，pending 时禁止所有策略/回放/暂停保持发布。用真正的队列和未完成 future 回归覆盖两个顺序。

### 2. 重锚过早武装；超时回滚无法阻止迟到效应

位置：policy_node `_cmd_takeover`（1027）、`_abort_takeover`（1126）；臂 `_reanchor_teleop`（1056）。

每个 reanchor 服务立即设置对应臂 `_armed=True`，其他臂还可能未完成。回滚只发 disarm 并清 pending，不撤销已经发出的 Trigger 请求。若服务在回滚的 disarm 之后执行，会重新武装该臂，而策略已恢复，出现双写可能。一次性 cancel future 也不能撤销已在服务端运行的效应。

另外，任意一次 `call_async` 同步抛异常，会在引擎已经禁用、pending 尚未建立时退出，没有事务回滚，引擎可一直被冻结。

修改方向：准备重锚与提交武装分离；使用事务代次/有效期并由服务端检查，或设置独立输出所有权门，在全部确认前及取消后禁止遥操输出。失败时取消未完成客户端任务、清理状态，并从实际姿态重新规划或保持暂停，不盲续旧动作。

### 3. X 回位旁路没有策略所有权检查

位置：`src/common/morla_bringup/morla_bringup/controller_workpos_gate.py`（107、140）。

该门只检查数采状态，未订阅策略状态。未录制而策略正在执行时，X 仍发 disarm，100ms 后发 init_direct。disarm 只拆遥操，不暂停 policy_node；臂回位轨迹与策略可能同时写 joint_commands。定时器触发时也不重新检查录制/所有权状态。

修改方向：回位请求统一经过仲裁；策略、回放或接管事务期间拒绝，或先停止策略并确认控制权释放。延迟回调必须重新校验，模式变化取消待执行回位。

### 4. 夹爪接管没有与策略当前值对齐

位置：`src/robots/astral/astral_gripper_teleop/astral_gripper_teleop/pinch_gripper_node.py`（272、300）。

重新打开门控后，夹爪立即使用 trigger/pinch 和遥操自身 `_ratio_cmd`。策略期间的实际夹爪指令没有同步到这里。策略保持 close=1、操作者扳机释放=0 时，首次遥操输出可为 open=0；即使启用限速，其起点也可能是旧遥操值，不是当前策略值。

修改方向：接管时同步当前执行值，加入 soft takeover（输入先匹配保持值才能控制）或以当前值为起点限速。夹爪无真实反馈时明确采用最近执行命令，并记录该语义。

### 5. 交还策略的首条动作绕过限速

位置：policy_node `_start_policy`（874、919）；`executor.py`（43、57）。

交还时 `_executor.reset()` 清掉 `_last`，SafeExecutor 只在 prev 存在时限速。因此首条策略动作没有相对实际姿态的限速约束。离线复现：1 rad/s、60Hz、目标偏差 0.5rad，reset 后首条仍为 0.5rad，而非 0.0167rad。

此外新策略首包推理完成前遥操仍可移动，首包观测可能已经过时；目前切换没有遥操 disarm 完成确认，跨节点信号也非原子提交。

修改方向：交还先固定机器人姿态并确认遥操停发，再取得观测/推理；首条执行器以交还时当前姿态及夹爪值初始化；慢推理、首包失败和停止请求都应有可取消状态。

## 完善闭环与远程能力

### 6. 缺少可直接训练使用的介入数据标记

位置：`src/common/morla_data_collect/morla_data_collect/data_collect_node.py`（293、590）及 pipeline。

数采主要记录 armed/disarm 等事件，没有订阅 policy state 并保存 policy/human/takeover_pending 区间，也没有逐帧 intervention / action owner。armed 可以来自普通遥操、暂停恢复等路径，不能等同人工纠正标签。当前 next_state 数据能用于一般模仿学习，但不能可靠区分自主段与纠正段、按纠正片段加权。

修改方向：raw 保存来源事件、事务和失败/断链原因；align 将模式映射到目标时间轴，清洗按 source indices 保留，OpenPI/ACT 导出沿用字段；历史数据无法确定的标 unknown，不推造标签。success/outcome 标记另行显式记录。

### 7. 远程数据“收到”不等于“新鲜”，按键路由状态也无失效机制

位置：`src/common/morla_input/morla_input/quest3_udp_mocap.py`（559）；`controller_start_gate.py`（98）；臂 reanchor（1056）。

TCP 将缓冲中的所有行按本次接收时间发布；协议未携带可用于拒绝陈旧输入的源采样时间/序号，UDP 也没有顺序过滤。远程拥塞后的旧输入会重新看似新鲜；接收超时看门狗不能覆盖该情形。reanchor 只检查腕数据对象存在，接受陈旧缓存；本次构造 60 秒前腕与关节状态仍返回成功并武装。policy 发请求前检查 fresh joint state 有帮助，但不能替代服务执行时检查，也不检查腕新鲜度。

grip 路由仅缓存 activity，无状态心跳超时、发布者活性或确认。启动尚未收到状态、状态丢失/畸形、重连保持按键等情况需要明确处理。两个节点（controller_start_gate 和 vr_collect_control）还都在 HUMAN 时发 release，应统一唯一发送方，另一个只抑制 A=start；否则一次 A 可产生重复 release。

修改方向：Quest 采样序号/单调时间、session ID 与时间同步，限龄/乱序/积压丢弃；重连必须松键再按；接管检查腕及关节新鲜度，策略状态失效时禁止绕过仲裁直接启动。

### 8. Wuji 灵巧手仲裁不完整（仅启用灵巧手时）

位置：`src/robots/wuji/wujihand_retargeting/wujihand_retargeting/wujihand_retarget_node.py`（181）。

retarget 每帧直接发布手 joint_commands，没有与夹爪同等的 disarm/owner 门控；若策略 schema 包含相同灵巧手，策略和 retarget 可同时发布。当前 policy 默认 left gripper/right none，此问题不应误报成当前夹爪配置必然发生。

修改方向：所有被策略控制的执行器采用一致所有权机制，灵巧手接管需同步手势、限制关节跳变，必要时支持独立通道接管。

## 验证

- 隔离 ROS Domain 91、ROS_LOCALHOST_ONLY=1；stub backend + synthetic feedback，未连接真机。策略 tests、grip/release/workpos 纯逻辑、数采 VR 控制测试：162 passed，4.76s。
- 既有臂 reanchor 脚本：7/7 passed；其中陈旧反馈回退 last command 是当前测试允许的行为，不是已具备新鲜度防护的证明。
- 临时复现脚本 `/tmp/astral-hitl-audit/reproduce.py`：8 个构造情形全部观察到上述行为。脚本借用真实方法，使用 fake futures/engine/pose/publisher，没有实例化硬件节点。迟到效应是模拟服务先后顺序，不是测得现场一定发生。
- 当前测试通过不覆盖这些边界。复现脚本不作为正式回归提交；修复时应先增加断言目标行为的正式回归。

建议顺序：修复接管事务与停止顺序 → 阻断 X 回位/灵巧手旁路 → 夹爪与交还首条动作连续性 → 完整介入数据标签 → 远程协议新鲜度与统一按键路由。
