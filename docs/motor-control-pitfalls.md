# 驱动层/电机控制坑（"电机突然自己动"定责手册）

> 2026-08~09 真机反复踩、2026-09-22 整理归档。
> 给新会话/接手者：臂/夹爪/头"突然动一下、回到不该回的位置、该动不动"——**先到这里对症状**，
> 这类问题根因多数在**板端目标/运动模式/命令流**三层，不在遥操或策略。
> 母本：`src/astral_robot_control/README.md`、`src/astral_arm_teleop/CLAUDE.md`、`astral_ws/CHANGELOG.md`
> （08-25~09-17 各条）。诊断资产：driver `driver_log_file`（`cmd`/`send`/`state`/`srv`/`spike` 五类记录，
> 用 spike 时间对齐 cmd/send/srv 定"谁先跳"）。

## 症状 → 根因 → 修法

| 症状 | 根因 | 修法/防护 | 数值/验证 |
|---|---|---|---|
| 阻尼释放后手动拖臂，再「一键就绪」/「归零」臂抽回**阻尼前位姿** | ① 阻尼(motion_mode=0)忽略 0x90 位置指令 → `~/home` 一次性归零在阻尼态静默无效；② 100Hz 控制定时器在 `command_timeout_s` 新鲜窗口内**持续重发缓存旧目标 P**，切回 POSITION 后陈旧 P 重新接管 | 三层：`_srv_home` 先切回 POSITION 再归零；`_clear_cmd_cache()`（ready/home/damping/estop/enable 时清缓存）；臂节点 operator disarm 取消进行中 homing；web 硬件模式端点先自动发 `/teleop/disarm` | test_driver_services 7→13 例全绿 |
| 阻尼拖臂后切回 POSITION，臂仍抽回旧位姿（**板端目标寄存器残留**） | 上轮只清了 ROS 侧缓存，漏了**板端内部保存的上一次目标** P——阻尼只是忽略 0x90，不清目标；切回 POSITION 板端立刻重新追踪 P | `seed_from_current=True`（one_click_ready）：POSITION 切换后、enable 轮询前，用最近实测关节 `set_target_positions` 重写板端目标；driver `_seed_target_from_current()` 在 `~/position`/`~/home` 的 set_motion_mode(1) 之后调用 | test_driver_services 13→16 例全绿（home/position 阻尼切回先重写实测位姿的顺序断言） |
| 点 web HOME 报"电机未使能"，实际已使能 | SDK `enable()` 轮询板端 `robot_powered` 位确认，该位此板常不置位 → 重复使能 + 确认位失灵 | `~/enable` **幂等化**（已上电直接成功）；未确认但**板端在线**（obs 帧持续）也算下发成功，仅真正离线才失败 | test_driver_services 7 例全绿 |
| HOME 归位"一卡一卡"地动 + 突然"抽一下" | HOME park 每拍 `q_cmd = 实测 + 一个 tick 步长`——等于"等实测挪一步才挪一步"的**阶梯采样**：命令只领先实测一个 tick 步长，被实测采样节拍门控 → 慢；命令按实测帧节拍一跳 → 粘滑，静摩擦突破时"抽一下" | park 与 init 一致改**纯开环匀速推进**（base=q_cmd）；反馈只做跟随便用——`homing_follow_tol`(0.25 rad) 冻结守卫：实测落后指令超阈值 → 冻结轨迹、q_cmd 不空跑，实测追近自动解除 | test_home_park 7→11 例全绿 |
| HOME"出了但没到 waypoint 就掉头"（切角） | park 轨迹纯开环、命令领先实测；途经点推进只看命令误差（<0.05 即走下一段），命令一到点立刻 180° 反向 → 实体在 waypoint 之前被"切角"（走廊失效）；且 `_go_home` 把 waypoints **正序**拼，应倒序 | 途经点推进改**双条件**：命令到点后钉住重发，等实测进入 `homing_via_tol`(0.12) 再推进（`homing_via_hold_s` 2.0s 兜底）；`_go_home` 路径改 `[::-1]` 倒序 | test_home_park 11→14 例全绿；闭环滞后模拟命令在 waypoint 驻留 1tick→等实体进入 0.12 rad |
| HOME"没到过 waypoint 姿态"（秒放行） | 门限 = "实测进入 0.12 即放行"太松且**不判停**——实体 0.6 rad/s 运动中命令到点瞬间实测恰擦过边界 → 秒放行，臂刚擦过就掉头 | 放行条件 = 实测进入 tol **且连续稳定 `homing_via_settle_s`(0.5s)**；运动中擦过 tol 连续计时清零重来 | test_home_park 14→16 例全绿；命令驻留 0.02s→0.52-0.68s |
| 夹爪抽搐 + 开度不到位 | `pinch_gripper_node` 同时发 `/command`(Float64) 和 `/joint_commands`(JointState)，driver 两个都订且各自映射 rad（0.8 vs 1.5 不一致）→ 两路 50Hz 交替覆盖同一目标 | pinch 节点**只发 Float64 闭合比**（0=开 1=合），rad 映射唯一权威收敛到 driver yaml；`joint_commands` 留给直接发弧度的适配器 | driver dry_run 全程只见 rad=1.500 单值 |
| 夹爪"开到最大马上闭合" | `open_rad=2.0`（机械硬止点）持续顶死 → 堵转 → 过流保护 → 电机失力回弹 | `open_rad` 回退 1.5（安全开度）；后按实机全开角升 2.5 | — |
| 夹爪极性相反（捏合反而张开） | 真机电机方向 0.8=张开、0.0=合拢，原 `open/close_rad=0.0/0.8` 颠倒 | 翻转为 `0.8/0.0`，两条路径（JointState 弧度 + Float64 比例）一并翻转 | — |
| 慢速平移"停一下走一下" | 电机低速静摩擦粘滑（speed 环 kp=0.04）+ geometric 最小关节速度优化压到死区 | 调 speed 环（⚠ 待验证）；`cmd_deadband_mrad` **实测无效**（硬件层） | 详见 [teleop-stick-slip-investigation.md](teleop-stick-slip-investigation.md) + [ik-solver-comparison](../src/astral_arm_teleop/doc/2026-09-17-ik-solver-comparison.md) |
| 单臂预设下缺席臂被拽向零位 | driver 控制定时器发现"左臂有新鲜指令、右臂 None"时给缺失侧**补零** → 100Hz 零目标持续拽向零 | arm-only 路径改为**只向有新鲜指令的一侧下发**（`_send_arm_side()` 按该侧电机 ID 子集），缺席侧不发命令 = 板端保持原目标 | test_driver_services 16→20 例全绿 |
| 电机 ID 语义错乱 | 0x31/0x32 是**头部**（head_yaw/head_pitch），不是夹爪；机械夹爪只走 0x97/0x98 | SDK `ROBOT_JOINT_NAMES` 末两位改 head；夹爪 `/joint_states` 改发最近指令角；下游命名同步 | — |
| 阻尼/急停后恢复路径混乱 | 急停=真断电 disable；重启无"使能"步骤，归位轨迹在电机失能期内部空跑领先实体 | web 收敛到 **HOME 按钮**（先 enable 再 disarm+收回零位）；启动只拉栈、不自动使能（自动使能实机 503/臂不动已回退） | test_home_park 5→7 例 |

## 定责方法论：driver_log_file（电机"抽一下"）

`astral_robot_control` 新参数 `driver_log_file`（空=关）+ `driver_spike_mrad`（默认 30）：
`kind` 区分——`cmd`（每个到达的 joint_commands，排上游）/ `send`（每控制拍实际下发+fresh 标志，
排陈旧重发）/ `state`（实测关节）/ `srv`（6 服务 + cache_clear + seed_from_current，运动模式切换=
抽动高危点）/ `spike`（命令/实测单拍跳变超阈值落一条，含跳前跳后值）。
**定位方法**：spike 时间对齐 cmd（上游到）/ send（driver 发）/ srv（服务调用）→ 定"谁先跳"。

## 通用教训

1. **"电机突然自己动"按三层查**：上游坏指令（cmd 流）→ driver 陈旧重发（send + fresh）→ 板端
   目标/运动模式（srv/cache）。不要只看 ROS 侧就下结论——**板端内部目标寄存器是独立状态**，
   阻尼不清目标、切回 POSITION 会重新追踪旧目标。
2. **位置控制被"贴实测"钳制会变阶梯采样**：命令永远只领先实测一个 tick 步长 = 慢 + 粘滑 + 抽动。
   唯一被实机验证的平滑路径是**纯开环匀速推进 + 冻结守卫**（反馈只做跟随）。
3. **运动模式切换（POSITION/阻尼/急停）是抽动高危点**：切换前先定住当前位置（seed_from_current），
   切换后清陈旧目标缓存。
4. **夹爪/双臂的"唯一权威"**：一个执行目标只能有一个映射源头（rad 映射收敛到 driver；缺侧不补零），
   双流覆盖 = 抽搐。
