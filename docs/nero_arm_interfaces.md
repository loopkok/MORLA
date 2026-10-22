# Nero 软件接口

## Quest 遥操启动与中断恢复（2026-10-20）

机器人启动后，先“准备并使能”；需要工作位时完成工作位动作，再次“准备并使能”切回 JS 流式模式。接通 Quest 并开启 MORLA 新应用的手部追踪，等待左右腕、关键点和两手实测反馈齐全，顶部“开始遥操”才允许提交。开始确认同时检查本次写入者状态和硬件状态，不能只凭 `armed=true` 认定实体已就绪。

Nero 腕输入超时统一为 0.5 秒；接收端按每条运动流检测超时或 TCP 断开并发送冻结事件。双臂与双手订阅根话题 `/quest3/input_epoch`，会话失效会清理旧输入和锚点。恢复数据不会自动继续运动；确认新鲜反馈后，重新点击“开始遥操”（手柄需先释放再按开始键）以实测当前位置重锚定。HUMAN 控制权心跳过期也会冻结，普通心跳不能自动恢复输出。

每侧遥操 `~/status` 新增 `stop_reason`，显示输入超时、控制权失效、会话改变等最近冻结原因；再次成功锚定时清除。零输入及重复目标复用已知 IK 解，避免同一个末端目标反复求解时发生冗余关节漂移。XHand 适配器状态的 `feedback_fresh` 表示收到有效且新鲜的原生状态；串口读取失败时不再刷新缓存反馈的时间戳，反馈过期时拒绝手指命令。

## 遥操工作空间开关（2026-10-20）

按用户要求，Nero 左右臂遥操暂时设置 `workspace_enabled: false`，同时跳过基座周围的球形半径及 XYZ 盒形工作空间投影。原有 `workspace_radius` 与轴向边界配置保留，日后在两侧 `nero_teleop_left.yaml` / `nero_teleop_right.yaml` 中改为 `workspace_enabled: true` 并重启机器人 Profile 即可恢复。该开关在节点初始化时读取，修改参数后须重启 Profile；仅重启 Web 不会更新正在运行的遥操节点。

该设置仅关闭笛卡尔工作空间裁剪。IK 可达性、关节限位、关节步长及驱动速度限制、已配置的碰撞检查、反馈/输入超时、控制权与急停处理保持原有逻辑。HOME 和工作位仍使用各自的关节点位轨迹。遥操启动日志及 `/{side}_arm/nero_arm_teleop/status` JSON 的 `workspace_enabled` 字段显示实际生效值。

此接口文档覆盖 P5 的 Nero 手臂 driver 与遥操作。其 ROS 话题使用统一具名接口；`pyAgxArm` 只由 `nero_robot_control` 的硬件适配器加载。ROS 构建、fake 运行和 profile 检查不要求导入该 SDK。

| 用途 | 每侧接口 | 类型/语义 |
| --- | --- | --- |
| 手臂命令 | `/{side}_arm/joint_commands` | `sensor_msgs/JointState`；必须提供严格有序契约中的 7 个 `name`，position 单位为 rad |
| 实测关节 | `/{side}_arm/joint_states` | `sensor_msgs/JointState`；SDK 回读，名字为 `{side}_joint1..7` |
| 实测机械臂法兰末端 | `/{side}_arm/flange_pose` | `geometry_msgs/PoseStamped`；SDK `get_flange_pose()` xyz+rpy 转换为 Pose 四元数，位置 m、姿态 xyzw；当前 Profile 声明 child frame 为 `{side}_gripper_flange`，SDK 与 URDF frame 的实际对齐仍待只读核验 |
| 兼容旧末端话题 | `/{side}_arm/tcp_pose` | 与 `flange_pose` 同一 PoseStamped 数据的 deprecated 兼容别名；历史名称里的 TCP 不代表单独标定的手/任务 TCP |
| driver 状态 | `/{side}_arm/nero_robot_driver/status` | `std_msgs/String` JSON；backend、连接/使能、反馈新鲜度、拒绝原因和最近错误 |
| 显式使能 | `/{side}_arm/nero_robot_driver/enable` | `std_srvs/Trigger`；只在连接且七关节实测反馈新鲜时接受 |
| 显式复位控制器急停 | `/{side}_arm/nero_robot_driver/reset` | `std_srvs/Trigger`；要求新鲜控制器状态与无待执行动作，复位后等待正常状态确认；不使能电机 |
| 显式位置动作 | `/{side}_arm/nero_robot_driver/position` | `std_srvs/Trigger`；仅在对应配置存在 `position_pose` 时可用，返回“已排队”不代表已到位 |
| 急停请求 | `/{side}_arm/nero_robot_driver/estop` | `std_srvs/Trigger`；异步请求 SDK electronic emergency stop |
| re-anchor 准备/提交 | `/{side}_arm/nero_arm_teleop/prepare_reanchor`、`.../reanchor` | `std_srvs/Trigger`；prepare 只捕获新鲜实测关节与 Quest 姿态并返回一次性 token；HUMAN 所有权消息携带 token 后才开启遥操作输出 |
| 遥操作状态 | `/{side}_arm/nero_arm_teleop/status` | `std_msgs/String` JSON；armed、所有者、新鲜度、输入 frame 和 solver |

解析 IK 的内部 FK/目标帧以及当前 Profile 的活动裸臂控制末端都是 `*_link7`。独立的 `*_flange_pose` 测量话题表示 SDK 回报的物理法兰位姿；URDF `*_gripper_flange` 是其候选命名帧，二者实体对齐仍待只读核验。URDF 固定链仍可用于 TF，或在配置显式选择法兰/手部工具帧时做坐标换算。PoseStamped 的 `header.frame_id` 表示父坐标系 `{side}_base_link`；child frame 由规范话题及 Profile 坐标契约给出。

`side` 为 `left` 或 `right`。默认 launch backend 为 `fake`；fake backend 使用配置中的显式 7 维初值，不打开 CAN。接入硬件时需显式传 `sdk_backend:=hardware`，并先按设备现场验收完成 CAN、固件、限速和运动方向检查。不得将 fake 初值当作机器人的实测姿态。

工作区内保留 SDK 源码于 `third_party/pyAgxArm`。只有在具备对应系统依赖的硬件部署环境中才运行 `scripts/install_pyagxarm_editable.sh`；它会将源码以 editable 方式安装到当前 `python3`。本次软件迁移没有安装供应商 SDK、没有初始化 CAN，也没有执行位置动作。

## 显式 CAN 初始化与冷启动

主机 SocketCAN 接口激活不代表机械臂控制器已经开启 CAN 反馈推送。驱动连接阶段只打开通信，不自动使能电机、不切入 `js`、不发送运动目标。Nero 真机在无反馈时，通过 Web「CAN 冷启动（含电机使能）」或 `/morla/control/initialize_can` 显式执行控制通道初始化；协调器会先冻结各 writer 并检查采集状态，再调用左右驱动的 `/{side}_arm/nero_robot_driver/initialize_can`。

冷启动遵循 SDK 首次使用指南与原 XNero 流程，调用电机 `enable()` 并请求 `set_normal_mode()` 打开 CAN 推送。适配器临时将 MOVE 设为 `0xFF`、MIT 设为 SDK 默认 `0x00`，不继承缓存的 JS/MIT 模式。界面明确提示会使能电机，需要操作员确认设备停止、允许电机上电保持。冷启动不发送运动目标，也不切入 JS；完成后软件运动许可仍关闭，后续「使能」才切入遥操控制模式。

冷启动不要求已有反馈；驱动软件已使能、存在运动或其他硬件请求时拒绝初始化。单次 SDK enable 尚未确认则由状态循环重试，默认总 SDK 截止时间 `sdk_timeout_s=2.0`，不无限等待。确认电机使能后，仍要求初始化之后发起的读取和新鲜的关节/法兰源数据，默认反馈总截止时间 `can_feedback_timeout_s=3.0`。失败或超时请求电子急停，协调器也向两侧请求急停；请求已发出不代表实测已停。外部急停会取消重试。`Trigger.success` 只表示接纳，驱动状态包含 `can_init_seq`、`can_init_status`、`can_init_error`、`can_motor_enable_confirmed`。后者只表示本次冷启动曾确认电机使能，不是持续读取的电机状态；`enabled` 仍代表软件运动许可。

SDK 适配器检查关节和法兰源时间戳，拒绝初始化前的缓存或超过 `max_feedback_age_s` 的旧数据，避免重复读取缓存而误报反馈新鲜。

Web 主流程：启动机器人 → 准备并使能（缺反馈时自动冷启动，确认后再启用控制）→ 等统一控制状态 completed → 开始遥操。独立冷启动入口位于高级维护。需要工作位时，在使能后移至工作姿态，确认到位，再显式使能并开始遥操。主机 CAN 接口仍须在启动 Profile 前配置正确。软件回归使用 fake SDK/独立 ROS 域，实际控制器的冷启动反馈恢复需现场验证。

控制器急停与 Profile 进程状态相互独立：停止或重启 Profile 不会清除控制器侧的急停锁存。真机 Nero 在 Web 的「控制器与急停状态」中执行明确的「复位急停」操作；驱动只有在复位请求后收到新鲜的正常控制器状态才报告成功。复位后电机仍未使能，需另行点击「准备并使能」。启动 Profile 不会自动发送复位命令。

## 工作位运动模式与到位确认（2026-10-19 修复）

### J 与 JS 的实际区别（2026-10-22 核对）

核对依据是工作区供应商 `third_party/pyAgxArm/docs/nero/nero_api.md`、Nero driver/parser 源码，以及部署主机实际安装的 pyAgxArm 1.0.0。两种 API 都传入7维原生关节绝对角（rad），使用相同关节目标报文；区别在控制模式：J 对应关节位置速度模式，JS 对应关节目标加 MIT/Follower 模式。MORLA 显式选择模式并关闭 SDK 自动切换，因此不能只根据 `move_js` 这个 Python 方法名判断实际模式。

- **J**：控制器对目标进行位置速度控制和平滑规划，`set_speed_percent()` 对此模式生效。适用于工作位、HOME 等明确点位；连续目标覆盖前一个目标，函数返回不表示到位，仍须核对实测误差和稳定时间。该规划不是环境避障。
- **JS**：控制器尽快跟随不断更新的关节目标，SDK 不提供轨迹规划和平滑；不能把 J 的5%速度设置当作 JS 限速。用于遥操时，由上位机生成连续目标并限制目标变化；MORLA 经过遥操安全滤波和驱动 `DriverCore` 关节目标变化率限制，驱动当前配置 `max_joint_velocity_rad_s=1.0`。这限制发送目标的变化率，不等价于实测关节速度闭环验收。供应商文档还指出 JS 相比位置速度模式控制刚度可能更低，实际保持误差仍受负载和固件影响。

原 Profile 的 `stream_position_mode` 默认 `js`；工作位/HOME 使用 J，返回遥操前显式准备并使能以恢复配置的流式模式。钢琴独立 launch 显式选 `stream_position_mode=j`：虽然驱动仍调用适配器的 `move_js()` 入口，适配器实际转成 `move_j()`，使用配置的5%速度并跳过重复的同编码保持目标；该选项没有全局启用。本次 QoS 修复不修改机械臂模式、速度、增益或轨迹。

### 工作位模式保持

`move_j()` 只发送关节目标，不等待实体到位。工作位运动必须保持 SDK 的 `j` 规划模式；在调用返回后立即切换为 `js`，会将尚未到达的远端目标交给无轨迹规划的直通模式，破坏工作位低速运动的预期。旧实现存在该错误，不能仅凭 `position_speed_percent: 5` 认定实体运动已限速。

修复后，驱动按 `position_speed_percent`（整数 1–100，当前配置 5）发送工作位目标，状态为 `waiting_arrival`，并持续正常读取实测反馈。各关节误差不超过 0.02 rad 且持续至少 0.2 秒后，中间点报告 `stage_arrived`、末点报告 `arrived`；到位后仍保持 `j` 模式和关闭的遥操命令许可。只有后续显式使能才恢复流式控制；位置运动或急停请求未完成时拒绝重新使能。

默认 `position_timeout_s=40.0`。位置超时、反馈过期或位置调用异常会撤销许可并请求 SDK 电子急停；外部急停把当前位置操作标为 `cancelled`。这些路径都不切回 `js`。协调器只接受新 operation 对应的 `position_seq` 与 `arrived` 状态作为工作位完成证据。

回归测试使用独立 ROS 域与 fake SDK，验证非阻塞目标发送、模式保持、实测到位、显式使能、超时、反馈丢失、SDK 异常与急停中断。软件回归通过不代表已完成实体速度/轨迹验收；实体测试前须重启 Profile 加载修复版本。

## 工作位与 HOME 的连续位置控制

状态字段 `enabled` 保留兼容语义，只表示 JS/遥操流式输出许可（同时提供明确别名 `streaming_enabled`），不表示电机实际失能。工作位或 HOME 动作开始时关闭该流式许可，但保留独立的 `position_permitted` 位置动作许可；到位后保持 SDK `j` 模式，可直接执行下一次 HOME 或工作位动作，不需要中间再使能或切回 `js`。

驱动通过 SDK `get_driver_states(1..7)` 的 `foc_status.driver_enable_status` 和每个样本自身时间戳读取实际电机状态，发布 `motors_enabled`、`motor_status_fresh`、`joint_enable_status` 与 `motor_status_error`。七个电机都必须有新鲜反馈且全部使能，控制器正常、关节反馈新鲜，同时位置许可有效，才能开始或继续位置动作。缓存的 `enable_status=enabled`、单独的控制器状态 0、旧到位状态均不能替代这些证据。

位置许可只在明确使能成功后开启；急停、复位、连接初始化、位置失败或超时会撤销它。即使这时还缓存着旧的“电机已使能”反馈，也不能继续 HOME。恢复后需重新明确准备并使能。工作位/HOME 正常到位不撤销位置许可；只有恢复遥操时才需要重新准备并使能以切回流式控制。

加载本修复须同时重启驱动与协调器所在的机器人 Profile。Web 将电机实测使能与遥操许可分开显示，缺少新电机状态字段时提示重启 Profile，不再把位置保持中的流式许可关闭显示成电机失能。


## Nero 工作位与 HOME：四点正反向轨迹

右臂原生角度，单位为度，按 J1–J7 排列：

```text
点 1：  0  25   0  123     0   0  -60
点 2：  0  25   0  123  -100  42    0
点 3：  0  25   0  123  -100  42  -70
点 4： 29  31  42  103  -120  24  -10
```

左臂按 `[-1,+1,-1,+1,-1,-1,+1]` 镜像生成：

```text
点 1：   0  25    0  123    0    0  -60
点 2：   0  25    0  123  100  -42    0
点 3：   0  25    0  123  100  -42  -70
点 4： -29  31  -42  103  120  -24  -10
```

2026-10-20 用户确认：原始前三点 J4=125° 改为 123°；点 2、3 的右 J6=45° 改为 42°，左臂对应 −42°，保持镜像且不放宽当前关节限位。

工作位按 **1 → 2 → 3 → 4** 执行，点 4 是最终工作位；HOME 按 **4 → 3 → 2 → 1 → 最终 HOME** 执行，第五段双臂均移至 `[0°,85°,0°,0°,0°,0°,0°]`。HOME 首段确认/移动至点 4，然后沿原路返回；没有额外 J1 避让步骤；反向四点之后追加 J2=85°、其他关节=0° 的最终收回动作。从任意当前位置发起工作位会先移至点 1；发起 HOME 会先移至点 4。这是固定关节点位路径，没有环境碰撞规划。

配置为各侧 `nero_driver_left.yaml` / `nero_driver_right.yaml` 的 `workpos_waypoints_deg`：四行原生点位按行展开为 28 个浮点数，单位为度。自定义配置已经是各侧原生角度，不再次镜像；修改路径应同时更新两侧并验证限位。`position_pose` 同步记录点 4 的弧度值供兼容工具使用，工作位以共享点位配置为准；HOME 先反向执行共享四点，再到固定最终姿态，`home_final_pose_deg` 状态字段公布该姿态。每次动作开始时冻结完整点位序列，全部通过有限值、形状和关节限位检查后才下发第一段。

每段均保持 SDK `j` 规划模式；`position_speed_percent` / `home_speed_percent` 默认 5%。所有配置机械臂实测误差 ≤0.02 rad 且稳定 0.2 秒后，协调器才下发下一段。每段超时由 `position_timeout_s` / `home_stage_timeout_s` 控制，默认 40 秒。反馈丢失、控制器异常、SDK 错误或超时终止动作并请求全部配置机械臂电子急停。直接驱动服务只执行当前段，日常使用 Web 或统一控制入口完成全路径。

驱动阶段服务：

- 工作位：`~/position` → `~/position_point2` → `~/position_point3` → `~/position_point4`。
- HOME：`~/home` → `~/home_point3` → `~/home_point2` → `~/home_point1` → `~/home_final_pose`。

已移除 `home_j1_deg`、`home_j2_deg`、`workpos_j1_deg` 及旧阶段服务。状态通过 `trajectory_revision=nero_four_points_home85_v2`、`workpos_waypoints_deg` 和 `position_kind` / `position_stage` / `position_stage_count`（工作位=4、HOME=5） 表达路径。`position_seq` 每段递增，中间段到位为 `stage_arrived`，末段到位为 `arrived`。Web 显示工作位四段及 HOME 五段进度及各侧点位，旧驱动缺少版本字段时禁止工作位/HOME并提示重启 Profile。

动作前准备并使能；执行时冻结遥操，采集中禁止发起。工作位与 HOME 可在位置许可有效时连续执行；完成后保持 J 模式，恢复遥操前再次准备并使能。更新后须手动重启机器人 Profile（驱动及协调器）和 Web、刷新页面；部署不会自动启动服务或执行真机运动。

## 限位附近的实测反馈与故障原因（2026-10-20）

四点路径前三点 J4=123°，位于原有命令上限附近。旧实现将任何越过命令上限的实测值作为无效数据丢弃，小幅测量/跟踪偏差也可能导致 0.5 秒后显示“反馈过期”；日志曾出现该错误后仍用上一帧确认到位。

实测值现在单独允许 0.1° 边界容差，保留原始角度用于显示与到位判断，不截断测量值。SDK/ROS 发出的运动目标仍严格受原限位约束；从实测值生成的遥操初始保持目标投影至原限位内。该容差不放宽四点目标或机械臂的运动范围。0.5 秒反馈时效、到位稳定确认、控制器/电机检查仍保留。

超过测量容差的反馈立即中止位置轨迹，记录 `Jx measured`、`limits`、`excess`，不再等待其被误报为通信过期；瞬时读取失败不使用上一帧完成到位，持续缺反馈仍触发停止。状态补充 `feedback_read_ok`、`feedback_read_error`、`feedback_limit_tolerance_deg` 及 `last_feedback_limit_violation`，保留最近超限细节。协调器不会以最新读取失败的状态推进下一点。

软件测试覆盖边界 +0.05° 保留原始反馈且不误停、+0.2° 明确超限停止、读失败时禁止缓存到位、命令仍严格限位。真实历史故障超限幅度未记录，不能据旧日志认定一定在 0.1° 内；若再次停止，按新增超限数值判断真实跟踪偏差或控制器问题，不能继续增加容差绕过保护。

## 控制器状态未知时的显式复位与恢复反馈

CAN 冷启动失败并请求急停后，控制器可能不再推送状态。Web 此时显示“状态未知/过期”，不能从未知状态推断已急停或已正常。新版在驱动传输连接仍在、遥操输出关闭且无位置动作/待执行 SDK 操作时，允许点击“复位急停 / 恢复反馈”。仍使用原有确认提示，复位可能让抬起关节失电，操作前需支撑机械臂。

SDK 仅发送复位和正常受控模式的 CAN 推送请求：`ctrl_mode=1`、`move_mode=0xFF`、`mit_mode=0`、`enable_can_push=1`，不调用电机 enable，不发送关节目标。此前缓存的关节/电机反馈失效，复位后必须有新的控制器状态回读。

双侧可分别为急停、未知、正常或抱闸未打开；正常/抱闸未打开且软件输出关闭的一侧确认无需复位，不重复发复位包。状态 0 或 6 的新鲜回读表示急停已解除，6 仍需单独“准备并使能”释放抱闸。其他已知故障、驱动未连接或操作未结束时拒绝复位。`reset_unknown_status_supported` 标记新功能，`reset_mode` 区分 `estop`、`unknown_status_recovery` 和 `not_needed`。

超过复位期限仍无新鲜回读时，操作报告失败并提示检查机械臂供电、USB-CAN 和 CAN 接线；不能因“复位命令已发送”显示成功，更不能自动转入使能或运动。恢复回读后，再独立执行“准备并使能”。若链路完全无帧，软件命令无法代替电源或物理接线恢复。
