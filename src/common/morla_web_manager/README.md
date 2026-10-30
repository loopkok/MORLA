# morla_web_manager — MORLA统一操作与监控

## 当前组合配置与启动（2026-10-30）

组合页/API v2 选择 descriptor 中的组件、端口与控制器；Web 与 CLI/ROS 使用公共 BindingEngine。原生设备桥通过 morla_native_bridge 注册工厂；采集进程通过 morla.processes 自动托管，输入框中可选 auto/managed/external，状态含 input_processes，原生设备 health 可含 processes。

scalar_map 支持比例或米制单自由度目标；pinch_filtered/trigger_filtered 显式选择自动范围、EMA、扳机曲线和输出 slew。轴映射来自 axis_names；新控制器可由已安装 controller_types 显示。更换设备同时核对单位、范围、关节顺序，不能只改品牌名。

Web stop 会关闭输入管理器；UR11 的“ROS stop 后输入到宿主退出才回收”属于无 Web 网关差异。旧 Profile/钢琴入口仍为独立兼容流程；不要把旧路由状态当成组合会话状态。

测试入口：[主机手册](../../../docs/testing/host_build_and_hardware_acceptance.md)、[报告模板](../../../docs/testing/host_acceptance_report_template.md)；软件与未测项见[实现记录](../../../docs/architecture_refactor/UNIFIED_IMPLEMENTATION.md)。


`morla_web_manager` 是 MORLA 的网页管理包。它以 robot profile 为入口，统一展示系统、启动仿真/真机遥操、管理数采与策略推理泳道、查看动态关节和流率、运行离线数据任务，并通过 `morla_bringup` 的控制协调器执行需要冻结/确认的机器人操作。

新增独立的「钢琴工作台」：曲谱选择/导入、时序预检、参数标定、单指及手腕试动、演奏和停止，核心仍复用同级 `MORLA-piano` 项目。操作与互斥规则见 [钢琴 Web 操作](../../../docs/piano/web-operation.md)，验证及边界见 [对抗性审查](../../../docs/piano/adversarial-review.md)。

## 启动与构建

```bash
cd /home/robot/loopkok/sdk/MORLA/src/common/morla_web_manager/web
npm ci && npm run build

source /opt/ros/humble/setup.bash
source /home/robot/loopkok/sdk/MORLA/install/setup.bash
ros2 launch morla_web_manager web_manager.launch.py
```

浏览器默认访问 `http://127.0.0.1:8080`。局域网访问可用 `web_host:=0.0.0.0`。环境变量前缀统一为 `MORLA_WEB_MANAGER_`，包括 `PORT`、`HOST`、`DIST` 和数据/日志根目录；旧 `ASTRAL_WEB_MONITOR_*` 只作为兼容回退。symlink-install 会从源码 `web/dist` 服务静态文件，普通安装使用 package share 下的 `web/dist`。

## Profile和运行泳道

在“机器人操作”页选择同一 robot profile 与 `sim`/`real` 模式。Profile目录由 `morla_core` 解析，Web列出state/action维度、机械臂/末端、相机和能力，并据此启动对应ROS launch。当前支持Astral与Nero配置；Astral MuJoCo支持Wuji手仿真，Nero/XHand支持MuJoCo仿真。profile选择会被`control_profile_id`/`profile_id`传递给遥操、采集和策略泳道。

“遥操/仿真”“数据采集”“策略推理”是独立启停泳道，可并行监控；启动参数均使用ROS argv，不经shell拼接。Web也会监视CLI启动的节点。profile/mode切换前端和后端同时检查活动launch、collector、policy、control coordinator及ROS command publishers；旧profile数据或writer仍活动时返回409，不会只凭Web自己的PID判断空闲。

模式和控制动作以ROS协调器结果为准。`/morla/control/*`服务成功表示操作已接纳，页面继续根据`/morla/control/state`的`operation_id`显示`accepted/freezing/waiting_services/executing/completed/failed/cancelled`。超时、writer ACK缺失、collector状态未知或服务失败都显示为失败，不把topic publish当作硬件动作成功。仿真不提供电机使能、阻尼、急停等真机能力，界面按profile禁用。

数据采集卡片可独立启动/停止collector，且会传入所选`profile_id`和`runtime_mode`。录制中禁止切换profile或执行回位。策略卡片控制policy/playback/HITL并展示profile/layout hash。监控实体来自当前profile和ROS实际发现的话题；关节名、维度和流率不再固定为Astral布局。摄像头与传感器缺失会按required/optional契约反映在状态和数据质量报告中。

Nero 真机冷启动：先配置主机 CAN 接口，再在 Web 启动机器人，点击「准备并使能」。协调器仅对缺少有效反馈的机械臂执行 CAN 冷启动（含电机使能），确认后再使能控制。独立连接初始化入口保留在高级维护，通过 `/api/v1/control/initialize_can` 调用；失败会请求双臂电子急停。仿真不提供此硬件操作。新增统一准备接口部署后需先停止机器人，手动重启 Web，然后启动新版机器人节点。

策略推理卡片可浏览机器人主机上的回放源：raw episode/session、aligned HDF5 与 LeRobot v2.1。
浏览器只读访问 `MORLA_DATA_ROOT`（默认 SDK 根目录下的 `morla_data`）、`~/morla_data` 和
`MORLA_WEB_MANAGER_REPLAY_ROOTS` 配置的额外目录（按系统路径分隔符分开）；符号链接越出
这些目录会被拒绝。raw 录制必须先停止并保存，随后可以直接回放，不需要先运行 align。

## 主要API

响应统一为`{ok, message, data}`；状态冲突返回HTTP 409。

| 方法 | 路径 | 用途 |
|---|---|---|
| `GET` | `/api/v1/health`, `/api/v1/state` | Web、ROS、launch泳道、profile、动态关节/流率和健康状态 |
| `GET` | `/api/v1/profiles` | Profile能力与state/action维度目录 |
| `POST` | `/api/v1/profile/select` | 选择profile及`sim`/`real`模式；活动writer/泳道阻止切换 |
| `POST` | `/api/v1/profile/launch` | 启动选定profile的遥操/仿真栈 |
| `POST` | `/api/v1/control/{operation}` | 统一控制协调：pause/resume/start/release/ready/enable/position/damping/home/workpos/workpos_direct/estop |
| `POST` | `/api/v1/collect/launch/start`, `/stop` | 数采泳道启停，自动沿用选定profile/mode |
| `POST` | `/api/v1/collect/control` | start/stop/pause/resume/next/discard等录制控制 |
| `POST` | `/api/v1/infer/launch/start`, `/stop`, `/api/v1/infer/cmd` | 策略泳道及policy/playback/HITL控制 |
| `GET` | `/api/v1/infer/replay/sources?path=...` | 在配置的数据根目录内只读浏览回放源 |
| `POST` | `/api/v1/data/jobs` | 提交profile感知的对齐、校验、导出等离线任务 |
| `WS` | `/ws/telemetry` | 推送统一UI快照、动态streams与日志尾部 |

OpenAPI文档由FastAPI提供在`/docs`。控制接口与ROS原生topic/service契约详见[`morla_bringup`](../morla_bringup/README.md)和[`morla_policy_inference`](../morla_policy_inference/README.md)。


### 统一机器人操作界面

默认进入「机器人操作」：选择机器人配置与环境 → 启动机器人 → 准备并使能 → 按需移至工作位 → 遥操。采集与策略仍在「监控与采集」中独立管理。运行期间锁定机器人选择，旧版 launch 预设与「仅应用配置」收纳至高级兼容选项，不再提供第二套日常启动入口。顶部保留遥操、暂停/恢复及急停；HOME 归位按能力显示在「准备与移动」主区域，阻尼与厂商就绪位于高级维护。失联、忙碌或能力不支持时禁止相应操作；急停在其他操作执行中仍可请求。

Nero 工作位和 HOME 使用同一四点路径：工作位 1→2→3→4，HOME 4→3→2→1→最终姿态 `[0°,85°,0°,0°,0°,0°,0°]`。点位 4 是工作位，每段等待全部配置手臂实测到位；默认 SDK `j` 模式、5% 速度。右臂为参考，左臂按 J1/J3/J5/J6 反号，详见 [Nero 双臂关节约定](../../../docs/nero_joint_conventions.md)。

Nero 的工作位 → HOME → 工作位可在同一次位置控制许可内连续执行，中间不重新使能、不切回 JS。HOME 的依据是七关节实测电机使能、新鲜反馈、正常控制器状态及有效位置许可，不再使用遥操流式许可 `enabled` 判断电机失能。急停、复位、位置失败/超时会撤销位置许可，必须重新准备。界面将“电机已使能”和“遥操输出已关闭”分开显示。新增状态字段需要重启机器人 Profile 加载。

「准备并使能」调用 `/api/v1/control/prepare`，由共享协调器执行能力驱动的准备计划。声明 `initialize_can` 能力且缺少有效反馈时，先执行冷启动并等待双侧确认，再执行 `enable`；反馈已有效时跳过冷启动；Astral 使用自身不回零的 `enable`，不调用可能归零的 `ready`。每阶段失败即终止，准备失败会请求急停，不自动继续遥操或移位。仿真不显示硬件准备。新机器人通过驱动能力和协调器路由接入相同操作入口，不在主界面增添专用启动按钮。

移至工作位保留既有控制许可策略：Nero 到位后仍保持 J 模式、关闭遥操流式输出许可并保留位置动作许可，需再次「准备并使能」后开始遥操。界面优化没有更改该运动安全行为。部署需停止 Profile、手动重启 Web 及 Profile，然后刷新页面；不可只更新前端继续使用旧协调器。

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
