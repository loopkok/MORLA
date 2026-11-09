# morla_web_manager — MORLA统一操作与监控

## 参考图工作站界面（2026-11-09）

默认进入浅色「遥操控制」。通过侧栏「系统编排」按输入设备、机械臂、末端、整机、相机、传感器分类添加组件，连接输入与执行器端口、选择兼容控制器；点击节点在右侧编辑设备、连接标定与控制链。画布布局仅属于界面，不改变安装变换。画布与属性区之间的淡线边界可左右拖动，双击恢复默认，也可聚焦后用方向键调整；表单根据宽度自动换列并保持正常字号。工作空间编辑统一在系统编排中进行，遥操页只显示当前已配置工作空间；未配置时显示进入系统编排的按钮，不自动跳转。

主画面在仿真执行器模式显示已添加的 MuJoCo 仿真相机，真机模式显示所添加相机的实际视频流。先配置并检查保存，再初始化；无相机、无会话和流错误均显示明确提示。采集与策略工作区沿用已有 API，完整运行控制和兼容工具仍可进入。遥操顶部固定显示运行前检查和启动工作站。会话与诊断只提供日志文件设置及工作站、teleop/collect/infer launch 实时日志，支持筛选、暂停、自动滚动和下载。详见[本轮实现与验证记录](../../../docs/testing/WEB_WORKSTATION_REFERENCE_UI_2026-11-09.md)。

## 当前组合配置与启动（2026-10-30）

组合页/API v2 选择 descriptor 中的组件、端口与控制器；Web 与 CLI/ROS 使用公共 BindingEngine。原生设备桥通过 morla_native_bridge 注册工厂；采集进程通过 morla.processes 自动托管，输入框中可选 auto/managed/external，状态含 input_processes，原生设备 health 可含 processes。

scalar_map 支持比例或米制单自由度目标；pinch_filtered/trigger_filtered 显式选择自动范围、EMA、扳机曲线和输出 slew。轴映射来自 axis_names；新控制器可由已安装 controller_types 显示。更换设备同时核对单位、范围、关节顺序，不能只改品牌名。

Web stop 会关闭输入管理器；UR11 的“ROS stop 后输入到宿主退出才回收”属于无 Web 网关差异。旧 Profile/钢琴入口仍为独立兼容流程；不要把旧路由状态当成组合会话状态。

无参数脚本 `tools/start_morla_web.sh`（桌面图标「MORLA 控制台」）启动完整控制台，公共配置与会话在「工作站」完成，不需要提前编写装配文件。脚本自动加载 Linux ROS 2 Humble 和仓库 install 环境；`OPEN_BROWSER=0` 不自动打开浏览器，`PORT=9090` 修改端口。复用已有服务前检查 `/api/v2/components`；更新代码后应先停止旧服务再重启。本地 MuJoCo 窗口在服务主机桌面打开，脚本不自动启动机器人或输入采集。六种输入/仿真场景见 [Web 操作手册](../../../docs/integrations/WEB_TELEOP_SIX_SCENARIOS_2026-11-02.md)。

「工作站」现采用六页流程：设备选型 → 连接与标定 → 遥操配置 → 数据记录 → 检查 → 运行。右侧设备面板选择对象，左侧只编辑当前设备；窄屏通过设备列表抽屉切换。顶部可打开、保存、另存为、载入预设及导入导出配置。切换页面或顶层标签保留草稿与会话状态。

先执行检查，再「保存并进入运行」，然后初始化工作站；初始化逐项显示会话、输入接收与设备就绪。开始遥操单独触发，以控制权和运行状态确认结果。运行期间配置只读，固定操作区保留暂停、急停、结束。请求超时或断网时先查询状态，页面不会自动重新提交运动操作。真实驱动使能位、输入频率和网络时延未由接口提供时显示暂无数据。

设备默认、装配显式值和部署覆盖分别显示。切换后端保留连接参数；切换控制链、移除设备、清理已核实无效字段均可预览并取消。保存冲突保留本地草稿，可另存为或打开最新版本核对。软件验收与现场复验范围见[2026-11-09 验收记录](../../../docs/testing/WEB_WORKSTATION_UX_ACCEPTANCE_2026-11-09.md)。

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

### CR5 MuJoCo 调试

组合配置不再提供 CR5 专用快捷按钮；CR5 sim 模板从「打开已保存配置 → cr5_quest_sim」恢复，或通过 `GET /api/v2/templates/cr5-sim` 取得包内模板后导入。模板包含 CR5 sim、Quest 输入和场景相机。预览、保存并启动后可查看实时 MuJoCo 画面；显式连接输入、激活后使用现有本地 IK 控制链路。页面重新打开会恢复当前会话及其保存配置。`GET /api/v2/sessions/{session_id}/cameras/{camera_id}/frame` 返回最新 JPEG，未生成帧时为 503，未知/非相机为 404；`GET /api/v2/sessions/{session_id}/cameras/{camera_id}/stream` 以 `multipart/x-mixed-replace` 推流（按 `(source_ns, sample_ns)` 去重，约 30 fps，会话结束自动断开），页面预览用它而不是轮询 `frame`。仿真相机可交互调整视角：`POST /api/v2/sessions/{session_id}/cameras/{camera_id}/view`，体 `{action: orbit|pan|zoom|reset, dx, dy}`，`dx/dy` 为指针位移像素（滚轮用 `dy`，向下为正=拉远）；相机移动排队在渲染线程执行，不阻塞 JPEG 编码。预览图左键拖动旋转、右键或 Shift 拖动平移、滚轮缩放，「复位视角」回到配置相机的位置。真机相机没有 `view`，返回 400；非相机实例返回 404。配置与启动说明见 [CR5 包说明](../../robots/dobot/morla_cr5_driver/README.md#cr5-mujoco-仿真与-web)。

### 公共仿真模式与显示选择（2026-11-02）

CR5/Nero/Astral 公共仿真支持运动学与物理动力学，并可选择网页、本地 MuJoCo viewer 或同时显示。CR5 新增独立 `cr5_mujoco_sim` 包；Nero/Astral 保留旧入口并新增公共 launch。配置、运行时切换和限制见 [公共仿真说明](../../../docs/integrations/SHARED_MUJOCO_2026-11-02.md)。

### CR5 IK 模式与诊断（2026-11-02）

CR5 专用控制绑定增加 differential/strict 选择，保存后重启会话生效。session state 的 `controllers[binding_id][plugin_id]` 提供线程安全求解报告，组合页展示原始目标的候选 FK 残差、奇异点接近度、阻尼、吸收位移和报告龄期。该残差不代表实机 TCP 测量精度；目标吸收后的保持也不会被标为原始目标已到达。见 [CR5 优化记录](../../../docs/integrations/CR5_DIFFERENTIAL_IK_2026-11-02.md)。

### 工作站配置表单（2026-11-02）

「工作站」（原组合配置）按设备选型、安装与标定、遥操作设置、数据记录、检查与运行组织。安装 XYZ/RPY、输入方向、TCP、控制轴、设备 IP/串口、控制组及记录投影均可通过表单生成，无需手写装配/映射 JSON。JSON 保留为可选文件导入导出与开发者编辑入口。运行期间配置只读；更换控制方式会重新生成对应参数，需要核对后保存新版本。

前端参数与几何转换测试：`cd web && npm run test:workcell`；发布前仍须 `npm run build` 更新 dist。表单仅生成配置，不自动完成标定或身份/停止核验。完整范围与浏览器验证见 [交互调整记录](../../../docs/integrations/WORKCELL_WEB_FORMS_2026-11-02.md)。

### 公共遥操运行诊断日志（2026-11-02）

公共 Runtime 每次启动默认自动保存输入、控制映射/IK 报告、命令回执、反馈与生命周期，不依赖数据采集。Web「工作站 → 检查与运行」可配置并查看/下载历史日志。Web 默认保存于 `<store>/logs/`，直接 Runtime 默认 `~/.local/state/morla/teleop/`，可用 `--log-dir` 或 `MORLA_DIAGNOSTIC_LOG_ROOT` 指定。日志异步写入，有丢弃和失败计数；反馈是缓存采样，旧独立 Profile 入口没有自动迁移。文件格式、配置、公共扩展接口和完整性边界见 [公共日志说明](../../../docs/integrations/COMMON_TELEOP_DIAGNOSTICS_2026-11-02.md)。

### 公共工作站统一操作（2026-11-02）

Quest 右 grip / 键盘 S / Vader M3：开始、重定零点、人工接管与 hold。右 A / A / A：开始录制或交还策略/回放；右 B / B / B：停止保存；右摇杆按下 / N / R3：丢弃本段；左 X / A+N / X：直接回工作位。键盘在运行控制中勾选启用，需页面焦点，输入框不触发；A/N 松开执行，A+N 不补发单键动作。原公共键位已替换。

「遥操作设置」可以填写各设备工作位与插补速度；录制中禁止回位，未配置拒绝移动。公共调度支持接管/交还，隔离旧预测，回放保留游标。更新后重启 Web 和 Quest 接收进程，检查并保存新版本配置，再启动；默认 Vader 旧映射可识别升级，自定义映射保留。详细语义、API 和验证见 [统一操作说明](../../../docs/integrations/UNIFIED_WORKSTATION_OPERATORS_2026-11-02.md)。

### HITL 确认与回滚（2026-11-02）

公共工作站接管与 astral_ws 的核心协议对齐：冻结输出 → 确认设备与所有映射的重锚 → 提交 HUMAN；失败/超时回滚，晚到 ACK 不生效。交还回放使用当前实测位置重锚 offset，交还策略先确认新观测的有效预测。保存接管前暂停状态，暂停不再销毁自动上下文；Web 提供“继续策略／回放”。工作站 Runtime 网关检查 owner、token、epoch、期限与资源。进度和原因在 operator.transition 及公共日志中可查。软件验证不等于实机验收，见 [HITL 对齐说明](../../../docs/integrations/WORKSTATION_HITL_ASTRAL_ALIGNMENT_2026-11-02.md)。

### 紧凑运行概览与机器人状态（2026-11-09）

遥操/采集页将当前工作空间、会话状态、修改状态和设备计数合并为一个摘要栏。当前会话只保留输入设备与控制器，控制链和采集/遥操跳转置于卡片标题行。右侧机器人状态按设备库分类展示当前添加的执行器，点击设备可展开反馈、状态码、报错和控制链报告；左侧画面下方依次为输入与控制权、会话记录。窄屏上下排列，保留原有启动、检查、录制及紧急操作接口。

反馈直接读取现有 v2 session.components，控制器报告按 binding ID 读取 session.controllers。硬件实测频率仅在 hardware、measured、fresh、非 synthetic 且无 fault 的反馈具备 device_status.sequence 和 sample_ns 时，用两次硬件帧计数增量/设备采样时间差计算；该值是观察窗口均值，非浏览器刷新频率，也非日志采样频率。未提供计数的驱动、仿真和测试数据均显示“暂无数据”。配置控制频率单独标注。求解耗时由控制器 completed_ns−sample_ns 计算，候选位姿误差不表示实机精度；现有公共会话接口没有完整 E2E 回执时间，端到端延迟明确标为“暂无数据”。未新增硬件写入路径或改动安全逻辑。

### Web UI 测试数据全流程（2026-11-09）

通过界面从空配置添加 Quest、FR3、夹爪和测试相机，建立控制链、配置安装关系、记录通道和日志后保存；完成运行前检查、启动、遥操/暂停、录制、实时日志、测试策略与人工接管/交还、刷新恢复、停止。可复用测试配置为 `webui_flow_fake_20261109`，全部设备为 fake。测试策略为本地 echo 桩，不是真实训练模型。过程、数据计数及截图见 [验证记录](../../../docs/testing/WEB_WORKSTATION_FULL_FLOW_2026-11-09.md)。

测试中修复了暂停状态文案、旧操作提示残留、策略运行/错误可见性。策略须在设备显式激活后启动，不自动使能；命令拒绝会终止策略，图像解码依赖在取得策略权限前加载。停止录制接口回传完整性失败原因，缺少动作通道的片段保持失败状态。
