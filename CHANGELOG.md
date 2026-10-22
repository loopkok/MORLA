# Changelog（MORLA）

## 2026-10-22

本节补记近期钢琴与 Nero / XHand 集成工作的最终行为。记录时区为 Asia/Shanghai，内容描述工作区及远端部署，不代表已创建发布标签。详细过程见 [钢琴调试日志](docs/piano/DEBUG_LOG_2026-10-20_22.md)。历史条目保留。

### 钢琴 Web、标定和演奏

- 通过独立 `MORLA-piano` 引擎、专用 worker / driver launch 接入钢琴标签页，提供曲谱库、CSV/JSON导入、时序/指法预检、准备、单指/移位测试和演奏。
- 增加机械臂关节、笛卡尔点动和12关节手部目标，支持演奏零位、悬停、按下及移位实测点的草稿记录；持久化配置与活动会话快照分离。
- 单侧标定工作位只校验所选侧。首次Web准备执行双臂MORLA四段轨迹、保存工作位及演奏零位；根据approach结束位置计算最后一段。
- 新增服务器演奏进度、当前动作、时间及音符/动作计数，停止/故障冻结，只有完成后显示100%。
- 再次演奏最终采用抬指后直接回演奏零位再播放，不再重复四段工作轨迹；换曲/速度后重新预检。回HOME、手动标定和故障不能绕过首次准备。
- 新增《欢乐颂主题（双手交替·40BPM）》预设，42音、约83秒、5次换位，左手0/−1、右手0/+1，固定手指和步数。

### Nero 驱动与轨迹

- 维护、复位、CAN初始化、反馈检查、控制权和位置运动分别确认；复位须取得新的控制器状态，不能因SDK调用返回就报告成功。
- 连续点动保留已有保持目标，避免反复用实测跟踪误差播种保持关节；校验新控制域的使能状态。
- 钢琴专用驱动启用原生J7优先帧顺序及J模式流式目标。保持原始途经点及SDK编码，不把固件触发推断写成已确认根因。
- 保留原生HOME第4→3→2→1点及最终J2=85°流程。
- 钢琴回位按钮最终另用：演奏零位 → J4=95° → J1=0° → J2=60° → `[0,85,0,0,0,0,0]°`。J4最后一段80%时长完成，仍满足命令速度约束；左右J2/J4同号。

### XHand、限位和当前容忍策略

- 原生手部反馈显式无力初始化、完整报文初始化、QoS及SDK诊断修正；保留设备故障原始字段。
- 当前串口按实体关联结果为左QWY、右YUK，写入Web默认值；先前YUK左/QWY右作为历史记录保留。
- 根据XHAND 1手册同步组件、适配器、Profile、活动URDF/MuJoCo与外部模板限位，保留legacy模型和冻结schema。
- 小偏差软件手姿参考保留原始读数，只允许明确匹配的边界接管；不等同设备内部标定或Flash校准。
- 到位容差改为3°，修正按下确认另有约1.146°隐藏门槛；普通确认失败在健康反馈下中止并保持，恢复失败仍急停。
- 应用户要求，双手 `arrival_check_enabled=false`；跳过不报告实测确认。反馈、SDK故障、目标限位和控制权检查仍启用。
- 当前共用跟踪误差阈值30°，同时影响臂和手。四指按下关节1=40°/关节2=30°，不改拇指、食指侧摆和悬停。
- 保存移位范围左−7～1、右0～7。左+2～+4因J5超限未放开，没有扩大机械臂硬限位。

### 会话与运行

- 空闲浏览器租约由180秒调整为无浏览器超时；执行动作或维护期间仍保留6秒失联保护，以及独立服务器/worker/驱动心跳。
- 返回页面重检控制权，新命令前续约；正常关闭/刷新页面和显式断开仍释放。状态失效后不保留“将自动恢复”的误导提示。
- 设备锁、ROS写入者互斥和自有进程清理保留，原机器人控制与钢琴不能同时占用同一硬件。

### 证据与未完成验收

手册限位阶段远端194项；外部引擎最近定向26项；回位9项；心跳4项；直接重播1个manager/worker集成用例；各批次前端构建通过。它们对应不同代码状态，不相加表述为当前全量验证。欢乐颂通过当前配置预检和纯软件完整演奏；用户报告过现场执行结束。新回位、最新直接重播及声音/节拍精度未记为独立真机验收通过。

2026-10-21曾发生SDK诊断Flash误写，详见 [事件记录](docs/piano/sdk-probe-incident.md)，本次补日志不删除或淡化。双臂红灯/无新控制器状态的通信故障没有足够证据确定根因。

## 2026-10-20—2026-10-21：钢琴初建与联调阶段

- 建立外部非遥操钢琴引擎，随后增加Web独立页面、在线标定、手部被动反馈初始化、维护状态拆分和进程互斥。
- 引入原生工作位途经点、实测到位判断及双臂阶段屏障；逐步排查急停锁存、反馈缺失、负读数接管和持有旧参数的会话。
- 保留 [初始集成摘要](docs/piano/integration-summary.json)、[标定集成摘要](docs/piano/manual-integration-summary.json)、[手姿参考记录](docs/piano/hand-reference-calibration-20261021.md)。阶段边界以这些历史文件和会话为依据，不编造精确逐日提交。

## 2026-10-15

**同步 Astral WS raw 遥操回放与回放源浏览**——对照 Astral `48642270361bf525d91b5b73f7bea893d9d918fe`，在 MORLA `morla_policy_inference` 中增加闭合 raw episode/session/`robot_data.h5` 自动识别，按 frozen profile 的 state/action block ID、stream indices 和 action indices 映射实测关节/末端状态；线性重采样关节、因果保持 ratio，校验公共时间窗、时间戳、样本数、采样间隙及 pause 事件，不读相机、不写 aligned 文件，并继续执行 MORLA 的 layout/kinematics 防错和 HITL/安全执行链。回放按录制帧率调度，超过 control rate 时拒绝启动并记录轨迹摘要/数据注记。`morla_web_manager` 新增数据根目录边界内的只读回放源 API 与网页选择器，支持 raw/aligned HDF5、LeRobot v2.1，阻止未保存段、unsupported dataset 和越界 symlink；回放命令标准化只处理动词，保留大小写敏感路径和空格。策略推理测试 156 项、Web manager 测试 44 项通过；两个 ROS 包定向构建成功，Vite 53 modules 构建成功，新增 Python 源定向 flake8 检查通过。raw 真机录制段尚未现场回放验收。

**同步 Astral WS 最新 Quest 输入、HOME 和工作位配置更新**——对照 Astral WS `38f9b458`、`41279f0e`、`54ad233b`，将三项变更移植到 MORLA 对应实现。① `morla_input.PacketGuard` 采用 `@q3m` 单调时间包头和 peer/nonce 绑定的 `@q3sync` challenge/response，按四时间戳估算 Quest-PC 单调时钟差、继续执行 0.5 秒源年龄和序号/session 防护，并把接受帧时间映射回 PC wall-clock；保留 MORLA 的新 session `quest3/input_epoch` 语义。ROS UDP/TCP 和 standalone 走同一检查器。`allow_legacy_packets` 仍默认为 false，显式 legacy 仅接收未封包原始行，旧 `@q3` UTC-ms envelope 不会绕过同步保护。② Astral HOME park 在倒序回到最后安全途经点后增加“其它关节保持、单独 J4 归零”中间点，再回全零；工作位和段间回位路径不变。③ 双臂工作位 J1 waypoint 由 −1.6 调整到 −1.2 rad，Pi05 standalone 示例同步。`morla_input` 与 Astral arm 两包构建通过，输入测试 10 项及 Astral arm 回归 49 项通过（arm 测试有一条第三方 `hppfcl` 弃用警告）；Quest 端协议源码已在 astral-tracking `a6e262c`，但 Unity APK 尚未构建/安装，设备侧握手仍待实机验证。

## 2026-10-12

**工作位/启动 init 轨迹增加实体到点确认**——`astral_arm_teleop`。此前路径推进只看命令位置 `q_cmd` 到达途经点，实体关节可能仍明显落后却已开始下一段。现工作位 `/teleop/init` 对每个途经点和最终 `init_pose` 都等待新鲜关节实测进入 `homing_via_tol`（默认 0.12 rad）并稳定 `homing_via_settle_s`（默认 0.5 s）；等待时间由 `homing_init_arrive_timeout`（默认每目标 12 s）限制。超时后停止在当前命令目标、保留原 `init_pose` 并提示检查电机使能与关节反馈，不跨越未确认的安全点。直达 init 路径同样确认最终目标。HOME park 保留原有 `homing_via_hold_s` 超时放行兼容语义；逐拍跟随冻结守卫仍只作用于 HOME park。左右臂配置和 README 同步说明。新增途经点稳定、最终目标等待及超时保持回归；**本次按要求未运行测试**。

**离线数据管线强化来源签名与默认失败保护**——`morla_data_collect.pipeline`。**背景**：旧 aligned 文件可能缺少 raw 来源签名或与当前 raw 已不匹配，却仍被预览、回放、诊断、清洗或导出流程消费；一键导出默认隔离移动失败段，也可能在质量检查失败后继续尝试导出。**做法**：① 新增 `require_source_signature`，要求 aligned 内记录的源签名与 episode 当前输入一致；签名覆盖 raw HDF5/meta 的大小与修改时间及 meta SHA256。align 的缓存跳过、prepare（含 aligned）、preview、quantify、repair、replay、validate 与 verify-aligned 均先检查签名；缺签名或源变更要求在工作副本显式重建，拒绝把旧缓存当作可信输入。② `verify-existing` 对来源不可信的 aligned 跳过清洗预览并标出原因，不把其结果累计成有效帧。③ `process-openpi`/`process-act` 默认不移动失败段；只有显式 `--quarantine` 才启用隔离。校验失败现在立即中止导出；帮助文本、pipeline README 和 Web 任务回归同步这一默认行为。④ next_state 校验补查最后一帧 action 必须等于末帧 state，避免终止 hold 被改坏后仍通过训练准入。新增来源签名缺失/过期、terminal hold 和默认/显式隔离回归；**本次按要求未运行测试或数据处理**。

**视频预览保留源帧时间并暴露 hook 错误**——`quest3_video_streamer`。ROS 图像预览继续使用消息 header 时间，webcam 预览使用 `VideoCapture.read()` 返回时记录的主机时间；时间随帧穿过非阻塞队列与 JPEG 编码，发布时不再改写成编码完成时间。preview 编码/发布和 ROS/webcam preview hook 异常由静默吞掉改为每个 publisher/source 仅告警一次，避免重复刷屏同时留下故障线索。新增源时间戳回归文件，**本次未执行**。

**Web Monitor 默认只监听本机**——`morla_web_manager`。host 默认值由 `0.0.0.0` 改为 `127.0.0.1`，避免服务启动后自动暴露到所有网络接口；需要局域网访问时可显式设置 `ASTRAL_WEB_MONITOR_HOST=0.0.0.0`，或 launch 参数 `web_host:=0.0.0.0`。README 与 launch 用法同步更新，并增加默认 host 配置回归；**本次未执行测试**。

**Quest 3 人在环接管、控制权仲裁与介入数据链路加固**——`morla_policy_inference`、`astral_arm_teleop`、`morla_bringup`、`astral_gripper_teleop`、`wujihand_retargeting`、`morla_input`、`morla_data_collect`。
**背景**：策略与人工控制切换边界可能同时写执行器；重锚服务在其他 writer 准备完成前即可武装，迟到响应也可能越过回滚；X 回位、头部和 Wuji 手部存在绕开策略仲裁的路径；夹爪策略转人工可能从旧遥操值跳变；Quest 网络接收按到达时间发布，无法识别旧包/乱序包；采集和导出缺少可用于区分策略与人工接管的逐帧标签。
**控制交接**：① 新增共享 `PolicyLease`，策略状态携带 epoch、递增序号、发送时间、transition ID、pending 标记和 reanchor token；writer 校验租约、拒绝过期/重复/乱序状态，并在串行门控生效后按 transition 回 ACK。策略/回放切换等待当前 schema 所有执行器 writer 确认。② 接管先调用 `prepare_reanchor` 更新臂锚点并取得一次性 token，但不开放输出；全部准备完成后提交 HUMAN，匹配 token 后臂 writer 才能武装。腕部与关节反馈须新鲜，失败/超时取消准备、保持当前位置并暂停旧策略，不能恢复可能陈旧的 chunk。③ pending 控制周期优先处理 stop，禁止策略/回放继续发目标；首轮策略规划移到后台，等待 writer ACK、规划完成和新鲜观测后才交还控制权。执行器以当前关节/夹爪目标 seed，交接提交周期不额外发送首帧；引擎拆除移出控制周期，stop 可中断等待。④ 臂、头、夹爪、Wuji retarget/HOME 均检查所有权租约；输入 header 默认按 0.5 秒限龄。策略运行时屏蔽人工 start、HOME、init 和 X 回位。夹爪以策略最近执行闭合比为软接管起点，要求新鲜输入靠近该值后再响应；Wuji 根据当前关节命令起步并按 `max_joint_rate` 限速。右手 A 的 release 统一由常驻 `controller_start_gate` 发送。
**Quest 输入**：ROS UDP/TCP 接收器和 standalone 接入共用 `PacketGuard`，识别 `@q3 <session> <sequence> <utc_ms>|` 包头，默认最大源年龄 0.5 秒，拒绝未来/过期、重复、乱序和旧 session 包；TCP 行缓冲限制为 1 MiB，controller 数值与 mask 也做有限值/范围检查。当前 `quest3_mocap.yaml` 为本次旧 APK 验证暂设 `allow_legacy_packets: true`；该兼容模式不提供源时间、序号和 session 防护，换装带新包头的 APK 后应改回 `false`。Quest/PC 时钟需同步；Windows Unity 6000.0 未用于重建或安装 APK，设备端效果仍待确认。
**介入标签与导出**：采集 episode 记录策略 ownership 事件；align 将 `control_owner`、`intervention`、`intervention_valid` 投影到帧，repair 按选中源帧同步标签，OpenPI LeRobot v2.1/v3 导出保留字段。`control_owner` 编码为 unknown=0、policy=1、human=2、playback=3、pending=4、idle=5；合成 hold 不标成人工，历史无事件段保留 unknown/invalid，不从动作轨迹推断。Parquet 将 intervention 两列写为 scalar bool、owner 写为 scalar int64（LeRobot feature shape `[1]`），避免 v3 reader 将列表列拒绝。详见 [HITL 审查](docs/quest3-hitl-audit-2026-10-12.md) 与 [修复和验证记录](docs/quest3-hitl-fixes-2026-10-12.md)。
**验证记录**：随附修复记录报告 policy inference 143 项、数据管线 117 项通过/1 项跳过、臂与遥操 70 项、Quest 3 项，以及 7 个相关 ROS Python 包 `colcon build --symlink-install` 通过。历史数据 episode 0/50/99 在副本中导出 611 帧，v2.1→v3、现代 LeRobot 加载和 ACT CPU 预处理通过；原始 HDF5 只读，且因旧段没有 ownership 事件，标签仍为 unknown/invalid。本次按用户要求未重新运行测试、构建或真机操作；修复记录中的数字为已有工作记录。

**采集暂停分段与相机时间戳语义修复**——`morla_data_collect`、`quest3_video_streamer`。
**背景**：旧暂停/继续仍把整段时间作为连续轨迹，对齐可能跨暂停间隔补帧；arm/disarm 分开保存会造成有效状态矛盾；采集抽头在 JPEG 编码后才取时间，时间戳混入队列与编码延迟。**做法**：① 暂停前落盘缓冲，继续时封存当前 episode 并创建新段；末尾暂停时间不计入段时长。任务文本在段开始时冻结，避免后续修改污染当前段。② arm/disarm 更新同一有效 armed 状态，段首记录状态快照，meta 标明 `effective_armed_v2`。③ 相机采样时间随帧穿过 collect hook、抽头编码队列和 ROS 发布；ROS 输入保留 header stamp，无有效 stamp 时回退主机时间；raw 相机 HDF5 新增 `arrival_timestamps`，meta 标出各流时间戳语义。④ webcam 在 `read()` 返回时记录主机时间；该值代表主机读帧时间，不代表曝光时间或跨设备同步精度。schema 同时校验 fps、相机标签、最大时间缺口与 hold 参数。**验证**：本次按要求未运行测试或真机采集；新增回归用例已提交，未执行。

**离线数据工具集中迁移并统一质量准入**——`src/common/morla_data_collect/morla_data_collect/pipeline/`。
**背景**：对齐、诊断、清洗、导出与迁移逻辑散落于仓库 scripts 和 ROS 包父目录，入口及安装行为不一致。**做法**：① 通过 `scripts/data_pipeline.sh`、安装后的 `data_pipeline`、`python -m morla_data_collect.pipeline` 统一分发 prepare/preview、quantify、align、validate、repair、replay、OpenPI/ACT 导出、v3 升级、相机迁移及来源验证；两种一键导出共用 `process.sh`。Python 路径按仓库位置定位并支持 `ASTRAL_DATA_PYTHON`。② 离线实现归入 pipeline 子包，旧 scripts 实现和父包模块移除；删除 `align_data`、`validate_data`、`convert_to_lerobot`、`replay_rerun` 四个旧 ROS 离线 console entry，改用统一入口。训练提示、策略回放导入、README 与排障文档同步迁移。③ 对齐采用过去最近 command 与夹爪零阶保持，记录来源网格/帧索引/源签名并拒绝失效缓存。清洗修复相机组内时间字段和 HDF5 属性读取，默认保护操作意图及非臂动作，保留 command action、源时间与帧来源。校验和导出覆盖 state/action/时间戳/schema/维数/相机与严重缺口，逐段准入并全量解码 JPEG；失败时停止而非静默跳过。④ `artifacts.py` 增加来源签名、路径重叠拒绝和暂存文件/目录提交，目录替换失败可恢复旧产物；质量报告与处理清单随导出保存。**验证**：本次未运行新测试、构建、数据处理或训练。10 月 10 日既有离线/ROS 测试、wheel 安装及 100 段只读检查和 5 段 1066 帧来源验证记录，详见下方同日条目及 `docs/data-pipeline-fixes-2026-10-10.md`，本次未复跑。

**Web 数据处理后台及界面**——`morla_web_manager`。新增 DataProcessTab 与独立 Python 子进程任务管理，调用统一数据管线；支持完整流程或单步、段号/相机映射、可选诊断与保守清洗、OpenPI/ACT 导出、v2.1→v3 升级、步骤状态、日志、质量报告、图片/状态预览和历史产物。REST 增加 session、任务提交/查询/取消与产物读取，WS 状态增加 `data_process`；限制为单活动任务，阻止处理正在录制/暂停/保存的 session，支持取消、服务重启中断标记和历史恢复。session 先复制到任务工作目录，Web 不自动隔离失败段；输入限定于数据根目录，产物读取限定任务目录。前端源码与已有 dist 一并提交，setup 安装静态资源并支持 ROS share 路径回退。部署配置：`ASTRAL_SDK_ROOT`、`ASTRAL_DATA_ROOT`、`ASTRAL_DATA_PYTHON`、`ASTRAL_DATA_CHECK_PYTHON`，静态目录可由 `ASTRAL_WEB_MONITOR_DIST` 覆盖。**验证**：本次未运行新增 Web 测试、前端构建或浏览器验收；10 月 10 日既有 REST/数据流程验证记录见同日条目，未复跑。

## 2026-10-10

**Web 数据处理板块**：新增 DataProcessTab 和后台任务管理，复用统一 CLI。支持完整流程/单步、段号选择、历史相机映射、可选保守清洗、OpenPI/ACT 导出、步骤状态、日志与质量报告、浏览器图像/状态预览、取消与持久历史。新增 prepare/preview 子命令，所有处理先建立真实工作副本；原始 raw 不改、不自动隔离失败段。前端 dist 随包安装，默认可定位。验证：23 项 Web 回归、84 项数据离线回归通过；npm build 通过。独立无硬件测试服务 REST 提交历史 5 段，13 步全部通过，导出 1066 帧，两格式及 ACT 真加载/预处理通过；原始 HDF5/meta SHA256 未变，结果在 web_jobs/20261010-160354-20b6d236。工具环境无浏览器，未完成交互式视觉验收。

**删除旧数据处理入口**：按用户要求移除 7 个旧 scripts/ 数据脚本、7 个父包兼容模块和 4 个旧 ROS 离线 console entry；实现仅保留在 pipeline/，命令统一为 data_pipeline。文档、训练提示和测试同步更新。验证：84 项离线测试、39 项 ROS 测试通过；colcon 重建后仅保留 data_pipeline 离线入口；真实 5 段 1066 帧来源检查通过。

**离线数据工具统一目录与入口**：对齐、校验、清洗、诊断、导出、回放、迁移与验证实现集中到 `morla_data_collect/pipeline/`；`scripts/data_pipeline.sh` / 安装后的 `data_pipeline` 提供子命令。两条导出共用 `process.sh` 编排，Python 路径按仓库位置推导并支持 `ASTRAL_DATA_PYTHON`。旧脚本、父包离线模块及旧 ROS 离线入口已删除，只保留统一入口。来源验证去掉本机目录硬编码，按帧索引核对全部图像与动作；空输入拒绝。数据格式与默认清洗规则保持不变。验证：91 项离线回归（含带空格路径的完整 OpenPI/ACT 一键链路）、39 项 ROS 回归通过；wheel 打包/临时安装通过；统一入口对真实 100 段只读检查、5 段 1066 帧端到端验证通过，raw 未修改。

**采集与数据清洗修复**：暂停继续分段、有效 armed 状态及任务冻结；相机编码保留采样时间并记录 arrival；command 因果对齐、夹爪零阶保持，缺口与失效缓存拒绝复用。清洗默认保护操作意图及非臂动作，保留 command action、真实相机时间戳和来源索引，显式处理历史相机别名。校验覆盖 aligned action/时间戳/schema，导出强制检查每段并全量解码 JPEG；输出失败不破坏既有产物。新增质量报告、处理清单与真实数据验证脚本。验证：100 段原始数据只读检查及清洗预览；5 段重建/清洗导出 1066 帧，LeRobot v3 加载成功，raw 未修改。详见 `docs/data-pipeline-fixes-2026-10-10.md`。


**独立推理部署适配：serve.py 部署机无关化 + init 超时可诊断 + 夹爪越界钳制**——
`morla_policy_inference/scripts/serve.py`、`pi05_standalone`。真机部署期间三处改进：
① **serve.py 去硬编码**——原来 `sys.path.insert(0, "/home/robot/loopkok/sdk/...")` 写死开发机
绝对路径，换部署机/用户名/目录即失效；改为按脚本自身位置定位（`Path(__file__).parents[1]`，
package/scripts 与 package 并列布局），不依赖 cwd 与用户名。
② **init 段超时可诊断**——`move_to_init` 超时原只报"timed out"；现带完整三方误差
（measured-target / command-target / measured-command gap）与具体数组值，可直接判断是目标
不可达、跟踪滞后还是指令超前；初始化期间每 2s 打进度日志；并修复超时早于首次采样时
`measured/gap` 未初始化即引用的隐患。
③ **夹爪越界从拒绝改为钳制**——`serve_pi05.validate_actions` 对模型输出的夹爪比例越界
（原 [-0.01,1.01] 外直接 ValueError 拒整个 chunk）改为钳到 [0,1] 并 WARN 记录越界帧数/范围；
非有限值/维数错误仍拒绝。**动机**：pi0.5 偶发输出 1.01~1.05 类轻微越界，真机因整 chunk 被
拒而停发——钳制后仅记录不影响连续执行；README 同步说明。
**验证**：py_compile 全过（serve/infer_astral_single/serve_pi05）。

## 2026-09-24

**OpenPI π0.5 guided RTC：旧块未执行尾段引导去噪（LeRobot RTCInferenceEngine 风格）**——
`morla_policy_inference`。**动机**：普通 pi05 checkpoint 换块时新块与旧块尾段互不知情，边界拉回/
内部接缝需靠节点端平滑；guided RTC 把旧块**尚未执行的绝对目标**随请求送到服务端，经 Astral
`DeltaActions`+归一化+维度填充后，在每个 flow-matching 去噪步约束新块前缀——边界由模型自身平滑，
**不需要 RTC 专项训练 checkpoint**。**做法**：① `serve.py` 新增 `--rtc-guided`/`--rtc-execution-horizon`
（默认 10）：预热同时编译普通与 guided 两条 JAX 图（不同图，普通预热不覆盖）；握手 metadata 带
`rtc_guided/rtc_horizon`；校验 horizon≥1、≤模型 action_horizon、guided 需 model=pi05+warmup。
② `RemoteBackend` 增 `rtc_guided/rtc_horizon`，`open()` 握手校验服务端须 `model=pi05`+`rtc_guided`+
horizon 匹配；`infer(rtc=...)` 发送 `rtc_prefix/rtc_prefix_valid/rtc_delay/rtc_max_guidance_weight/
rtc_schedule`。③ `ActionEngine` 增 `rtc_queue_threshold`(30)/`rtc_execution_horizon`(10)/
`rtc_max_guidance_weight`(10)/`rtc_prefix_schedule`(linear,zeros|ones|linear|exp)：剩余行≤阈值即启动
引导推理；prefix=已提交轨迹尾段（不足按末行重复补满 horizon），`delay` 按历史最大 RTT×fps 估；
`_install` 在 guided 下若 `consumed>=rows`（延迟超返回horizon）拒绝；planner 循环 guided 用
`remaining>threshold` 判闲（对齐 LeRobot RTCInferenceEngine）。④ 节点：engine_mode=rtc+model=pi05
自动启用 guided（要求 backend=remote），并**自动关掉时序融合与 anchor 平滑**（避免对模型引导结果再
叠加轨迹混合）；启动行自报 `rtc_guided/rtc_queue/rtc_horizon`。**验证**：policy 套件（engine RTC
参数校验/prefix 构造/安装守卫）+ 编译通过；真机部署须服务端/节点 horizon 一致（握手强校验）。

**WN 相机固件更新 + 双相机迁 USB2 后全链路验证通过（30fps ×3 路、0 损坏）**——
`quest3_video_streamer`/数据采集链路。**背景**：此前 USB3@120fps 下相机编码器 ~1% 坏帧、固件拒绝 30fps
（详见 09-23 条目）。**本次**：① 厂商更新固件——两 WN 相机在 USB2/USB3 都枚举 MJPG 720p 30/60/120；
② 用户将两台 WN 相机全部移到 USB2 口（right_wrist=usb-0:2.2.2→video8、left_wrist=usb-0:2.2.3→video6），
D435i 挪到 USB3（usb-0:1.4，彩色节点 1.4:1.3→video4）；③ `label_aliases` 按新 by-path 更新（base→
1.4:1.3、left_wrist→2.2.3、right_wrist→2.2.2）。**实测**：两台 WN 720p30 各 200/200 帧、error_dc=0；
**base 1080p30 全链路**：base 27.3fps/collect 30.04Hz、left 27.8/30.03、right 27.8/29.97，Corrupt JPEG=0；
**base 720p30 全链路**：base 28.0/29.99、left 27.8/29.98、right 27.8/30.04，Corrupt JPEG=0；端到端
collect 三路各 40 帧全解码、0 条带伪影。**结论**：坏帧问题随固件更新+USB2 迁址彻底消失（此前 base
19-26fps 是旧固件右腕 120fps 吃 CPU 所致，现右腕 30fps 释放后 base 1080p30 稳定），**base 无需降档
720p**，当前配置可正常采数/推理。

**policy_inference 远程协议加固：有界超时 + action_dim 握手校验 + 响应形状/NaN 校验**——
`morla_policy_inference`。**动机**：服务未启动或推理无响应时 `WebsocketClient` 会一直重连/阻塞读，
节点无从报错退出；且无法在握手阶段发现"客户端/服务端 action_dim 不一致"（错配要到首帧 infer 才崩）。
**做法**：① `server_connect_timeout_s`/`server_infer_timeout_s`（默认 5s/3s）：连接在期限内重试
（0.2s 间隔）、握手与单次推理读取超时即报错；`infer` 超时关闭该连接再抛错。② `RemoteBackend.open()`
握手读取服务端 metadata 并校验 `action_dim`（不匹配拒绝并关闭连接）；`serve.py` 握手 metadata 补发
`action_dim`，客户端兼容只发 `model` 的旧服务。③ 动作响应形状严格校验（必须 2-D 且 rows≥1）并拒绝
NaN/Inf。④ `last_server_timing` 兼容独立 pi0.5 服务的 `server_ms` 字段（缺 `server_timing` 时）。
**验证**：新增 `test_client.py`（不可达服务有界超时、旧 metadata + infer 超时关闭连接）+
test_backend 3 例（server_ms 透传、错误 action_dim 握手拒绝并 close、空/NaN 动作拒绝）。

**pi05_standalone execute 模式：初始化路径 + MOVE/START 交互确认 + 安全断开（不下 DISABLE）**——
`src/pi05_standalone/`。**动机**：原 execute 从任意实测位置直接切模型动作，无安全起始位；`--ready`
调用 SDK `one_click_ready` 会把所有关节归零（危险）；`close()` 走 SDK 公开 `disconnect()` 会先发
DISABLE——观察/推理运行后断开也可能让活动中的臂掉落。**做法**：① execute 要求机器人预先
WORK/POSITION/enabled，从实测位置以 `init_joint_vel_rad_s`(0.6) 每关节上限先走 `init_waypoints` 再走
`init_pose`（config 新增 init_* 段）；每点须新鲜关节反馈、`init_arrive_tol_rad`(0.05) 内稳定
`init_settle_s`(0.3s) 才前进，跟随误差超 `init_follow_tol_rad`(0.25) 暂停轨迹，单段超
`init_timeout_s`(40s) 终止。② 运动前交互终端输入 `MOVE`、到位后再输入 `START`（同一终端）才开始
推理；`--duration` 从 START 后计；START 前校验臂仍停在 init_pose（偏离 >0.05 拒）。③ `--ready`
彻底禁用（拒绝）。④ `close()` 绕过 SDK 公开 `disconnect()`（会先 DISABLE），直接停本地线程 + 关
UDP socket，不下电/运动指令。⑤ Planner 兼容独立服务 metadata（有无 action_dim 均可）并读
`server_ms` 计时。**部署信息**：GPU 机 `lukang@192.168.1.249`（独立端口 8002，checkpoint
`pi05_astral_lora/single_pick_place/15000`），机器人 `nvidia@192.168.0.5`。
**验证**：test_standalone 增 3 例（execute 初始化路径+MOVE/START 顺序、缺 init_* 拒 execute/拒绝
--ready、未到位前进超时）+ close 不下 DISABLE；离线用例全绿。

## 2026-09-23

**新增独立 Astral pi0.5 推理包 `src/pi05_standalone/`（最小路径，定位卡顿用）**——
不导入 `astral_ws` 其他包、不启动 ROS 2；机器人端依赖 `astral_robot_sdk`+OpenCV+websockets，
GPU 端依赖 `VLA/openpi`，两端可不同 Python 环境。**动机**：在完整 ROS 策略栈（callback 组/融合/
插值/队列）之外提供一条刻意**无融合、无插值、无 ROS 队列**的最小推理链，JSONL 逐行记录输入年龄、
控制延迟与实际动作，与 `pi_plan_trace.jsonl` 互相印证，用于定位"换 chunk 拉回"之外的纯链路卡顿。
**构成**：`serve_pi05.py`（GPU 端 websocket 服务：校验 8 维 state / `base_0_rgb`+`left_wrist_0_rgb`
双槽 224×224×3 / 非空 prompt，夹爪 ratio 校验并钳 [0,1]，预热一次）+ `infer_astral_single.py`
（机器人端：`observe`/`infer`/`execute` 三模式，30Hz 主循环 + 后台模型请求 + `replan_after_rows=10`
提前请求、新 chunk 从首行切换、用尽无新结果即停；每步 `max_joint_vel_rad_s/control_hz` 限速并记原始
动作与实际命令；`letterbox_bgr_to_rgb` 等比缩放补黑边 224）+ `protocol.py`（msgpack ndarray 线协议）
+ `config.example.yaml`（SDK 板卡地址 + 相机 by-path + 任务文本 + 夹爪初始比例）。**约定**：胸口
RealSense→`base_0_rgb`、左腕→`left_wrist_0_rgb`；8 维绝对动作前 7 维送左臂电机、末维夹爪
`2.5*(1-ratio)` rad；夹爪无反馈用启动时初始值/最近下发值。**约束**：与 `astral_robot_driver` 不能
同时跑（都绑 SDK UDP 端口）；默认不上电/归零，`--ready` 才 `one_click_ready`（使能并归零）。**验证**：
开发环境无相机/真机，仅离线静态/假设备用例（`test_standalone.py`：wire roundtrip、letterbox 几何/
RGB、action 校验）；真机按 observe→infer→execute 逐阶段验证。

**USB3@120fps 相机 JPEG 损坏根因定位 + 诊断脚本 `scripts/diag_camera_usb3_fps.sh`**——
`quest3_video_streamer`。**动机**：同款 WN 相机 USB2 口可选 720p30 无损坏；USB3 口只有 120fps 且
录像经 ffmpeg 报大量 JPEG 错。**定位（纯离线分析，无相机本机）**：ffmpeg 报错分两类——①
`unable to decode APP fields` 为**良性**：相机每帧写空载荷 APP1（`FFE1 0004 0000`），ffmpeg 4.4
`mjpeg_decode_app` 解析失败即报（删除 APP1 后错误消失）；两相机、每帧都报，libjpeg/OpenCV 全忽略。
② `error dc`/`overread` 为**真实熵编码损坏**：仅 USB3@120fps 出现，video0_ubs2（USB2,442 帧）0 帧损坏
vs video9_ubs3（USB3,1563 帧）12 帧（0.77%；前次录像 7/287≈2.4%）。损坏表现为 1-3 个 MCU 行横向条带
（Huffman 失步到 RST 重同步），帧完整（SOI→EOI 齐全）、非截断、与高码率帧无相关。**机理判断**：
UVC 传输有 CRC、出错帧默认被 uvcvideo 丢弃（`nodrop=0`）——能完整送达的损坏是相机 JPEG 编码器在
120fps 下间歇性故障（传感器/编码器跟不上），不是 USB 丢字节。**待实机验证**：`--set-parm=30` 是否
被 USB3 相机接受；USB2@120fps 对照（损坏跟帧率还是端口）；dmesg 无 USB 错（佐证相机侧）。脚本
固化以上三步并输出 VERDICT。**做法**：新增 `MORLA/scripts/diag_camera_usb3_fps.sh`（S_PARM 30
测试 + USB3 录制 + USB2@120 对照录制 + ffmpeg error dc 计数 + dmesg USB 错误检查）。**后续实测
（Jetson 报告 2026-09-23）**：① ffmpeg `-f v4l2` 实时解码 120fps 报 `Found EOI before any SOF`/
`No JPEG data`/`bad vlc`/dts 洪水 = 软解过载+缓冲覆盖，**只在实时路径、文件里没有**；② ffmpeg
`-c:v copy` 采集仅 59.6%（V4L2 输入层解析开销致丢帧），v4l2-ctl 采集 96.5%——但 **v4l2-ctl 纯净
采集的 `test_v4l2_15s.mjpg` 离线解码仍有 error dc**，实锤损坏为相机编码器自产、与采集工具/传输
无关；③ 相机实测 ~130fps（超标称）→ 编码器过载。全工程图像链路（采集/推理）核实**无 ffmpeg**，
均走 OpenCV V4L2（`webcam_source` cv2 排干线程+令牌桶），ffmpeg 丢帧结论不影响本项目。剩余待
实测：USB3 `--set-parm=30` 是否被接受、USB2@120 对照（帧率 vs 端口）。**决定性实测（2026-09-23
下午）**：① USB3 相机 `--set-parm=30` **被固件打回 120**（get-parm 仍 120/1）→ USB3 口上无软件
手段拿 30fps；② USB2 相机 `--set-parm=120` 录制 1200 帧（~118-119fps，160MB，帧均 134KB）
**error dc = 0** → 同样 120fps，USB2 口干净、USB3 口 ~1% 损坏。**判定：损坏跟 USB3/SS 链路相关
（SS 下相机还超发 ~130fps），不是帧率本身；且 USB3 口无法协商 30fps** → 唯一干净的 720p30 路径是
**USB2(HS) 链路**：最省事 = USB2-only 数据线（无 SS 对）插同一 USB3 口 usb-0:1.3 → 链路强制 HS →
相机 HS 描述符出 30fps，且 by-path `usb-0:1.3` 不变 → **label_aliases 零改动**；备选挪 USB2 口 /
USB2 hub（by-path 变，改一行 label_aliases）。验证：换线后 `--list-formats-ext` 见 30fps、`--set-parm=30`
生效、录制 error dc=0。**受控亮场景补测（2026-09-23 晚，SSH 背靠背录制）**：① 帧大小差异**复现且
定位机制**——USB2@120 中位 134KB（DQT0[0]=q15 粗量化）vs USB3@120 中位 169KB/均 252KB（q11 细量化，
胖尾 554KB），场景亮度/细节一致（95-107 / 2335-2513）→ **固件按链路带宽自适应量化档位**（SS 细量化
高质、HS 粗量化省带宽）；② **USB2@120 亮场景 0 error dc / USB3@120 亮场景 5/400（1.25%）** → 损坏与
USB3/SS 链路绑定结论用有效内容补强（此前 usb2_at120 文件被黑帧污染，已弃用该数据点）；③ `v4l2-ctl -L`
确认**无 JPEG 质量/帧率控件**，量化档位不可改 → 相机侧无法"做成 USB2 一样"，但 collect_tap 统一重编码
quality-90 使训练数据不受原始帧大小影响，且换 USB2 链路后相机自动用 HS 档位（q7@30/q15@120），
输出与 USB2 相机一致。

## 2026-09-22

**修复 `TEST_0922-1626` 第二个 chunk 安装崩溃及 planner 静默死亡**——
`morla_policy_inference/engine.py`。本轮 `coeff=0 + anchor_tol=0.05` 在第二次 plan 的 anchor blend
报 `ValueError: assignment destination is read-only`：msgpack 从不可变 `bytes` 恢复 ndarray，原
`np.asarray` 未取得可写所有权；planner 线程退出后仅执行完首个 50 行 chunk，随后持续重发末指令。
现于引擎所有权边界复制实际安装的 `rows × action_dim` 小数组，并将融合、anchor 与计数更新改为事务式
安装，全部成功后才替换旧 chunk，异常时恢复 ensemble 计数、旧 chunk 和消费索引。planner 外层新增
兜底：未预期异常写 `planner_failed` trace/metrics，已接受尾段执行完后由 `EngineStateError` 驱动节点
安全 stop，不再无限 hold。关闭 anchor、ACT temporal ensemble、queue_sync/queue_async/RTC 的数值及
时序语义不变；50×8 float64 复制约 3.2KB，基准约 0.25µs/plan。新增只读远端 buffer、anchor 开/关、
ensemble 组合、非法/空 chunk 拒绝、后处理回滚和 planner 异常注入测试。**验证**：推理包完整 119 tests passed，compileall
与 `git diff --check` 通过。

**整理 ACT 推理卡顿排查记录为独立文档**——`docs/act-inference-stutter-investigation.md`。
从 `morla_policy_inference/CLAUDE.md`"推理优化历程"章节展开：四层叠加原因（参数没生效 /
换 chunk 断流 / 收敛拉回尖峰 / 训练数据节奏 / 网络带宽）+ 两个 ACT 专属机制（单行 chunk、
摩擦伪影）+ 排查标准 checklist + 诊断资产。CLAUDE.md 加指向；CHANGELOG 留档。

**新增 pi0.5 每次 plan 的端到端可审计 trace，验证观测旧/非原子快照与 chunk 切换**——
`morla_policy_inference`。D2 表明训练集动作本身均在硬件范围内；先前日志中的越界是当前模型 raw
output，不能反推训练数据有问题。此前 C1 的 left_wrist timeout 已由 callback 隔离修复，但“图像流
fresh”不等于模型请求中的 state/双相机是同一时刻，也无法从旧日志验证约 95ms remote RTT 期间的
action 消费补偿。新增 `plan_trace_log_file`，`log_dir` 自动生成 `pi_plan_trace.jsonl`；每次 plan 以
非阻塞后台 JSONL 记录请求 state、相机/状态源龄期、RTT/server timing、`snap_i/consumed/resume_i`、
完整 server raw chunk、最终 installed chunk 及旧/新边界动作。诊断写盘不会阻塞 planner 或控制线程，
队列满时 fail-open 丢弃 trace。未改变模型采样、时序融合或 YAML 策略参数。**验证**：engine + runlog
Python 单元测试 24 passed；语法编译通过。

**补充 ACT 历史卡顿与 pi0.5 的跨模型归因**——`docs/2026-09-21-pi05-inference-investigation.md`。
回查 2026-09-14 ACT 原始指令/metrics：未启用平滑时 5--17 个大步全在换 chunk 附近，已分类的 7 个中
6 个为旧开环 command 向实测 state 的收敛拉回；state↔command 稳定相位差约 194--214ms。故 pi0.5
当前的“旧 chunk 偏离实测→新 plan 拉回”不能首先归因于 flow-matching，先以新 plan trace 定责共同的
观测/执行时间轴。ACT test5 的平滑参数真正生效后尖峰清零；pi0.5 D2 刻意关闭相同参数作原始基线，
两者不能直接比较。left_wrist 旧帧是 C1 已证实风险，但旧 ACT 日志缺少逐帧龄期，未将它倒推为 ACT
历史卡顿的既定原因。

**`TEST_0922-1505` 首次以逐 plan trace 定责 pi0.5 的运行中抽搐**——
`pi05_infer_test_log/TEST_0922-1505/20260922-150517`。完成后立即 stop 的 10.23 秒窗口内 300/300
tick 都正常 emit（cmd dt p95 37.7ms），无 state/image gate、hold、queue empty 或遥操竞争；driver
接收的 arm command 与 policy 指令逐值一致。13 次 plan 的 state p95 龄期 12.33ms、base/left_wrist
image p95 32.08/19.17ms、双相机 stamp p95 skew 14.98ms；RTT p50/p95 为 98.50/101.13ms，实际
`consumed=2--3` 行与该延迟匹配，首次 selected row 在下发时与 state 的偏差通常 ≤0.063rad。反而旧
chunk 尾部在边界已偏离当前 state 0.10--0.16rad，新 plan 拉回实测造成最大 0.163rad arm jump；raw 和
applied arm 指令相同，安全层未参与。这与 ACT 的收敛拉回共因一致，排除“持续旧图像/RTT 补偿失效”作为
本轮主因。下一项单变量测试为 `async_prefetch_ahead: 25 → 40`，保留 `coeff=0`、`anchor_tol=0`，先
降低旧 chunk 开环年龄而不固定 pi0.5 采样或用平滑掩盖问题。

**修复 pi0.5 policy 命令永远 IDLE、left_wrist 在 policy 端被饿死**——
`morla_policy_inference/node.py`。`TEST_922-1101` 中 streamer 对 base/left_wrist 均稳定发布
约 30fps（queue_full=0），policy 却收到 base 28.7fps、left_wrist 2.86fps（最大 gap 2.37s），
`pi_cmds.jsonl` 为 0 行；Web 显示已发送及终端直接 pub 都不触发状态切换。**根因**：所有高频
image callback、30Hz control timer 与可靠 cmd subscription 仍共用 Node 默认
`MutuallyExclusiveCallbackGroup`，故 4 线程 executor 实际串行化，base JPEG decode 长期占用等待集。
**修复**：每台相机独占一个 serial callback group，使两路解码并行；state/diagnostics 回调使用
reentrant group；cmd/task 独占 command group，control/status 共享独立 serial group 保持仲裁单写。
同时加 `cmd_rx seq`（DDS 收到）→ `cmd_exec seq`（tick 执行）日志与 control JSONL 的
`cmd_execute/cmd_queue_delay_ms`，并在启动行打印解析后的 `cmd_topic` 与 callback 布局。新增回归覆盖
callback 隔离和命令留痕。**验证**：policy node-flow 21 passed。

**C 轮恢复策略执行，并补齐模型原始动作/安全层动作的可审计日志**——
`TEST——0922-1137` 中 `policy` 命令 29ms 内切到 POLICY，base/left_wrist policy 接收恢复到
28.98/29.77fps，图像年龄 p95 均约 34ms；由此排除相机采集、DDS 接收及 image timeout 作为本轮
卡顿主因。控制稳态下实际下发 579 条指令，30Hz 间隔 p95 37.6ms；首 plan 仍有一次 307ms callback
停顿。发现旧 `pi_cmds.jsonl` 只记录安全处理后的值，无法区分 pi0.5 原始输出、时序融合和
clip/slew 的影响；现对每个发生安全修正的动作记录 `raw_targets`、`safety_events`，并同步写入
`pi_control.jsonl`。控制行为不变。**验证**：node-flow + executor 29 passed。

**Web 推理卡片接入 Launch 实时日志；pi0.5/224 成为全链路默认**——
`morla_web_manager`。推理泳道原本已采集 subprocess stdout/stderr 到独立的 `LaunchManager`，但
`infer_launch.log_tail` 未在卡片渲染，`/api/v1/logs` 也未返回推理的完整环形缓冲，故 Web 启动失败时
仍要回终端查日志。现将通用日志面板复用于推理卡片：实时显示最新 800 行、可复制显示内容；下载从
REST 获取推理泳道完整缓冲（默认最多 8000 行）。系统页下载同步包含遥操/数采/推理三段。并将前端
表单、Pydantic 请求默认值和后端 `policy_launch_args()` 默认值统一改为 `model=pi05`、
`camera_image_size=224`，避免旧客户端或缺字段请求回退为 ACT/480。**验证**：web pytest 与 Vite
生产构建通过。

## 2026-09-21

**streamer `_setup_collect_taps` 参数名错配修复（`diagnostics_hook` vs `diagnostic_hook`）**——
`quest3_video_streamer/streamer_node.py`。**动机**：C 轮测试启动 streamer 即崩
`TypeError: _setup_collect_taps() got an unexpected keyword argument 'diagnostics_hook'`——
pipeline diagnostics 链路并行改动时，main 调用传 `diagnostics_hook=`（带 s），而函数定义与
CollectTapPublisher 都是 `diagnostic_hook`（不带 s），参数名不匹配 → 启动即死。**做法**：
main 调用处改回 `diagnostic_hook=diagnostics_hook`（与定义/CollectTapPublisher 一致）。**验证**：
streamer 测试 39 passed；语法 OK。**部署**：机器人侧 git pull + colcon build quest3_video_streamer。

**Web“记录推理日志”自动收齐相机全链路诊断，无需额外终端**——
`quest3_video_streamer` × `morla_policy_inference`。streamer 新增 BEST_EFFORT
`~/diagnostics`：每 5 秒按相机发布 capture FPS/read failure，以及 collect tap 的 submit/rate skip/
queue full/encode/publish FPS 与耗时。policy 在 `log_dir` 模式自动创建第四份
`camera_diagnostics.jsonl`，逐帧记录 `policy_rx` 的 gap、payload bytes、JPEG decode ms/result，并
把上游 capture/tap 事件合并到同一 epoch 时间轴。落盘使用独立后台线程和有界非阻塞队列，不在
图像 callback 做磁盘 I/O。新增 policy 合并日志与 tap 结构化统计回归；C 轮现在只需 Web 勾选
日志，目录自动得到 metrics/cmd/control/camera 四份 JSONL。

**分析 pi0.5 B 轮 base-only：断流消失，但定位到稀疏时序融合的周期性内部接缝**——
`inference_test_logs/inference/20260921-171159_pi05_muteleft_testB`。34.62 秒 POLICY 的
1015 tick 全部正常 emit（29.45Hz，最大间隔 78ms），joint/base 全 fresh、左腕全 muted、无
state reject/hold/queue empty，故肉眼抽搐不是控制断流。动作却有 28 次 >0.1rad/步尖峰，全部
精确落在 chunk 安装后第 22～23 tick：`prefetch=25` 下融合覆盖 `new[0:25]`，随后切入未融合的
`new[25:]` 形成接缝；安装当拍自身最大仅 0.042rad，anchor 无法覆盖后续接缝。记录新增 D 轮：
保持其余参数不变，仅设 `temporal_ensemble_coeff=0.0` 验证。base-only 无法抓取也表明左腕近场
视觉不可永久移除，最终应恢复训练视角/裁剪或用新视角数据微调。回查 A 轮的 4 个 >0.1rad
尖峰也全部位于安装后第 22～24 tick，确认该接缝不是 mute 左腕才出现。

**补充 pi0.5 C 轮左腕超时分层诊断方案 + 图像话题 BEST_EFFORT 测速**——
`docs/2026-09-21-pi05-inference-investigation.md`、`scripts/hz_best_effort.py`。B 轮完成后恢复双相机，
同步对照 V4L2 capture、collect tap、DDS 到达间隔和 policy freshness 四层时间线，判定超时发生于
设备/USB、JPEG/发布、DDS 还是 policy callback；明确 C1 不改门限和图像参数，必要时再以关闭
WebRTC/预览作为单变量 C2。测速脚本新增 `--msg cimg` 支持 `sensor_msgs/CompressedImage`，避免
Humble `ros2 topic hz` 的 RELIABLE 默认值无法匹配 BEST_EFFORT 图像 publisher；新增
`--log-file/--report-s/--gap-ms`，按墙钟时间写 JSONL，并在长 gap 恢复时立即留证。文档明确完整
C 轮共 6 个文件，streamer 的 capture/tap stdout 需从 Web Launch 日志另存，不能误以为推理
`log_dir` 会自动收集上游进程输出。

**修复 Web 启动推理节点后“开始策略”偶发无反应的 DDS 发现竞态**——
`morla_web_manager`。**根因**：`/policy_inference/cmd` 有意采用 VOLATILE QoS，但 Web 在启动节点后
立即单次 publish，不等待订阅者发现，并无条件向前端返回成功；若用户点得早，命令会静默丢失。
**做法**：保持 VOLATILE（禁止旧 `policy` 在节点重启后被重放而自动运动），发送前最多等待 2 秒
直到 publisher 发现订阅者；超时返回 503 而非假成功；前端在 `/policy_inference/state` 离线时禁用
策略控制按钮。新增 2 例测试覆盖“发现后只发一次”和“无订阅者不发送/返回失败”。同时分析
`inference_test_logs/inference/tetsA`：POLICY 的 211 tick 中 58 次均因 `image:left_wrist` stale
被 gate 拒绝，joint/base 全程 fresh、engine 未耗尽；本轮卡顿及最终自动暂停已定责到左腕图像流
间歇与 `image_required=true` 的组合，而不是 GPU 推理速度。

**推理控制卡顿增加逐 tick 可归因日志 `pi_control.jsonl`**——`morla_policy_inference/node.py`
+ launch/config/tests/README。**动机**：现有 `pi_cmds.jsonl` 只记录新 target，无法区分 0.1～0.9s
指令空档究竟是 joint state 超过 `obs_timeout_s`、`_state_ok()` 静默拒绝、ROS timer/callback
阻塞，还是 `_resend_last()` 保持旧指令。**实现**：① 新增 `control_diagnostics_log_file`，每个
控制 tick 记录 `tick_interval_ms`、超期量 `tick_late_ms`、`callback_ms`、最终 `action/events`、
各 state/image 源的 age/status、门限、FSM 状态、相机帧数和 engine pops/plans/remaining；
② state gate 拒绝写 `state_rejected + missing`，feed obs 异常不再完全不可见；③ 所有
`_resend_last(reason)` 标明 `policy_paused`/`playback_paused`/`engine_no_row`/`playback_end_hold`；
④ `pi_cmds.jsonl` 增加 `send_kind`、`hold_reason`、`control_seq`，并把 resend 的实际发布也记录；
⑤ `log_dir` 自动生成第三个 `pi_control.jsonl`。新增回归覆盖 stale state 与 paused resend，且
保留原指令流/绘图格式兼容。

**`mute_cameras` 完成 pi0/pi05 端到端 `image_mask=False` 相机消融**——
`morla_policy_inference/node.py` + `VLA/openpi/src/openpi/policies/astral_policy.py`。**动机**：把腕部图
改成全黑但保留 key 会令 OpenPI 设置 `image_mask=True`，仍是训练分布外输入，不能等价验证
base-only。**做法**：① 节点对静音 label 跳过 freshness 门并从 `ObsBatch.images` 删除，
`RemoteBackend` 因而不发送对应 camera key；② OpenPI `AstralInputs` 按请求实际存在的 key 构建
`parsed`，对缺失固定槽补 `zeros_like(base)` 且 mask=False；若所有相机都缺失则明确报错；③ 当前
yaml 已准备为 A/B 第一轮 `mute_cameras: "[]"`，第二轮仅改为 `'["left_wrist"]'`。**验证**：客户端缺腕不阻断且 wire key 省略；服务端
3 例覆盖缺腕 mask=False、正常双相机 mask=True、全相机缺失拒绝。已同步到 GPU 主机
`lukang@192.168.1.249:~/loopkok/VLA/openpi_astral`，远端实测 mask 为 `{base: true,
left_wrist: false, right_wrist: false}`；旧文件备份为 `astral_policy.py.bak-20260921-1625`。
部署时 29999 服务未运行且 8001 端口空闲，下次按原脚本启动即加载新实现。

## 2026-09-20

**streamer `_device_to_index` 修复 + `_scan_spec` 不再用块 device 覆盖扫描值——修
auto_scan 起崩（`ValueError: invalid literal for int() with base 10`）**——
`quest3_video_streamer/streamer_node.py`。**动机**：base alias 改 by-path 后，重启 streamer
在 `_build_sources→_scan_spec→_device_to_index` 崩（错误值就是块里的 by-id/by-path 路径）。
**根因**：① `_scan_spec` 在 auto_scan 分支用 `_get_param(f"{label}.device", dev["device"])`
让 yaml 块 device（by-path/by-id）**覆盖**扫描到的 `/dev/videoN`，违背"device 被扫描值取代"
不变量；② `_device_to_index` 只认 `/dev/videoN` 或纯数字，遇到 v4l 符号路径直接 `int()` 崩。
**做法**：`_scan_spec` 改用 `dev["device"]`（auto_scan 一律用扫描到的 /dev/videoN，块 device
仅固定配置/兜底用）；`_device_to_index` 加 v4l symlink realpath 解析 + `-video-indexN` 后缀
退化，纯数字/`/dev/videoN` 行为不变。**验证**：新增 `DeviceToIndexTests` 6 例（devnode/纯
int/by-path 后缀/by-id 后缀/realpath 命中/非法抛错），test_label_aliases + test_quest_layout
22 passed。**部署**：机器人侧同步后 colcon build + 重启 streamer。

**base（realsense）alias 改 by-path 物理口位——修 ACT 推理"收不到 base 图、跑不了"**——
`quest3_video_streamer/config/params.yaml`。**动机**：web 启动推理节点后 `collect/base` 0 帧、
`gate_state configured` 只有 left_wrist/video6，节点只剩单路图 → 旧 ACT 少一路输入跑不了。
**根因**：base 的 label_alias 用 **by-id 序列号** `254843065994-video-index0`，但 D435i 是
复合 USB 设备（彩色+IR+depth 多接口），内核编 by-id 时 **index 串位**——实测 by-id 的
video-index0 落到 `0:2.1:1.0` 的深度节点（`v4l2-ctl` 实锤 Z16 depth，非彩色），而真正的
彩色（YUYV，`0:2.1:1.3`，/dev/video6）在 by-id 里**没有条目** → 序列号 alias 钉不上彩色，
realsense 以内核名 video6 裸奔。**做法**：base 的 alias + base 块 device 改用 by-path
`platform-3610000.usb-usb-0:2.1:1.3-video-index0`（彩色口位），与左右腕 USB 相机同策略；
left_wrist/right_wrist 块 device 同步改 by-path（原 /dev/video0/video2 硬编码），right_wrist
口位实机 scan 填 `0:1.3:1.0`（原占位 0:1.4 错）；docs/camera-label-semanticization.md §1/§7
同步。**验证**：test_quest_layout 3 passed；yaml 解析 cameras=[base,left_wrist,right_wrist]、
三块 device 均为 by-path。**部署**：机器人侧 git pull + colcon build quest3_video_streamer +
重启 streamer → gate_state 应含 base。

**驱动层诊断 JSONL 日志（`driver_log_file`，抓"电机抽一下"）**——`astral_robot_control`。
**动机**：遥操/没遥操时臂/夹爪/头偶发"突然动一下"（含头电机抽），需要驱动层数据定位
"谁先跳"（上游坏指令 vs driver 陈旧重发 vs 板卡/机械）。遥操 JSONL 只记录臂命令，
覆盖不到夹爪/头/非遥操时段。**做法**：driver 节点新参数 `driver_log_file`（空=关）+ 
`driver_spike_mrad`（默认 30），照遥操/推理模式写 JSONL，`kind` 区分——
`cmd`（每个到达的 joint_commands 臂/全/头/夹爪，排上游）、`send`（每控制拍实际下发+
fresh 标志，排陈旧重发）、`state`（实测关节）、`srv`（6 服务 + cache_clear +
seed_from_current，运动模式切换/重播种=抽动高危点）、`spike`（命令/实测单拍跳变超阈值
落一条，含跳前跳后值）。**launch 透传**：`astral_drivers`/`dual_arm`/`full_teleop` 加
`driver_log_file`；**log_dir 模式自动派生 `{run_dir}/driver.jsonl`**——web 勾选「记录遥操
日志」即自动带上驱动层日志（web 无需改）。新 `test_driver_log.py`（DriverJsonlLog +
spike_mrad 纯单测）。**验证**：节点 dry_run 冒烟——喂慢速小步+一次 0.3rad 突跳，
cmd×6/send×6/state×1/srv×1/spike×1，spike 正确捕获 298mrad（含 before/after）。**用法**：
`ros2 launch astral_robot_control astral_drivers.launch.py driver_log_file:=/tmp/driver.jsonl`
或 web 勾选记录遥操日志。定位方法：spike 时间对齐 cmd/send/srv。


**serve.py pi05 分支加进程内 XLA 预热 + 磁盘编译缓存**——`morla_policy_inference`。
**动机**：pi05 首轮推理要 XLA 编译（2-5 分钟），此前在 serve 进程外单独 warmup 无法加速
serve 本身（编译结果随进程退出丢失）；换 checkpoint/重启 serve 每次都重编。**做法**：①
`_warmup_policy`（加载模型后 dummy infer 做掉编译，`--warmup` 默认开 / `--no-warmup` 关）；
② `jax_compilation_cache_dir` 指到 `~/.cache/jax`（与训练侧同目录，跨进程复用已编译图）。
**验证**：语法 + fake-policy 单测（state_dim=8、2 相机槽、推理 1 次）；训练主机真实
checkpoint `create_trained_policy` + infer 通过（warmup ok）。部署要求不变：pi05 serve 需
≥32GB RAM 机器（采集机 15GB 装不下），`POLICY_CONFIG` 必须=训练配置名（LoRA 用
`pi05_astral_lora`），`camera_image_size`=224。

**serve.py 加 `--capture-dir` 诊断捕获**——把每次 pi05 推理的输入图像（base/left_wrist，
每 30 次存 ~1Hz）与 state/prompt（meta.json）+ 输出首行动作（actions.jsonl）落盘。**动机**：
pi05 真机推理"末端抓向侧边"（肩关节 j1 z=-3.0 / j2 z=+2.3 显著偏出示范抓取分布），模型
离线复现示范 MAE 0.0028 正常、serve 配置正确——需要实时输入帧对比训练帧定位视图不匹配。
**验证**：fake-policy 单测（每 30 次存图、actions 全量、格式正确）。已 scp 到训练主机。

## 2026-09-18

**`repair_aligned.py` 相机名归一化——修复 camera 迁移遗留的 meta 不一致**——
**背景**：当日相机 label 迁移（videoN→base/left_wrist）迁移了 raw camera_data.h5/aligned/
v2.1/v3 的文件键名，但 **meta.json 冻结的 schema.cameras 仍是 video8/video0**（迁移脚本没改
meta）→ 旧数据 raw 处于"文件=base/left_wrist、meta=video8/video0"的不一致态。跑
`vla_process_openpi.sh` 时 validate F2 全段 fail（camera_data.h5 按 schema 名找不到
video8/video0）→ 全部 quarantine，convert 报 "no aligned episodes"。
**做法**：`repair_episode` 输出时把 meta.json 的 `schema.cameras` 归一化为源 aligned 实际相机
组名（顺序=参考相机在前）——修复段自洽，validate F2 与下游 convert 都以实际组名为准。
**验证**：pick_place_merged_repaired_v3 重跑 repair，100 段 meta 归一化
（['video8','video0']→['base','left_wrist']）；validate 0 fail / 100 warn（W5 既有 WARN）；
openpi v2.1 转换正常（key=base/left_wrist，与当前 openpi camera_map 一致）。
**遗留**：`migrate_camera_labels.py` 未同步 meta.json schema.cameras——存量 raw 源段仍有该
不一致（修复输出已自洽；后续建议 migrate 脚本补 meta 迁移，或重跑 repair 即得一致段）。

**相机改名/加右腕后的脚本适配审计 + `verify_aligned.py` 修复 + openpi v2.1 修复版产物**——
**动机**：相机 label 语义化（videoN→base/left_wrist/right_wrist，且新增右腕）后，要确认
数据处理的"转换/修复/量化"脚本是否有硬编码相机名假设。**审计结论**：① convert_to_lerobot/
_lerobot_v3/_act 全走 `schema.cameras`（meta）+ 动态组名，**无硬编码**；② repair_aligned 动态
`_camera_groups` + meta 归一化，任意相机名/数量（2/3 路）都对；③ quantify_cmd_state 只读
state/action/streams，**不碰相机**；④ **`verify_aligned.py` 硬编码 `f["left_wrist/images"]`**
（只验左腕一路，改名/3 相机漏检或崩）→ **改为遍历全部相机组**动态探测，零错位对抗复跑
ALL PASS（2477 帧）；⑤ 部署侧 run_act_e2e/correctness/benchmark、serve SLOT_MAP 仍 2 相机
假设——属推理部署范畴，等 3 相机模型部署再适配（`pi05_astral_3cam` 配置已留）。
**产物**：openpi v2.1 修复版 `pi/pick_place_merged_repaired_v3`（100 段 / 20883 帧 / 224，
key=`observation.images.base/left_wrist`，与当前 openpi camera_map 匹配）——从
`raw/pick_place_merged_repaired_v3`（natural 修复）转换，与 ACT v3 数据集同源一致。
**后续**：openpi 训练前重算 norm stats（`compute_norm_stats.py --config-name pi05_astral_lora`）
+ 迁移到 RAM≥32GB 目标机软链。


**相机 label 语义化重构：videoN → base/left_wrist/right_wrist，全流水线同步 + 旧数据迁移 + 右腕启用**——
`quest3_video_streamer` × `morla_data_collect` × openpi × `morla_policy_inference`。**动机**：
换口后 videoN 编号漂移（video8→video6）暴露 videoN 是"内核号"不是"角色"；label_aliases 已把设备
钉成稳定语义名（by-id/by-path），干脆把 label 本身改成位置名。**做法**（三处决策）：
① label 改名：streamer params `label_aliases` 目标 + 覆盖块 → `base`（realsense 彩色，by-id
序列号+index0）/`left_wrist`（by-path 端口）/`right_wrist`；data_collect/policy_inference/
openpi 的 cameras、camera_map、图像列名同步（模型槽 `base_0_rgb`/`left_wrist_0_rgb` 是 pi0.5
固定输入，**不动**，只改它们指向的 label）；② 右腕全面启用：data_collect cameras 加
right_wrist，openpi 新增 `pi05_astral_3cam`/`pi05_astral_lora_3cam`（camera_map 3 槽）供新数据，
旧 2 槽配置（pi05_astral/pi05_astral_lora）保留供旧数据（右腕槽零填充 mask=False）；推理侧
policy_inference 当前 2 槽（匹配已部署的 2 相机模型），3 相机模型部署后再加 right_wrist；
③ 旧数据迁移：`scripts/migrate_camera_labels.py`（video8/video0/video2 → base/left_wrist/
right_wrist，处理 raw camera_data.h5 + aligned_data.h5 + v2.1/v3 的 info.json features + videos
目录，parquet 无图像列不用动），**实测 408 处改名**（raw 100 episode × 2 h5 + pi + act）。
**验证**：openpi 4 配置加载 camera_map 正确（2 槽/3 槽）；全套件绿——policy_inference 108、
streamer 16、convert_to_act 8、node_guards 15、**adversarial 8**（schema↔openpi 金标准）；
迁移后抽查 raw/pi/act 键名全对。**对抗审查（同日）发现并修复 2 处**：① `serve_policy.env`
的 `SLOT_MAP` 值被误改成 collect label（base/left_wrist）——serve 把图喂到
`observation.images.<值>` 键，值必须是**模型 `input_features` 图像键**（旧 ACT 模型=
video8/video0），已改回 `{base_0_rgb: video8, left_wrist_0_rgb: video0}` + serve.py
`--slot-map` 默认值同步 + 迁移文档 §5 记录该区分；② 数据**变体目录未迁移**（
pick_place_merged_repaired/_v2/_v3/_smooth、act dropped/smooth/repaired/_v2/_v3 仍 videoN
键）→ 补迁移（raw 变体各 400 处 + act 变体各 4 处），verify_aligned（源 vs repaired 零错位
对抗）复跑 **ALL PASS**（2477 帧）。全套验证：openpi `create()` 直读迁移后真实数据解析
camera_map OK、对抗配置 8 例、推理链路（节点 collect label vs serve 模型特征键 vs 模型
input_features）三方匹配。**后续补全**：右腕 alias 口位已按实机 scan 填（by-path
`0:1.4:1.0-video-index0`）；两条 USB alias 补 `-video-index0`（多节点相机防误改兄弟节点，
实测 index1 兄弟不被误改）；streamer `cameras` 兜底列表改 `[base, left_wrist, right_wrist]`
（原来只有两个腕部、缺 base），块名统一语义名 + 合并重复键（原命名块与 auto_scan 覆盖块
同名重复、device 被 PyYAML 吞掉 → 合并每相机一个块）。**⚠ 注意**：右腕相机实插前建议
`data_collect.cameras` 先只留 [base, left_wrist]（3 路录到空右腕会被
validate 隔离）。

**相机换 USB 口后 realsense 内核编号漂移 → 启用 `label_aliases` 钉回 video8**——
`quest3_video_streamer`。**症状**：realsense（base 相机）换口后 `/dev/videoN` 重排，
彩色节点从 video8 变 video6（`ls /dev/video*` + `python3 -m quest3_video_streamer.scan`
实测：video6 YUYV=realsense 彩色、video0 MJPG=左腕；D435i 4 个 by-id 节点 index0=彩色/
index1-3=IR/深度）。若不处理：采集 `collect/video8` 无源 → base 图像缺失；openpi 训练
`camera_map` 指向不存在的 `observation.images.video8` 列 → `unknown_cams` 校验直接抛错；
推理 `collect/video8` 无帧 → serve 端 base 槽位零填充（退化）。**修法**：启用 params.yaml `label_aliases`，**混合策略**——realsense 走 by-id、USB 相机走
by-path：`"254843065994-video-index0=video8"`（realsense 序列号唯一可靠、跟随设备，换口/内核
重排都还是它；末尾 `-video-index0` 精确钉彩色，不误中 D435i 的 IR/深度 index1-3）+ 
`"platform-3610000.usb-usb-0:2.2:1.0=video0"`（左腕 USB 相机：廉价相机 by-id 序列号是假的
[Generic_USB_Camera_200901010001 所有同款一样、再插同款只显示后插那台]→ 序列号不可靠只能按
物理口位钉）。**验证**：YAML 解析 OK；模拟 `apply_label_aliases`——realsense video6→video8 ✓、
IR/深度不误改 ✓、左腕 video0 保持 ✓、**新插同款 USB 相机（同假序列号、别的口）不被改名** ✓。
生效后 data_collect.yaml / policy_inference.yaml / openpi config.py 的 video8 引用**全部零改动**。
**部署**：重启 streamer；日志 `label_aliases: alias ... matched ...` 是命中提示非错误；web 视频
卡片应显示 video8=realsense。⚠ 非 symlink 构建需重新 `colcon build`。**换 realsense 机体**：
改第一条规则里的序列号（按新 scan 填）。

## 2026-09-17

**`repair_aligned.py` v3 重写（保时序摩擦移除，默认）+ `--mode uniformize`（修好版 v2）**——
**动机**：v2 弧长均匀化两次回放反馈"整体速度还是被加快、夹爪猛合猛放、有意停顿很快走"。
根因：均匀化把整段重定时到单一速度（均值/p90），实测慢速段 p10 提速 4 倍（0.003→0.012
rad/帧）；夹爪 ratio 过渡（真实数据实测 11 帧/0.37s ramp）被压没成 1-2 帧。v2 思路本身对
"训练更快执行"有用，但对"复现操作者"是错的。
**做法**：① **natural 模式（默认）= 保时序摩擦移除**：只删摩擦型停顿帧（全臂+夹爪都停、
非意图、连续≥min_hold=2），其余帧 1:1 保留、**速度=自然速度**；夹爪过渡 ±5 帧保护（抓取
上下文不丢）；意图停顿（keep-intent）时长 1:1。② **uniformize 模式 = 修好版 v2**：臂按
`--speed-ref` 弧长匀速化（0=自动取臂维均值），但**夹爪/意图停顿保护 1:1**（不再猛合），
摩擦段照删——供"工程化提速"的数据烘焙。③ `--speed-ref` 从 natural 移除、只属 uniformize。
**验证**：自测 12/12（natural 7 + uniformize 5：摩擦删/夹爪留/意图留/保时序/零错位）。
真机 pick_place_merged 全 100 段：natural **21321→20883（-2.1%）**、臂速度 p10/p50/p90 与
原始逐位一致（0.003/0.014/0.0325）、夹爪变化帧 17→17、平段 5.3%→0.5%、零错位全过；
uniformize（自动均值）**-13.0%**（更快用途）。v3 输出 `pick_place_merged_repaired_v3` →
ACT 480 转换中。**遗留**：夹爪 ratio 是指令回显（无真实反馈），物理闭合滞后数据层无法改，
natural 保自然时序 + 保护窗避免脱节；真机回放待用户验证。


**电机指令最小步长地板（`cmd_deadband_mrad`）+ 遥操 JSONL 跟手/慢速跟踪指标**——
`astral_arm_teleop`。**现象**：geometric vs urdf_numerical 遥操对比（两份 JSONL），
慢速移动 geometric 肉眼"一顿一顿"。**根因**：电机最小可靠步长 ~1 mrad（机械/固件死区，
**更新 2026-09-17 二次真机 A/B：`cmd_deadband_mrad` 实测无效**——floor1_geo 指令层步长已
全抬到 ≥1mrad（整臂全低 1% vs floor0 22%、db_nudge 58% 拍），但慢速窗实测关节平段占比
三方法都 ~35-40%（j3：floor1=25% / floor0=14% / urdf=18%），floor 未改善反使 j3 变差
（地板硬抬→过冲振荡）；且 1mrad 测量分辨不出 geometric/urdf 差异。**结论：电机硬件间隙
问题暂无法解决**（调增益/摩擦/地板均无效），`cmd_deadband_mrad` 保留参数但标注实测无效、
默认关。详见 `doc/2026-09-17-ik-solver-comparison.md` §7。
实机验证调 speed 增益无效）；geometric 精确解+最小关节速度优化，慢速时把指令压到死区下
（匹配速度段实测：**24% 拍整臂 7 关节全 <1 mrad、单关节 51~69% <1 mrad**，中位步长
0.55~0.97 mrad）→ 电机不执行 → 攒误差跳一下；urdf 的"浪费型"关节运动（123 vs 10 mrad/拍，
12 倍）反而每拍超死区 → 平滑。**做法**：① 新增 `cmd_deadband_mrad`（mrad/拍，0=关）+
`cmd_deadband_min_vel`（mm/s 门控，手停不蠕）：VR 目标速度达门限的运动中把
`0<|dq|<floor` 的关节步抬到 floor（方向保持），让电机每拍有步长去跟；热改支持；loop 记录
加 `db_nudge` 标志、Latency 计数。② 遥操 JSONL 加 solver 对比指标：`pos_err`/`ori_err`
（命令 FK vs 滤波目标）、`psi_err`（臂角跟随误差）、`vr_vel`（慢/快打标）、metrics 加
`track`（slow_frac + 慢/快段 pos/ori_err p95）。**验证**：节点 dry_run——floor=1.0 时非零
关节步 413 个仅 1 个 <1mrad（0.2%）、最小步长正好 1.000、db_nudge 59/60 拍；floor=0（默认）
行为不变（98% 子死区步、0 次 nudge）；test_teleop_log PASS。**副作用**（文档明示）：慢速带
轻微恒定微动、TCP 可能略超目标速度；真机 A/B 从 1.0 起试。**遗留**：电机死区本身在机械/
固件层，需联系厂商（调 speed 环无效已锁定）；数值求解器仍无臂角约束（肘乱跑）。


**接管臂角改方案B（保持到操作者手臂真的动） + 右手 A 释放移到常驻 gate**——
`astral_arm_teleop` × `morla_bringup`。**现象**（真机复测 09-17-11:06，5 次接管）：方案A
（reanchor 重锚臂角到当前配置 + EMA 过渡）虽消除了首拍跳变（cmd==state，FK 验证一致），
但**接管后肘仍自行摆动 0.07-0.42 rad/1s**（body_joints 62Hz 新鲜、腕部跟手零延迟
|vr−cmd|≤3mm）——EMA 过渡被感知为"卡一下/臂自己在动"；且操作者手臂角度 ≠ 策略留下
的臂角，遥操"怪"。**修法**：①方案B（`reanchor_elbow_hold=true` 默认开）：reanchor 后
肘**保持接管时刻臂角**，直到操作者手臂方向相对 reanchor 时刻变化超
`reanchor_elbow_release_thresh`(0.2 rad) 才切回人肘 EMA 跟随——肘不自行摆动；
②右手 A 的 HUMAN 释放原本在 `vr_collect_control`（仅数采栈运行），推理会话没启数采栈
→ A 无响应。把 release 路由移到**常驻遥操栈**的 `controller_start_gate`（新纯函数
`decide_release_action`，订 `quest3/right_controller_joy`）。**验证**：`test_reanchor_teleop.py`
新增 `test_elbow_hold_keeps_config_until_operator_moves`（阈值内保持/超阈值释放）；
`test_start_gate_logic.py` 新增 4 例 release 路由；全量套件绿。**待真机**：确认方案B
（臂角保持 + 手臂动则跟随）是否消除"卡/怪"。

**HITL 接管肘部重构修复（方案A：reanchor 重锚臂角参考）**——`astral_arm_teleop`。
**现象**：策略运行中点击「接管」进入 HUMAN，机械臂"原地等待一会"后**肘部突然重构**
（手腕 TCP 不动、整段前臂/肘换姿势），然后才响应手柄；用户预期"接管后原地不动、
增量遥操"。**根因**（真机日志实证，`inference_test_logs/20260916`）：`_reanchor_teleop`
只重锚了 robot_init（位置），没重锚**臂角（psi）**；`human_elbow_mode=hard` 的
`solve_hard` 在精确人臂角上解算，reanchor 时操作者手肘举着 → 第一拍就把臂角摆到
当前手臂角度。三次接管实测：指令空档仅 11-56ms（臂没"等"），但 0.7s 内肘部关节
重构 **0.26-0.32 rad**，TCP 钉在当前位（FK 逐拍验证一致）。**修法**：`geometric.py`
新增 `elbow_direction(q)`（肩→肘单位向量，`R03 @ v_se_hat`）；`_anchor_origin_to_measured`
（`_reanchor_teleop` 与 `_start_teleop` 重锚分支共用）末尾把 `_elbow_dir_vr` 重灌为
当前上臂方向（`R_vr_to_arm.T @ u_se`）——首个 `solve_hard` 解在当前臂角（零跳），
随后 `_on_body_joints` EMA（τ=0.15s）平滑过渡到实时手臂。**验证**：新用例
`test_elbow_direction_reanchor_keeps_config`（`elbow_direction` 与候选解 psi 一致，
`solve_hard`@重锚方向 worst config move 0.0005 rad）、`test_reanchor_reseeds_elbow_to_current_config`
（`_elbow_dir_vr` 被重锚到当前配置方向）全绿；顺带修了 `test_reanchor_teleop.py`
缺 `_tlog` stub（节点 09-16 加 `_tlog_event` 后测试骨架没跟上）。**用户真机 A/B**：
方案A 效果不满意则切方案B（reanchor 后保持当前臂角，直到手臂方向变化超阈值）。

**~1.1s 接管状态显示延迟：加时间戳日志 + 提前发布 HUMAN**——`morla_policy_inference`。
**现象**：reanchor 武装遥操（如 28.58s）到 web 显示 HUMAN（29.71s）稳定差 ~1.1-1.3s
（三次一致）；期间状态话题仍 POLICY，操作者以为没接管。**排查**：29.198s 的 POLICY
发布证明 `_tick` 未在 teardown 中阻塞 → 更可能是 reanchor 响应/提交延迟。**本轮修法**：
①接管流程三处加**精确时间戳日志**（`_cmd_takeover` send → `_poll_takeover` all-reanchor-done
→ `_commit_takeover` committed，`time.strftime('%H:%M:%S.%f')`）；②防御性：`_commit_takeover`
在 FSM 切 HUMAN 后、`_teardown_engine()` 前**立即 publish state**——即使拆引擎阻塞（join
planner+关后端可达 ~2s），web 也立即显示 HUMAN。**验证**：推理包全套 108 例全绿。**待真机**：
下一次复现抓 policy_node 终端三行时间戳，钉死"响应慢 vs 提交慢 vs teardown"后针对性修。

**VR 键位语义：推理活跃时 grip=HUMAN 接管、右手 A=HUMAN 释放**——`morla_bringup` ×
`morla_data_collect`。**动机**：策略运行中误按 grip 会 `/teleop/start` 重新武装遥操，
与策略**双写** `/left_arm/joint_commands`（150Hz 遥操盖掉 30Hz 策略）；改为推理活跃
（`/policy_inference/state` activity∈{policy,playback}）时 grip 发
`/policy_inference/cmd`="takeover"（不发 /teleop/start+/armed）；右手 A 在 HUMAN 时发
"release"（交还控制权），其余仍发采集 start。**修法**：新纯逻辑 `start_gate_logic.py`
`decide_start_action`（policy/playback→takeover，否则 teleop_start）；`controller_start_gate.py`
订 `/policy_inference/state`（latched）按决策路由 grip；`vr_collect_logic.py` 新增
`decide_release_or_collect`（HUMAN→release，否则采集路由）；`vr_collect_control.py` 订
`/policy_inference/state` + 新增 `/policy_inference/cmd` 发布器。**验证**：新增
`test_start_gate_logic.py` 4 例 + `test_vr_collect_control.py` 扩 5 例，全绿；B/摇杆语义不变。
**设计假设**：grip→接管只在推理**活跃**时生效，IDLE/HUMAN 下 grip 仍走 /teleop/start
（起策略前可手动摆位）；HUMAN 下 grip=重标定（现状）。

## 2026-09-16

**遥操 jsonl 新增 `kind=event` 生命周期/接管记录**——`astral_arm_teleop`。**现象**：HITL
takeover 的 armed/disarm/reanchor/start/fault 只打终端日志，jsonl 里只有 loop 断档能间接推断，
单看遥操文件无法还原接管时序（用户反馈"takeover 没有记录"）。**做法**：`teleop_log.py` 新增
纯函数 `teleop_event_record(event, side, **fields)`（kind=event、t=墙钟、side、自由字段）；
节点加 `_tlog_event` 辅助并埋在五个事件点：`_on_armed`（armed）、`_on_disarm`（disarm +
`disarm_reason` + `homing_cancelled`）、`_start_teleop`（start + `reanchored`）、
`_reanchor_teleop`（reanchor 成败 + reason）、VR 看门狗（fault + `timeout_s`）。零开销路径
（日志关）不碰。**验证**：`test_teleop_log_run_dir.py` 4→**7 例**全绿（event 结构/自由字段/
t 默认现在）；README kind 表加 event 行、五类→六类；py_compile 通过。

**`quantify_cmd_state.py` 修复 + `repair_aligned.py` 意图感知重采样（--keep-intent）**——
**动机**：① `--aligned` 的"零步占比"语义错——量的是 `|action 绝对位|<阈值`（关节在零位，
对 gripper=开位），不是"没动"（实测 gripper 62.9% 假零步 vs 真 delta=0 11.3%）；②
`_cross_lag` 无守卫——静止关节近恒定序列任何 shift 都"相关"，返回噪声 lag 错移 cmd 平段
标志→分型错分；③ 用户要的"意图滤波"落地——repair 默认把所有停顿一视同仁压缩，会毁掉
任务合法停顿（抓握保持/放置等待）。
**做法**：① `analyze_aligned` 改收 state，`action_zero_prop` 用 delta=action-state 判定
（无 state 退化用 action 平段）；② `_cross_lag` 加方差守卫（任一方 std<1e-4 → 0）+
相关度守卫（bestc<0.5 → 0），classify 阈值提成 `--intent-thresh/--friction-thresh` 参数；
③ `repair_aligned.py` 新增 `--keep-intent`：读 raw `*_cmd` 流（优先 schema 臂块同名，防双臂
拿错侧），互相关对齐后把 state 停顿分型——意图型（cmd 同步停）帧在重采样度量里每帧推进
speed_ref（时长 1:1 保留），摩擦型照常压缩；去重豁免意图停顿帧（连续重复=时长正确表示）。
**验证**：两脚本自测全过（quantify 9 例含静止关节 lag=0 新例；repair 7 例含意图保留/摩擦
压缩/对照/零错位）。真机 pick_place_merged 全 100 段：普通 repair -27.3%（平段 0.6%）、
`--keep-intent` -24.6%（平段 5.3%=保留的意图停顿），两者零错位对抗 ALL PASS（3529 帧）。
**对抗性审查**：滞后 ±0.2s 意图仍识别、全静止双路径安全（keep-intent 保留 ~n 帧/无 intent
退化复制）、边界停顿不崩、NaN 不崩、混合停顿零错位——ALL PASS。**遗留**：意图停顿在输出
有 ~0.2s 边界过保留（最近帧选帧的边缘效应，无害）；分型质量依赖对齐可靠性（已有守卫）。


**日志落盘改"每次运行独立目录"：`log_dir`/`log_tag` 取代 /tmp 固定文件**——`astral_arm_teleop` ×
`morla_policy_inference` × `morla_web_manager`。**现象**：遥操/推理诊断日志默认写 `/tmp/pi_*.jsonl`、
`teleop_teleop.jsonl`——重启即丢、多跑互相追加到同一文件、无法区分本次记录。**做法**：两个 launch
新增 `log_dir`（根目录）+ `log_tag`（事件名）参数，每次 `ros2 launch` 自动建
`{log_dir}/{YYYYMMDD-HHMMSS}[_tag]/` 运行目录并落文件——遥操按侧拆 `teleop_teleop_{left,right}.jsonl`，
推理落 `pi_metrics.jsonl`+`pi_cmds.jsonl`；显式 `teleop_log_file`/`metrics_log_file`/`joint_stream_log_file`
仍优先（向后兼容）。新增共享小助手 `run_log_dir`（arm 侧进 `teleop_log.py`、policy 侧进新 `runlog.py`，
两侧各带 4 例单测：空 root 关闭/目录创建/stamp 格式/tag 消毒）。web「记录遥操日志」与推理日志开关从注入
固定 /tmp 路径改为注入根目录 `<ws>/inference_test_logs/{teleop,inference}`（`ASTRAL_WEB_MONITOR_LOG_ROOT`
可覆盖）。**验证**：arm `test_teleop_log_run_dir` 4/4、policy `test_runlog` 4/4、web 全套 15/15、policy
全量测试脚本 exit=0；全仓无旧常量残留、改动文件全部 py_compile 通过。用法：`log_dir:=<ws>/inference_test_logs/teleop log_tag:=pick_place_test7` → 目录 `.../teleop/20260916-153012_pick_place_test7/`。


**扳机夹爪映射改线性：`trigger_gamma` 1.4→1.0**——`astral_gripper_teleop`。**现象**：手柄扳机前半段闭合量很小、后半段才明显闭合，手感和"半程=半闭合"直觉不符。**根因**：扳机整形链的 `trigger_gamma=1.4`（幂曲线）把前半行程压细——按到 50% 闭合比只有 0.5^1.4≈0.38，前一半行程只贡献约 1/3 行程。**做法**：yaml `trigger_gamma: 1.0`（代码 `!=1.0` 时跳过幂运算，为严格线性；死区重标定后 1:1）。顺手修正 yaml 头注释过期值（gripper_open_rad 1.5→2.5）。`max_ratio_rate: 2.5`（输出限速）不受影响，快速扣扳机仍会按斜率逼近。节点无热改回调，改后需重启节点。


**删除空壳功能包 `astral_urdf_ik` / `astral_analytic_ik`**——两个包仅含 `COLCON_IGNORE`
（colcon 忽略占位），无代码、全仓零引用（IK 已由 `astral_arm_teleop` 的 geometric/analytic
实现，见该包）。直接删除，源码树 21 包 → 19 包。


**废弃 `compress_pauses.py` + repair 的 drop 模式（收敛到单一弧长选帧法）**——
**动机**：09-16 实测对比——compress_pauses 删停顿段但每段保留 4 帧过渡帧，压缩后平段占比
反而从 11.3%→20% 放大；repair 的 drop 模式与之同思路同病；修复后的 resample（臂维弧长均值
+去重）已覆盖全部需求（平段 0.6%、不加速、跳变不放大、零错位）。**做法**：删除
`scripts/compress_pauses.py`；`repair_aligned.py` 移除 `--method drop`/`drop_frames`/
`--min-speed`/`--min-keep`，脚本收敛为"弧长匀速化选帧"单一功能（`--speed-ref`/`--fps`/
`--dry-run`）；`verify_aligned.py` 只验 resample 输出。文档同步：data_collect README/CLAUDE.md
第 5 层、policy_inference README/CLAUDE.md 的压缩脚本引用全部改为 `repair_aligned.py`（带新
数值 -27%/平段 0.6%）。**验证**：语法 OK、dry-run 21321→15503（-27.3%）与修复后一致、残留
引用 grep 清零（仅无关测试名）。**遗留**：历史 CHANGELOG 条目保留作演进记录；旧数据产物
`*_smooth`/`*_dropped` 目录仍可转换训练（不再推荐新数据走此路径）。


**动机**：用户用 repair 重训后"丝滑了不少但速度变快非常多、大抖动被放大"——旧默认
speed-ref=p90（≈1.9x 均值）把整段数据提速并放大每帧跳变（实测每帧最大跳变 p50
0.019→0.043、>0.05 rad 占比 16%→40%）；且弧长把夹爪 ratio（无量纲，占 15%）和关节
rad 混算，污染速度参考。
**做法**：① 读 meta.json schema 的 state_blocks 生成臂维掩码，弧长/速度判定只用臂关节
（ee/waist/head 排除）；② 默认 speed-ref 由 p90 改为 session 臂维弧长**均值**（总弧长/总
帧数：重采样后帧数≈原始、总时长不变，只匀掉停滞不加速）；③ 去重连续重复帧——弧长网格间距
小于快速帧弧长跨度时多个网格点就近映射到同一原始帧，留下 ~30% 假"停顿"帧（新伪影），去重
只删无信息副本、零错位不变。
**验证**（pick_place_merged 全 100 段，真实重跑）：帧数 21321→15503（**-27%**，原 p90 为
-51%）；平段(delta=0) 11.3%→**0.6%**（均值无去重时反而 33.9%，去重后归零）；每帧最大跳变
p50 0.019→0.025、>0.05 rad 占比 16%→**17%**（不再放大）；零错位对抗全过（state/图像同源、
时间戳单调、action=next_state，3363 帧 ALL PASS）。speed-ref 可调：0.035~0.07 区间压缩稳定
21-36%、平段 0%，>0.07 放大效应回归。**遗留**：驱动层静摩擦粘滑（speed 环 kp=0.04）是
物理根因，见本日 SDK 排障条目；`action_source: command` 意图语义是采集侧根治方案（未实测）。


**采集数据"意图 vs 摩擦"量化工具 `quantify_cmd_state.py`**——`MORLA/scripts/`。
**动机**：遥操粘滞/静摩擦 → state"粘-滑"（平段+突跳）→ next-state 语义 action[t]=state[t+1]
把摩擦伪影直接做成回归目标 → 模型学到"停-跳"节奏（真机固定卡点的数据侧源头）。要决策
"标签平滑 vs 切 command-source"，需要先把每个停顿量化成**意图型**（cmd 也停=操作者/任务
真实停顿，训练该保留）还是**摩擦型**（cmd 平滑移动、state 平段突跳=摩擦伪影，训练该去掉）。
**做法**：新脚本读 raw `robot_data.h5` 的 `/streams/*_cmd`+`/*_state`（各自时间戳，最近邻
重采样到共同网格）→ 逐关节输出 state/cmd 平段占比（速度口径 flat_v=0.02 rad/s）、粘滑频率、
突跳幅度分布（p50/p90/max）、每个停顿（≥min_pause）分型意图/摩擦/混合；`--aligned` 模式读
aligned_data.h5 的 /action 输出 next-state 零膨胀统计；`--out` JSON 全明细、`--plot` PNG
（state vs cmd 曲线 + 平段着色：绿=意图、橙=摩擦）。聚合只算峰值 cmd 速度 ≥ min_peak_v
（默认 0.1）的**活跃关节**（全程低速的小关节恒判平段会污染聚合/分型），逐关节仍全列。
**自测**：`--self-test` 合成数据——cmd 平滑斜坡+state 粘滑→摩擦型（13 个）、cmd+state
同停→意图型、aligned 零占比 80% 全 PASS。**过程中抓到并修 3 个方法 bug**：① 平段阈值用每格
步长会把 0.05 rad/s 慢速移动误判成平段→改速度口径；② 最近邻重采样在比源采样率细的网格上
重复样本→慢速流重复点速度 0→误判平段，改**折叠重复源样本、速度在真实采样间隔上算**
（`_distinct`）；③ 慢关节污染聚合→`--min-peak-v` 活跃关节聚合。**验证**：raw/aligned/plot
三路径冒烟通过（合成 60s episode：主关节 j0 state平段 26.6% vs cmd 20.1% = 6.5pp 摩擦份额、
7 个意图停顿、粘滑 1.3Hz，符合注入场景）；py_compile OK。脚本入 git（
`scripts/quantify_cmd_state.py`），归档 README 工具表已同步。**实跑（pick_place_merged
100 episode，`--align` 对齐后）**：state 平段中位 34.9% vs cmd 平段 9.3% → **摩擦份额
~25.6pp**、粘滑 11.7Hz、1316 个停顿分型 = 意图 **13%** / **摩擦 38%** / 混合 49% →
训练数据 next-state 标签的假停顿以摩擦为主（非操作者犹豫），标签平滑/切 command-source
可捞回 ~25pp 丢失动作。**`--align`（分型前互相关对齐，默认开）**：实测 **cmd 领先 state
~120ms**（物理跟踪滞后，与推理侧 210ms 同向；初版把符号标反误报"state 领先 200ms"，
独立验证 corr(S(t),C(t+lag)) 峰 -121ms 定死方向）——不对齐时 state 停顿窗口看的 cmd
错位一个跟踪滞后，短停顿被错分（对齐后意图 7%→13%、摩擦 55%→38%，混合区增大=过渡停顿
更真实）；自测场景4 验证对齐纠正错分。
归档见 `inference_test_logs/teleop/SUMMARY.md`。

**慢速平移"抖"根因实锤 + 排障全记录文档化**——`astral_arm_teleop` × `astral_robot_sdk`。
**结论**：慢速遥操"停一下走一下"的抖，根因在**驱动层**——电机低速静摩擦粘滑（speed 环
kp=0.04 微弱，慢速产不出扭矩/阻尼），与遥操/IK/指令精度无关；速度前馈(0x95 PV)无效。
**过程（可复现链路）**：① 遥操 JSONL 诊断日志（teleop_log_file，09-14 已建）真实数据——
慢速段实测关节 30~56% 停帧、12~19Hz 粘滑、突发>1 rad/s，上游计数器干净→指向驱动层；
② 顺带发现并修 Quest App `F4`(0.1mm) 量化→F7（精度瑕疵非主因）；③ SDK 直连复现
`repro_stickslip.py`（干净三角波仍 70% 停帧/21Hz/滞后3.4°→驱动层实锤，pos vs pv 对照
70.7% vs 69.7%→速度前馈无效）；④ `read_gains.py` 读电机四环 PID 级联 position(35)→
speed(0.04)→iq(2) 定位 speed 环微弱；⑤ `set_pid.py` 调 speed 环 kp / `calib_friction.py`
摩擦补偿 / `repro --kp/--kd` MIT 增益。**⚠ 验证未完成**：17:31 重跑 pos 74.8% 平段无改善，
待 set_pid 改完 speed 环后重跑 repro 对照（平段显著下降才算修好）。**文档**：本包
README「遥操不平滑排障全记录」+ CLAUDE.md「低速粘滑排障全记录」；SDK demos 4 个脚本
（repro_stickslip/read_gains/set_pid/calib_friction）已在 astral_robot_sdk 仓库提交推送。

## 2026-09-15

**推理优化历程文档化——README「推理质量优化」+ CLAUDE.md「优化历程实录」**——
`morla_policy_inference`。把 09-14 真机推理优化全过程的认知沉淀成文档：**README** 新增
「推理质量优化（真机卡顿→流畅）」章节——症状画像（换 chunk 冲一下/边界尖峰/固定卡点/慢速
一卡一卡）、根因链（①yaml 命名空间未生效 ②引擎锁跨推理 ③收敛拉回 ④训练数据节奏）、参数体系
表（coeff/anchor_tol/control_interp/prefetch/jpeg 各自机制与调优方向）、诊断三板斧
（metrics+joint_stream 落盘 → plot_inference_curves 换 chunk 关联 → last_plan_ms−server_timing
拆延迟）、实测对比（test4→test5：尖峰 7→0、RTT 140→56ms）、数据层治本（compress/repair）。
**CLAUDE.md** 新增「推理优化历程（2026-09-14 实录）」时间线——按排查顺序讲清每个机制的
**为什么**与关键教训（参数自报防复发、起点用最后已发出行 _i-1、时序融合不平滑"预测 vs 执行"、
固定卡点=训练数据节奏、选帧零错位 vs 插值必然错位）。**验证**：纯文档，无代码改动。

**数采质量优化历程文档化——README「数据质量优化（采集→训练）」+ CLAUDE.md「质量优化历程」**——
`morla_data_collect`。把采集→训练链路质量认知沉淀成文档：**README** 新增 5c 章节——症状画像
（慢速抖/3fps 无人知/空录/坏段/训练后固定卡点，各对应防线与根因）、三层防线体系（实时告警/
离线隔离/转换两级自检）、训练数据节奏修复（compress_pauses/repair_aligned 对比表）。
**CLAUDE.md** 新增「数据质量优化历程」时间线——五层演进（上游 F7/F6 精度 → 实时防线 → 控制面
与目录 → 转换自检 → 训练数据节奏），讲清每层的**为什么**与关键教训（"数据坏了要当时知道"、
"模型是示范的镜子"、repair 选帧零错位 vs 插值必然错位的设计决策）。**验证**：纯文档，无代码改动。

## 2026-09-14

**真机推理调参：`async_prefetch_ahead` 25→5、`control_interp` 2→3**——`morla_policy_inference`。
yaml 生效参数调整（配合时序融合/切换平滑实测）：prefetch=5（每 45 步重规划，重规划 ~0.7Hz——
更少推理、网络压力更小；观测新鲜度略降但融合/锚点已兜住）；control_interp=3（90Hz 插值下发，
30Hz 大步再拆细）。serve_policy.env checkpoint 指回 pickup_act_480/080000（部署目标模型）。
数值为真机 A/B 可调项，改 launch 参数或 yaml 生效。

**遥操诊断 JSONL 日志 + web 记录开关**——`astral_arm_teleop` × `morla_web_manager`。
**动机**：慢速平移"停一下走一下"卡顿排查需要遥操链路逐级数据（VR 原始 / 滤波 / 命令 /
实测关节 / 人肘 psi / IK 计数），此前只能靠 tune 话题实时看或 `ros2 topic echo`。照
`morla_policy_inference` 的 `metrics_log_file`/`joint_stream_log_file` 模式：遥操节点新参数
`teleop_log_file`（空=关，零开销），非空则行缓冲追加写 JSONL，`kind` 区分五类记录——
`loop`（每 armed 控制拍：vr/filt/cmd TCP 位、q/q_state、psi_ref、vr_age/ik/loop_ms、
ik_fail/ik_sat/hard_fallback/ws_clip/reach_clip）、`wrist`（每腕位事件，upstream 阶梯证据）、
`state`（每实测关节事件）、`body`（每 body_joints 人肘方向 raw+EMA）、`metrics`（~2s
LatencyMeter 窗口 ms 统计+计数）。写失败绝不抛（不打断控制流）；`destroy_node` 关闭句柄。
launch 透传：`full_teleop.launch.py` → `astral_dual_arm_teleop.launch.py` 新增
`teleop_log_file` 参数，共享基路径按侧拆 `_left/_right`（`teleop_log.py:side_log_path`）。
web：预设启动请求 `StartRequest` 加 `log` 字段，System tab 启动按钮旁加「记录遥操日志」
勾选框（仅 morla_bringup/astral_arm_teleop 预设生效，注入
`teleop_log_file:=/tmp/teleop_teleop.jsonl`）。**验证**：新 `test_teleop_log.py` 4 例全绿
（side_log_path 拆路径、JSONL 写入/空路径禁用、写失败安全、LatencyMeter.snapshot 非破坏）；
web_monitor test 11→**15 例**全绿（新 `test_teleop_log_preset.py` 覆盖白名单注入/非白名单跳过/
空路径跳过/源预设不被改）；`npm run build` tsc+vite 通过。**节点/launch 级验证**（正确解释器
= 系统 ROS `/usr/bin/python3.10`，`PYTHONPATH` 在 source ROS 后**追加**而非覆盖）：
① 节点 dry_run 冒烟——喂 30 拍 1mm 慢速平移 + 1 帧 joint_states + body_joints，日志产出
`wrist×30 / loop×30 / state×1 / body×1`，loop 记录 18 字段齐全、vr/filt/cmd 数值正确
（30mm×scale→19.5mm、EMA 收敛后 filt≈cmd）、`ik_fail=0`；`metrics` 记录经强制窗口触发验证
（ms 统计 ee_r/ik/loop/vr_age + counts）。② `--show-args` 确认两个 launch 都声明
`teleop_log_file`。③ 端到端 `ros2 launch ... arm_side:=left teleop_log_file:=/tmp/teleop_e2e.jsonl`
（symlink-install 重建后），节点日志 `teleop jsonl log -> /tmp/teleop_e2e_left.jsonl`、文件已建——
**launch 参数全程透传 + 按侧拆路径实锤**。**坑**：dual_arm launch 裸路径直接跑时 `from
astral_arm_teleop.teleop_log import side_log_path` 解析期 ModuleNotFoundError（包不在
sys.path）→ 加 try/except 内联兜底（生产安装路径仍走单测过的包内函数）。**用法**：web 勾选
记录启动遥操 → `/tmp/teleop_teleop_{left,right}.jsonl`；CLI 直接
`ros2 launch morla_bringup full_teleop.launch.py ... teleop_log_file:=/tmp/x.jsonl`。

**对齐数据完整修复 `scripts/repair_aligned.py`——弧长匀速化重采样（插补）替代删帧**——
`astral_ws/scripts`。**动机**：compress_pauses.py（raw 层删帧）会让相邻帧位移变大、动作变
"跳"，且删帧后时间戳出现 gap（validate W2×100）；用户要求对**对齐后**数据做完整修复，优先
插补而非删除。**做法**：新脚本处理 `episode*/aligned_data.h5`，主方法 **resample**——把
「快-停-快」轨迹按累计运动量（弧长 = 8 维位移和，夹爪动作也计、不被压缩）重新映射成**匀速**：
目标每帧弧长 = `--speed-ref`（默认 0=自动取原数据 p90 速度），帧数 = 总弧长/speed-ref；关节
数值线性插值（平滑），相机帧取最近原始帧（JPEG 无法插值）、`src_offsets/src_timestamps/
quality` 同步取最近；时间戳均匀 `1/fps`（W2 gap 消失）；next-state action 重建。
`--method drop` 保留删帧法（静止段删中间帧，每段 `--min-keep`）。**验证**：全量 pick_place_merged
**21321→10489 帧（-50.8%，20s）**；时间戳间隔 33.3ms/std 0/严格单调；validate **W2 gap 100→0**、
W3 仅 1、0 fail（W5 armed coverage 为原数据警告与修复无关）；速度分布更均匀（std 改善）。
**坑**：resample 时间戳初版误用弧长单位（间隔 1000ms）→ 修正为 1/fps 秒。**用法**：修复后
`vla_process_act.sh`（align 检测 aligned 存在即跳过）→ 训练；`--speed-ref` 调帧数（更小=更
温和更多帧），`--fps` 调输出帧率。

**repair_aligned 对抗修复 2 处（图像-关节错位）**——`astral_ws/scripts`。① 相机帧选择
`searchsorted`（取上界）在静止段/边界选到更远帧，关节(弧长插值) vs 图像(离散最近帧)弧长偏差
实测 p50 2.2°/p90 6°/max **12.7°**（>半帧）——改 `argmin` 精确最近，压回 p50 0.9°/p90 2.7°/
max 5.7°（≤~半帧，插补固有代价）；② `speed_ref` 默认从"每 episode 独立 p90"改 **session 全局
p90**（预扫描所有 episode 合并，实测独立 p90 跨段差 1.3x 尺度不统一）。修复后 repaired 数据
重跑（-50.9%）并重转 v3。

**repair_aligned v2 重构：插值→选帧，两个模式严格零错位 + 对抗验证**——`astral_ws/scripts`。
**动机**：插值（关节弧长插值 + 图像最近帧）无论 argmin 多准都有 ≤半帧弧长的固有 state-图像
错位（实测 max 5.7°），对精确定位任务（插试管）是训练信号污染。**做法**：resample 改为
**弧长均匀选帧**——每个重采样点取"弧长最近原始帧"，state/action 与图像**严格同源同一原始帧**
（零错位）；静止段弧长≈0 被压缩、帧间位移≈speed_ref（速度近似均匀）。代价：相邻新帧可能重复
（静止段）或快慢不均（原始帧离散），但绝无错位。drop 本就走选帧（同零错位）。新增
`scripts/verify_aligned.py` 对抗验证工具：断言输出 state 每行精确等于原始某帧（非插值）、图像
bytes ∈ 原始帧集合（同源）、时间戳严格单调、action=next-state。**验证**：resample(选帧)
-50.9%、drop -8.4%；**对抗验证 ALL PASS**（repaired 1714 帧 + dropped 2967 帧四断言全过）。
两模式 v3 重转中。

**训练数据停顿压缩 `scripts/compress_pauses.py`——删掉录制中的长时间停顿/慢速段，让 ACT 学到更流畅的轨迹**——
`astral_ws/scripts`。**动机**：真机推理的固定卡点（到试管前/夹取后/放置前/释放后）＝ 训练数据里操作员
在任务阶段转换处的停顿/减速被模型忠实复现（实测训练集 20.6% 帧速度 <0.008 rad/帧、208 个慢速段；
真机 test5 慢速段与训练数据阶段一一对应）。引擎参数只能平滑换 chunk 跳变、改不了模型在特定状态预测
低速的行为——治本须改数据。**做法**：新脚本在 **raw 层删帧**（robot_data.h5 / camera_data.h5），
输出新 session；后续 `vla_process_*.sh` 基于删帧后 raw 重新对齐/转换（next-state 语义在 align 阶段
重建；相机 JPEG 字节原样保留、**零重编码**）。判定用 `left_arm_state` 流相邻帧 max|Δ|，`--speed-thresh`
（默认 0.008=只压完全停顿；0.02=连减速也压，实测删 80% 过度）、`--min-pause`（默认 3 帧）、
`--min-keep`（默认 4 帧/段，保留短暂过渡感）、`--dry-run`。**验证**：单 episode 结构校验（各流/相机
同步删帧、JPEG 保留）；全量 pick_place_merged 100 episode **54721→35614 帧（-34.9%，2168 停顿段，
14s 完成）**；align 100/100、validate 0 fail/100 warn（W2 时间戳 gap/W3 帧跳变是删帧预期后果，
v2.1 按帧序重建无碍）。**用法**：压缩 → `vla_process_act.sh raw/pick_place_merged_smooth ...` →
`act_train.sh` 重新训练；A/B 用原始 vs smooth 数据集对比真机卡点。**坑**：robot_data.h5 是
`streams/<流>/values+timestamps` 两层结构（曾误把子 group 当 dataset → U10 字符串）；camera 的 vlen
JPEG 数组须复用源 dtype 写入。

**修复：yaml 顶层命名空间 ≠ 节点名 → 整份参数被 rclpy 静默丢弃，推理长期按代码默认值跑**——
`morla_policy_inference`。**症状**：真机连续 4 次测试（test1-4）"改了 yaml 参数（coeff/
tol/interp/jpeg）运动却不见对应变化"；本次 test4 实测指令流仍 30Hz（`control_interp: 2`
未生效）、尖峰仍在。**根因**：yaml 顶层键是 `morla_policy_inference:`，而 launch 节点名是
`policy_node`——rclpy 按节点名匹配 `--params-file` 的段，键不匹配**整份参数被静默丢弃**
（Humble 无单键回退），节点回落到 `_declare_params` 代码默认值：`control_interp=1`、
`temporal_ensemble_coeff=0.0`、`chunk_anchor_tol=0.0`、`jpeg_transport=false`。即**此前所有
真机测试的时序融合 / 切换平滑 / 60Hz 插值 / JPEG 全都没生效**（host/port、camera_image_size、
metrics/joint_stream 日志因 launch 显式传参而侥幸正确）。对照 data_collect：其 yaml 顶层键
= `data_collect` = 节点名，正确。**做法**：yaml 顶层键 `morla_policy_inference:` →
`policy_node:`（+ 注释警示）；node 启动行扩展为**生效参数自报**（ctrl/coeff/anchor_tol/
anchor_blend/prefetch/jpeg/backend/host:port），未来任何 launch 一眼可见实际参数、防复发。
**验证**：launch 默认（不传 interp）→ `ctrl=60.0Hz coeff=0.01 anchor_tol=0.05 anchor_blend=4
prefetch=40 jpeg=True`（此前 interp=1/coeff=0/tol=0/jpeg=false）；launch 传 `control_interp:=2`
仍 60Hz（显式覆盖照常生效）；全套件 108 例全绿。**注意**：修复后 yaml 值（coeff 0.01、
prefetch 40、interp 2、jpeg true）将**首次真正生效**——test4 声称的 coeff 0.05 / prefetch 25
从未跑过，要测哪组用 launch 参数或改 yaml。

**上行 JPEG 传输优化 `jpeg_transport`**——`morla_policy_inference`。**动机**：远程推理
RTT 主项是上行带宽×载荷（1.38MB 原始 RGB，WiFi ~106ms/直连 ~14ms），GPU 推理仅 ~8ms。
**做法**：节点保持"解码→letterbox→RGB"不变（像素无改），`RemoteBackend` 在发送前把每个
camera 槽位 `encode_jpeg`（quality 92）成字节 + 载荷加 `image_format="jpeg"`（新
`image_codec.py`：encode/decode/normalize，cv2 优先 PIL 回退）；serve `_handle` 收到后
`normalize_request_images` 解码回 RGB 并 pop 标志（ACT/pi05 两分支照旧消费 RGB，不污染
openpi AstralInputs）。载荷 1.38MB→0.45MB（benchmark 随机噪声最差情形；真实相机图
~0.1-0.2MB，~7×）。**开关**：node/yaml/launch `jpeg_transport`（yaml 默认 true；false =
现状 RGB，兼容直连 openpi 官方 serve 的退路）。**像素**：节点 letterbox 后 RGB 再编码，
二次 JPEG 有损（训练数据本身是 collect tap 的 JPEG-90 解码，模型对其鲁棒）。**验证**：
4 例单测（jpeg on 发字节+标志、off 保持 RGB、round-trip 保真、normalize 解码+移除标志）、
协议 bytes 往返（msgpack bin）、全套件 104→**108 例**全绿；**端到端**：benchmark `--jpeg`
ALL PASS + e2e `--check-server --jpeg` preflight PASS。benchmark/e2e 加 `--jpeg` 以测真实
部署路径。**对抗性审查修 2 处**：serve `normalize` 移入 try（JPEG 解码失败优雅回错误串）；
e2e 补 `--jpeg`（此前 --check-server 不测 JPEG 路径）。

**服务端推理分项进节点指标：engine `server_timing` 透传 + `[MET]` 行 `srv=`**——
`morla_policy_inference`。**动机**：真机排障要拆"端到端 RTT（发送观测→收到 action）vs
服务端推理"——此前 `last_plan_ms`（端到端，remote 实测 avg ~140ms）在 state/metrics 有记录，
但服务端分项只存在 backend 内部、要单独跑 benchmark 才看得到。**做法**：引擎 `_run_plan`
infer 后把 `backend.last_server_timing` 透传到 `engine.stats.server_timing`（serve.py 返回的
prep/pre/infer/post/total_ms）→ 进 `/policy_inference/state` + `metrics_log_file`；`[MET]`
终端行加 `srv=<total>ms`。无该属性的 backend（Stub/Inproc）→ None 不崩。**验证**：新增 2 例
（带 server_timing 的 backend 透传到 stats、无则 None），全套件 102→**104 例**全绿。
README 指标口径（last_plan_ms=端到端、RTT−total=网络+序列化 ~130ms）与 CLAUDE.md 指标表同步。

**换 chunk 切换平滑 `chunk_anchor_tol`——消除收敛拉回/模型突变切换尖峰**——
`morla_policy_inference`。**症状**：temporal_ensemble 已开仍残留换 chunk 尖峰（真实
pi_cmds 7 个 >0.1 rad 全在换 chunk 边界，多收敛拉回）。**根因**：时序融合只平滑"新旧
预测"，不平滑"预测 vs 执行"——旧 command 开环 chunk 内漂移（重规划观测过时 ~130-160ms +
预测漂移累积），重规划时新预测一步追向实测（收敛拉回，跳后新 chunk[i0]≈state）；初版"从
实测 state blend"对收敛拉回无效（跳后 dev<tol 不触发），真正跳的是**旧 command 漂移量**。
**做法**：`_install` 加 `chunk_anchor_tol`（yaml 默认 0.05，>0 开）——续播起点偏离
**正在执行的旧 command** >tol 时，前 `chunk_anchor_blend`（默认 4）行从旧值线性过渡到
新轨迹，切换差摊到 nblend 行；node/yaml/launch 全链路透传。**验证**：3 例单测（旧值起步
blend=0.56/0.74/0.8、tol 内不触发、关闭硬切换）；**真实数据离线模拟**（对 pi_cmds 7 个尖峰
应用 blend=4/tol=0.05）——0.1-0.2 rad → **全部 ≤0.04 rad**（<0.1 阈值）；全套件 99→**102
例**全绿。README/CLAUDE.md 同步。**对抗性审查修复 2 处**：① `old_row` 原取 `_chunk[_i]`
（下一行、还没发给机器人），blend 起点超前一行——改为 `_chunk[_i-1]`（最后**已发出**的行，
机器人正在执行它），测试锚点同步（_chunk[19]），期望值不变；② `chunk_anchor_blend` 补
launch 透传（此前只在 yaml）。

**`plot_inference_curves.py` 升级：尖峰换 chunk 关联 + 收敛拉回/模型突变分类**——
`astral_ws/scripts`。**动机**：真机诊断（temporal_ensemble + async_prefetch_ahead=25 +
control_interp=1，801 行 / 26.8s 实测）发现 7 个 >0.1 rad 尖峰**全部落在换 chunk 边界**
（行号 %25≈3，metrics `engine.plans` 递增处 ±0.5s 内 7/7），且多数是**收敛拉回**——command
长期偏离 state（开环 chunk 预测漂移），重规划时新预测一步猛追回真实位置。**做法**：脚本
新增 `--metrics`（读 `engine.plans` 递增处 = 换 chunk 时刻）→ 每个尖峰标「距最近重规划」
（判是否换 chunk）；按「跳变后 command 距 state」分类——`< 0.6×步长` = 收敛拉回、否则
模型突变；步长面板红点标尖峰（橙=收敛/红=模型突变）+ 绿虚线标换 chunk；`--self-test`
合成数据自测分类逻辑（行20 收敛拉回+换chunk、行50 模型突变+非换chunk 全 PASS）。
**验证**：真实 pi_cmds.jsonl + pi_metrics.jsonl——7/7 换chunk、**6/7 收敛拉回**（cmd-state
0.003~0.055）、1/7 模型突变（行578 cmd-state≈步长，主动跳离），与人工逐条核对一致。
**诊断结论**：残留尖峰 = 换 chunk 时收敛拉回，temporal_ensemble 边界融合只把新旧 chunk
差异减半未消除；对症 `control_interp=2/3`（插值细粒度，追 state 逐步逼近）+ 
`async_prefetch_ahead=40`（每 10 步更勤重规划，预测漂移积累更少）。

**推理时间曲线诊断：joint_stream_log 同轴记录 state + `plot_inference_curves.py`**——
`morla_policy_inference`。**需求**：真机推理时查看 state/action 时间曲线、state↔action
错位时间、输出动作平滑度。**做法**：① 节点 `joint_stream_log_file` 每行指令值之外**同轴
加记观测 state**（`_state_ok()` 只读取，layout 顺序），旧日志无 state 自动降级；
② 新 `scripts/plot_inference_curves.py`：读 jsonl 画 7 臂关节 state(蓝)vs command(红)
时间曲线 + 动作步长(平滑度)+夹爪 PNG，并算 **state↔command 错位**（逐关节互相关最佳
滞后，+ = state 领先）、**动作平滑度**（每行最大步长分布 + >阈值尖峰及时刻/角速度）。
**验证**：合成数据（command 领先 state 15 行）实测报 -15 行/499ms 且 corr=1.0（先发现
符号反了已修）；真实 pi_cmds.jsonl 报 12 个 >0.1 rad 尖峰（2.6~6.4 rad/s，即"冲一下"点）；
套件 99 例全绿。**坑**：matplotlib DejaVu 无 CJK 字形，图内文本用 ASCII（终端输出可中文）。

**web 推理模块白屏修复：engine.server_timing 嵌套对象进 React child + 顶层 ErrorBoundary**——
`morla_web_manager`（前端）。**症状**：web 启动推理节点 → 开始策略后界面白屏，刷新仍白屏。
**根因**：推理节点一跑，WS 遥测里 `engine` 出现嵌套对象 `server_timing:{prep_ms,pre_ms,infer_ms,...}`，
`InferenceCard` 引擎参数渲染 `<b>{v}</b>` 把对象直接当 React child——React 抛
"Objects are not valid as a React child" → 整棵树卸载白屏；节点持续推遥测，每次加载都崩。
**做法**：①`InferenceCard` 新增 `fmtStat` 安全格式化（标量原样、对象压平为
`{k1:v1 k2:v2}` 摘要），engine 参数与 cam_frames 两处渲染都用它；②新增顶层
`ErrorBoundary` 包住 App——任何组件渲染异常不再整树白屏，显示错误卡 +「重试渲染」按钮
（WS 数据流模块层继续跑），console 记 componentStack 便于定位。**验证**：Node SSR
（react-dom/server + esbuild）喂入**确切崩溃数据**（engine.server_timing 嵌套对象）渲染
InferenceCard + 整棵 MonitorTab——修复前抛错、修复后正常且 server_timing 压平渲染、无
[object Object] 泄漏；前端 `npm run build`（tsc+vite）通过；web_monitor pytest 11 例全绿。
**部署**：机器人侧重建前端 dist（`cd web && npm run build`）后刷新页面即可，后端无改动。

**launch 默认值反模式修复 + `control_interp` 透传（yaml 默认 1→2）**——`morla_policy_inference`。
① **launch host/port 默认值静默覆盖 yaml（bug）**：`host`/`port` 的 launch 默认值此前是
`127.0.0.1`/`8000`，而 launch 无条件把整包参数 append——`ros2 launch` 部署时即使用户在 yaml
改了 host/port，也会被 launch 默认值覆盖、连到 127.0.0.1:8000（data_collect 不变量 1 的反
模式：launch 默认值不该盖 yaml）。**修法**：默认值改空串、只 append 显式传入的键（host/port/
checkpoint_dir），cmd_topic 保留默认（keyboard 依赖且与 yaml 一致）。② **control_interp 透传**：
launch 补 `control_interp`（此前只能在 yaml 改），yaml 默认 **1→2**——2 = 60Hz 线性插值下发，
把 30Hz 的大步拆半（真机快动作"卡一下"对症，配合 chunk_anchor_tol/时序融合）。**验证**：
py_compile；launch 语义回归——不传 host/port 时回落 yaml（无 127.0.0.1:8000 覆盖）、显式传才
覆盖；policy_inference 全套件 108 例全绿（engine 行为不变）。

## 2026-09-11

**融合频率参数化 `async_prefetch_ahead`**——`morla_policy_inference`。把"每多少步
重规划并融合"从引擎硬编码默认（chunk//2=25）暴露为 yaml/launch 可配：`每
(action_chunk − async_prefetch_ahead) 步融合一次`。默认 25（每 25 步）；yaml 当前设
**40 = 每 10 步融合一次**（重规划 1.2→3Hz，观测更新鲜 + 重叠 40 行融合更充分，直连网线
下无压力）。0 = 自动 (chunk//2)。仅 queue_async 生效。node/launch 透传 + 新增
`test_async_prefetch_ahead_reaches_engine`，套件 98→**99 例**全绿。

**推理移出引擎锁：消除换 chunk 指令空档 + 修正续播对齐**——`morla_policy_inference`。
**症状**：真机推理时"固定时刻突然冲一下"（夹爪闭合上提、放完回程等，都恰逢换 chunk 重
规划）。分析 `pi_cmds.jsonl`：6 个 >0.1 rad 的步长**每个都紧跟在 106~149ms 的指令空档
之后**，且 plans 同时 +2——"先停着等 ~120ms、再冲过去追"。**根因**：`_run_plan` 锁跨
`backend.infer()` 持有，慢推理（远程网络 ~120ms，`last_plan_ms` 实测 120~164ms）期间
控制线程 `tick()` 拿不到锁 → 指令断流；且安装时 `i0=round(latency_ms×fps)` 按"机器人
前进了 4 行"估算，但锁内推理时控制线程被堵、机器人**实际没动** → 超前跳（叠加进 lunge）。
**做法**：① 推理移出引擎锁——锁内只快照观测 + 记 `_i_snap`，释放锁推理，再取锁安装
（queue_sync 的 `_plan_blocking` 外层仍持锁，阻塞重填语义不变；queue_async 的
`stop()` 本就 join planner 再关后端，无竞态）；② 续播索引用**实测 `consumed = _i −
_i_snap`**（推理期间控制线程真实消费行数），替代 latency_ms 估算；③ 时序融合对齐锚点
改用 `_i_snap`（`old[_i_snap+k] ↔ new[k]` 时间语义不变，前 consumed 行 stale 混合由
i0 跳过）。**验证**：新增 3 例回归（200ms 慢推理下 tick() 不被堵 >100ms；慢推理后从
`new[consumed]` 续播而非 new[0]/超前跳；融合+锁外推理锚点对齐不崩），套件 95→**98 例**
全绿。效果：换 chunk 空档从 ~120ms（网络时长）→ 0（控制线程从缓冲照常发），网络延迟
与指令流解耦。

**openpi（pi0.5）训练迁移交接：本机 15GB RAM 装不下 pi05_base，部署清单 + 文档全树同步**——
`docs/openpi-training-deploy.md`（新）× 顶层 CLAUDE.md × CHANGELOG。**背景/验证**：完整跑 openpi
数据→训练链路——重指数据集软链（原断链到已删目录）、重算 norm stats（CPU 666 窗口）、**真实数据语义
金标准**（delta 掩码 [T×7,F]、截断 8 维、相机槽位 base/left_wrist 有像素 + right 零填充、action
delta/绝对语义 k=0,1 OK、tokenized_prompt 存在）全绿、`test_adversarial_configs` **8/8** 全绿、
JAX CUDA 重启后恢复。**关键发现（对抗性审查）**：① 训练冒烟三次**内核 OOM（SIGKILL）**——pi05_base
权重 11.6GB 必须整体进 RAM，本机 15GB − IDE ~5GB = 10GB 可用，实测 anon-rss 9.0/10.25/10.85GB 三连
被杀（`journalctl -k` 实锤）；降 batch/`PREALLOCATE=false`/`MEM_FRACTION` 均无效（只影响显存）。
**结论**：openpi 训练需 **RAM≥32GB** 机器；ACT（0.2GB 模型）留本机无碍。② 连带发现
`OPENPI_DATA_HOME=/home/robot/openpi/ckpt` 错误指向导致每次重下 11.6GB 权重（慢网络卡 1h+），正确
权重在 `~/.cache/openpi`（12GB 完整，`maybe_download` 秒级命中）；建议 unset。③ CLI 坑：
`--no-wandb-enabled`（非 `=false`）、checkpoint 存在需 `--overwrite`。**做法**：写交接文档
`docs/openpi-training-deploy.md`（目标机硬性要求 / 迁移三件套+体积（代码几百MB、数据集 58MB、权重
12GB）/ 准备步骤 / 训练命令 / 5 个坑 / 验收清单）；顶层 CLAUDE.md 训练速查改指该文档。
**遗留**：目标机验证训练冒烟通过后，把 `docs/openpi-training-deploy.md` 的验收结果回填。

Quest3 → Astral 双臂 + Wuji 双手。从 `xnero_ws-main` 迁入。各包 README 内亦有对应条目。

时间均为北京时间。

**web 监控 tab 新增「策略推理」模块（配置化泳道 + 按钮化键盘控制面）**——
`morla_web_manager` × `morla_policy_inference`。**动机**：用户要求——推理包实机测完但一直命令行
启动（launch 参数 + 单独键盘节点），要在数采模块下方加一个推理模块：可配置 GPU 主机 IP/端口/图像
尺寸/是否记录日志，用按钮实现键盘全部功能，UI 美观。**做法**：①`monitor_node` 加推理话题桥——
发布 `/policy_inference/cmd`（String 动词，同 `policy_keyboard` 契约）+ `/task`（latched），订阅
`/policy_inference/state`（latched）镜像进 ui_state `infer`（带 stale 龄期标记）；**纯话题控制面，
CLI 启动的推理节点同样可控**（同数采卡片模式）；②`web_server` 加推理泳道 `_policy_mgr`（配置化
构建 `policy_inference.launch.py` 命令，`start(check_orphan=False)`——推理节点设计上与遥操共存、
takeover 仲裁，不能被"有遥操 launch 在跑"挡掉）+ 端点 `POST /api/v1/infer/launch/start|stop|
restart`（配置经 `InferLaunchRequest`：backend_type/model/host/port/camera_image_size/engine_mode/
log）、`POST /api/v1/infer/cmd`（白名单 policy/playback/pause/resume/takeover/release/stop 及
`playback:<源>`）、`POST /api/v1/infer/task`；日志开关 → `metrics_log_file=/tmp/pi_metrics.jsonl`
+ `joint_stream_log_file=/tmp/pi_cmds.jsonl`；③前端新增 `InferenceCard`（数采卡片下方）——启动
配置表单（主机/端口/尺寸/引擎模式/模型族/日志开关）+ 命令按钮（开始策略/回放[可选源]/暂停/恢复/
接管 HUMAN/释放/停止，对应键盘全部按键）+ 任务输入 + 实时状态（模式徽标/engine 指标/延迟/
exec_events/error/stale）。**对抗性审查修复**：推理泳道 launch 参数来自 web 输入而 `LaunchManager`
用 `bash -c` 拼命令——**host 等含 shell 元字符 = 命令注入**；`policy_launch_args` 加白名单清洗
（`[^A-Za-z0-9_.:\-]` 剔除，port/尺寸先 int 化），回归测试锁死。**验证**：新增 `test_infer_control.py`
**11 例全绿**（launch 参数默认/覆盖/int 化、注入清洗无 shell 元字符残留、cmd 白名单全动词+playback
带源+垃圾拒绝）；web_monitor 套件全绿；前端 `npm run build`（tsc+vite，50 模块）通过；后端
py_compile 通过。**待办（机器人侧）**：同步构建 `morla_web_manager` + 前端 dist 后实机冒烟——
配置 host/尺寸 → 启动推理节点 → 按钮发 policy/pause/stop → state 实时刷新、日志文件落盘。

## 2026-09-10

**节点关节指令流自动记录 `joint_stream_log_file`**——`morla_policy_inference`。
**症状**：排障真机卡顿需要实际的关节指令流（30Hz 步长/节奏），但 metrics_log_file 只记
state 高层指标，不含指令值；`ros2 topic echo` 又因 BEST_EFFORT depth=1 需手加 QoS flag 且
输出 YAML 难解析。**做法**：节点新增参数 `joint_stream_log_file`（默认空=关闭）：非空时每次
实际下发（`_send_targets_from`）把带时间戳的各话题指令值（JointState 7 维位置 + 夹爪比值）
追加写该文件（JSON 行，与 metrics_log_file 相互独立）；launch 新增透传。**验证**：新增
`test_joint_stream_log_writes_commanded_values`（下发 2 拍 → 文件含 `/left_arm/joint_commands`
7 维浮点行），套件 94→**95 例**全绿。**用途**：`joint_stream_log_file:=/tmp/pi_cmds.jsonl`
后可直接分析"每 25 行（0.83s）边界步长是否偏大、指令到达间隔是否均匀"。

**引擎级 ACT 时序融合 `temporal_ensemble_coeff`**——`morla_policy_inference`。
**症状**：真机 POLICY 一卡一卡（换 chunk 时指令硬跳）。已排除网络（直连网线后延迟
116→21ms 卡顿依旧）、GPU 争用、数据抖动、夹爪 clip（实测模型输出仅越界 ≤0.008）。
**做法**：借鉴 lerobot `ACTTemporalEnsembler`（ACT 论文 Algorithm 2：重规划的新 chunk 与
历史预测做在线指数加权平均，权重 wᵢ=exp(-coeff·i)，coeff>0 旧预测权重大→新 chunk 无法
瞬间跳变）。lerobot 版每 tick 重规划，我们的引擎 ~chunk/2 行才重规划一次，故把 update()
推广为**部分消费后重规划**：新 chunk 头部与旧缓冲未消费尾段（时间对齐）按 ACT 权重平均，
尾部直接追加（engine.py 新增 `TemporalEnsembler`，numpy 实现）。引擎新增参数
`temporal_ensemble_coeff`（0=关默认，>0 开启；yaml 默认 0.01=ACT 推荐值），node/yaml/
launch 全链路透传，`reset()` 清融合计数。**验证**：新增 2 例（注入 0→1 硬跳变 chunk：
无融合步长 1.0、融合后 <0.9 且确实降低最大步；首块采用+reset 清计数），套件 91→**93 例**
全绿。**诚实备注**：离线仿真喂录制观测显示切换步无明显变化（录制观测下切换本就平滑，
新[0]≈旧[24]），真机卡顿源于真实观测的闭环发散，离线复现不了——真机 A/B
（`temporal_ensemble_coeff:=0.01` vs `0.0`）才能定论；若 A/B 无改善说明跳变不在 chunk
边界，需抓 joint_commands 定位。**对抗性审查修复 2 处**：① `TemporalEnsembler.update`
重叠行数只按 `min(M-i, n)` 取，旧缓冲短于 chunk_size（StubBackend 4 行/截断响应）时
`old[i+k]/counts[i+k]` **越界 IndexError**——改为 `min(max(0,len(old)-i), n)`，新增
`test_ensembling_short_chunk_does_not_crash` 回归（修复前红、修复后绿）；②
`engine.reset()` 的 `ensembler.reset()` 原在锁外，与 planner 线程锁内的 `_install` 改计数
竞态——移入锁内。套件 93→**94 例**全绿。

**GPU 主机 serve 启动脚本 `serve_policy.sh`**——`MORLA/scripts/`。包装
`serve.py` 为可运维入口：参数**外置**在 `serve_policy.env`（PY/MODEL/CHECKPOINT_DIR/HOST/
PORT/ACTION_DIM/SLOT_MAP/...），改参不动脚本；CLI 可临时覆盖（`--port`/`--config`）。**启动前
两道冲突检查**：① pidfile 认"是否本脚本旧实例"（是→默认拒绝、`--force` 优雅停旧再起新）；
② 端口被**他人**进程监听→拒绝并列出占用者（`ss -ltnp`）。`--status`/`--stop` 配套。
**验证**（全路径实测）：正常启动→真实推理往返 ALL PASS→stop 端口释放；重复启动拒绝；
`--force` 停旧起新（pid 更换）；他人占端口拒绝。**踩坑**：bash `${VAR:-默认}` 里不能放
JSON——`${SLOT_MAP:-'{"a":...}'}` 解析时词内 `}` 被当闭合符，**变量已设也被尾部 `"}` 污染**
（实测 54→56 字符、serve 报 JSON Extra data）；默认值改用 `:?` 必填，注释已写明。

**节点指标自动记录 `metrics_log_file`**——`morla_policy_inference`。**症状**：真机推理想
看延迟/引擎指标（`latency_ms.loop/obs_age`、engine pops/plans/plan_ms/remaining、
exec_events），只能手动 `ros2 topic echo /policy_inference/state > file`，且 echo 输出是
YAML 非 JSON 行、传感器话题还常因 QoS 不匹配收到 0 条。**做法**：节点新增参数
`metrics_log_file`（默认空=关闭，零开销）：非空时每次 `_publish_state`（1Hz + 状态变化）
把带时间戳的 state JSON 追加写该文件，并在终端（launch output=screen）打印一行摘要
`[MET] ... loop_avg obs_avg pops plans plan_ms rem exec`；launch 新增 `metrics_log_file` 透传，
yaml 已加注释项。**验证**：新增 `test_metrics_log_writes_file`（两次发布=两行 JSON，含
t/latency_ms/engine），套件 90→**91 例**全绿。顺带修 `run_act_correctness.py` 残留的
`LerobotActBackend` 旧类名引用（重构漏网）。

## 2026-09-09

**ACT 三种训练模式一键脚本 `scripts/act_train.sh`**——`astral_ws/scripts`。**动机**：用户已分别做过
从头训/接着训（resume）/已有模型加数据微调三种训练，要求统一成一个可配置参数的一键脚本。
**做法**：`act_train.sh --mode from_scratch|resume|finetune` + 通用参数（dataset/checkpoint/
output-dir/steps/batch/lr/chunk/n_action_steps/kl/num_workers/job/wandb）；按模式自动组装
lerobot-train 命令——from_scratch 默认 lr 3e-5/steps 80000 且禁带 checkpoint；resume 用
`--resume`+原 output_dir+新 steps（lr/bs 沿用 checkpoint，文档提示 batch 须与原 run 一致）；
finetune 用 `--policy.path`+新数据集+新 output_dir（默认 lr 1e-5/steps 30000）。含参数/模式校验
与 `--dry-run`（只打印命令不跑）。**验证**：`bash -n` 通过 + 三种模式 dry-run 组装逐条核对 +
负例拦截（finetune 缺 checkpoint / from_scratch 带 checkpoint 均拒绝，exit=2）。默认 wandb offline。

**BEST_EFFORT 话题测速 `scripts/hz_best_effort.py`**——`astral_ws/scripts`。**动机**：
`ros2 topic hz` 在 Humble 没有 QoS 选项，默认 RELIABLE 订阅匹配不上 BEST_EFFORT 发布者
（如 policy_node 发的 `/left_arm/joint_commands` 就是 BEST_EFFORT depth=1），永远报 0 条；
排障卡顿要看指令流实际到达率/间隔。**做法**：新增脚本——BEST_EFFORT depth=1 订阅，滚动
打印到达率 + 相邻消息间隔 min/mean/max/std；`--msg js|float|str` 按话题选、`--window` 滚动
窗口。**验证**：py_compile + ROS 环境可 import。


**推理包重构：传输/模型解耦命名 + 统一 serve.py + 自包含 client**。`backend_type` 原来用
`"openpi"|"act"` 把传输和模型名混在一起（openpi 实为远程传输、act 实为进程内加载，且都支持
任意模型），与后续多模型扩展冲突。**做法**：① `backend_type` 改为表达**传输**（`remote` 连
serve.py / `inproc` 进程内 / `stub` 冒烟），新增 `model` 表达**模型族**（`act` / `pi05` / 后续，
决定 serve/inproc 加载路径）；类重命名 `OpenPiServerBackend→RemoteBackend`、
`LerobotActBackend→InprocBackend`（体现本地非远程），**不留** "openpi"/"act" 旧别名；
② **统一 serve**：`scripts/serve.py --model act|pi05` 单入口替代 serve_act.py/serve_policy.py
分离（act=lerobot InprocBackend，pi05=openpi create_trained_policy，同一 websocket 协议）；
③ **client 自包含**：新增 `protocol.py`（vendor openpi `__ndarray__` 序列化器）+ `client.py`
（websocket client），节点 RemoteBackend 不再依赖外部 openpi_client——**Jetson 无需再装
openpi-client**，也去掉 serve_act 的 `_OPENPI_CLIENT_SRC` sys.path hack。④ node/yaml/launch
同步：`backend_type: remote` + `model: act`，launch 新增 `model` 透传。**验证**：90 例单测全绿
（新增 protocol round-trip + wire-key 锁定、make_backend remote/inproc/stub、RemoteBackend
payload）；`serve.py --model act` + RemoteBackend(自包含 client) 真实 ACT 推理返回 (50,8) 绝对。

**移除非 ROS 推理实现（决策：主用 ROS 侧）**。删除：`runner.py`/`session.py`/`hw_io.py`/
`scripts/runner_demo.py`/`scripts/act_inference.py`/`scripts/robot_session_cli.py`/
`config/robot_session.yaml` + `tests/test_hw_io.py`/`tests/test_session.py`（共 9 文件），
并清 CLAUDE.md/README 的非 ROS 章节。包回到纯 ROS 侧结构（node/keyboard/backend/engine/
executor/controller/replay/robot_io + scripts/serve_act.py）。`s` 键盘问题（launch 子进程
stdin=/dev/null）按「单独终端 `ros2 run morla_policy_inference policy_keyboard`」解决。

**真机推理"policy blocked: missing left_gripper_ratio"修复——夹爪状态三级种子**——
`morla_policy_inference`。**症状**：机器人侧纯 POLICY 部署（无遥操在发）按 policy 直接被拒
`incomplete obs, missing=['left_gripper_ratio']`。**根因**：夹爪观测订 `/left_gripper/command`
（命令回显约定），节点对 ratio 有两级种子（收到过命令 / 自己发过命令），但**首次进 POLICY 时
两者皆空**（teleop 不在 → 无人发命令；节点还没输出过动作）→ 鸡蛋问题死锁。**做法**：加第三级
种子——订阅 driver **每拍都发**的 `/{side}_gripper/joint_states`（last-commanded rad 回显，
从未命令过则 0.0），按 rad↔ratio 线性映射换算回 ratio（新参数 `gripper_open_rad/closed_rad`，
**须与 driver yaml 一致**，本机 2.5/0.0）；优先级 = 收到命令 > 自身命令 > driver 回显。
**验证**：test_node_flow 12→**14 例**全绿（driver 回显 rad=0.4→ratio 0.5 且完整进 POLICY、
无任何夹爪源仍拒绝进策略），全套件 **106 例**全绿；yaml 同步 cameras=[video8,video0] 与新参数。
**部署**：机器人侧同步包代码 + colcon build 后重启 policy_node，确认 driver yaml 的
open/closed_rad 与节点参数一致。

**夹爪冷启动种子语义修正——"从未命令"回显 open_rad（全开）而非 0.0（全合）**——
`astral_robot_control`（driver）。**动机**：用户指出整机上电时夹爪实际是**全开**的；原 driver
对"从未命令过"的夹爪回显 0.0 rad，经 policy_node 的 rad→ratio 换算得 1.0=全合——与物理现实
相反；且 0.0 rad 在话题层与"真命令到全合"无法区分。**做法**：driver state 定时器的夹爪回显
改为"从未命令 → 回显 `open_rad`（按侧取 `*_gripper_open_rad`，本机 2.5）"，节点换算自然得
ratio 0.0=开；命令过则照旧回显真实最后命令。**验证**：`test_driver_services.py` 20 例全绿；
policy_node 侧换算链路（open_rad→ratio 0.0）由 node_flow 的 driver 回显种子用例覆盖。**部署**：
driver 与 policy_inference 两个包都要同步 + colcon build。


**新增 `morla_policy_inference` 非 ROS 真机推理 Session（`hw_io.py` + `session.py` +
`scripts/robot_session_cli.py` + `config/robot_session.yaml`）——脱离 ROS2 完成完整真机推理 Session。**
**动机**：`runner.py` 已是纯线程的非 ROS 完整编排（FSM/引擎/安全层/回放/HITL），但输入输出没接
真实硬件——`runner_demo.py` 只喂合成观测、把动作丢进列表，无法实机使用。**做法**：新增三层 I/O
装配——① `hw_io.py`：`RobotIO`（`astral_robot_sdk` 适配：读 18 维关节反馈按 schema 拼状态、
夹爪 ratio 回显、绝对动作行按块下发 `set_target_positions(左臂 motor ids)`/`set_gripper_angle`
（ratio→rad 映射，单臂不命令右臂）、阻尼/位置 HITL 带 `seed_from_current` 防回跳、
`dry_run` 无硬件模式）+ `CameraIO`（每相机后台线程：V4L2/pyrealsense2，帧 letterbox 到模型
尺寸，**单相机失败不拖垮 Session**）；② `session.py`：`RobotSession`（泵线程按 fps 喂观测 +
`on_action` 下发/记录 + HITL 阻尼接管映射 + estop 兜底，SDK 调用异常不杀控制线程）+ 
`Hdf5SessionRecorder`（写 **aligned_data.h5 兼容格式** `/action`+attrs{fps,schema}，可直接
`load_replay` 回放本次 Session）+ `SessionConfig`/`load_session_config`/`build_robot_session`
（yaml 装配，schema 段必须与 `data_collect.yaml` 一致）；③ `scripts/robot_session_cli.py`：
键盘 CLI（s/y/t/空格/n/h/g/x/e/q + 1s 状态行到 stderr）。**后端两选**：openpi 远程（大 VLA
走 GPU 主机 serve_policy/serve_act，机器人侧只连 websocket）/ act 进程内（Jetson py3.12
lerobot env，需装 astral_robot_sdk + pyrealsense2）。**数值**（dry-run + stub 冒烟）：策略
30Hz 动作下发、引擎 loop 0.5-0.6ms、POLICY→PAUSED→POLICY→IDLE 全通、HITL 阻尼→位置切换
正确、56 帧记录可被 `load_replay` 回读（action_dim 8/fps 30/schema 齐全）。**顺带修一个潜伏
bug**：`robot_io.assemble_state` 对 waist/head 用 `body[(14,16)]` 当二元索引（必 IndexError，
现有部署关腰/头从未触发），改 `slice(*body_slice)`（先红后绿，新增用例
`test_waist_head_sliced_from_body_state`）。单测 104 例全绿（新增 18 例 hermetic：RobotIO
状态拼装/单臂只发左/ratio→rad/HITL/dry-run、CameraIO letterbox、记录→回读一致、
RobotSession 编排/HITL、config 解析）。**部署注意**：真机参数（控制板/相机）本机无法验证，
Jetson 侧需装 `astral_robot_sdk`（act 进程内还要 + pyrealsense2 + lerobot env）；`camera_map`
与 ACT 图像键就是 `observation.images.video8/video0`（`make_backend` label→label 路径正确）。

**修复 `morla_policy_inference` ACT 后端加载真实 checkpoint 的 bug + 本机全链路验证**——
`LerobotActBackend.open()` 原用 `PreTrainedPolicy.from_pretrained()` 直接加载，但 lerobot 0.6.2
（`/home/robot/loopkok/lerobot` 与 `VLA/lerobot` 两个 checkout 行为一致）里 `PreTrainedPolicy`
是**抽象基类**，直接调用必挂 `Can't instantiate abstract class`。**根因**：正确用法是经
`factory.get_policy_class(config.json["type"])` 解析出具体类 `ACTPolicy` 再
`from_pretrained`；`test_backend.py` 从未用真实 checkpoint 覆盖过 `LerobotActBackend.open()`
（只测 openpi fake client / stub / factory），这条路径零覆盖。**修法**：`open()` 改读
`checkpoint_dir/config.json` 的 `type` → `get_policy_class` → 具体类 `from_pretrained`
（`pre/post` 处理器加载不变）；新增 2 例 hermetic 回归（`sys.modules` 伪造 lerobot，无
torch/无网络）：`TestLerobotActBackendLoad::test_open_resolves_concrete_policy_via_factory`
（先红后绿）+ `test_open_unknown_type_raises_policy_error`。**验证**（本机 4090D + ROS
humble + conda lerobot env）：① 后端层——真实 checkpoint
`astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model` 加载 0.5s 到 cuda:0，合成
480×480 图像推理出 8 维绝对动作（夹爪 [0.33,0.33]）；② 引擎层——50 tick 产出 50 行平滑
绝对动作（最大步长 0.0085 rad，夹爪 [0.37,0.43]），ACT 内部 50 行队列只真推理 1 次（236ms）
其余缓存弹出（≤5ms）；reset 清空 ACT 队列后 tick 从新观测重规划（队列 49=新 50-chunk）；
③ 节点层——colcon 可执行文件 + 参数文件（含 JSON `camera_map`）在本机 ROS 下启动成功
（schema=8D arms=['left']），cmd→POLICY→指令流→pause 夹持（Δ=0）→resume 重规划→stop→IDLE
全 PASS，顺带实测了运行期观测门控（关节停发 >1s 自动暂停、resume 缺新鲜观测被拦，均正确）。
单测 81 例全绿（69 非 ROS + 12 node_flow）。**注意**：ACT 模型 preprocessor 无 resize，图像
须 480×480（节点 `camera_image_size` 须设 480，非默认 224）。

**环境硬墙 + ACT 远程 serve 化（`scripts/serve_act.py`）——真实 ACT 端到端跑通**。**动机**：
「完整节点 + ACT 单进程」在本机不可行——lerobot 0.6.2 要求 py3.12（代码真用了 PEP 695 泛型
`def f[T](...)`，py3.10 解析期 SyntaxError，shim 无法绕过），而 rclpy（ROS Humble）只有 py3.10
绑定；版本约束不可调和。**做法**：新增 `scripts/serve_act.py`，在 py3.12 lerobot 环境里复用
修好的 `LerobotActBackend` 加载 checkpoint，起一个**与 openpi websocket 协议完全兼容**的
server（msgpack_numpy、连接先发 metadata、请求 state+images+prompt、响应 absolute actions、
字符串响应即错误）；node 侧**零改动**直接 `backend_type=openpi` + `host/port` 连它
（`OpenPiServerBackend` 现成）。**踩坑**：①`msgpack_numpy.Packer` 无 `unpackb`（模块级函数）；
②openpi_client **自带 vendored msgpack_numpy**（键 `__ndarray__`），与 pip 版（键 `nd`）线上
不兼容——server 必须用与 client 同源的 vendored 版，否则解出 dict；③编辑残留重复的
`_make_packer` 定义导致旧 pip 版覆盖新 vendored 版（后定义覆盖前定义）。**验证**（本机
4090D）：py3.10 `OpenPiServerBackend` client → serve_act（py3.12 + cu130 torch）真实推理回传
8 维绝对动作（首推 187ms）；完整节点端到端——policy_node（backend_type=openpi,
camera_image_size=480）+ 合成关节/夹爪/480² 图像驱动，**真实 ACT 驱动指令流 123 条/99 个不同值、
夹爪比值 [0.25,0.5]、pause 夹持 Δ=0、resume 重规划、stop→IDLE，全 PASS**。环境处置：torch
2.11.0+cu130 + torchvision 0.26.0 装入 ros2 conda env（与训练 env 同版本，满足 lerobot `<2.12`
约束）；ros2 env 里 pip 强装的 lerobot 因 py3.10 无法解析已卸载。**部署模型**：ACT = serve_act
进程（py3.12, GPU 主机）+ 节点（py3.10, 机器人侧）远程连；pi0.5 走原 openpi server 同理。
**验证脚本收拢进仓库**（原 /tmp 脚本会丢）：`MORLA/scripts/run_act_integration.py`
（模型侧：后端 get_policy_class 加载 + 推理 + 引擎分块/队列/reset，py3.12 环境跑）与
`MORLA/scripts/run_act_e2e.py`（节点侧全链路 + `--check-server` 预检，py3.10 + ROS 跑），
均已实测 ALL PASS；包 CLAUDE.md/README、顶层 README、根 CLAUDE.md 均补交接文档。

**完整端到端 + 推理正确性 + 远程链路指标（真实数据集 + 真实模型，本机 4090D）**。新增
`MORLA/scripts/run_act_correctness.py`（正确性）与 `run_act_benchmark.py`（链路基准）。
**正确性方法**：从 `astral_data/act/pick up and place_480` 读真实 episode（observation.state +
video8/video0 视频帧，用 PyAV seek 按需解码，**禁止整段解码——单文件含全部 50 段 ~1 万帧 ≈15GB/相机，
全量解码会 OOM 卡死**，已踩过），喂训练好的 checkpoint 对比录制的 next-state 动作。
**踩坑（关键）**：`LerobotActBackend.infer()` 走 `select_action` 内部 50 行 action 队列——
不清队列时后续 infer 只吐上一 chunk 缓存行、不重新推理。**逐帧评估必须每帧先
`backend.reset()`**（节点运行时此行为正确=ACT 自管节奏，已验证；但逐帧正确性测量会被缓存行
污染，MAE 0.20→0.004）。**结果**：ep0/ep5/ep20/ep46 全 ALL PASS，整体 MAE **0.004~0.005 rad**
（≈0.3°，对齐偏移 0 确认 next-state 语义），与训练 L1=0.025 吻合——**模型能在训练数据上以
亚毫弧度精度复现录制轨迹，推理结果正确**。**链路指标**（250 请求远程推理）：0 失败、
吞吐 198 req/s（1.38MB/请求 → 274MB/s 上行）、稳态单请求 RTT **4.3ms**、冷启动真推理
**201ms**（warm 后 <50ms）、GPU 显存 ~957MiB；节点端到端**实测控制率 29.6Hz**（引擎
pops/4s，~30Hz 达标）、engine last_plan_ms 4.4ms、pause 夹持 Δ=0、resume 重规划、stop→IDLE。
**serve_act 优雅断开**：客户端正常关闭 ws 时 `_handle` 未捕获 ConnectionClosed → 误报
"handler failed" 刷屏，已加 try/except 捕获（0 error）。

**真实部署脚本审计——发现并修复 5 个运行时会踩的坑（本机 4090D 实测）**。
① **陈旧队列跨会话（critical）**：serve_act 的 ACT 后端常驻，`select_action` 的 50 行队列
跨客户端连接残留——节点 HITL 接管或 stop→重启后重新进 POLICY，会先重放最多 ~1.7s（50×33ms）
**接管前的陈旧动作**，可能造成危险跳变。实测：客户端 B 用完全不同的观测连上，第一条响应与
A 会话残留只差 0.0007（拿到的是 A 的缓存行）。**修法**：serve_act 每个新连接建立时
`backend.reset()`（openpi 协议无 reset 消息；节点每次 POLICY 会话正好一个新连接），修复后
B 与 A 残留差 0.1156（新推理）。② **launch 不透传 `camera_image_size`**：`ros2 launch`
部署 ACT 会静默用 yaml 默认 224，与模型 480 不匹配直接崩——launch 补 `camera_image_size`
透传。③ **launch 不透传 `engine_mode` + ACT 必须 queue_sync**：ACT 后端每次返回 1 行
（select_action 队列），queue_async 预取节流把控制率压到 **13Hz**（实测 pops/s），
queue_sync 才是 **29.6Hz**——launch 补 `engine_mode` 透传，文档/yaml 明示 ACT 用 queue_sync。
④ **console script 进 `bin/` 而非 `lib/<pkg>/`**：缺 `setup.cfg`，`ros2 run`/`ros2 launch`
找不到可执行文件（README 里早有的 "No executable found" 坑根因）——补 setup.cfg
（`[install] install_scripts=$base/lib/<pkg>`，与 morla_data_collect 一致），launch 部署验证
通过。⑤ **openpi_client 需手动 PYTHONPATH**：`pip install --user -e openpi-client --no-deps`
（其 numpy<2 约束过旧，numpy 2.2.6 实测兼容）免 PYTHONPATH。另：backend.py 的
"lerobot not importable" 错误改提示 serve_act 路径（py3.10 跑进程内 ACT 的运行时护栏）；
yaml 加醒目注释（本机 ACT 必须远程 + camera_image_size 480）。**验证**：`ros2 launch
... backend_type:=openpi camera_image_size:=480 engine_mode:=queue_sync` + serve_act +
run_act_e2e 全 PASS（控制率 29.6Hz、preflight 30ms、pause Δ=0）。

**远程链路指标补全埋点**——所有指标可观测、有数值。① **服务端分项**：`LerobotActBackend.infer`
加 `last_timing`（prep/pre/infer/post/total ms），serve_act 随响应返回 `server_timing`，
`OpenPiServerBackend` 捕获 `last_server_timing`；② **节点端到端**：node `_policy_tick` 测
`loop_ms`（新观测→指令发布处理耗时）与 `obs_age`（策略作用的真实关节反馈龄期，排除夹爪
自回显污染），入 `/policy_inference/state` 的 `latency_ms`；③ **基准脚本补全**：握手耗时、
本地序列化（vendored msgpack pack/unpack）、网络 vs 模型分离（RTT−server_total）、线缆探测
（小消息 RTT 下限）、RTT 抖动（std/跨度）；④ serve_act 加连接计数（断线/重连观测）。
**实测（本机 4090D，250 请求）**：RTT p50 3.9ms / p95 5.1ms / p99 12.1ms；服务端分项
prep 1.68 + pre 0.22 + infer 1.16（median 0.33，真推理抬均值）+ post 0.07 ≈ total 3.14ms；
**网络+序列化 p50 1.94ms**（线缆下限 0.14ms，大头是 1.38MB 上传）；本地序列化 pack 0.26ms /
unpack 0.04ms；握手 39ms；吞吐 207 req/s；GPU mem ~1GiB util mean 6-10%；**节点端到端
obs_age avg 16.5ms + loop avg 7.9ms ≈ 观测→指令 ~24ms**（受 33ms 控制周期约束）。

**方案 A（ACT 返回完整 chunk）+ 引擎锁饥饿修复——queue_async 达 30Hz、网络降 96%**。**方案 A**：
`LerobotActBackend.infer()` 从 `select_action`（每次 1 行 + 内部 50 行队列）改为
`predict_action_chunk`（一次前向返回完整 n_action_steps 行），引擎拿回分块权；
`temporal_ensemble_coeff` 非 None 时回退 `select_action`（保留时序融合）。**收益**：①
queue_async 从 13Hz 提到 30Hz（引擎按 50 行 chunk 预取）；② 图像上传从"每行一次"变
"每 chunk 一次"→ **网络降 ~96%**（e2e 实测 plans=5/4s=1.25 chunk/s vs 原 30 次/s）；③
节点 loop avg 从 7.9ms 降到 **1.26ms**（引擎本地弹 chunk，无逐 tick 网络）；④ 周期性
200ms 真推理停顿消失（warm infer 仅 8ms，且 queue_async 后台预取）。**对抗性审查发现并修复
引擎锁饥饿 bug（critical）**：`_planner_loop` 把 `self._stop.wait(0.005)` 写在
`with self._lock` **内**——planner 空闲时几乎 100% 持有引擎锁（5ms 等待+立即重获），控制线程
`tick()` 在锁上饿死 → queue_async 即使瞬时推理也只有 ~15Hz。此前被 select_action 的 1 行
chunk（planner 一直在重填循环）掩盖，方案 A 让它进入空闲路径后彻底暴露（真实 ACT 只有
0.5Hz）。**修法**：`Event.wait` 移出锁外（空闲判定在锁内、等待在锁外）。验证：stub 50 行
chunk 15→**4900Hz**、真实 ACT 0.5→**3000Hz**、e2e queue_async **30.0Hz**、loop 1.26ms、
正确性 MAE 0.0036 不变；新增回归 `test_queue_async_control_thread_not_starved_by_planner`
（0.5s >200 pops，修复前 ~7）。基准：3888 actions/s（130× 余量）、RTT p50 15.3ms std 3.5ms、
每 chunk 服务端 10.4ms、GPU util 23%。**部署变化**：ACT 不再要求 `engine_mode:=queue_sync`，
默认 queue_async 即 30Hz（launch 无需传 engine_mode）。

**绝对动作语义守卫（`abs_action_min_scale`）——对抗性审查发现信任边界后加固**。审查结论：
Jetson 侧对两后端（openpi 内部 delta / ACT 绝对）统一按绝对动作处理，delta↔绝对转换完全在
server 层——但节点**原先无任何"返回的是绝对"断言**（信任 backend 契约）。若 openpi server 漏
`AbsoluteActions` 输出变换或错 checkpoint，会静默返回 delta → 机器人到错误目标。**加固**：
引擎 `_check_absolute_semantics`：机器人明显离开零位（max|state|>1，即臂关节）时，chunk 首行
目标值须保持量级（`arm_scale` = 在这些维上 max|action| ≥ 阈值 0.5），否则 `PolicyError` →
引擎报错 → 节点安全 stop（不静默跳到错误目标）。只查 |state|>1 的维（夹爪 state∈[0,1] 自动
排除），接近零位 fail-open。**对抗迭代**：初版用「|action-state| 必须小」判别，被真实模型的
OOD 输入误报（合成 state 让模型预测向均值，|action-state| 达 1.05）；改为「离开零位时动作
量级不得塌缩」后，真实模型在所有真实量级 state 下 arm_scale≈1.7（3.4× 余量）、delta≈0.03
（16× 余量）。集成/e2e 改用真实位姿（含 d3≈-1.9）喂守卫并全部通过；测试合成 OOD 状态禁用
守卫（部署中状态恒在分布内）。新增 3 例单测（拒 delta / 收绝对 / 关闭）。`abs_action_min_scale`
yaml 可配（默认 0.5，≤0 关闭）。单测 85 例全绿。

**独立 ACT 推理脚本 `scripts/act_inference.py`（无 ROS、可移植）**。单文件、**只依赖
lerobot**（`get_policy_class` / `make_pre_post_processors` / `prepare_observation_for_inference`），
不 import 任何 ROS/astral 包——`PYTHONPATH=VLA/lerobot/src` 或任意 lerobot 环境可跑，参考
VLA/lerobot 官方推理路径（`lerobot_eval.py`）。`ActPolicy` 类：load（get_policy_class +
from_pretrained，含方案 A 的时序融合守卫）→ `predict_chunk`（一次前向完整 n_action_steps 行，
绝对量）/ `select_action`（单行，官方逐行路径，独立观测需 reset）；全部参数（action_dim /
image_keys / image_size / n_action_steps / temporal_ensemble）从 checkpoint config.json 的
input/output_features **自动推导**，CLI 可覆盖。三模式 CLI：`--scratch`（合成冒烟）/
`--state`+`--image label=path`（单次推理，可存 .npy）/ `--dataset-dir`（真实数据批量 +
逐帧 reset 对比录制动作 MAE）。**验证**（PYTHONPATH=VLA/lerobot/src）：三模式全通过，ep5/46
MAE 0.0052/0.0036 与正确性脚本一致。**对抗性审查修复**：① `device` 参数与同名 property 冲突
→ 删 property；② preprocessor 用 checkpoint 保存的 device（cuda）→ `--device cpu` 时观测被
pre 搬回 cuda 与 cpu 策略冲突 → 加 `preprocessor_overrides={"device_processor":{...}}` 强制
一致（参考 lerobot_eval.py），cpu 实测通过。

**非 ROS 完整推理包 `runner.PolicyRunner` + `runner_demo.py`**。推理包的编排层（node.py）依赖
rclpy，但核心（backend/engine/executor/controller/replay/robot_io）本就纯 Python——新增
`PolicyRunner` 用**纯线程控制循环**替代 ROS 定时器/话题/回调，复用全部纯核心，覆盖推理包全部
功能：三种引擎节奏、绝对动作 + 语义守卫、SafeExecutor、FSM（POLICY/PLAYBACK/PAUSED/HUMAN）、
回放（h5/v2.1）、暂停恢复重规划、HITL 接管交还、延迟/引擎指标。**I/O 可插拔**：`feed_observation
(state, images, prompt)` 喂观测（任意频率）、`on_action(row)` 收**安全处理后的完整绝对动作行**
（你下发真机/仿真）、`on_acquire_control/on_release_control` 镜像 disarm 电平门（外部遥操可接）、
`request("policy|playback|pause|resume|takeover|release|stop")` 命令。**真机/无 ROS 都能跑**：
demo 用真实 ACT 进程内后端（lerobot 环境，无需 serve_act）全链路 PASS——policy 30Hz →
pause 夹持 → resume 重规划 → takeover HUMAN 静默 → release 回 POLICY → playback 回放 →
stop；stub 后端（任意 python，无模型）同样全 PASS。**对抗性审查修复**：① `_resend_last` 把
逐流 CmdTarget（7/1 维）发给 on_action → 改为重发最近一次完整行；② `on_action` 给原始行而非
安全层处理值 → 按 schema 块序把 clip/slew 后的流拼回完整行再回调（消费者拿到实际下发值）；
③ 控制率测量用 on_state tick 计数（避免 resend 干扰）；④ demo 断言按 FSM 实际语义修正
（release 回到被打断的活动）。

## 2026-09-07

**左手柄 X 键「段间回位」+ web 数采卡片同功能按钮（episode 间免手摆放物品）**——
`morla_bringup`（新 `controller_workpos_gate` 闸门）× `morla_web_manager`（数采卡片按钮）×
`astral_mujoco_sim`（sim launch 同启）。**动机**：单人数采的段与段之间，人要离开手柄去重新摆放
物品——需要一只手柄按键"停止跟随 VR 并回到工作位"；web 侧同功能按钮方便不戴头盔时操作。
**做法**：①新闸门节点镜像 `controller_start_gate`（grip→start）对称——左手柄 X（primary,
`buttons[0]`，此前完全空闲）按下沿 → 发 `/teleop/disarm` + `/teleop/init`（即 web「工作位」的
信号序列：停止跟随 + 沿 init_waypoints→init_pose 回位，臂侧接口全现成，零改动）；②**录制保护**：
闸门订 `/data_collect/state`（latched），RECORDING/PAUSED/SAVING 时按 X 忽略并节流告警（防手臂
回位毁掉正在录的段）；无数采节点运行（纯遥操）时照常生效；③`full_teleop.launch.py` 无条件接入、
sim pipeline 随 `with_start_gate` 接入；④web 数采卡片按钮组加「段间回位」（`POST
/api/v1/teleop/workpos` 复用既有端点），仅遥操 RUNNING/PAUSED 且**非录制中**可用（录制保护与 X 键闸门一致）+ confirm 弹窗，键位小抄补
"左手柄 X"。纯决策拆 `controller_workpos_logic.py`（无 ROS）。**对抗性审查发现并修复**：①
**disarm/init 跨话题无保序竞态**——`/teleop/disarm` 会取消进行中的 homing（arm_teleop 刻意行为），
背靠背连发可能"先收 init 启动回位、再收 disarm 取消回位"致 X 静默失效；闸门改为 **disarm 立即发 +
0.1s 一次性定时器发 init**（镜像 web workpos 的 disarm→sleep(0.1)→init 时序），连按 X 取消旧待发
init。②depth=1 BEST_EFFORT 订阅连发不消费时中间跃迁被最新帧覆盖（真实 DDS 行为）；③
**rclpy 同进程 intra-process 双投**：闸门在 spin 回调内发布时同进程订阅会收到 2 份
（intra 快路径 + DDS 环回各一，~6ms 间隔）——纯测试环境怪癖，非生产缺陷（跨进程单收）。
**验证**：纯逻辑 pytest 10 例 + 节点集成 6 例（rclpy，本机 ROS humble 实测**16/16 全绿 ×2 连跑**；
节点测试用独立 client 捕获节点 + 直接驱动 `_on_joy`（回调内发布会双投，主线程调用单投，确定性），
含 disarm 先于 init 的时序断言、连按取消待发 init）；`colcon build morla_bringup` 后**跨进程
DDS 冒烟 ×3 稳定 PASS**（子进程精确 PID 启停，零残留）——独立闸门进程 + rclpy 客户端：
IDLE+X → disarm×1+init×1（0.1s 间隔）、RECORDING+X → 忽略并节流 WARN；前端 `npm run build`
（tsc+vite）通过；smoke_test.sh 加 workpos 端点探测（遥操停止时 409 非 404）。**待办（机器人
侧）**：实机冒烟——按 X 回位 → 再 grip 重标定开始下一段；录制中按 X 确认被忽略。

**遥操完整启动命令 + 操作步骤文档化并全树同步**——`morla_bringup/README.md`（主）× 顶层
`CLAUDE.md`（采集→训练速查）× `morla_web_manager/README.md` × `morla_data_collect/CLAUDE.md`。
**动机**：用户要求把"完整遥操启动（web + CLI 两条路径）与遥操步骤"写成文档，一处权威、各处引用。
**做法**：①`morla_bringup/README.md` 新增「完整遥操启动（web/CLI）」+「遥操操作步骤（数采 episode
循环）」两节——web 路径（起监控 → 预设启动 → 数采卡片启节点/设目录/任务 → 工作位）、CLI 路径
（单左臂/双臂/双夹爪/仿真/数采五条命令）、七步 episode 循环表（grip→A→任务→B→X→摆物→grip）、
键位速查（VR/web/键盘/CLI 四端等价）、顺序要求（先 grip 再 A 防 W5；B 后等 IDLE 再 X；回位完再
grip）、安全收尾（急停/暂停/HOME）；②顶层 `CLAUDE.md` 采集→训练速查加遥操启动两条路径 + episode
循环摘要；③`morla_web_manager/README.md` 启动节后加「完整数采流程」指引；④`morla_data_collect/
CLAUDE.md` 新增「数采完整流程」节（含 X 与数据的关系：`_accepting` 门控 + `dq.clear` + 录制拦截
三保险）。**验证**：文档与代码契约逐条核对（命令与 presets.yaml/launch 参数一致、键位与
vr_collect_logic/controller_start_gate/workpos_gate 一致）；无代码改动，无需跑测试。

**段间回位改直达（X / 数采卡片「段间回位」不经途径点）+ 工作位保持途经点**——
`astral_arm_teleop` × `morla_bringup` × `morla_web_manager`。**动机**：真机测试后用户要求——
段间回位（左 X / 数采卡片）现在先回 init_waypoints 再到 init_pose，改为**直接**关节空间直线
插补回工作位（不经途经点）；而系统 tab「工作位」按钮保持途经点路径。**做法**：①臂节点新增
`/teleop/init_direct` 订阅 + `~/init_direct` 服务，`_go_init(direct=True)` 时 `_homing_path` 仅
`[init_pose]`（直达），`direct=False`（原 `/teleop/init`/工作位）保持 `init_waypoints → init_pose`
——两路共用同一 homing 慢速轨迹机（限速/限位/到点判定相同），disarm→0.1s→init 时序沿用；
②`controller_workpos_gate` 的 `init_topic` 默认改 `/teleop/init_direct`（X 走直达）；③web 新增
`POST /api/v1/teleop/workpos/direct`（disarm→0.1s→发 init_direct），数采卡片「段间回位」按钮改调
直达端点，系统 tab「工作位」保持 `/api/v1/teleop/workpos`（途经点）；④`config.py` 加
`TOPIC_INIT_DIRECT`、`monitor_node` 加 `publish_init_direct`、client.ts 加 `teleopWorkposDirect`。
**验证**：新增臂节点级测试 `test_arm_teleop_node_init.py` **7/7 全绿**（直达路径=1 点=init_pose、
途经点路径=途经点+init_pose=2 点、回调触发、level guard、homing 中拒绝、只读参数 init_waypoints
经 `rclpy.init(args)` 构造期注入）；morla_bringup **16/16**（闸门节点测试改订 `/teleop/init_direct`）；
跨进程 DDS 冒烟 **×3 PASS**（IDLE+X → disarm×1 + init_direct×1；RECORDING+X → 忽略；零残留）；
前端 tsc+vite build 通过；`colcon build morla_bringup astral_arm_teleop` 后冒烟。**待办（机器人
侧）**：同步构建三包后实机冒烟——X 直达回位、工作位途经点回位、录制中按 X 忽略。

**数采卡片「本次采集数据」实时下拉窗口（实时看采了多少、在哪个文件夹）**——
`morla_data_collect` × `morla_web_manager`。**动机**：用户要求——设定目录/任务旁加一个可实时
查看本次采集数据量的下拉窗口，实时看"采了多少数据、在哪个文件夹采的"。**做法**：①采集节点
`/data_collect/state`（1Hz 定时器 + 控制事件即时补发）新增 5 字段——`folder`（完整目录路径
`save_root/session`）、`episode_counts`/`camera_counts`（**当前段已写精确样本数/每相机帧数**，
来自写盘器 `counts()`，不受 HDF5 chunk 预分配取整影响）、`episode_bytes`/`session_bytes`
（**磁盘占用**，含预分配——语义为"占了多少磁盘"；session 累计**增量累加**，段结束 close 时加、
切 session 归零，避免 1Hz 全目录重扫）；②前端 `types.ts` 加字段、`DataCollectCard` 设定目录/任务
下方加可折叠「本次采集数据」面板（点标题展开收起，折叠态也带 session 总 MB 实时徽标；展开显示
文件夹路径 / session 累计 / 当前段时长·样本·帧·磁盘 / 每流每相机明细）。**验证**：
`test_node_guards.py` 29→**30 例全绿**（新用例覆盖：IDLE 空计数、录制中精确样本/帧数、磁盘占用
>0、session 累计=当前段、切 session 后 folder 变 + 累计归零）；data_collect 节点套件 30/30；
前端 `npm run build`（tsc+vite）通过；py_compile 通过。**待办（机器人侧）**：同步构建
`morla_data_collect` + 前端 dist 后实机冒烟——录制中下拉实时增长、文件夹路径正确。

## 2026-09-04

**AV1 并行度可配：`ASTRAL_AV1_LP` 环境变量（默认 2 保内存红线，20 核机可放开）**——
`morla_data_collect`。**动机**：用户问"为什么转换这么慢"——实测 20 核机器只用了 2 核
（SVT `Level of Parallelism: 2`，内存红线），AV1-224 ≈78 帧/秒、720 ≈8 帧/秒；要求针对本机
放开核数。**做法**：`encode_video` 的 `lp=` 从硬编码改为 `_svt_av1_params()` 读
`ASTRAL_AV1_LP`（默认 2；非法值回退 2；>2 时 stderr 提示 ~0.6GB/lp 峰值内存），
`lookahead=16` 不变；所有入口（脚本/模块直调）自动生效。**验证**：新增
`test_svt_av1_params_env_override`（默认 lp=2 / 放开 lp=8 / 非法回退）通过；CLAUDE.md
不变式 6 与 README §5 补说明。**注意**：正在运行的后台转换进程不受影响（旧代码已加载），
配置对下次运行生效；本机当前 IDE 占 10GB（可用 ~4GB），放开建议 lp≤4（~2.4GB），
要 lp=8 需先关 IDE/opencode。

**convert_to_act 分辨率选项化：224/480/720/原生（0）+ 原生同 shape 预检**——
`morla_data_collect` × `scripts/vla_process_act.sh`。**动机**：用户要求 ACT 转换输出分辨率可
选（224/480/720/原生）。机制本已存在（`--image-size` 任意整数、0=原生），真正的坑是**原生模式下
各相机分辨率不同**（video8=1080p、video0=720p）会破坏 ACT 的"全部相机同 shape"要求，此前要到结构
自检才报通用错。**做法**：①`convert_to_act` 新增 `_probe_native_shapes`——原生模式（image_size=0）
转换前先解各相机首帧 JPEG 探测分辨率，不一致即 `ValueError` 并提示改用 224|480|720 或统一采集
分辨率；②`run_pipeline` 内部把 `image_size==0` 归一化为 `None`（此前只有 CLI 路径做了，直连调用会
漏判）；③help/docstring/脚本注释写明 224 默认/480/720/0=原生选项。**验证**：`test_convert_act.py`
6→**8 例**全绿——新增 原生不一致提前报错（注入 32×32 vs 64×48 两相机）、480 端到端（结构自检 0
issue + 产物视频实测 480×480）；`bash -n` 通过；README §5b 补分辨率选项说明。真实数据 480 全链路
+ conda lerobot 深度自检后台运行中（5 段 2 相机）。

**数据目录三文件夹重组（raw/ pi/ act/）+ 剔除 video2 + 清理旧转换产物**——
`astral_data` × `morla_data_collect` × `scripts/`。**动机**：用户要求——不要 video2（原生+转换都
不要）、删掉之前几次转换产物、统一到 astral_data 下建 原始/pi/act 三个文件夹、后续按 session 各进各的。
**做法**：①5 段 raw（061-065）剔除 `camera_data.h5` 的 video2 组 + meta.json schema.cameras 同步为
[video8,video0] + 删旧 3 相机 aligned_data.h5（对齐按新 schema 重建）；②删除全部旧转换产物
（astral_data_lerobot/v3、astral_data_act/v2、default_task_openpi/act 共 6 目录）；③建三目录
`astral_data/{raw,pi,act}/`，raw 移入 `raw/default_task/`；④`data_collect.yaml` `save_root` 改
`~/astral_data/raw`（未来采集直接落 raw/{session}）；⑤README §4 加目录约定块、§2 save_root 行、
§5b 示例路径、包/顶层 CLAUDE.md 同步。**验证**：从新 raw（2 相机）端到端重建——
`vla_process_openpi.sh raw/default_task pi/default_task`（v2.1, `cams=2`）与
`vla_process_act.sh raw/default_task act/default_task --check-python lerobot`
（v3, conda lerobot 深度自检 `OK episodes=5 frames=1391 cams=[video0,video8]`）全绿；
全树 `find -iname "*video2*"` 零残留。**遗留**：`astral_data/default_task_ep58-65.tar.gz`（ep58-65
备份包）与 `~/astral_data`（home 根旧 junk：default_task quarantine + verify_*）未动，确认后另清。

**openpi/ACT 一键脚本拆分：`vla_process_openpi.sh` + `vla_process_act.sh`（同 raw、独立输出）**——
`scripts/`。**动机**：用户要求"处理脚本与 sh 脚本都要清晰：从同一个采集原始数据文件夹来，
openpi 和 ACT 分开，转换后的文件夹也分开"。**做法**：删掉合并的 `vla_process_session.sh`，
拆成两个脚本，**入参都是同一个 raw 会话目录**，输出独立文件夹（建议 `<session>_openpi` /
`<session>_act`）：①`vla_process_openpi.sh <raw> <openpi输出>`——对齐→校验(默认隔离)→转 v2.1
（openpi 直接消费）；②`vla_process_act.sh <raw> <act输出>`——对齐→校验(默认隔离)→
`convert_to_act`（v3 + 两级自检），带 `--check-python`/`--keep-v21`/`--overwrite`；两脚本共享
同样的"坏段默认隔离 + 隔离后中止"门禁。引用同步：`openpi_train.sh` 提示、包 CLAUDE.md/README
一键脚本段、`convert_to_act` docstring。**验证**：两脚本 `bash -n` 通过；**同一 raw**
（default_task，5 段 1391 帧）端到端各跑一遍——openpi → `default_task_openpi`（v2.1，5 段）、
act → `default_task_act`（v3，conda lerobot 深度自检 `OK episodes=5 frames=1391`）全绿。

**convert_to_act 增 `--v21-root` 快速入口 + vla_process_session.sh 的 --act-output 改走 convert_to_act**——
`morla_data_collect` × `scripts/vla_process_session.sh`。**动机**：用户问"keep-v21 中间产物是什么、
与 openpi 转换什么关系、脚本同步改"——明确 openpi/ACT 共享 raw→v2.1 中间层：openpi 直接消费 v2.1，
ACT 在 v2.1 之上升版 v3 + 自检。**做法**：①`convert_to_act` 输入改为 `--session`/`--v21-root` 二选一
（`--v21-root` 跳过对齐/重编码，直接升版+自检，秒级）；②`vla_process_session.sh` 的 `--act-output` 从
直接调 `convert_to_lerobot_v3` 改为调 `convert_to_act --v21-root $OUTPUT`（复用本步 v2.1，不重复编码），
新增 `--act-check-python` 透传深度自检，`--overwrite-v3` 透传为 `--overwrite`；③README §5b 补 v21-root
用法。**验证**：`test_convert_act.py` 4→**6 例**全绿（新增 v21-root 复用路径、--session/--v21-root 互斥）；
离线回归 59 例全绿；**真实端到端** v21-root 复用现有 v2.1 → `/home/robot/loopkok/sdk/astral_data_act_v2`
仅 **7.6s**（对比全链路重编码 ~2min），conda lerobot 深度自检 `OK episodes=5 frames=1391 cams=[video0,video2,video8]`；
`bash -n` 语法通过。

**新增 ACT 专属转换 `convert_to_act.py`：raw 会话目录 → 官方 lerobot ACT 可训数据集（内置两级自检）**——
`morla_data_collect`。**动机**：用户要求"转 ACT 不要写通用 v3 升版，要专门的 ACT 转换，且保证无缝接
官方 lerobot ACT 训练"。**前提澄清**：官方 `lerobot-train --policy.type=act` 的输入格式就是 LeRobot
v3（无独立于 v3 的 ACT 专用格式），所以"专属"体现在**硬保证 + 自检**而非新格式。**做法**：新模块
`convert_to_act.py`——输入 raw session（含 episode*），内部复用 align_data → convert_to_lerobot(v2.1,
临时目录, --keep-v21 可留) → convert_to_lerobot_v3 → **结构级自检**（stats.json 含
observation.images.{每路}+state+action 的 mean/std（ACT VISUAL/STATE/ACTION MEAN_STD 硬需求）、
各相机同 shape、tasks 非空、parquet+视频可读可解码，不过即退出）；**深度级自检**
（`--check-python <现代lerobot python>`）：用该解释器真装载 `LeRobotDataset` + 构建 ACT 预处理管线 +
逐帧解码。输出 = 官方 v3 布局，脚本结尾打印 `lerobot-train --dataset.root=... --policy.type=act` 命令。
**验证**：`test_convert_act.py` 4 例全绿（全链路转出布局关键件齐 + 结构级自检 0 issue + 深度自检源码
可编译、stats 缺图像统计负例报错、tasks 空负例报错、无 --overwrite 拒绝覆盖输出）；离线回归 35 例全绿；
**真实数据端到端**（default_task 5 段 1391 帧）→ `/home/robot/loopkok/sdk/astral_data_act`，conda
lerobot 深度自检输出 `OK episodes=5 frames=1391 cams=[video0,video2,video8]`——现代 lerobot 真装载 +
预处理管线构建成功，即"无缝衔接官方 ACT"的实测证明。

**数采 web 端可运行期切换录制目录（session）——换目录不再重启节点**——
`morla_data_collect` × `morla_web_manager`。**动机**：web 泳道 launch 把
`session:=default_task` 写死，所有段都进 `~/astral_data/default_task/`——用户按 A 录完
几段后在自己的 `~` 下"找不到数据集"（其实都在 default_task 里），换目录只能改 launch
重启节点（pending task 也丢）。**做法**：① 节点新增 latched 话题 `/data_collect/session`
（与 `/data_collect/task` 同模式）——目录名安全校验（非空/无路径分隔符/非 `..`/不以
`.` 开头/≤64 字符），**仅 IDLE 生效**（录制/暂停/保存中拒绝并计入 `ignored` 的
`set_session`）；切换后 state 立即 latched 重发（卡片头部 session 实时更新），
**meta.json 新增 `session` 字段**（段级溯源，离线侧无视）；launch/yaml 的 session 仍是
初始值（重启回落 default_task）。② web 桥：`DC_TOPIC_SESSION` → monitor 发布器
`publish_dc_session` → REST `POST /api/v1/collect/session {text}` → `api.collectSession`；
卡片任务文本同排新增「录到目录（当前: xxx，仅空闲可换）」输入 + 「设定目录」按钮
（成功了 toast、输入清空）。**验证**：`test_node_guards.py` 12→**14 例**全绿——IDLE
切换后段落新 session 目录（段号独立从 000000 起）且 meta.json 带 session、录制中切换
与 6 种非法名（a/b、a\b、..、空、.hidden、65 字符）被拒且计入 ignored、IDLE 再切成功；
`npm run build`（tsc+vite）通过 dist 重建；后端 py_compile 通过。机器人侧需同步
`morla_data_collect`（节点话题）与 web dist。

**工作位/HOME 后夹爪无响应——latched disarm 把 pinch 仲裁门关死，「开始遥操」补发 /teleop/armed 开门**——
`morla_web_manager` × `morla_bringup`。**症状**：no-right-arm 预设 → 点工作位 → 开始遥操，
手柄对夹爪无响应（此前正常）。**根因**：工作位/HOME 流程发 **latched** `/teleop/disarm`
（门控语义），pinch 仲裁门只认 `/teleop/armed=true` 重开；而「开始遥操」（web 按钮 / 左手
gripClick 闸门）只发 `/teleop/start`，从不发 armed——点过工作位后门永久关死。**做法**：
① web `POST /api/v1/teleop/start` 先发 `/teleop/armed=true` 再发 start（臂节点未校准会忽略
armed，无害）；② `controller_start_gate`（gripClick 入口）同步一并发 armed（新参数
`armed_topic` 默认 /teleop/armed）。验证：py_compile、web monitor 4 例全绿。

**单臂预设下缺席臂被拽向零位——driver 缺侧"补零"改为只发有指令的一侧**——
`astral_robot_control`。**现象**：web 预设选 no-right-arm（单左臂）启动后点「工作位」，
**右臂也抽了一下**（右臂不在预设里、停在某非零位姿，却被命令拽向零）。**根因**：driver
控制定时器发现"左臂有新鲜指令、右臂 None"时给缺失侧**补零**（`right=[0.0]*7` 后
`move_arm_js(left, zeros)`）——工作位/HOME/遥操一发流（150Hz），缺席侧实体臂就被 100Hz
零目标持续拽向零位；双臂模式平时两侧都在发所以不显形。**做法**：控制定时器 arm-only 路径
改为**只向有新鲜指令的一侧下发**——双侧都新鲜仍走 `move_arm_js` 一次下发（不变）；仅单侧
新鲜走新 `_send_arm_side()`（SDK `set_target_positions` 按该侧电机 ID 子集下发，与
`move_arm_js` 同一 0x90 指令通道）。缺席侧不发命令 = 板端位置保持维持原目标（原位保持），
与预设搭配：no-right-arm 时右臂不再被任何左臂轨迹打扰。**验证**：`test_driver_services.py`
16→**20 例**全绿——新增控制定时器 4 例：左新鲜只发左电机 ID 集（无 move_arm_js）、右新鲜
只发右、双侧新鲜仍 move_arm_js 双发、无新鲜零下发。README 话题契约补"缺侧不补零"。

**fix：monitor 节点补 import `TOPIC_INIT`——上条「工作位」条目遗留的启动 NameError**——
`morla_web_manager`。**症状**：monitor_node `__init__` 里 `create_publisher(Bool,
TOPIC_INIT, ...)` 引用的常量未在 import 区导入（上条改动只加了 config 定义与
`publish_init` 调用，import 列表漏 `TOPIC_INIT`），monitor 节点启动即抛
`NameError: name 'TOPIC_INIT' is not defined`，web 整体不可用。**做法**：import 区补
`TOPIC_INIT`（config.py 中已定义，无其他缺失）。**验证**：模块可导入（rclpy 环境）。

**web「工作位」按钮：启动不再自动归位，回初始位改手动触发**——`astral_arm_teleop` ×
`morla_web_manager`。**动机**：web 启动会拉起节点并自动走 init_waypoints → init_pose，
机器人动不动不该由"启动栈"决定，且操作员常要先把臂摆开/上电再决定回位时机；改为启动只
拉节点、回工作位由按钮显式触发。**做法**：① 节点新增 `_go_init()`（与 `_go_home` 对称：
disarm → init_waypoints **正序** → init_pose，启动同款 init 轨迹机，不加 park 门控），
入口 = 全局一次性 `/teleop/init` + 单臂 `~/init`（VOLATILE，迟到不触发）；`_finish_homing`
init arrived 置 `_at_init_pose` 标志；② **`/teleop/start` 守卫**：臂不在工作位时先
`_anchor_origin_to_measured()`（从 `_reanchor_teleop` 提取的共享重锚 helper）把原点锚到
当前实测再 arm——从任意位姿 start 不再向启动位 FK 锚点跳变，遥操纯增量开始；③ 左右 yaml
`move_to_init_pose: false`（注释说明；CLI 需要可 `:=true` 覆盖）；④ web：`POST
/api/v1/teleop/workpos`（disarm + 0.1s + 发 `/teleop/init`，**不调 `~/enable`**——按用户
要求）＋ SystemTab 预设卡片「工作位」按钮（sky 色，HOME 旁，confirm 弹窗）；前端重建
dist。**验证**：`test_home_park.py` 16→**21 例**全绿（新增 _go_init 正向路径构建/disarm、
busy 拒绝、init arrived 置位/超时与 park 不置位、start 不在工作位时重锚+告警、在工作位时
不重锚）；reanchor 5 例全绿（共享 helper 重构回归）；web monitor 4 例、py_compile 通过。
README（arm_teleop 工作位节 + web REST/话题表）与 CHANGELOG 同步。机器人侧需重新
colcon build（astral_arm_teleop + morla_web_manager）。

**HOME/启动途经点调整：init_waypoints 改远摆位 [-1.6, ±0.2, 0, -1.92, ∓0.2, 0, 0]**——
`astral_arm_teleop` 配置。**动机**：HOME 收回时臂到途经点的摆臂动作不明显（原途经点
[-1.0, 0, 0, -2.2, 0, 0.46, 0] 与 init_pose 的 j1 只差 0.65 rad），实机确认需要更明显的
"到过折叠途经点"行程。**做法**：左右 yaml `init_waypoints` 改为：左
`[-1.6, 0.2, 0.0, -1.92, -0.2, 0.0, 0.00]`、右按 init_pose 镜像规律（j2/j5 反号）
`[-1.6, -0.2, 0.0, -1.92, 0.2, 0.0, 0.00]`。数值全在 URDF 限位内（J1 -1.6 < ±2.0、
J4 -1.92 < [-2.26, 0]），不会触发 clip。**影响**：该参数同时作用于启动 init 与 HOME park
（同一 `_parse_init_waypoints`）；HOME 段 init_pose→途经点 j1 差 0.65→**1.25 rad**，
行程 1.05→**~2.1 s**（摆臂更明显）；w1→零 j4 1.92 rad（原 2.2）略快。**机器人侧需重新
colcon build 生效**（astral_arm_teleop 的 yaml 非 symlink 构建为拷贝）。

**日志降噪：夹爪仲裁门 disarm 提示只打状态变化；capture driver-side fps 稳定时不再 5s 刷屏**——
`astral_gripper_teleop` × `quest3_video_streamer`。**动机**：HOME/暂停期间夹爪仲裁门长期关闭，
pinch 节点每秒打一条 "disarmed by arbitration gate"（3 相机 x 5s 窗口的 capture fps 同理），
正常状态刷屏。**做法**：① pinch 新增 `_set_gate()`——门状态**变化**才打日志（进入 disarm 打
"not publishing (waiting for open)"，重开打 "publishing resumed"），删 1s 节流重复；②
`webcam_source._capture_loop` 的 driver-side fps 改为 5s 统计窗口 + **偏差 >15% 才报**（掉速/
降级立即可见）+ **≥60s 心跳保底**（证明线程存活），稳定满帧不再刷屏。**验证**：两文件
py_compile；streamer 纯测试 24 通过（1 例环境性失败：uv venv 无 aiortc，与本次无关）。

**HOME 途经点放行日志加"实测距途经点"误差**——`astral_arm_teleop`。HOME/init 途经点放行的
`Via N/N reached` WARN 追加实测关节距刚放行途经点的 max 误差（`.3f rad`），实机核对
"实体到没到过途经点"无需再看动作——日志直接给数字。`test_home_park` 放行断言同步。

**HOME 归位仍"没到过 init_waypoints 姿态"——实机日志定案：门限秒放行 + 只擦过不停下；改"到位 + 稳定驻留"放行**——
`astral_arm_teleop`。**现象**（实机 HOME 日志）：`Homing 3 segment(s)` 路径含 w1 无误，但
`Via 1/2`(t=0.01s) → `Via 2/2`(t=1.06s)——命令 1.05 s 走完 init_pose→w1 后**立即反向**，
无任何驻留；用户观察：臂"先到另一处再回零"，从没真正到过 w1（[-1.0,0,0,-2.2,0,0.46,0]）
的关节姿态。**根因**：上一版门限 = "实测进入 `homing_via_tol`(0.12) 即放行"太松且不判停——
实体 0.6 rad/s 运动中，命令到 w1 瞬间实测恰已擦过 0.12 边界 → **秒放行**；臂刚擦过 w1
（差最多 ~7°）就掉头下零，位置保持从未在 w1 停住。**做法**：放行条件改为**实测进入 tol
且连续稳定 `homing_via_settle_s`（新参数默认 0.5 s）**——命令全程钉在途经点，实体在位置
保持下真到位停下；运动中擦过 tol（err 再出 tol）连续计时清零重来；`homing_via_hold_s`
改为途经点总等待上限（默认 2 s），实测流不可用时按纯命令驻留 settle 时长放行。启动 init
不加门限不变。**验证**：`test_home_park.py` 14→**16 例**全绿（新增/改写：到位后不满
settle 不推进、中途离开 tol 清零重计、实测流缺失按命令驻留放行；hold 超时放行保留）；
闭环滞后模拟：实体距 w1 最近距离 0.033-0.047 rad→**0.000-0.025 rad**，命令在 w1 驻留
0.02 s(1 拍)→**0.52-0.68 s**（τ=0.05-0.35 s 全档）；reanchor 5 例全绿。左右 yaml 补
`homing_via_settle_s`，README 同步。机器人侧重新 colcon build 生效。

**数采卡片加「动态状态提示行 + 操作键位/门控小抄」**——`morla_web_manager` 前端。
**动机**：卡片此前只给按钮不给"当前状态能做什么/为什么按键没反应"的语境——VR 摇杆按下=丢弃
（删文件不可逆）等门控语义只写在 README，操作者在 IDLE 下误按摇杆、SAVING 期按 A 只会看到
"没反应"。**做法**：①按钮行下方动态提示行（online 显示，按 state 出文案）——IDLE：可开始、
段号接续与复用说明；RECORDING：可停止保存/下一段/暂停/丢弃 + VR B=停止保存、摇杆=丢弃不可逆、
非法状态按键被忽略；PAUSED：缓冲已落盘可继续/停止/丢弃；SAVING：收尾写盘中按键会被忽略计入
「忽略指令」勿急（左侧琥珀色竖条强调）；②卡片底部键位小抄——VR A/B/摇杆 + 键盘
s/q/d/n/p/t 的键位与各自合法状态、门控忽略语义、"要确认弹窗的破坏性操作用卡片按钮"。
**验证**：`npm run build`（tsc+vite）通过，dist 重建（index-Dxq7FFA1.js）。

**VR 采集控制改键：A=开始 / B=停止保存 / 摇杆按下=丢弃**——`morla_data_collect`。
上一版键位（摇杆=start / A=next）实机试用手感不合，用户改配：**A 键(IDLE) → start；
B 键(录制中) → stop&save；摇杆按下(录制中) → discard**（删除当前段并回 IDLE，语义同
键盘 `d`）。上升沿 + 状态门控机制不变（同帧多键：低号键优先、非法跳过）；`discard` 是
破坏性操作（删文件无确认），门控保证 IDLE 下误按摇杆不动作。**验证**：`test_vr_collect_control.py`
重写 12 例全绿（A/IDLE=start、摇杆/RECORDING=discard、B 不变、三态门控、同帧 A+B→start/
stop 与 摇杆+A→discard 的优先级、长按不重复）；`test_vr_collect_node.py` 集成 3 例全绿
（IDLE: A→start、B/摇杆门控；RECORDING: 摇杆→discard、B→stop、A 门控）。README/CLAUDE.md/
launch 注释同步新键位。

**遥操加"硬跟人肘"模式（human_elbow_mode=hard）+ astral_pim_ik 录制转换器——"训练臂角复现人手姿态"链路打通**——
`astral_arm_teleop` × `astral_pim_ik`。**动机**：用户要最终训练出的臂角符合人手 Quest3 增量遥操姿态
（无肘跟踪也运行）；遥操原本的 `use_human_elbow` 只是 ψ_human 软先验（w=2.0 与 w_vel=1.0 连续性混合
+ 局部窗/逃逸迟滞），肘"半跟人半跟惯性"，正是"遥操姿态怪"的主要嫌疑。**做法**：① teleop
`GeometricIKSolver` 新增 `solve_hard(T, ψ)`——精确解在人臂角上（`_collect_solutions(T,ψ)` + 最小
warm-start 选支，无网格/1D QP/逃逸；镜像 pim_ik 的 `GeometricArmAngleSolver.solve`，因
pim_ik→teleop 依赖不可反向），成功同步连续性 state 保证回退平滑；② 节点新参数
`human_elbow_mode: soft|hard`（默认 soft 保兼容，左右 yaml 置 **hard**），人肘方向新鲜且伸直
门控通过时走 `solve_hard`，不可行/奇异/超龄期自动回退软路径，`[Latency]` 行加 `hard_follow`/
`hard_fallback` 计数，参数热改可用；③ pim_ik 新增 `convert_sessions.py`——遥操录制
`robot_data.h5` 的 `{side}_arm_state`（实测 q）→ FK 得 T_ee、`psi_from_config` 得标签
（schema 同 `generate_dataset`），meta pause/resume + 帧跳变 + 超 URDF 限位帧自动断段
（本机 verify/default_task 存量段全是等差假数据 q4 超限，已被正确拒绝），train/val **按整段会话
留出**（`--val_sessions`）；eval 加 `--no_split`。**验证**：`test_geometric_ik` 10→**11 例**
全绿——hard 保真：双臂各 12 样本肘方向偏差 L 0.0118°/R 0.0002°、位姿 <0.05mm、不可达→None、
确定性；pim_ik `test_deterministic_solver` 8→**9 例**——teleop `solve_hard` ≡ pim_ik `solve`
**逐位 0.00e+00**（双臂 25 样本）。整链排演（合成会话 → convert → 8 epochs 训练 →
`--no_split` 整段评估）跑通。**硬模式是录制前提**：执行的 q 的臂角 == 喂入的 ψ_human，故训练
标签零额外录制流。真机 runbook 见 pim_ik `CLAUDE.md` §7.1。机器人侧需重新 colcon build 生效。

**VR 采集控制：右手柄 摇杆按下=开始 / A=下一段 / B=停止保存**——`morla_data_collect`。
**动机**：单人采集时双手都在遥操，录段控制（开始/分段/停止）要伸手去键盘或 web，打断操作
节奏。**做法**：新增 `vr_collect_logic.py`（纯决策：上升沿 + 状态门控，无 ROS 可离线单测）
+ `vr_collect_control.py` 节点——订 `quest3/right_controller_joy`（mocap 已发布，
buttons=[primary(A)/secondary(B)/stickPress/...]）与 `/data_collect/state`（latched），
**摇杆按下(buttons[2]) 上升沿 + IDLE → start；A(buttons[0]) + RECORDING/PAUSED → next；
B(buttons[1]) + RECORDING/PAUSED → stop**。长按不重复；状态不合法/采集未运行（未收到
state）时按键静默忽略（info 日志说明），不给采集节点发无效命令刷 warning；与 web 卡片/
键盘控制器三面并存（离散一次性命令，采集节点状态机幂等）。随 `data_collect.launch.py`
同启（`vr_control:=false` 可关，同 keyboard 参数模式）。**验证**：新增 `test_vr_collect_control.py`
纯逻辑 12 例全绿（上升沿/长按不重复/释放再按/三种状态门控/state 未知/同帧多键/短数组不越界）；
`test_vr_collect_node.py` rclpy 集成 3 例全绿（真实话题契约：latched state 门控 → control
命令序列 start/next/stop、录制中摇杆被门控、长按不重复）；node_guards 9 例、离线套件 30 例
回归全绿。README/CLAUDE.md 控制面与文件地图同步（三面→四面）。

**HOME 归位"出了但没到 waypoint 就掉头"修复：途经点加实测到达门限 + park 路径改倒序**——
`astral_arm_teleop`。**现象**：点 web 紫色 HOME 能正常回到零点，但途经点与启动移动不成反向——
预期先到 init_pose、再经 init_waypoints、再到零；实际**伸向 waypoint 但没到位就转去零位**
（用户确认"出了但没到就掉头"），且**经过位置与启动路径不是反向**。**根因**（两个独立缺陷）：
① **切角**——park 轨迹纯开环，命令 q_cmd 领先实测（板端跟踪误差+链路延迟）；途经点推进只看
命令误差（<`init_arrive_tol` 0.05 即走下一段），命令一到点立刻 180° 反向，实体会被"切角"在
waypoint 之前掉头（waypoint 是防刮桌安全走廊位姿，切角 = 走廊失效）；冻结守卫只兜 >0.25 rad
的失能/堵转，0.05~0.25 的常规滞后静默切角。② **顺序**——`_go_home` 把 init_waypoints **正序**
拼在 init_pose 与零之间，而 README 明示收回应**倒序逐点**（启动路径的反向）；单一途经点配置
（当前左右臂 yaml 各 1 个）掩盖了顺序错误。**做法**：① park 途经点推进改双条件——命令到点后
**钉住重发，等实测也进入 `homing_via_tol`（新参数默认 0.12 rad）再推进下一段**；实测流不可用
或等满 `homing_via_hold_s`（新参数默认 2.0 s，防负载静差拖死归零）则照旧放行。段间运动仍纯
开环——不退回"每拍从实测迈步"的阶梯采样（c3941c5 教训）。启动 init 不加门限（实测没动不能
卡启动）。② `_go_home` 路径改 `_parse_init_waypoints()[::-1]`（倒序），docstring/注释同步。
**验证**：`test_home_park.py` 11→**14 例**全绿——新增 park 途经点等实测到位再推进（钉住重发
断言）、实测长期不到超 `homing_via_hold_s` 放行、init 不加门限三例；路径构建测试断言改为
**倒序**（[init_pose, WAY2, WAY1, 零]）；闭环滞后模拟：旧行为命令在 waypoint 只停留 1 tick
(0.02s) 即反向，修复后钉住等实体进入 0.12 rad 才走下一段；reanchor 5 例全绿。左/右臂 yaml 补
两参数；**机器人侧需重新 colcon build 生效**（本机 install/ 为 9/3 旧构建，连 /teleop/home 都无）。

**astral_pim_ik 收尾：首次真实执行 torch 路径暴露并修复 3 个真 bug + F1 兜底落地 + F2 关闭 + 小规模训练闭环打通**——
`astral_pim_ik`。**背景**：该功能包此前在无 torch 机器上开发（文档引用的 `/opt/anaconda3/envs/MujocoSim`
本机不存在），`network.py`/`kinematics.py`/`train.py` 只过语法 + 静态审查、从未执行。本机实测
`arm_sdk`（pinocchio 3.8 + torch 2.13 cpu）与 `lerobot`（pinocchio 3.4 + torch 2.11 cu130 + 4090D）
均有 torch，全部未完成项可补。**现象①（9D 编码行列序 bug）**：首次跑 `test_network.py` 即 2 FAIL——
`transform_to_9d` 把 (3,2) 旋转块行主序拉平得 `[r00,r01,...]`，与 docstring/`rotation_6d_to_matrix`
的列主序 `[r00,r10,r20,r01,r11,r21]` 不符，恒等位姿编码成退化矩阵。**修**：拉平前 transpose +
非恒等旋转 round-trip 断言。**现象②（L_elbow 变量遮蔽）**：真跑 `train.py` loss 卡 ~14 不降、网络
几乎不学 ψ——`PhysicsInformedLoss.forward` 里 `B, W, _ = pred_psi.shape` 把形参 `W`（腕部张量）
遮蔽成窗口长度 int，肘误差对 int 广播成 ~13 m 假项（同批数据手动肘误差 mean 0.11 m vs `loss_fn`
报 12.97）。**修**：解包改 `T` + 回归测试「perfect ψ → L_elbow<1e-3」（实测 1e-4 m）。
**现象③（IID 数据不可学）**：IID 随机位形数据 ψ 学不动（误差 52.8°、loss 平台 ~1.15）——单帧
T_ee 不决定 ψ（肘在 S-W 轨道圆上自由），窗口网络需要帧间运动信号。**修**：新增
`generate_trajectory_dataset`（平滑关节漂移，`travel`/`wobble` 控速，schema 同
`generate_dataset`），文档明确训练必须用轨迹数据。**F1 兜底**：`solve(fallback_scan=True, n_scan=36)`
网格扫最近可行 ψ（默认关保持 None 契约，救援计入 `fallback_count`），`solve_trajectory` 透传，
test_deterministic_solver 新增用例 8/8。**F2 关闭**：torch/numpy 肘点一致性 **9.58e-07 m**（<1e-5）。
**训练闭环实测**（lerobot/4090D，时间相干轨迹 20k 帧/40 ep）：ψ err 27.1°（median 19°）、
Joint MAE 10.0°、solve 率 74%、位姿 <0.06 mm；oracle（真值 ψ）100% solve——stage-2 几何 IK 精确，
残差为合成数据宽肘先验的信息瓶颈，真机遥操录制是生产路径。5k 帧 seed 对照 ψ err 15.2°（方差来自
小 val 集）。**包布局修复**：`package.xml`/`setup.py` 原埋在 `src/robots/astral/astral_pim_ik/src/robots/astral/astral_pim_ik/`
（多一层 src），上移到包根 `src/robots/astral/astral_pim_ik/`——与其余 20 包一致，colcon 可发现，PYTHONPATH 与
文档对上了。**验证**：test_network 4/4、test_deterministic_solver 8/8、test_dataset 3/3、
test_pipeline 2/2、test_geometric_ik 10/10（回归）；全部实测数字见包 `CLAUDE.md` §5.1 与
`docs/adversarial_review.md`（F9/F10/F11 三条新发现）。

**阻尼释放后再就绪/归零仍回跳——根因在板端陈旧目标，补"切回 POSITION 先重写实测位姿"**——
`astral_robot_control` × `astral_robot_sdk`。**现象**：启动后点「阻尼释放」正常，手动拖臂后
再点「一键就绪」或「归零」，臂仍**抽一下快速回到阻尼释放前的位置**（上轮"陈旧命令流抢占"
三层修复后依旧）。**根因**：上轮修的是 **ROS 侧**（driver 缓存 `_clear_cmd_cache` + 臂节点
disarm 停流），但漏了**板端内部的目标寄存器**——阻尼（motion_mode=0）只是让板端忽略 0x90
位置指令，并不清空其内部保存的"上一次目标"（= 阻尼前位姿 P）；手动拖臂只改实测、不改板端
目标。`~/ready`（SDK `one_click_ready` 内部 `set_motion_mode(POSITION)`）与 `~/home`
（显式 `set_motion_mode(1)`）一切回 POSITION，板端立刻重新追踪 P → 臂抽回旧位姿；此时
`enable()` 轮询最长 3s，期间臂就停在 P 上，归零要么迟到要么因电源位未确认被跳过。**做法**：
① SDK `one_click_ready(..., seed_from_current=True)`——POSITION 切换后、enable 轮询前，用最近
一帧实测关节角 `set_target_positions` 重写板端目标，POSITION 进入即保持当前位置，随后归零
照常下发（默认 False 保持旧行为）；② driver 新增 `_seed_target_from_current()`（`_read_q18` →
`split_full_q` → `move_arm_js` 双臂），在 `~/position` 与 `~/home` 的 `set_motion_mode(1)`
之后、显式目标之前调用——「位置保持」真正保持被拖拽后的当前位置，「归零」先定住再收回；
③ `~/ready` 传 `seed_from_current=True`。**验证**：`test_driver_services.py` 13→**16 例**全绿
（新增 home 阻尼切回先重写实测位姿再归零的顺序断言、position 阻尼/非阻尼两例都重写目标）；
SDK `one_click_ready` 签名向后兼容（旧调用不带参行为不变）。

**真机遥操手感/可达调优（当前 yaml 生效值；配置项，无 A/B 数字，留待实机复核）**——
`astral_arm_teleop`。左右 yaml 各四处：`workspace_radius` 0.55→**0.65**（腕目标球约束
放宽，末端可达范围更大）；`reach_margin` 0.01→**0.001**（肘伸直软墙基本放平，逼近 URDF
伸直极限才挡）；`motion_scale` 0.8→**1.0**（VR 位姿增量 1:1 无缩放）；`rot_smoothing`
0.25→**0.35**（旋转低通加强，τ≈14→19 ms @50 Hz）。`arm_teleop` README 参数默认值说明
同步为当前值。

**数采"坏段默认隔离 + 空录/吞指令在线告警"——堵坏数据进训练集的通路**——
`morla_data_collect` × `morla_web_manager` × `scripts/vla_process_session.sh`。
**动机**：对抗性审查 start→stop→start→stop 边界时序（逐拍验证端头/端尾/接缝均干净：
尾部先 drain 后 close 不丢帧、头部清缓冲+门控不收旧数据、缝内帧按段独立对齐零伪影），
真正毒化通路在别处——①一条龙 validate **默认不隔离** fail 段（align 的 too-short 阈值
仅 `n_frames < 2`≈33ms，0.5~2s 短段/体检差段直接进转换产物进训练集）；②源未就绪空录
无即时告警（low_fps 只管参考相机半速，管不到数值/图像全 0）；③SAVING 期被吞的 start
无留痕（web 按钮有 disabled 视觉，键盘/VR 没有——"以为开了实际没录"）。**做法**：
①`vla_process_session.sh` validate 默认 `--apply`（fail 段移入 `session/quarantine/`，
移动不删除可逆；旧 `--apply-quarantine` 兼容 no-op），隔离后仍有 fail 即 `exit 1`
中止转换；`--no-quarantine` 显式放行旧行为；②节点录制启动 ~2s 空录检查（段级一次性，
阈值对齐 F5）：数值/图像全 0→"未收到任何样本"、无数值→teleop 未发布、无图像→抽头
未发布，进 state JSON `empty_warning` + 节点 `EMPTY-REC` WARN + web 红条（begin 复位/
end 清空）；③状态机拒绝的指令计数入 state JSON `ignored`（start/stop/discard/next/
pause/resume 六键；pause/resume 原静默 no-op 现也计数留痕）；④web 卡片新增
`empty_warning` 红条 + `ignored` 忽略指令 chip，`types.ts` 同步（`mapUiState` 整对象
透传无需改）。**验证**：`test_node_guards.py` 9→**12 例**全绿——空录全空 e2e 订阅
`/data_collect/state` 断言 JSON 带 `empty_warning` 且丢段后复位、部分缺失只告警缺失侧
（有数值无图像→图像文案）、数值+图像流入不误报（`_quiet_start_checked` 确认检查真执行）、
吞指令计数 `{"start":1,"pause":1,"stop":1}` 进 JSON；`test_collect_smoke.py` 2 例通过；
前端 `npm run build`（tsc+vite）通过、dist 重建。机器人侧无需重装（脚本/话题/前端改动），
web 新 dist 需同步部署机。

## 2026-09-03

**HOME 归零卡顿/抽动修复：park 从"贴实测迈步"改纯开环 + 冻结守卫**——`astral_arm_teleop`。
**现象**：上一轮修好"电机未使能"后 HOME 第一次真正跑起来，但臂**慢慢的一卡一卡**地动，还
**突然抽一下**，和当初启动归位被贴实测钳制时的异常很像。**根因**：HOME park 分支每拍
`base = 实测关节 state_q`、`q_cmd = 实测 + 一个 tick 步长`——命令永远只领先实测一个 tick 步长，
等于把平滑位置控制改成"等实测挪一步才挪一步"的**阶梯采样**：实测经板端读码→UDP→ROS→节点有
几十 ms 延迟，q_cmd 实际推进被实测采样节拍门控 → 速度被压到远低于设定的 0.6 rad/s（慢）；
命令按实测帧节拍一跳一跳 → 粘滑/卡顿，静摩擦突破时"抽一下"。这套"贴实测"钳制当初为启动 init
引入、因同样症状已回退成开环，但 **park 保留至今，而 HOME 此前从未真正成功跑过**（先报未使能、
再被陈旧命令流抢目标），直到这次才暴露。**做法**：① park 与 init 一致改**纯开环匀速推进**
（base = q_cmd，唯一被实机验证的平滑路径）；② 反馈只做**跟随便用**——新增参数
`homing_follow_tol`（默认 0.25 rad）：park 运行中实测落后指令超过阈值（电机失能/堵转没在跟随）
→ **冻结轨迹**，q_cmd 不空跑一路领先到零位，领先被钳在 tol 内；实测追近（恢复跟随）自动解除
继续收回，不会猛扑；正常跟随稳态滞后远小于 tol，守卫不介入；③ 状态字段 `_park_frozen` 每次
`_go_home` 重置。**验证**：`test_home_park.py` 7→**11 例**全绿——新增健康跟随开环平滑推进、
实测停住轨迹冻结（领先钳在 tol 内）、实测追近后解冻继续收回三例；reanchor 5 例全绿；README
`homing_track_state` 语义说明同步。

**阻尼/归零/一键就绪"陈旧命令流抢占"修复**——`astral_robot_control` × `astral_arm_teleop`
× `morla_web_manager`。**现象**：启动后点「阻尼释放」正常，手动拖臂后再点「一键就绪」，
臂直接回到**阻尼释放前的位姿**；阻尼释放后点「归零」**无效**。**根因**（非重复使能）：
① 阻尼 = 板端 motion_mode=0，位置指令被忽略——`~/home` 只发一次性 `set_all_joints_zero`，
在阻尼态静默无效；② 归零/一键就绪只是**一次性目标**，而 100Hz 控制定时器在
`command_timeout_s` 新鲜窗口内持续重发缓存的"阻尼前目标 P"，或遥操臂节点仍在 armed/
启动归位 homing 持续发流——一旦运动模式切回 POSITION，陈旧 P 立刻重新接管、覆盖刚下发的
归零 → 臂回到阻尼前位姿。**做法**（三层）：① driver `_srv_home` **先切回 POSITION**
（motion_mode=1）再归零（阻尼后归零生效），未上电明确报"先 ~/ready"而非静默；② driver
新增 `_clear_cmd_cache()`——`~/ready`/`~/home`/`~/damping`/`~/estop`/`~/enable` 执行时清掉
缓存目标，100Hz 重发只复读显式目标（零位），不再把陈旧 P 顶回去；③ 臂节点 **operator
disarm 取消进行中的 homing/park**（`_on_disarm` 中 `_homing=False`，否则取消命令流仍持续
下发覆盖后续归零）；④ web 五个硬件模式端点（急停/阻尼/一键就绪/归零/位置保持）**先自动
发布 `/teleop/disarm`** 再调 driver（与 HOME 端点同款）——手动模式即交还控制，之后用
「开始遥操」重新捕获 vr_init。**验证**：`test_driver_services.py` 7→**13 例**全绿（阻尼中
归零先切 POSITION/缓存清理/未上电报错）；`test_home_park.py` 7→**9 例**全绿（disarm 取消
进行中 homing）；reanchor 5 例、web monitor 4 例全绿；前端提示同步并重编 dist。

**HOME 使能修复：~/enable 幂等化 + 板端在线即算下发成功**——`astral_robot_control` ×
`morla_web_manager`。**现象**：启动正常到设定位置（电机已使能）后点 web **HOME**，报
"电机未使能"、HOME 不下发。**根因**：HOME 端点先调 driver `~/enable`，而 SDK `enable()`
靠轮询板端 obs 帧的 `robot_powered` 位确认——该位在此板子常不置位（SDK demo 同款
"enable 未确认仍继续"）；auto_ready/一键就绪已使能后再 enable = **重复使能 + 确认位失灵**
→ 服务回 "enable not confirmed" → web 把已使能误报成未使能。**做法**：① driver `_srv_enable`
**幂等化**——`_robot_powered` 已 True 直接成功返回（跳过 WORK→POSITION→enable 重复下发，
仅当运动模式非位置时补切 position）；② 未确认但**板端在线**（obs 帧持续）也算下发成功
（命令已送达，对齐 demo「未确认仍继续读反馈」），仅真正离线才硬失败；③ web
`_ensure_driver_enabled` 注释/文档同步（HOME 先 enable 再 disarm+park 语义不变）。
**验证**：新增 `test_driver_services.py` 7 例全绿（已上电零下发/阻尼补位置/确认 OK/在线未确认
=成功/离线=失败/dry_run/未连接），arm home-park 回归 12 例、web monitor 4 例全绿。

**web 启动流程回退（急停恢复方案收敛到 HOME 按钮）**——`morla_web_manager` ×
    `astral_arm_teleop`。**现象**：上一条「启动/重启自动调 `~/enable` + 启动归位贴实测」实机
验证无效——web「停止→启动」后直接 **503 service** 且臂不动。**根因**：① `/start` `/restart`
在 launch 起来后同步等 `~/enable` 确认（12s 预算），driver 尚未 ready 时超时抛 503——启动本应
"只把栈拉起来"；② 启动 init 归位也被 `homing_track_state` 钳到实测关节，电机一旦没立刻跟上，
q_cmd 逐拍被实测拽回 → 臂"原地不动"。**做法（回退 + 保留）**：① `/start` `/restart` **回退**
到改前——不再自动调 `~/enable`、不再 503；`~/enable` 服务本身保留，仅 **HOME** 端点使用
（先使能再 disarm+收回零位）；② 臂节点 `_homing_tick` 的贴实测钳制**收窄到 HOME park 专用**
（`self._homing_mode == "park"`），启动 init 归位恢复改前**开环逐拍推进**——web 启动只拉栈，
电机使能交给 driver auto_ready/一键就绪，臂正常往 init 走；急停后的"使能+收回零位"统一走 HOME
按钮。**验证**：`test_home_park.py` 由 5 例扩到 7 例——新增 `test_init_homing_advances_even_
when_state_frozen`（实测冻结时 init 仍开环推进，50 tick 前进 ~0.1 rad）与 `test_park_homing_
holds_until_robot_moves`（park 模式下实测不动 q_cmd 不空跑，50 tick 误差不动），全绿；arm teleop
回归 13 例通过；web monitor 单测 4 例通过。

**真机遥操手感调参：两级低通串联改单级**——`astral_arm_teleop` × `astral_robot_control`。
**现象**：遥操小动作"先顿后冲"（先迟钝一下再冲上去），静止又有发颤。**原因**：teleop
`pos/rot_smoothing` 与 driver 板卡 `lpf` 是**两级串联低通**，时间常数叠加、噪声抑制与跟手
矛盾。**做法**：噪声抑制收敛到 teleop 一级，板卡近直通——teleop `pos/rot_smoothing` 0.4→
**0.25**（按 50Hz 标定 ≈ τ14ms）；driver `lpf_alpha` 0.35→**0.85**（100Hz ≈ ~2ms 近直通）。
注释记录回退阶梯：静止发颤回 0.35~0.4 / `lpf 0.6~0.7`；大动作仍觉肉再降 0.15~0.2 / 升 0.95
（`lpf_enable:false` = 全直通）。三处 yaml（左右臂 + `astral_robot.yaml`）同步。

**web 急停后恢复不归位修复 + HOME（归位到零）按钮**——`astral_robot_control` ×
`astral_arm_teleop` × `morla_web_manager`。**现象**：web 端急停（真断电 disable）→ 停止 →
再启动，机器人不回零，直接锁在当前位置。**原因**：急停后电机失能；重启时 driver `auto_ready`
的 one_click_ready 未可靠确认上电，且启动仅起节点、无"使能=遥操版一键就绪"步骤，臂节点的
归位轨迹在电机失能期"内部空跑"领先实体。**做法**：① driver 新增 `~/enable` Trigger 服务
（WORK→POSITION→enable，**不回零**——遥操恢复归位路径专用，避免与 teleop 正在发布的
joint_commands 轨迹目标抢）；② 启动/重启带真机 driver 的预设后，monitor 等待 `~/enable`
服务出现并幂等调用（`DRIVER_ENABLE_WAIT_S` 12s 预算，无 driver/sim 自动跳过）——
`_preset_has_driver` 按 `with_arm_driver` 或 package 判定（**实机 503/臂不动后已回退**：
web 启动只拉栈、不再自动 enable，见当日顶部条目）；③ 臂节点归位贴实测 joint_states
（新参数 `homing_track_state` 默认 true）：fresh 且贴近 q_cmd 时每拍从实测位姿迈步，电机未使能
不动时 q_cmd 不再空跑，使能晚到不会猛扑（**启动 init 路径已回退开环**，此钳制仅保留 HOME park，
见当日顶部条目）；④ 新增 **HOME / park-to-zero**：臂节点订阅
`/teleop/home`（一次性 volatile，语义同 `/teleop/start`）+ `~/home` 服务，`_go_home` disarm
后沿 **init_pose → init_waypoints → 零位** 慢速收回，到零位把机器人原点锚到 FK(零)，超时保持
当前 q_cmd 绝不硬发零目标（新单臂/双臂通用测试 `test_home_park.py` 5 例全绿）；⑤ web monitor
`config.py` 增 `TOPIC_HOME`/`DRIVER_SRV_ENABLE`，monitor_node 发布 `publish_home` 并把
`enable` 纳入 `_DRIVER_SERVICES`；⑥ `web_server.py` 新增 `POST /api/v1/teleop/home`（先使能
后 disarm+home，仅 RUNNING/PAUSED 可用），前端预设管理区「启动/停止/重启」旁加 **HOME** 按钮
（`api.teleopHome`，purple，与急停区分）。真机流程：急停→停止→启动（拉起栈，不再自动使能）→
按需点 **HOME**（自动 enable + 收回零位）→ 一键就绪/`/teleop/start` 重新遥操。

**新增推理功能包 `morla_policy_inference`（策略部署 / 数据真机回放 / 人在环路）**。
本包接遥操数采与训练上游做推理闭环：模型无关后端（openpi 远程 ws `host/port` / lerobot ACT 进程内
`checkpoint_dir` / stub 冒烟），**换模型=只换后端参数**；三种引擎模式 `queue_sync`（阻塞重填）/
`queue_async`（后台预取）/ `rtc`（后台滚切+min_tail+延迟补偿跳行）；布局真源
`morla_data_collect.schema.CollectSchema`——观测订阅/state 向量/`split_action` 指令拆分全自动，
换机器人配置只改同 schema 参数。回放读 LeRobot v2.1 目录或 `aligned_data.h5`，`PlaybackSession`
步进 + 中断续播 offset 重锚。HITL：`_cmd_takeover` 先对每个遥操节点调 `~/reanchor`（效应）全部成功
才提交 HUMAN（仲裁先效应后提交）；暂停→增量 VR 接管→交还策略/回放；release/resume 时策略按实况
重规划、回放按实况重锚，避免陈旧 chunk 跳变。`astral_arm_teleop` 新增 `~/reanchor` 服务（同步重记
机器人原点=FK(当前关节) 与 VR 零点=当前 VR 位姿）。安全兜底 `SafeExecutor`（NaN/比值 [0,1]/
关节限位/`max_joint_vel` 限速）独立于策略。修复三个易踩坑：rclpy 无 struct 参数 →
`camera_map` 用 JSON 字符串；`__init__` 顺序清空 `_backend_cfg` 导致 make_backend 缺参；re-anchor
`service_is_ready` 误判就绪后阻塞等响应 → `call_async`+3s 轮询。验证：最初 74 例单测全绿（含进程内
mock 机器人 + stub 后端 + 真实 Trigger 服务的 node_flow：策略起停/暂停恢复重规划/回放两路径/
接管成功与失败路径），另有 `astral_arm_teleop/test_reanchor_teleop.py` 覆盖 `~/reanchor`。
实现期后续修复/补强（同批次）：① `pinch_gripper_node` 新增 `disarm_topic` 仲裁门
（默认 `/teleop/disarm`，空串=旧行为），策略在 POLICY/PLAYBACK 期间独占夹爪指令话题、进入
HUMAN 时 policy_node 发 disarm=False 放行真人捏合/扳机——否则夹爪遥操与策略抢写
`/left_gripper/command`；② `ActionEngine.stop()` 在锁内 close backend、queue_sync 边界重填
整段持锁推理，消除"控制线程推理中另一线程停引擎→并发关后端"竞态；③ 刚进播放立刻暂停且尚无
指令时 `_cmd_pause` 兜底 `_hold_current()`（以实测姿态冻结）；④ 引擎/节点恢复语义：暂停后
resume 策略重建 chunk、回放重锚 offset（防陈旧行跳变）。MuJoCo 真话题端到端冒烟
`scripts/run_inference_sim_smoke.py`（astral_mujoco_sim + policy_node stub 全链路：policy 驱动
arm/gripper→pause 夹持值稳定→resume 恢复变化→h5 回放逐帧一致→播完自动 IDLE）**全部通过**。

**对抗性审查修复（首轮 14 项，本轮全处理）**——`morla_policy_inference` × `astral_arm_teleop`
× `astral_gripper_teleop`。① **`/teleop/disarm` 电平语义冲突（critical）**：arm/head 遥操
`_on_disarm` 对任何 Bool 都 disarm，而 policy 进 HUMAN/IDLE 发 disarm=false 开闸，把刚
re-anchor 武装的遥操又拆掉 → arm/head 只认 `Bool(true)`，与 web/pinch 消费方对齐；② stop→
IDLE 发 disarm=false 重开仲裁门（TRANSIENT_LOCAL 闩锁不再永久关死夹爪门）；③ pinch 门控回归
web pause→resume：pinch 新增 `arm_topic`（`/teleop/armed`=true 也恢复），默认配置同时设
disarm+arm；④ `_cmd_playback` 先 load+`action_dim`/`state_names` 校验再 request→teardown，
失败原地保持（不再"POLICY+引擎 None 静默冻结"）；⑤ `_start_policy` 先验证 obs→冻结旧引擎→
新引擎成功后 disarm+换新，失败 `revert()` 恢复（Controller 快照含 `_interrupted`）；⑥ POLICY
运行期 state 连续缺失 >`obs_stale_stop_s` 自动暂停，杜绝陈旧观测盲推；⑦ `~/cmd` 入队 + 控制
定时器持 `RLock` 串行 drain+dispatch（仲裁回调与控制循环不再交错双写，接管改为事件驱动轮询
re-anchor、失败事务性 re-disarm）；⑧ 接管先验新鲜 state，避免陈旧 q_cmd 锚点跳变；⑨
`_run_plan` 整段持锁推理，`stop()` 锁内 close，消除 infer/close 竞态；⑩ 二次回放不再跳过
末帧保持（`_hold_end` 在 playback/stop 清理）；⑪ 回放带 schema 时校验布局命名顺序（同维不同
块序拒绝）；⑫ smoke 驱动反馈泵按 disarm 闩锁门控、断言策略夹爪比值且不再被 0.5 回显污染、
回放 leg 断言轨迹末帧=记录末行。验证：全套 79 例单测全绿（新增 obs 陈旧自动暂停/disarm 电平
顺序/接管失败事务性回滚/回放 schema 拒绝/连续回放保持 等 node_flow 回归），
`astral_arm_teleop/test_reanchor_teleop.py` 5 例全绿。

**LeRobot v2.1→v3.0 升版转换器（现代 lerobot ACT 直接训练）**——`morla_data_collect`
新增 `convert_to_lerobot_v3.py`：把 openpi 用的 v2.1 数据集**原样升版**为 v3.0，
供 `VLA/lerobot`（>=0.6，ACT 与其它策略共用读取管线）直接训练——其读取侧对 v2.1
直接 `raise BackwardCompatibilityError`。布局/数值语义逐字段镜像官方
`scripts/convert_dataset_v21_to_v30.py`：data/episodes 按 chunk 分段合并
`file-*.parquet`、同 chunk 的段视频用 PyAV `ffconcat` **流拷贝串接不重编码**、
时间窗按各段实测时长叠加、tasks.jsonl→`tasks.parquet`、episodes.jsonl→
`meta/episodes/chunk-000/file-000.parquet`、v2.1 专用 info 字段清掉换成 v3.0
模板；`stats.json` 原样沿用（无需重算）。**源 v2.1 目录只读**，输出独立目录默认
拒覆写（`--overwrite` 重建）。验证：与官方脚本同源产物逐文件一致；现代
`LeRobotDataset` 加载 3 段 1828 帧、逐帧解码逐像素与官方产物一致；`lerobot-train
--policy.type=act` 冒烟 2 步在 CPU 上真实跑通并落 checkpoint（前向/反向/保存全过）。
`vla_process_session.sh` 增加 `--act-output <dir>`（可加 `--overwrite-v3`）一条龙
串接升版。新增 `test_convert_lerobot_v3.py` 7 例回归：目录布局/info 字段迁移、
tasks.parquet、数据 parquet 行数+非视频列集不变、episodes 元数据区间/tasks/
length、视频合成帧数=各段之和+时间窗首尾相接、stats 原样拷贝、守卫（覆写拒绝/
非 v2.1 源/输出在源内）与源目录只读。离线回归全套 59 例通过。

**改采集配置后同目录重转的静默污染修复**——`convert_to_lerobot.py` ×
`convert_to_lerobot_v3.py`。对抗审查发现：改 `data_collect.yaml`（加右臂/换手/
段数变少）后把 v2.1 重转进**同一输出目录**，旧 `data/`、`videos/` 的 episode
文件不会被删（只截断 meta jsonl），而 v3 升版按目录遍历文件 → 把**旧布局的行**
（如单臂 8 维）与新高维行混进同一个 parquet，静默产出损坏数据。修复两层：
①`convert_session` 视输出目录为数据集专属目录，开始前清掉旧 `data/`、`videos/`
（与官方"输出即替换"语义一致）；②`convert_to_lerobot_v3` 以
`meta/episodes.jsonl` 为权威 episode 清单，数据/视频文件集与清单不一致即响亮报错
（列出多余残留下标与缺失下标并提示换新目录），不再静默跳过或合并。新增 2 例回归
（重转清残留后升版一致性 / 脏源拒绝），离线回归全套 **61 例**通过。

**sim README 对齐当前默认 + 关节 3/5 限位放宽**——`astral_mujoco_sim` × `astral_arm_teleop` × `astral_robot_description`。①README：默认求解器描述从 `urdf_numerical` 修正为 **`geometric`**（yaml 实际默认，含 `use_human_elbow` 先验说明）、init_pose 描述改为「代码默认非零位、`astral_mujoco_sim.yaml` 覆盖为零位看 homing 过程」、MJCF 表补 `mjcf_path` 回退逻辑。②URDF/MJCF/`analytic.py` 四处一致放宽 joint3/5：±1.57 → **±2.2689 / ±1.7802**（`astral_arm.urdf`、`astral_robot.urdf` 及各自 `.pin.urdf`、内嵌 `RobotMain_URDF.pin.urdf`、`astral_dual.xml` 关节 range + actuator ctrlrange）。③`test_geometric_ik` 满伸 sweep 用例对齐逃逸迟滞设计：>0.15 rad 跳变仅当 `esc_active`（分支逃逸）时放行并打印 note，无逃逸的真跳变仍 FAIL；离散化与 seed 选择同为 5mm，避免更细步长撞进合法的窄姿态口袋。

## 2026-09-02

**roll 逃逸迟滞参数化 + 逃逸可观测**——`astral_arm_teleop`。`ik_escape_after_frames`（默认 4）/`ik_return_after_frames`（默认 20）从 `geometric.py` 常量提升为节点参数，工厂/求解器全链路透传，可经 `ros2 param set` 热改（调大 = 越不易甩肩但"卡住不跟"窗口越长；调小返回 = roll 结束更快回人臂角）。逃逸发生时打一次 WARN 并计入 `[Latency]` 行 `psi_escape` 计数——真机肩部突动先查这个，不再是无声的瞬间跳变。验证：`test_geometric_ik` 9 例全 PASS（含 `roll_escape_hysteresis`），工厂注入 7/33 生效、默认 4/20 不变。

**指纹查询 CLI：`python3 -m quest3_video_streamer.scan`**——在机器人上直接打印每路采集设备（节点名/设备路径/fourcc）的全部稳定指纹（by-id → by-path → sysfs），抄子串进 `label_aliases` 即可，替代手工 `ls -l /dev/v4l/by-id/` 再对节点。无采集设备时提示 no capture-capable device。无相机环境冒烟通过。

**label_aliases：auto_scan 换口/重插防漂移落地（别名表方案）**——`quest3_video_streamer`。在昨日「auto_scan: false + 命名块 + by-id」备选写法之上，补上两全方案：保持 `auto_scan: true` 的自动发现，新增 `label_aliases` 参数（字符串数组 `"指纹子串=稳定label"`），扫描完成后按设备指纹（`/dev/v4l/by-id` → `by-path` → sysfs 名，子串匹配，先列先赢）改写 label；改写发生在 per-label 覆盖块查找之前（`video8.preset` 等同名块自动生效）。语义细节：目标名=匹配设备当前内核名 → 合法 no-op（固化现状，且消耗该设备，后续规则不再对其生效）；目标名撞上**其他**设备的内核名 → 拒绝并告警（防止重名）；一条规则匹配多台 → 只改第一台并告警；规则零命中 → 告警（指纹写错或相机未插，开录前看日志一眼）。`scan.py` 新增 `stable_fingerprints()`（纯函数）与 `apply_label_aliases()`（支持预置 fingerprints 注入）；`streamer_node` 声明 `label_aliases` 参数并接入 `_build_sources` 的 auto_scan 分支。新增 `test_label_aliases.py` 12 例（指纹顺序/兜底 mock、命中/未命中/固化现状/多设备/冲突/畸形规则/集成断言——rclpy 缺失自动 skip），streamer 全套 32 例通过。使用：`params.yaml` 加 `label_aliases: ["Intel_R._RealSense=video8", ...]`，换口/重插/重启后 data_collect 与 openpi camera_map 永远零改动。

**VLA 一键脚本 + 相机 label 防漂移 + 默认配置切单左臂**——`scripts/` × `morla_data_collect` × `quest3_video_streamer`。①新增 `vla_process_session.sh`（对齐→校验→转 LeRobot 一条龙，走 openpi uv venv，`--image-size`/`--apply-quarantine` 可选）与 `openpi_train.sh`（norm stats 重算 → 训练，`lora|full` 二选一，前置检查数据集软链与 pi05_base 权重）。②`params.yaml` auto_scan 覆盖块加「换 USB 口防漂移」注释写法：`auto_scan: false` + `/dev/v4l/by-id/` 稳定路径 + label 沿用 `video8/video0/video2`，数据集 key 不变下游零改动。③`data_collect.yaml` 默认切实际采集配置：单左臂 + 左夹爪、`cameras: ["video8", "video0", "video2"]`（auto_scan 节点名，双臂配置注释保留）。回归：转换/schema/web 31 例 + node_guards 9 例 + 对抗配置 8 例（openpi uv 环境跑通）全绿。

## 2026-09-01

**换配置全链路对抗测试 + openpi 侧布局硬编码清除**——`morla_data_collect` × `VLA/openpi`。新增 `test_adversarial_configs.py`（8 例，独立临时 HF_LEROBOT_HOME，不碰真实数据集）：双臂+双夹爪+三相机（16 维）、左臂+wuji 灵巧手+腰+头（31 维，贴近上限）、双臂+腰头+15fps+非对称相机映射（右腕映射/左腕零填充）、NaN 段校验隔离后训练侧只剩健康段、31 维 norm stats 全量计算+归一化回归，以及三个负例（35 维超限必须拒 / camera_map 指向不存在相机必须拒 / 缺 meta 清晰报错）。每例在 openpi 侧逐维验证：delta 掩码与 schema 期望逐位一致、delta 语义为「相对 chunk 起始 state」（含 k≥1 帧）、`AstralOutputs` 截断维=state_dim、相机槽位按 conftest 颜色指纹验证接线正确、空 task 段 default_prompt 兜底分词与正常 task 段不同。同步修复：openpi `LeRobotAstralDataConfig` 原来写死 8 维截断、`(7,-1)` delta 掩码、video8/video0 相机映射——现改为 `create()` 时从数据集 `meta/info.json` 现读（state 维度/names 推导，`_ee_` 保持绝对其余转 delta，相机走 `camera_map`）；并修正环境变量口径：pinned lerobot 0.1.0 只认 `HF_LEROBOT_HOME`（设旧名 `LEROBOT_HOME` 会直接 raise），meta 解析与文档同步对齐。全套 53 例回归通过。


**转换期图像 letterbox 到 224×224**——`morla_data_collect`。`convert_session` 新增 `image_size`（默认 224，`--image-size 0` 保留原分辨率）：视频落盘前按 openpi `resize_with_pad` 的几何约定等比缩放 + 对称黑边（余数归下/右），训练时 `ResizeImages(224,224)` 变恒等操作，模型输入与"全分辨率入库 + 在线缩放"逐像素一致，但省掉加载侧解码大图 + 缩放的 CPU 开销；图像统计口径同步改为 letterbox 后的帧。存量 3 段重转后数据集 283MB → 20MB（-93%），openpi 管线加载验证通过（原生 224 直入）。新增 2 例测试（letterbox 几何对齐 openpi 约定 / 默认 224 端到端尺寸与黑边断言）。

**转换产物 parquet 去掉视频 struct 列（OpenPI 下游适配）**——`morla_data_collect`。v2.1 规范的 parquet 只含非视频列（视频帧由读取侧用 `timestamp` 列 + `meta/info.json` 的 `video_path` 模板解析，`get_hf_features_from_features` 对 `dtype=="video"` 直接跳过）；此前为"兼容性"多写的 `struct{path,timestamp}` 列会让 openpi 锁定的 lerobot 0.1.0 在 `hf_transform_to_torch` 的 `torch.tensor(dict)` 处崩溃（`Could not infer dtype of dict`）。转换器删列、存量 3 段 parquet 原地重写（未重编码视频）、schema 测试改为断言**不得**出现 `observation.images.*` 列。修复后 openpi 数据管线端到端验证通过：1828 样本加载、AV1 视频解码、action=next_state 语义保持（max diff 0.0）、pi05 三相机槽位映射 + 空 prompt 兜底分词正确。

**转换管线 OOM 修复：SVT-AV1 限内存 + 图像统计流式化**——`morla_data_collect`。实机 3 段 1080p 数据转换在小内存机（15GB）上被 OOM 杀掉（exit 137，峰值 RSS 6.3GB，且管道下游静默表现为"只转出 1 段、meta 全空、退出码 0"——教训：该命令勿接 `| tail` 看结果，要看退出码）。两处根因：① SVT-AV1 默认按核数并行 + 深 lookahead，1080p 内部缓冲数 GB → 加 `svtav1-params: lp=2:lookahead=16`（参数不识别则退化无参打开，再不行回退 h264）；② 图像统计 `np.stack(≤100 帧)` ≈620MB/相机 → 改 `ImageStatsAccumulator` 流式累加（每通道 sum/sumsq/min/max，输出与堆叠计算数值等价，新增等价性回归 2 例）。修复后峰值 RSS **6.3GB → 1.14GB**，3 段 1828 帧完整转出。

**cmd 流时间戳修复：改用到达时刻**——`morla_data_collect`。validate 对 9/1 上午实机 3 段数据报 F4 全灭：`left_arm_cmd` ~45% 样本戳完全重复（值不同）。根因：teleop `_publish_q` 把 `joint_commands` 的 `header.stamp` 打成上游 VR 输入戳（供 web 延迟面板算 E2E），IK 定时器（~60Hz）比 VR（~30Hz）快时同戳复用。采集端 `*_cmd` 流改为一律用到**达时刻**——指令样本时刻语义本就是「发出时刻」，且 VR 戳比真实发出早一个管线延迟，混用两时钟还会给 command 模式 action 对齐引入交错偏差。state 流仍用 header.stamp（驱动侧时钟）。新增 2 例回归（cmd 用到达时刻严格递增 / state 保留戳）。存量 3 段测试数据的 raw cmd 戳仍是旧的（next_state 模式不消费 cmd，不影响对齐/转换）。

**采集抽头从推送开关解耦**——`quest3_video_streamer`。此前 `~/collect/{label}` 与 preview 共用运行时门控：`push_enabled=false`（或相机被 `active_cameras` 静音）时采集流也断，「关了推送开关想省 CPU 却采不到图」。改为**唯一准入条件 = 话题有订阅者**（编码本来就按需，无订阅零开销），录不录由 `morla_data_collect` 状态机决定；preview/WebRTC 仍走门控不变。效果：推送开关现在只控制「看」（Quest + web 实时画面），与「录」完全正交。

**采集抽头限流器换代：令牌桶**——`quest3_video_streamer`。0.8 容差后仍有 2-3/s 误杀（捕获线程调度抖动偶发 20ms 间隔对）；改令牌桶（容量 2、速率=max_fps）：源≈目标时抖动被吸收、长跑 99%+ 透传，源更快仍限 30/s。仿真：30fps±5ms 抖动 99.4% 透传、60fps 源稳态 30.0/s。实机录制 ~28.8/27.6/27.3fps（中位 29-30）→ 修复后 tap 发布 29.8-29.9/s、rate_skip=0，采集支路 30fps 满速透传实锤。

**回传降载再减半：push_max_width 960→640**——`quest3_video_streamer`。960 宽下 WebRTC 实测 10-13fps/路；降 640（360p，编码像素较 1080p −84%）后 13-17fps/路。降载只作用于 WebRTC track（等比缩放 + 黑帧同尺寸保编码 context），采集抽头全分辨率全速不受影响。yaml 注释内调参阶梯同步更新。

**采集抽头限流器抖动误杀修复**——`quest3_video_streamer`。setter 修复后实机：capture 三路 30fps 钉死、queue_full=0，但 tap 只放行 18-20/s——限流器硬卡整周期（33.3ms），30fps 源到达间隔 33.3±5ms 抖动，早到几 ms 的帧被 rate_skip 误杀 1/3。改 0.8×period 容差：≤37.5fps 的源全通过，更快源仍限在目标附近。**同时记录里程碑**：修复后 WebRTC H264 回传达 10-13fps/路（5.5Mbps，此前 1-2fps），采集/捕获/推流首次三线并发满速。

**全案根因定案：rosidl `data` setter 逐字节校验吃光 GIL**——`quest3_video_streamer`。**py-spy 实锤**：三路 collect-tap 线程全部停在 `sensor_msgs/msg/_compressed_image.py:183-184` 的 genexpr——`msg.data = bytes` 触发 rosidl setter 的 `all(isinstance(v, int) ...)` + `all(0<=v<256 ...)` 两遍逐字节 Python 校验（200KB JPEG ≈ 40 万次迭代/帧，持 GIL 数百 ms）；3 路 × 30fps 把 GIL 占满，捕获线程/事件循环/WebRTC 全部饿死。**这解释了全部历史症状**：采集 video8 3fps、推送开关 ON/OFF 的 16↔30fps（无 Quest 也复现）、三种编码配置吞吐不变（编码器从来无罪——VP8/x264 修复仍保留，是真实加速只是非瓶颈）。**修复**：`msg.data = array.array('B', jpg.tobytes())` 走 setter 的 array 快路径（C memcpy），本地实测 10.1ms → 0.016ms（630×）。collect_tap + preview 两处都改。Jetson cv2 5.0 多线程扩展比 3.9×，GIL 释放正常，洗清嫌疑。

**相机 eager start：采集/预览与 Quest 视频解耦**——`quest3_video_streamer`。此前相机源仅在 WebRTC track 首帧时惰性打开——Quest 不开 video feed，采集抽头就拿不到任何图像。新增 `eager_start_sources`（yaml 默认 true）：service 启动即打开全部源，Quest 连接后 track 复用已开源（`WebcamVideoSource.start`/`RosImageSource.start` 补幂等守卫）。同时支撑"无 WebRTC 负载时采集支路满速"的对照实验（三组编码配置吞吐不变已证瓶颈非编码器，疑似 aiortc 媒体面 Python/GIL 开销拖垮全进程，待 py-spy 实证）。

**H264 上线后流塌缩修复：x264 线程爆炸**——`quest3_video_streamer`。**实机证据**：重排生效（`codec 首优 video/VP8 -> video/H264` ×3、协商 H264）后，流从 ~3fps 一路塌缩到 **0.0fps/0kbps**，且捕获/采集线程照旧被饿死。**根因**：x264 默认 `threads=1.5×核数`——12 核 Jetson 每编码器 18 线程、3 路共 54 线程在弱 ARM 核上 convoy（zerolatency 的 sliced-threads 同步开销在小核上被放大）；对比 VP8 基本单线程所以只是慢不是塌。**修复**：x264 options 加 `threads=2`（3 路共 6 线程，给捕获/采集留核）；本地 540p veryfast threads=2 实测 215fps（x86），Jetson 预期每路 15-30fps。另加补丁幂等守卫（模块重复 import 时不再自包装递归）。

**H264 强转失效根因定案与修复**——`quest3_video_streamer`。**实机证据**：`offer video codecs: ['VP8','rtx','VP9','H264','AV1','H265',...]`——Quest **明明提供 H264**，应答却仍是 VP8（`h264_forced` 空转实锤）。**根因（aiortc 1.15 源码实证）**：`setCodecPreferences` 在应答路径无效——`createAnswer` 直接用 `setRemoteDescription` 时算好的 `transceiver._codecs`（offer 顺序，VP8 在前），发送端 `RTCRtpSender` 以 `_codecs[0]` 建编码器。**修复**：弃用 setCodecPreferences，改为在 setRemoteDescription 之后、createAnswer 之前直接重排 `transceiver._codecs`（H264 及其 RTX 伴随提前；条目来自协商公共集即 offer 参数深拷贝，无 fmtp 不匹配风险）；公共集无 H264 时打日志保持 VP8。新增重排日志 `codec 首优 VP8 -> H264`。新增 2 例单测（重排序+RTX 伴随、无 H264 回退）。

**低帧率根因定案与修复：VP8 软编码挤爆嵌入式 CPU**——`quest3_video_streamer`。插桩证据链：tap `publish=3.4ms`（DDS 无罪）、编码线程有活跑不动（encode 20ms 吞吐仅 7.5/s）、捕获线程进程内 17-20fps（单跑 30）、**协商日志显示三路全是 VP8**（初判 Quest 未提供 H264；当日实机日志证伪——offer 含 H264，实为强转代码在应答路径无效，见上方 H264 条目）——aiortc 给 libvpx 设 `cpu-used=-6`（比默认更慢的画质向档位），三路软编在 Jetson 上吃光 CPU 并阻塞默认 executor，全进程线程互相饿死。修复三件套：① **VP8 `cpu-used=-6→8` 补丁**（realtime 档位上限提速，镜像 aiortc 1.15 原生 options 仅改此一项）；② **x264 preset veryfast 补丁**（dormant：Quest 端 app 开 H264 后立即受益）；③ **回传降载旋钮** `push_max_width`/`push_fps`（默认 960/30 ≈ 编码像素 -59%），track 级等比缩放 + 黑帧同尺寸保编码 context，采集抽头全帧全速不受影响。两补丁带 `encoder_patch_status()` 启动自证行（结构不符静默回退会显示 OFF）；`apply_offer` 新增 offer 编码列表日志（Quest 报了什么一眼可见）。新增 `test_push_throttle.py`（11 例，含补丁激活断言）。调参阶梯：960/30 → 960/15 → 采集时 mute 腕部轨。

## 2026-08-31

**E2E 延迟：QoS 只留最新帧**——腕位 / body / 关节状态 / 指令流改为 `BEST_EFFORT` `KEEP_LAST` **depth=1**。mocap 发布、teleop 订阅、driver/monitor 指令订阅对齐。旧 depth 10/20 会在 IK 跟不上时把旧腕位排队，表现为「手已经停了臂还在走」。Joy/头/hips/landmarks 同步改，避免 BEST_EFFORT 发布对不上 RELIABLE 订阅。

**Web Launch 日志**——环形缓冲 500→**8000**，WS 实时推尾 **800** 行；新增 `GET /api/v1/logs` 拉全量。系统 tab 日志窗 240px→**70vh**，复制/下载。

**运维清场 + 夹爪刷屏**——`jetson_teleop_start.sh` 强清补 `controller_start_gate`、`data_collect`、`keyboard_controller`、`ik_solver`。夹爪 `log_interval_s` 默认/yaml/launch 均 **0.0**，关掉每 2 秒一条的状态日志。

**左臂 TCP 偏置撤回**——`tcp_offset` Y **−0.147 → 0**（与右腕一致）。此前为贴 Quest 左手视觉中心，实测不必。

**数采段边界速率不为负**——writer 每段新建计数归零；`_prev_counts` 随段清零，delta 再 `max(0, …)`，避免下一段第一秒打出负 Hz。

**数采防护体系 + launch 参数优先级修复**——`morla_data_collect` / `morla_web_manager`。①launch 参数改空串默认 + OpaqueFunction：**yaml 成为唯一默认来源**，此前 launch 默认值（如 `arms` 默认 `left,right`）静默盖掉 yaml——"改了 yaml 却采出旧 schema"即此机制；CLI/预设显式传参仍可覆盖。②**domain 单例锁**：`/data_collect/control` 谁订阅谁开录是双份数据的根因（残留节点 + 再启动 = 双开同录）；节点对 `/tmp/morla_data_collect_domain{N}.lock` 取排他锁，第二个节点构造即拒绝，session 不同也不放过。③**段号原子占位**：mkdir 抢号撞号让位——老残留进程不持锁，段号互斥由文件系统原子性兜底（此前一个 start 双节点各写 000000/000001）。④**多节点红条**：web 按 `/data_collect/state` 发布者计数 >1 即提示残留。⑤**低帧率告警**：录制中参考相机实率 < dataset_fps/2 → state `low_fps_warning` + 日志节流 WARN + 卡片红条。⑥web 重启端点等旧进程真退出再启（单例锁窗口）。另修：cameras 逗号字符串曾被逐字符拆解；jpeg_quality 参数此前被静默忽略。

**auto_scan MJPG 确定性优先（非帧率根因，实机已证伪带宽假设）**——`quest3_video_streamer`。MJPG/JPEG 提至 110 分消除平票依赖枚举序的不确定性。**实机验证**：d435i 彩色节点仅播 YUYV（无 MJPG 可优先）、三相机分属不同总线且 1080p YUYV 单跑满 30fps——采集低帧率非带宽问题；双订阅者收到完全相同帧集合证明丢失在 streamer 进程内部（抽头总共只发了那么多）。`test_scan_mjpg.py` 保留作为确定性回归。

**Web 数据采集卡片 + 独立泳道**——监控页顶部录制控制（开始/停/下一段/暂停/丢弃 + 任务文本）；数采 `LaunchManager` 与遥操预设解耦，互不挡启动。`/teleop/start` 订阅改 VOLATILE 才能收到 web 一次性触发。详见 8/28 VLA 条目补记。

**Web 预设：单左臂 + 左夹爪**——`presets.yaml`「Left arm + left gripper (no right arm)」。`full_teleop` / `astral_dual_arm_teleop` 新增 `arm_side:=left|right|both`（默认 both）；left 时不启右臂遥操节点，mocap 只发左腕。

**Web 管线延迟面板**——`morla_web_manager` 系统页。只读：mocap 腕姿 stamp 龄期、`ik_solver_*/ik_status` 求解耗时、`joint_commands` stamp 作为 VR→指令端到端；左右各一列，带 Hz / FAILED / 过期灰显。

**左臂 TCP 偏置**——`astral_arm_teleop_left.yaml` `tcp_offset` Y **−0.147 m**（右腕仍为 0）。

**Quest 面板：D435i 缩回、勿挡左侧腕部**——`quest3_video_streamer`。分辨率仍是 1080p30，未改。

- 8/28 `x=0.05` / `size=0.88` 半宽 ≈1.09 m，盖住左上/左下腕部约一半。改回 `x=0.50` / `size=0.60`（间隙约 0.42 m）。腕部左侧叠放不动。重启 streamer 并让 Quest 重连。

**D435i IR 节点带 UYVY 也不当彩色**——`scan.py`。红外立体旁路常挂 UYVY（packed IR 不是 RGB），旧打分会当彩色、靠更低 video 号赢过 RGB 节点。节点上只要有 GREY/Y8I 等 IR fourcc 就打 10 分丢掉。

**大幅 roll 逃逸迟滞**——`geometric`。末端 roll 超腕行程时局部 ψ 窗闪空，旧行为同帧全局逃逸再被 `psi_ref` 拉回，肩部抽搐。

- 局部窗连续空 **4** 帧才允许全局逃逸；逃逸后驻留，home 窗连续可行 **20** 帧才返回（~150 Hz 下约 27 / 133 ms）。冷启动立即全局。`test_geometric_ik` 加 `roll_escape_hysteresis`。

**geometric 默认 + 人臂肘先验**——`astral_arm_teleop`。左右 yaml / 节点默认 `solver_type: geometric`，`use_human_elbow: true`。

- 现象：只跟腕 6D 时冗余臂角由连续性随便锁一支，人转肘平面机器人肘不动；人臂伸直时 IOBT 肘偏置会被当成满权重 ψ，放下手臂肘拧到固定错角。
- 订 `quest3/body_joints` 的 `{side}-arm-upper/-lower`，大臂方向旋到 `*_base_link` 得 `psi_ref`，局部 ψ 窗以它为中心、评分加 `w_psi_ref·|ψ−ψ_ref|`（默认 weight 2.0）。人转腕带肘 → 机器人肘跟转；人只转腕 → 肩肘不动。超时/缺名/不可行自动回退纯连续性。
- 伸直度门控 `human_elbow_min_sin=0.15`（肘尖离肩腕线 ≲4 cm）关先验；肘方向 EMA 0.15 s。Latency 行报 `psi_off`。
- 调参：`pos/rot_smoothing` 0.5/0.6→0.4，`max_joint_vel` 4→6。tune 图加「几何 臂角」按钮。假人臂 `body_joints_sim`（elbow_circle / wrist_spin / combined）。

**肘伸直软墙**——`geometric`。全伸展 S/E/W 共线、q4 撞限位 → IK 无解 → 臂卡一下。

- `reach_margin=0.01`：腕目标径向钳在肩心 `l_se+l_ew−margin` 内，手感是墙不是顿；`sin α < 0.05` 时 ψ 网格塌成上一帧单候选。Latency 行报 `reach_clip`。

**免 DH 几何臂角闭式 IK**——`astral_arm_teleop` 新增 `solver_type: geometric`（当时 yaml 默认仍是 `urdf_numerical`；同日稍后改为 geometric 默认，见上）。

- 现象：`analytic_dh` 靠硬编码 MDH + 关节翻转，和 SolidWorks URDF 对不齐；`urdf_numerical` 贴 URDF 但每拍 LM，遥操 150 Hz 偏重。
- `ik/geometric.py`：q=0 时用 Pinocchio 从 URDF 抽出肩/肘/腕中心与关节轴（`left_base_link` / `right_base_link`），POE 正运动学 + 臂角 ψ + Paden-Kahan 子问题闭式求 q。无 DH 表、无 theta 偏置、无轴翻转；输出已是硬件约定，节点不 `flip_q`。
- 连续解沿用 `analytic.py` 的局部 ψ 窗 + 迟滞 + 全局回退。工厂别名 `swe` / `poe` / `geometric_arm_angle`。
- 离线套件 `test_geometric_ik`：FK 对 Pinocchio 到浮点精度、FK→IK 回环、限位、不可达、轨迹连续性、双臂/单臂工厂。

## 2026-08-28

**Quest 面板：腕部靠左缩小，RealSense 居中放大**——`quest3_video_streamer`。

- 左上/左下腕部 `x -0.6→-0.95`、`size 0.38→0.28`；D435i `x 0.5→0.05`、`size 0.6→0.88`。
- auto_scan 无同名 yaml 块时按 fourcc：YUYV 居中 1080p，其余当腕部左侧叠放 720p（换 USB 口编号变了仍是这套布局）。

**D435i auto_scan 勿推红外**——`quest3_video_streamer`。

- 彩色节点常是 `VIDEO_CAPTURE_MPLANE`，扫描只 enum 了普通 capture，RGB 格式列表为空；同机红外 GREY 得分更高，推流变成红外。
- 同时 enum mplane；USB 接口 `1-2:1.0` / `1-2:1.3` 归到同一设备再比分；得分低于彩色阈值的红外/深度组直接丢掉。

**VLA 数据采集包**——新增 `morla_data_collect`；`quest3_video_streamer` 加采集抽头；`morla_web_manager` 集成录制控制卡片。

- web 集成：`morla_web_manager` 监控页新增**数据采集卡片**（开始/停止保存/下一段/暂停/丢弃 + 任务文本 + 录制状态徽标与流率/掉帧显示），REST `/api/v1/collect/{control,task}` 桥接 `/data_collect/*` 话题（纯话题接口，CLI 启动的采集节点同样可控）。
- **数采节点独立泳道（与遥操预设解耦）**：`web_server` 新增第二个 `LaunchManager` 专管数采 launch，`POST /api/v1/collect/launch/{start,stop,restart}` + ui_state `collect_launch`（state/preset/uptime/pid）；数采卡片右上角直接启停节点。数采是纯订阅者，启动跳过孤儿检测（`start(check_orphan=False)`）；`_find_orphan` 对 `morla_data_collect` 命令行对称豁免——两条泳道任意顺序起停互不阻塞。「系统」tab 遥操预设下拉过滤 `morla_data_collect` 条目（避免占用互斥的主泳道），其 presets.yaml 条目保留为泳道启动命令来源。
- 修复：数采节点遥操事件订阅 QoS 由 latched 改为 VOLATILE（DDS 订阅端 durability ≤ 发布端；`/teleop/start` 在 web_monitor 是刻意的 volatile 一次性触发，latched 订阅完全收不到——事件是跃迁语义，仅损失订阅前的历史值）。
- 修复：`replay_rerun` 适配 rerun ≥0.24 API（`set_time(timestamp=)`）+ h5py 数据集句柄逃逸（文件关闭后逐帧读图必崩，此前被缺包 skip 掩盖）。

- 目标：为 OpenPI pi0.5 采集训练数据。四段式：采集（raw HDF5，与遥操并行）→ 离线对齐（严格 1/fps 网格，action=state[t+1] 可切 command）→ 清洗校验（11 条规则 + quarantine 隔离）→ LeRobot **v2.1** 导出（逐字段对齐 `VLA/lerobot @0cf86487`，纯 pyarrow+av 手写、不依赖 lerobot 包）；另附 Rerun 回放。
- schema 采集前配置并冻结进 meta.json：**臂侧可选（`arms`: 左/右/双臂）**、左右末端独立 gripper/wuji/none（跟随所属臂）、腰/头可选纳入；state 布局 `[左臂7?, 右臂7?, 左EE?, 右EE?, 腰2?, 头2?]`，夹爪用闭合比（0..1，同 pi0.5 dim6 约定）。
- 控制：键盘热键（s/q/d/n/p/t）与 `/data_collect/control` 话题双通道；`/data_collect/state` latched JSON 报各流帧率/丢弃数；任务文本每段一条、录制中可改。
- streamer 抽头 `~/collect/{label}`：全分辨率 JPEG，**仅在有订阅者时编码**（不采集零开销），与 preview 同线程模型、跟随门控；参数 `collect_tap`/`collect_tap_fps`/`collect_tap_quality`。
- 测试：36 项离线单测（schema/align/validate/convert 逐字段校验 v2.1 布局）+ ROS 进程内冒烟（假话题 start/stop/discard/pause 全流程）。

**夹爪开合限速 + 扳机整形**——`astral_gripper_teleop`。

- 现象：手柄扳机控制夹爪"一扣到底"，中间态几乎不可见。链路（Unity `Axis1D` F3 模拟量 → Joy → ratio → 驱动每拍绝对角度 0x97/0x98）全程比例，但**没有任何速度整形**，夹爪伺服对每个目标全速赴位，快扣时宏观上就是直接合死。
- `pinch_gripper_node` 新增合并输出限速 `max_ratio_rate`（默认 **2.5 ratio/s**，全行程 ≥0.4 s；≤0 关闭），trigger/pinch 两路合并后的比值按拍斜坡下发；首拍直取，避免持握中恢复时先松一下。比例性保留：扳机停半路夹爪停半路。
- 扳机路径补整形链（此前无任何滤波）：死区重标定 `[trigger_deadzone,1]→[0,1]` → `trigger_gamma`（默认 **1.4**，前半程更细，便于半捏）→ `trigger_ema_alpha`（默认 **0.4**，去 Joy 抖动）。
- 参数在 `gripper_teleop.yaml`，节点启动时读取（无热改回调，改后需重启节点）。

**肘折偏置，方便内收**——`astral_arm_teleop` `urdf_numerical`。

- 现象：J4 卡死少了，但肘经常伸太直（`ik_q4_max=-0.25` 只剩约 14°），J3 对末端几乎没力臂，手难往内收。左 J2 内收侧 URDF 只有 0.3 rad，软限位 0.18 还吃掉大半。
- `ik_q4_max` −0.25→**−0.45**；新增单向折肘 `ik_w_fold=0.015` / `ik_q4_fold=-1.20`（比这更直才罚，约 2 mm 级位置代价；位置误差仍优先）。限位铰链按「到 0 的短边」缩 margin，J2 内收不再一碰就顶。

**暂时撤回「贴上一帧」默认值**——`astral_arm_teleop`。求解器仍支持这些项，默认改回改之前：`ik_w_ori=0.3`、`ik_w_reg=1e-4`、`ik_dq_max=0`、`ik_w_pref=0`。yaml 注释里留了 0.40 / 0.02 / 0.30 / 0.004 以便恢复。J4 帽和工作球不动。

**驱动 SESSION 低通 + 建议 100 Hz**——`astral_robot_control`。

- yaml 新增 `lpf_enable` / `lpf_alpha`，connect 与 `~/ready` 后调 SDK `set_lpf`（进 WORK 可能重置，故每次 ready 重写）。默认开、`alpha=0.35`（100 Hz 约 19 ms）。
- `obs_hz` / `ctrl_hz` / `control_rate` / `state_publish_rate` 50→**100**（与遥操 150 对齐的起步档；UDP 稳再升 150）。

**URDF IK 贴上一帧，少换怪构型**——`astral_arm_teleop` `urdf_numerical`。

- `ik_w_reg` 1e-4→**0.02**（贴 `q_prev`；肩/肘权重大、腕小）。另：`ik_dq_max=0.30` 每拍关节步进盒，挡住一次解跳到另一支 IK；`ik_w_pref=0.004` 冗余时往 `init_pose` 靠。**`ik_w_ori` 提到 0.40**（在强正则下跟 Quest 腕朝向；0.12 会漂）。位置权重不变。tune 正则滑条范围扩到 0.1。

**URDF IK 约束 Joint4 勿伸直卡死**——`astral_arm_teleop` `urdf_numerical`。

- 原因：`left/right_joint4` URDF 上限是 **0**（完全伸直）。LM 只有 `w_reg=1e-4` 贴上一帧，跟手外伸时 q4 顶死在 0，肘奇异，再收不回来。
- 不改 URDF。IK + SafetyFilter 使用 `ik_q4_max`（默认 **−0.25 rad**，约 14° 余弯）；另加近限位铰链残差 `ik_w_limit`。`ik_q4_max≥0` 关闭帽、回到 URDF 0。可热改 / tune 滑条。
- **边界跟手**：顶到 `ik_q4_max` 后不再因 `ik_ok=false` 丢解冻住。LM 有限解一律下发，q4 停在余弯、其余关节继续跟可达面上的手；肘贴上限时姿态权重降到 0.2。延迟日志 `ik_sat` 表示在边界上滑、`ik_fail` 才是真失败。

**`workspace_radius` 接到 IK 前**——`astral_arm_teleop`。

- 末端目标相对 `left_base_link` / `right_base_link` 原点超半径则径向收到球面再求解。yaml 默认 **0.55 m**（工作位 |p|≈0.44 m，零位下垂≈0.49 m，关节限位内最远≈0.59 m）。≤0 关闭。可 `ros2 param set` 热改。
- 延迟日志增加 `ee_r=…mm`（夹之前的目标距离）和 `ws_clip` 次数。

**URDF 数值 IK 改到臂基座系**——`astral_arm_teleop` `urdf_numerical`。

- 此前 Pinocchio `oMf` 在 universe（= `body_link` 躯干）求解。现 FK/IK 位姿改到 `left_base_link` / `right_base_link`（肩安装座），与 DH 解析解同一类臂基座系。
- `*_base_link` 相对躯干只有平移、无旋转，Quest `robot_world` 增量方向仍可与 `vr_to_arm_rot=I` 相加；`robot_init` 原点会从躯干原点变为肩原点（日志 EE0 的 z 不再含 0.412 m 肩高）。

## 2026-08-28（超时）

**超时统一 1.5s**——`astral_arm_teleop` `data_timeout`、`astral_robot_control` `command_timeout_s` 均改为 1.5s（代码默认 + yaml）。VR 断连后 teleop 再撑 1.5s 才 disarm；teleop 停发后 driver 再 1.5s 才停刷电机目标。

## 2026-08-27（四）

**Homing 途经点（先抬再伸，避桌）**——`astral_arm_teleop` / `astral_mujoco_sim`。

- 启动不再关节空间直达 `init_pose`（那条弧会刮桌）。新增 `init_waypoints`（扁平 7×N，硬件约定）：按序走完再到工作位。当前左右各一个途经点 `[-1, 0, 0, -2.20, 0, 0.46, 0]`。
- `init_timeout` 15→40s（两段慢速）。MuJoCo 默认起始改为零位，便于看 rest → via → init。

## 2026-08-27（三）

**视频扫描按像素格式挑彩色节点（修 RealSense 无画面）**——`quest3_video_streamer`。

- 问题：插上 D435i 后 auto_scan 按"最小编号"选中深度节点 `/dev/video4`（Z16），OpenCV 打不开 → 该轨反复黑帧重试刷屏、Quest 无画面。
- `scan.py`：同一物理设备多采集节点时，用 `VIDIOC_ENUM_FMT` ioctl 枚举**像素格式**打分（彩色 YUYV/MJPG/NV12=100 > 红外 GREY/Y8=10 > 深度 Z16=0），自动选中彩色节点（D435i → video8 一类）；深度-only 节点直接丢弃。`v4l2-ctl` 仅作补充，没有该命令也不退回选 video4。并输出 `force_mjpg` 建议（YUYV 节点强制 MJPG 会让 cv2 打不开）。sysfs 名含 RGB/Depth/Infrared 作为无 fourcc 时的回退。
- `streamer_node.py` `_scan_spec`：`force_mjpg` 默认采用扫描建议；新增 `{label}.device` 覆盖项。`params.yaml` 新增 `video8` 显式块（D435i 彩色、右侧布局）。
- 修复：`hook_gate_visibility()` 接线被误删导致 web 改勾选后 Quest 面板不跟随，已恢复。

## 2026-08-27（二）

**静音轨 Quest 面板缩小到边缘 / 可隐藏（track_visibility）**——`quest3_video_streamer` / `astral-tracking`。

- 需求：web 取消勾选后 Quest 面板显示黑色，希望"什么都不显示或变得很小"。视频帧本身无 alpha，改为 PC 经信令通道发 `track_visibility {enabled: [...]}` 消息、Quest 端按 label 处理被静音面板。
- `quest3_video_streamer`：`StreamGate` 新增 `on_change` 钩子；service 新增 `send_track_visibility()`——门控每次变化即推（经 `loop.call_soon_threadsafe` 跨线程调度），进入 playing 时也同步一次初值；总开关关闭 = enabled 空表（全部静音）。黑帧仍照常发（2fps 保活，恢复无重连）。
- `astral-tracking`（Quest app）：`VideoStreamManager.HandleTrackVisibility`——被静音面板默认**缩小为视野左下角的小条**（多路横向排开；`minimizeMutedPanels`/`minimizedScale`/`minimizedOrigin`/`minimizedSpacing` 可在 Inspector 调；关掉开关则**整个隐藏**）；恢复勾选时还原原布局。旧版 app 忽略该未知消息（行为不变=黑面板）；**新行为需在 Unity 重新构建 APK 并安装到头显后生效**。

## 2026-08-27（前条）

**安全修复：`/teleop/armed` 闸门收紧（审查 C2）**——`astral_arm_teleop` / `morla_web_manager`。

- `_on_armed` 不再无视消息内容任意武装：仅 `Bool(true)` 生效；未标定（无 vr_init）拒绝并告警——顺带堵住 web「恢复」latched 残留让新启动节点自动武装的旁路。
- 新增 disarm 原因区分：操作员「暂停」→「恢复」可直接 armed（零点仍有效，体验不变）；**VR 看门狗超时 disarm（fault）→「恢复」被拒绝**，必须重新 `/teleop/start` 重标定 vr_init——断连后手可能已移动，沿用旧零点会产生计划外快速移动。fault 状态粘性：fault 后点「暂停」不会降级，只有 start 能清除。
- `morla_web_manager` README 更新「恢复」按钮的生效条件。
- 冒烟（状态机桩件测试）：`data=false` 不武装 ✅；未标定拒绝 ✅；暂停→恢复 ✅；fault 拒绝恢复 ✅；fault 后暂停不降级 ✅；start 清除 fault 后暂停/恢复如常 ✅。

**安全修复：VR 断连即 disarm + 全链路 NaN 屏障（审查 C5/C6）**——`astral_arm_teleop`。

- **VR 看门狗改为 disarm**：`data_timeout` 超时不再只是静默停发——立即 `_armed=False` + ERROR 日志，恢复必须重新 `/teleop/start`（顺带重标定 vr_init、重置 EMA，消除重连跳变）。此前断连后 1.5s 内手臂仍按冻结的最后一帧完成在途动作，且重连后无重闸门直接恢复。`data_timeout <= 0` 可关闭（语义与驱动 `command_timeout_s` 一致，不推荐）。
- **`data_timeout` 默认 1.5→1.0s**（代码 + 左右 yaml）：容忍 VR 链路短暂抖动（亚秒级断连不误触发）；超时前 teleop 持续发冻结目标（driver 跟随），teleop 停发后 driver 的 `command_timeout_s`(0.5) 接管保持。运行中可热调（`ros2 param set`）。
- **NaN 屏障三处**（此前全链路无 isfinite 检查，一帧 NaN 永久污染）：① `_on_wrist` 拒绝非有限 position/四元数——此前 NaN 四元数穿过模长校验（`NaN > 0.1` 为 False）且 `Rotation.from_quat(NaN)` 会在回调里抛 ValueError；② `PoseProcessor.update_vr_pose` 入口拒绝非有限输入——EMA 吃进 NaN 后 `a·NaN+(1-a)·x=NaN` 永不恢复；③ `SafetyFilter.filter` 入口 isfinite 检查——非法帧保持上一合法指令、不更新 `prev_q`（此前 NaN 经 clip/速度门双双放行并写入 prev_q，恢复后第一帧速度限制失效），info 新增 `invalid` 标记。
- 冒烟：SafetyFilter NaN 保持 / 恢复后限速完好 / Inf 阻断 / 无历史回零 ✅；PoseProcessor 丢弃 NaN 位置与旋转、正常更新仍跟踪 ✅；三文件 py_compile ✅。

**Web 实时相机画面（JPEG 抽帧 + MJPEG 转发）**——`quest3_video_streamer` / `morla_web_manager`。

- `quest3_video_streamer` 新增 `preview.py` `PreviewPublisher`：每路相机发布 `~/preview/{label}`（CompressedImage，BEST_EFFORT，默认 10fps/640宽/q65）。**延迟隔离设计**：捕获线程只做 gate 检查 + 速率限制 + maxsize=1 队列 `put_nowait`（忙则丢）；JPEG 编码在独立 daemon 线程——不进捕获线程（不拉长采集周期）、不进 asyncio 循环（不干扰 Quest RTP 节奏）；跟随门控（静音路不发）。参数 `web_preview`/`_fps`/`_width`/`_quality`。两个 source 适配器加 `preview_hook` 抽头（webcam 抽原生 BGR 免回转，ros 抽转换后 RGB）。
- `morla_web_manager`：按 gate_state 相机列表动态订阅 preview 话题（BEST_EFFORT depth1）；新增 `GET /api/v1/video/feed/{label}`（MJPEG multipart，浏览器 `<img>` 原生支持）与 `GET /api/v1/video/snapshot/{label}`（单帧）；视频卡片新增「实时画面」开关 + 勾选相机的 320px 预览网格，取消勾选即停。
- 延迟预算：Quest 链路 ≈0 影响（共享读 + 独立编码线程）；控制链路 ≈0（Jetson 每路 ~2-5% 单核）；DDS ~0.3-0.6MB/s/路；web 预览自身延迟 ~0.2-0.4s（监控用途）。
- 冒烟：preview 话题建立✅；monitor 动态订阅✅；CLI 发测试 JPEG → snapshot 200 返回原图✅、feed multipart 帧格式正确✅、未知 label 404✅。

**视频设备自动扫描（web 免配置可选）+ 懒打开**——`quest3_video_streamer` / `morla_web_manager`。

- `quest3_video_streamer` 新增 `scan.py` + `auto_scan` 参数（默认 true）：启动时枚举 `/dev/video*`（ioctl `VIDIOC_QUERYCAP` 查 `V4L2_CAP_VIDEO_CAPTURE` 跳过 metadata 节点，按物理设备 sysfs 父级去重），label = 节点名（`video0`…），面板一行网格自动排布——**不再写死设备节点/中文名**。同名 yaml 块可覆盖单路字段；一台没扫到回退 `cameras` 列表；`cameras` CLI 覆盖联动 `auto_scan:=false`。D435i 多采集节点取第一个不一定是彩色，需显式块（README 已注）。
- **懒打开 + 失败黑帧**：sender 启动不再打开全部相机；每轨首个未静音帧才 `source.start()`，打开/读帧失败 → 黑帧 + 1s 退避重试（设备没插/被占用不再拖垮 sender，插上即恢复）。
- `~/gate_state` JSON 增加 `cameras:[{label,device,source,preset,sysfs_name}]`——web 端不再解析 streamer 的 yaml。
- `morla_web_manager`：`/video/status` 在线时改用 gate_state 的相机列表（exists/sysfs 每次轮询实时刷新）；**离线时 monitor 自行扫描主机设备，可预选**（latched `~/active_cameras` 在 streamer 启动后生效；总开关服务仍需在线）；在线判定加 `count_publishers` 活性检查（latched 残留不再误报在线）。VideoCard 离线时勾选/路数可用、仅总开关禁用。
- 冒烟：无相机主机 → auto_scan 回退 wrist 列表✅；gate_state 带 cameras✅；web 在线/离线切换正确✅；离线预选 latched 下发 → streamer 启动拾取并对未知 label 告警过滤✅。

## 2026-08-26

**视频回传并入完整遥操 + 运行时可门控（web 可选路/开关）**——`quest3_video_streamer` / `morla_bringup` / `morla_web_manager`。

- `quest3_video_streamer` 新增**运行时推流门控**（`gate.py` `StreamGate`）：服务 `~/set_push_enabled`（SetBool 总开关）、latched 话题 `~/active_cameras`（逗号 label 子集，空=全部）、latched 状态 `~/gate_state`（JSON）。门控在 track 帧级生效：被关的轨改发 2fps 黑帧（Y=16/U=V=128，几乎不占带宽，Quest 面板变黑，恢复即时）——不做 SDP 重协商，相机集合仍在启动时固定。
- 修复 `StreamGate.set_active` 嵌套取非重入锁死锁（topic 回调卡死 spin 线程，服务也连带无响应）。
- 默认相机改为实际硬件：`cameras: ["wrist_left","wrist_right"]`（当前两路 USB，无 RealSense）；d435i 配置块保留。`multi_camera.launch.py` 新增 `cameras` CLI 覆盖；仅当 d435i 在列表且 source≠v4l2 才拉 realsense 节点。
- `full_teleop.launch.py`：`with_video:=true` **默认**带视频回传；启动时自动 `adb reverse tcp:8000/8765`（无 adb/无设备仅 WARN）；`video_cameras:=` 透传。仍需 Quest 端开启 video feed。
- `morla_web_manager`：新增 `GET /api/v1/video/status`（配置相机列表 + `/dev/videoN` 在线/sysfs 名 + gate 实时状态）、`POST /video/push`、`POST /video/cameras`；monitor_node 订 `~/gate_state` 镜像进 `ui_state.video_gate`；系统页新增「视频回传」卡片（总开关 + 路数下拉(快捷勾前n路) + 逐路勾选 + 在线点，streamer 离线时禁用）。
- 冒烟：streamer 起 2 源 + gate 初始化✅；`set_push_enabled false`→gate_state push_enabled=false✅；`active_cameras:='wrist_right'`→active=[wrist_right]✅、空串恢复全部✅；web 三端点全通✅、`ui_state.video_gate` 随 WS 下发✅。

**夹爪开度 1.5→2.5 + web 机器人按钮可用时机文档化**：

- `astral_robot.yaml` 左右 `gripper_open_rad` 1.5→**2.5**（真机实测目标全开角；若顶死回弹则回调 ~2.0）。之前"2.0 开到最大马上闭合"实为双流竞态所致（已修），非硬止点过流。sim echo 配置同步 2.5。
- `morla_web_manager` README 写清机器人模式按钮（急停/阻尼/位置保持/就绪/归零）可用时机：运行中✅、已暂停✅（driver 还活着）、已停止❌503 属预期（driver 随栈退出）——**先启动栈再点按钮**；运行中仍 503 则查 Jetson driver 是否旧代码（`ros2 service list | grep astral_robot_driver` 应列 5 个 Trigger）。
- 文档写明上电瞬态：ready 把夹爪归零(0)→遥操首条指令开到 open_rad，像"抽一下"，正常。

**真机修复：夹爪抽搐 + 开度不到位（双流竞态）**——`astral_gripper_teleop` / `astral_robot_control` / `morla_web_manager` / `morla_bringup`。

- 根因：`pinch_gripper_node` 同时发 `/command`(Float64) 和 `/joint_commands`(JointState)，driver 两个都订且各自映射 rad（ pinch 节点配置 vs driver 配置），Jetson 上两份配置不一致（0.8 vs 1.5）→ 两路 50Hz 流交替覆盖同一目标 → 夹爪按控制频率抽搐，且 pinch 端旧配置 0.8 导致"张开但不到位"。
- 修复：pinch 节点**只发 Float64 闭合比**（0=开 1=合），rad 映射唯一权威收敛到 driver `astral_robot.yaml`（1.5/0.0）；`joint_commands` 话题保留给直接发弧度的适配器。`gripper_teleop.yaml` 删 `open_rad/closed_rad`。
- `morla_web_manager`：左夹爪指令频率改听 `/left_gripper/command`（Float64），`TopicPair` 加 `cmd_kind`；状态显示仍走 driver 回显的 `/left_gripper/joint_states`（即 driver 实际用的 rad）。
- `head_teleop_node`/`controller_start_gate` 回退昨日多余的双 QoS 订阅（RELIABLE 发布↔BEST_EFFORT 订阅本就兼容，双订阅只会触发 incompatible QoS 警告噪音）；CLI `echo` BEST_EFFORT 话题用 `--qos-reliability best_effort`。
- 冒烟：pinch 只发 /command；driver dry_run 全程只见 rad=1.500 单值（无交替）；head 节点 0 条 QoS 警告、start 正确捕获 head_init=[0.1,0.05]、摇杆回中指令=[0.1,0.05]；web monitor 夹爪 cmd 49.4Hz health=ok。
- ⚠️ Jetson 注意：本修复要求 driver 端 `astral_robot.yaml` 的 `left/right_gripper_open_rad` 正确（1.5）；拉代码后需重新 `colcon build`（非 symlink 构建的 install 里 yaml 是构建时拷贝，旧值会残留）。

`morla_web_manager`：接入头部通道（配合 `morla_bringup/head_teleop_node`）。

- `config.py` `TOPICS` 新增 `head`（`/head/joint_states` + `/head/joint_commands`）——snapshot/健康巡检自动携带头部；**不设 cmd 频率下限**（启动前/手柄掉线时头部指令合法为 0Hz，设下限会误报"偏慢"）。
- 监视 tab 新增「头部 (yaw/pitch)」JointPanel、指令频率序列加"头部"、新增「头部 yaw/pitch」实时折线图；健康巡检面板加 `head→头部` 标签。
- README 话题表/WS 负载示例/Web UI 功能同步；冒烟：snapshot 含 `head`（values/cmd_hz/state_hz/health=ok, expected_hz=0）。

`morla_input`：README 补手柄 Joy 量程/符号约定（实测）。

- 话题表后新增「手柄 Joy 量程与符号约定」小节：`axes=[trigger,grip,stickX,stickY]`，stickX 左−1/右+1、stickY 后−1/前+1，回中 0；buttons 6 位 mask 顺序。实测左右手均达 ±1.0 满量程、回中干净 0.000，建议下游死区 0.05~0.1。

`morla_bringup` / `astral_mujoco_sim`：右手柄摇杆 → 机器人头部（yaw/pitch），绝对位置控制，与臂遥操共用 `/teleop/start` 闸门。

- 新增 `morla_bringup/head_teleop_node`（`with_head_teleop:=true` 默认）：订 `quest3/right_controller_joy`，`axes[2]`=stickX（左=-1 右=+1）→ yaw、`axes[3]`=stickY（前=+1 后=-1）→ pitch。弹簧回中摇杆用**绝对控制**：`head_target = head_init + stick*scale`，摇杆回中→头回初始位，`/teleop/disarm`→停发由 driver 位置保持。
- **启动记 `head_init`**：`require_start_signal:=true`（默认）下等 `/teleop/start`，触发时读 `/head/joint_states`（真机为 OBS 真实头角）记为基准，避免跳变；无该话题回退 `init_head_*`（默认 0,0）。发 `/head/joint_commands` 给 `astral_robot_control`（driver 唯一硬件权威）。配置 `config/head_teleop.yaml`：`yaw_scale`/`pitch_scale`（带符号，方向反了改符号）、限幅、deadzone、publish_rate、input_timeout。
- 非侵入：只订已有话题、只发 driver 已在订的 `/head/joint_commands`，不动 driver/臂/夹爪/灵巧手。Joy 与 joint_states 改**双 QoS 订阅**（真机 BEST_EFFORT + CLI RELIABLE），`ros2 topic pub` 注入也能测；`controller_start_gate` 同步补 RELIABLE 订阅。
- sim：`astral_sim_pipeline.launch.py` 加 `with_head_teleop`；`mujoco_sim_node` 新增 head echo（订 `/head/joint_commands`→回显 `/head/joint_states`+每秒日志 `[Head] cmd yaw/pitch`），MJCF 无头关节故 viewer 不动头，用于验证映射。
- 冒烟：start 前 `/head/joint_commands` 无输出；start 后 stickX=+1→yaw 0.8、stickY=+1→pitch 0.4、stickX=-1 stickY=-1→[-0.8,-0.4]；sim echo `[Head] cmd yaw=-0.4 pitch=0.2` 正确。真机首测需确认 yaw/pitch 正负号（反了改 scale 符号）。

`astral_mujoco_sim`：sim 管线接入手柄遥操 + 外部启动开关，便于在仿真里测手柄功能。

- 新增 `controller_start_gate`（`with_start_gate:=true` 默认）：左手柄 grip 键 → `/teleop/start`。
- gripper include 注入 `controller_joy_topic=quest3/left_controller_joy`：左 trigger 模拟量 → 左夹爪（新鲜时优先，否则回退 pinch）。
- `with_gripper` 描述更新为 pinch/trigger；package.xml 加 `morla_bringup` exec_depend；README 补「手柄集成（sim）」。
- 冒烟：sim 启动后 controller_start_gate(grip→/teleop/start)、pinch_gripper_node(joy=...left_controller_joy axis=0)、arm 节点 require_start_signal=true(等待 start) 均正常，无报错。

`morla_bringup` / `astral_gripper_teleop`：手柄接入遥操——左 grip 键作外部启动开关，左右 trigger 模拟量控夹爪。

- 新增 `morla_bringup/controller_start_gate` 节点：订 `quest3/left_controller_joy`，左手柄 grip 键（mask bit 5 = gripClick）按下沿发 `/teleop/start`（Bool true，RELIABLE+VOLATILE 一次性），等价 web「开始遥操」/CLI `/teleop/start`；再按一次重新记零点。`full_teleop.launch.py` 默认启动它。
- `pinch_gripper_node` 新增 trigger 合并：参数 `controller_joy_topic`/`trigger_axis`(0)/`trigger_deadzone`(0.05)/`trigger_invert`。订 `Joy`，`axes[trigger_axis]`(0=开..1=合)作为闭合比；手柄 Joy 新鲜时优先 trigger，否则回退 pinch（同侧手柄/裸手互斥，二者互补）。保持单发布者到 `/{side}_gripper/*`，无冲突。
- `gripper_teleop.launch.py` 暴露 `controller_joy_topic` 参数；`full_teleop` 为左/右 gripper 分别注入 `quest3/{side}_controller_joy`。
- `morla_bringup` package.xml 加 `sensor_msgs`/`std_msgs`，setup.py 加 entry_point。
- 冒烟：左 grip 按下 → `/teleop/start` data:true ✅；trigger=0.8 → `src=trigger close=0.80` ✅。

`morla_input`：接收 Quest 端新增的手柄按键/摇杆推流，发布 `sensor_msgs/Joy`。

- 新增 `_process_buttons_line`：解析 ``{Left|Right} buttons:, trigger, grip, stickX, stickY, mask`` → `Joy`。
  - axes=[trigger, grip, stickX, stickY]；buttons=mask 6 位 [primary(X/A), secondary(Y/B), stickPress, menu, triggerClick, gripClick]。
  - 话题 `quest3/left_controller_joy` / `quest3/right_controller_joy`；在 side-filter 前分发，双侧手柄始终转发。
  - 手柄位姿行（`controller:`）原有逻辑已处理（`controller_as_wrist` 时镜像到 wrist_pose）。
- **修复 mask 解析 bug**：原 `len(parts) >= 6` 误判（parts 实为 5 字段）导致 mask 恒为 0、按键全丢；改为 `>= 5` 读 `parts[4]`。
- `package.xml` 加 `sensor_msgs` 依赖；README 话题表补 Joy 说明。

`morla_bringup` / `astral_gripper_teleop`：右手新增「捏合→右夹爪」模式，`right_hand_source` 加 `gripper` 选项。

- `full_teleop.launch.py`：`right_hand_source` 增加 `gripper`（右捏合→右夹爪、不起 Wuji）；新增右 `gripper_teleop.launch.py hand_side:=right`（条件 `right_hand_source==gripper`）；`use_wuji` 改为 `right_src in (quest3,glove)`，gripper/none 均不起 Wuji；mocap `publish_landmarks_right` 对 gripper 仍为 true（右 pinch 需要 landmarks）。
- `astral_gripper_teleop`：`pinch_gripper_node` 本就支持 `hand_side:=right`（订 `hand_landmarks/right`、发 `/right_gripper/*`），无需改节点代码；README 补左/右两侧用法。
- CLI：`right_hand_source:=gripper with_gripper:=true`；Web：`presets.yaml` 新增「双臂+双夹爪(无灵巧手)」预设。
- 冒烟：`right_hand_source:=gripper` 启动后左/右两个 `pinch_gripper_node` 均起（rad=[1.5,0.0]），Wuji 节点 0 个。

`astral_gripper_teleop` / `astral_robot_control`：夹爪全开角度 2.0→1.5 rad（左右均改）。

- 现象：`open_rad=2.0`（机械硬止点）时遥操手一张开，夹爪开到最大后立即闭合——持续顶死硬止点 → 堵转 → 过流保护触发 → 电机失力回弹至合拢。
- 修复：`open_rad` 回退到 1.5（安全开度，接近全开但不顶死）。`astral_gripper_teleop/config/gripper_teleop.yaml` 与 `astral_robot_control/config/astral_robot.yaml`（`left/right_gripper_open_rad`）同步改为 1.5。
- 更新两包 README 真机全开值描述。

`astral_arm_teleop`：外部启动闸门 `require_start_signal` 真机默认改为开启。

- `config/astral_arm_teleop_left.yaml` / `_right.yaml`：`require_start_signal: false` → `true`。真机启动推流后臂节点只跟踪 `vr_current`、不自动记零点/不 arm；手摆好初始位姿后由 web「开始遥操」或 `/teleop/start` 记 `vr_init` 并 arm。
- `astral_mujoco_sim/launch/astral_sim_pipeline.launch.py`：`require_start_signal` launch arg 默认保持 `""`（透传 yaml=true），sim 同样默认开启外部闸门，统一真机/仿真流程。
- launch arg 非空时覆盖 yaml；full_teleop / dual_arm 默认 `""` 透传 yaml（=true）。
- 更新 `astral_arm_teleop/README.md` 默认值描述。

## 2026-08-25（续）

`astral_robot_control` / `morla_web_manager`：**真急停 + 阻尼释放**。原"急停"实为软 disarm（与暂停同效），改为硬件级断电；并补「阻尼释放」用于遥操收尾手动拖臂回 home。

- `astral_robot_control` driver 新增服务 `~/damping`（`motion_mode=0`，阻尼可拖拽）与 `~/position`（`motion_mode=1`，位置保持）；`~/estop` 语义明确为真断电（SDK `e_stop`/`disable`，臂失去保持力）。更新 driver docstring 与 README 服务表。
- `morla_web_manager` 后端：`monitor_node` 新增 `call_driver_service(name)`（缓存 Trigger 客户端，后台 spin 线程完成 future，web 线程轮询 done）；`config.py` 新增 `DRIVER_NODE` + 5 个 driver 服务名（`ASTRAL_WEB_MONITOR_DRIVER_NODE` 可配）。`web_server` 新增 `POST /api/v1/robot/{ready,home,position,damping,estop}`（`asyncio.to_thread` 调用，driver 未就绪返回 503）；**移除**旧 `POST /api/v1/estop`（软 disarm，与 `/api/v1/pause` 重复）。
- `morla_web_manager` 前端：`ControlBar` 急停按钮改为真断电（`api.robotEstop`，确认弹窗文案强调断电）；新增「阻尼释放」按钮（`api.robotDamping`）。`SystemTab` 新增「机器人模式」卡（一键就绪/归零/位置保持/阻尼释放/急停断电）。`client.ts` 移除 `estop`，新增 `robotReady/Home/Estop/Damping/Position`。
- 典型流程：遥操中 → 停止（臂保持末位姿）→ 阻尼释放（手动拖回 home）→ 位置保持/归零；急停=断电，恢复需重新一键就绪。
- 非侵入性：机器人模式按钮仅**调用 driver 已暴露的服务**，driver 是硬件权威，monitor 不新增硬件写入路径、不改 driver 逻辑。
- 冒烟测试：driver `dry_run:=true` + web_monitor 8091，`/api/v1/robot/{damping,position,ready,estop}` 均返回 `dry_run: skipped ...` 成功；旧 `/api/v1/estop` 已 405。
- 前端已 `npm run build` 重新生成 `web/dist`；两包 `colcon build --symlink-install` 通过；lint 无错误。

## 2026-08-25

`morla_web_manager`：Web UI 大幅增强——急停、Toast 通知、实时图表、健康巡检面板、系统运维 tab。全程非侵入（只读已有话题、只发已有 `/teleop/disarm` 等控制话题、不动其他包）。

- **急停**：`POST /api/v1/estop` 无条件发 `/teleop/disarm`（软暂停）；前端红色常驻按钮 + 确认弹窗。与 launch 状态无关，CLI 启动的遥操也可急停。
- **重启**：`POST /api/v1/restart` 停止后重新启动当前预设（仅对经本监控启动的预设有效）。
- **健康巡检**：`monitor_node` 新增 state 话题频率计数 + `health` 汇总（每实体 stale/state_hz/cmd_hz/expected_hz/slow/status + 总体）；`config.py` 新增 `EXPECTED_RATES_HZ`（`ASTRAL_WEB_MONITOR_EXPECTED_HZ` 可配）。修复原 lambda 闭包捕获循环变量导致指令频率计数失效的 bug。
- **ui_state / health 端点**新增 `state_rates_hz` 与 `health` 字段。
- **Toast 通知**：`useToast` store + `ToastHost`，替换原 `alert()`，成功/失败浮窗自动消失。
- **实时图表**：`historyStore` 环形缓冲（200 样本）+ `ChartPanel` 手写 SVG 折线（指令 Hz、臂关节0 角度），无第三方图表库。
- **三 Tab 重构**：监控（关节面板 + 图表）/ 健康（HealthPanel）/ 系统（预设管理 + 重启 + 日志控制台）。`Tabs` 组件 + `MonitorTab`/`HealthPanel`/`SystemTab`。
- **顶栏状态徽标**：WS / ROS / 数据 三色 pill（`useWsConnected`）。
- 前端已 `npm run build` 重新生成 `web/dist`。

`morla_web_manager`：接入 `astral_arm_teleop` 的外部启动闸门，Web 界面可手动发 `/teleop/start`。

- `config.py` 新增 `TOPIC_START="/teleop/start"`。
- `monitor_node.py` 新增 `publish_start()`（RELIABLE + VOLATILE，非 latched 一次性触发，避免晚加入臂节点收到旧 start 自动开始）。
- `web_server.py` 新增 `POST /api/v1/teleop/start`（**无条件发送** `/teleop/start`，不检查 launch 是否经本监控启动——臂节点是唯一裁判：homing 中或无 VR pose 时会忽略并告警。因此无论遥操由本监控预设启动还是从外部 CLI 启动，此端点均可用）。
- 前端 `ControlBar` 新增"开始遥操"按钮（琥珀色，running/paused 时可用）；`api.teleopStart()`。
- 前端已 `npm run build` 重新生成 `web/dist`。

`astral_arm_teleop`：新增**外部启动闸门** `require_start_signal`，修复 Quest3 "start stream" 时手抬着点按钮导致首帧 `vr_init` 记在错误位置、机器人一上手就偏的问题。

- `PoseProcessor`：新增 `auto_calibrate` 开关（默认 true=旧行为，首帧自动记零点）；新增 `calibrate_from_current()` 用当前 pose 记 `vr_init`。
- 节点：新增 `require_start_signal` 参数（默认 false，不破坏 sim/旧流程）。true 时启动不 arm、不自动记零，只跟踪 `vr_current`；订阅全局 `/teleop/start`（Bool true，双臂同启）+ 服务 `~/start`（Trigger，单臂带反馈）；收到信号时用当前 `vr_current` 记 `vr_init` 并 arm。homing 中或无腕姿时拒绝并告警。重发 start = 重新记零点（re-center）。
- yaml（left/right）新增 `require_start_signal: false`；`astral_dual_arm_teleop.launch.py` 与 `morla_bringup/full_teleop.launch.py` 暴露 `require_start_signal` launch arg。
- 真机用法：`require_start_signal:=true` 启动 → 手摆好 → `ros2 topic pub --once /teleop/start std_msgs/msg/Bool '{data: true}'`。

`astral_robot_sdk` / `astral_robot_control`：纠正电机 ID 语义——`0x31/0x32` 是**头部**（head_yaw/head_pitch），不是夹爪；机械夹爪只走专用命令 `0x97/0x98`（`set_gripper_angle`），在 18 轴关节流里没有槽位。

- SDK：`ROBOT_JOINT_NAMES` 末两位改为 `head_yaw/head_pitch`；新增 `HEAD_IDS=[0x31,0x32]`、`move_head_js(yaw,pitch)`、`move_waist_js(front,side)`；`GRIPPER_IDS` 保留为 `HEAD_IDS` 的兼容别名（旧导入不报错，但语义已是头）。
- `astral_robot_control`：`joint_layout` 末两位改 `head_yaw/head_pitch`，新增 `HEAD_JOINT_NAMES`/`HEAD_NS`（`GRIPPER_JOINT_NAMES` 保留为别名）；driver 新增 `/head/joint_commands`→`move_head_js`、`/head/joint_states`；18-DoF `/astral/joint_states` 末两位改为头；夹爪 `/joint_states` 改发最近一次指令角（夹爪无 OBS 槽位）；`_on_grip_js` 期望名改 `f"{side}_gripper"`。yaml 新增 `head_ns`/`enable_head_cmd`。
- 下游：`astral_mujoco_sim`（自带 14-DoF 臂名 + 独立夹爪 echo，不碰 18-DoF/头）、`astral_arm_teleop`（仅 14-DoF 臂）、`morla_web_manager`（仅 launch 预设）、`morla_bringup` bringup 均无需改代码；无包再以 `GRIPPER_IDS/GRIPPER_JOINT_NAMES` 当夹爪电机 ID 下发。

`astral_gripper_teleop` / `astral_robot_control` / `astral_mujoco_sim`：修复夹爪两个问题。

- **极性相反**：真机夹爪电机方向为 0.8=张开、0.0=合拢，原 `open_rad/closed_rad=0.0/0.8` 导致捏合(close_ratio=1)→0.8→夹爪反而张开。翻转为 `0.8/0.0`：捏合→0.0(合)、张开→0.8(开)。`close_ratio` 语义不变(0=开,1=合)，`/left_gripper/command` Float64 仍 0=开 1=合。两条路径(pinch 节点 JointState 弧度 + driver Float64 比例)的 `open/closed_rad` 一并翻转，避免互相覆盖；sim 同步。
- **行程太小**：原 `open_dist_m/close_dist_m=0.08/0.015` 固定映射常与用户实际捏合距离范围不符，只用到中间一小段。新增 `auto_range`（默认开）：带遗忘地跟踪实测捏合距离 min/max，把当前距离映射到真实范围，用满夹爪行程；`open/close_dist` 退化为先验/回退，关掉则回退固定映射。新增 `close_ratio_from_range`；`_update_envelope` 指数遗忘包络 + `min_span` 兜底；日志加 `range=[lo,hi]mm` 便于观察。

`morla_input`：修复 IOBT 开启后下游遥操 IK 失败（`ik_fail`、`vr_rx`/`vr_age` 延迟爆炸）。根因是 `frame_id` 在 `robot_world`↔`robot_body` 间切换导致 `astral_arm_teleop` 的 `PoseProcessor` 跨系做 delta（零点在世界系、当前在躯干系 → 米级跳变 → 不可达目标）。在源头（mocap）解决，`astral_arm_teleop` 不再需要身体系特判（已回退 `7c77555`）。

- **sticky latch**：`_iobt_active()` 一旦见过 body 包即置 `_body_ever_seen=True`，此后即使 body 包瞬时丢包（>0.4s）也不再回退世界系，消除运行中 `frame_id` 抖动（实测 26 次切换）。
- **hold gate**：新增 `_pose_frame_settled()` 与 `_WRIST_SETTLE_S=1.0`。启动后 head/wrist/controller 在"帧确定"前不发布：收到首个 body 包 → 躯干系；或 1s 内无 body 包 → 世界系。从源头消除启动期 world→body 翻转，下游第一帧 wrist 即在正确系，`vr_init` 不会被错系污染。landmarks 为腕局部系、与参考系无关，不受闸门影响。
- 独立脚本 `quest3_mocap_standalone.py` 同步 sticky latch。

## 2026-08-24

重命名：`astral_quest_teleop` → **`astral_arm_teleop`**（纯臂遥操 + IK，去掉夹爪 include）；`astral_bringup` → **`morla_bringup`**（整机编排：mocap + 双臂 + 左夹爪 + 右 Wuji）。夹爪 include 从臂包移到 `morla_bringup`。

`astral_arm_teleop` 内部命名统一：节点 `morla_bringup_arm_node` → `astral_arm_teleop_node`，节点名 `morla_bringup_{left,right}` → `astral_arm_teleop_{left,right}`，配置 `morla_bringup_{left,right}.yaml` → `astral_arm_teleop_{left,right}.yaml`（避免与新 `morla_bringup` 包混淆）。

新增 **`morla_bringup`**（原 `astral_bringup`）：整机遥操只拼现有包（双臂 + 左夹爪 + 右 Wuji）。`right_hand_source:=quest3|glove`；手套时 mocap 不再发 `hand_landmarks/right`。`astral_dual_arm_teleop` 增加 `with_mocap`，避免两份 mocap。

新增 **`astral_gripper_teleop`**：Quest3 左手捏合 → 左夹爪，与 mocap / 臂 IK 解耦。

- `pinch_gripper_node` 订 `hand_landmarks/left`，拇指尖–食指尖距离映射为 `/left_gripper/command`（`Float64`，0=开 1=合）和 `/left_gripper/joint_commands`（弧度）。
- `astral_robot_control` 收上述话题，调用 SDK `set_gripper_angle`（CMD 0x97/0x98，非电机 ID）；`move_arm_js` 不带动夹爪。
- 夹爪 include 原在 `astral_dual_arm_teleop.launch.py`（`with_gripper:=true`），现移到 `morla_bringup/full_teleop.launch.py` 编排；臂包不再含夹爪。以后换硬件：新节点发同一 `command` 即可。

## 2026-08-21

`morla_input`：配合 Quest **astral-tracking** 坐标系修复，下游不再二次转换参考系。

- Quest 端把 `FullBody_Hips` 骨头系（+X下/+Y前/+Z左）用常量四元数 `(-0.5,0.5,0.5,0.5)` 重定向为躯干系（+X右/+Y上/+Z前）；`_lastBodyReferencePose` 与 body 包里的 hips 朝向均用重定向后的值。
- Quest 端 `BuildAndSendPacket` 对非 hips 关节直接 `ToBodyRelative`，body 包里非 hips 关节已是 hips 相对；hips 仍发世界系。
- `quest3_udp_mocap._process_body_line` 移除 `pose_in_parent_frame`：hips 发世界（`quest3/hips_pose`），其余关节直接 `unity_pose_to_robot`，PoseArray 中 hips 为 identity 根。整棵树统一在躯干系，轴映射 `unity_pose_to_robot` 由"用错轴"变为正确。
- 独立脚本 `quest3_mocap_standalone.py` 同步去掉二次 `pose_in_parent_frame`，避免可视化把已是 hips 相对的关节再转一遍。

## 2026-08-20

适配 Quest **astral-tracking** 新推流：Mixed 下手柄与手可同时有数据；IOBT 开时发全身关节，且头/手/手柄已在 hips 身体系。`quest3_udp_mocap` 重新解析 `Left/Right controller:` 与 `body iobt`。

- Mixed：同侧 Quest 只发手柄或手（手柄优先）；左右可一边 ctrl 一边 hand。`controller_as_wrist` 默认把手柄写到 `quest3/{side}_wrist_pose`，臂 IK 不用改。另发 `quest3/{side}_controller_pose`、`quest3/input_mix`。
- IOBT：`quest3/hips_pose`（世界系）+ `quest3/body_joints`（转到 hips 系）+ `quest3/body_joint_names`。头/腕/手柄的 `frame_id` 在收到身体包时切到 `robot_body`/`vr_body`。landmark 仍是腕局部。
- TCP `listen` 提到 8，覆盖手×2 + 头 + 手柄 + 身体多路连接。
- 独立脚本 `quest3_mocap_standalone.py` 同步解析与可视化。

新增 **`quest3_video_streamer`**：把 PC 相机画面经 WebRTC 推到 Quest 3，遥操时看机器人第一视角。与 `morla_input` 独立（信令 `:8765`，mocap 仍是 `:8000`）。自 `hand-tracking-sdk` 视频部分拆出，不依赖该 SDK、不用 venv。

- 单条 peer connection 多路 track：默认 D435i 1080p30（v4l2 `/dev/video8`）+ 左右腕 USB 720p30。
- 可调全在 `config/params.yaml`（相机列表、source/device/preset/fov/layout）；launch 只加载 yaml。
- 延迟：源端 `next_frame()` 节流、webcam 后台线程抓帧、D435i 默认真连 v4l2（可选 `d435i_source:=ros` 走 realsense 节点）。
- Quest 用 `video_config` 的 `fov_h_deg` + layout 摆 3D 面板。码率下限/默认/上限 3/5/12 Mbps。
- `quest3_udp_mocap` 增加 `[Mocap Downlink] kbps`，与视频上行 `total_kbps` 可对照链路占用。
- 与 HTS 同跑时 `enable_mocap_tcp` 保持 false，避免抢 TCP 8000。

```bash
ros2 launch quest3_video_streamer multi_camera.launch.py
# Quest：astral-tracking 填 PC 信令（Wi-Fi 或 adb reverse tcp:8765 tcp:8765）后 Start Stream
```

## 2026-08-19

相对 `f6300cf` 的未提交改动一并入库。当日另做：

- **移出 WebXR**：`astral_webxr` 不再在本工作空间内（现位于 `../astral_webxr`）。臂/手 pipeline 去掉 `input_source:=webxr`，只走 HTS `quest3_udp_mocap`。
- **撤回 PC 端 IOBT/混控**：`quest3_udp_mocap` 不再解析 `body iobt` / `controller` 行，也不再发 `quest3/body_joints`、`hips_pose`。Quest HTS 工程里的 IOBT/混控代码未改。

## 2026-08-17

相对 `f6300cf` 之后、当日完成的遥操改动（此前未 commit）：

### 跟手调参 UI（`teleop_tune_plot`）

- 布局、中文字体、按钮颜色；横轴改为「过去 8 s → 现在」。
- 热改滑条 / DH↔URDF 热切仍沿用 08-14 的话题与参数（见下文 08-14 节）。

### 配置改走 yaml

- 臂 IK 默认改为 **`urdf_numerical`**（`morla_bringup_{left,right}.yaml`）。
- `protocol` 以 `quest3_mocap.yaml` 为准（默认 `tcp_wired`）。
- `astral_sim_pipeline` / `astral_dual_arm_teleop` 的 `solver_type` / `protocol` / `convert_to_robot` 留空则**不覆盖** yaml。

### 删除旧单进程入口

- 去掉 `morla_bringup_node.py`、`morla_bringup.yaml`、`astral_quest_teleop.launch.py`、`astral_real_pipeline.launch.py`。
- 真机只留 `astral_dual_arm_teleop`（2× `morla_bringup_arm_node`）。
- `setup.py` 用 `_existing()` 过滤 glob，避免删 yaml 后 colcon 仍拷已失效的安装链接。

### 真机启动走到 init_pose

- `move_to_init_pose` 默认开：从当前 `joint_states` 按 `init_speed_percent`（10% × `max_joint_vel`）关节空间插值到 yaml `init_pose`，到位（0.05 rad）或超时 15 s 后再跟手。
- `init_speed_percent` 是本节点相对 `max_joint_vel` 的比例，**不是** SDK `set_speed_percent` / `move_j`。
- `dry_run` 跳过 homing。

## 2026-08-14

此版本使用 Quest3，逆解使用 DH 以及 URDF，双臂遥操成功。

- 输入：Quest3（`morla_input`，`convert_to_robot:=true`）
- 臂 IK：`analytic_dh` 与 `urdf_numerical` 均可
- 双臂：`morla_bringup_{left,right}` 进程并行 → `/{side}_arm/joint_commands`

### 延迟测量与 150 Hz 控制环

在 mocap / 单臂遥操 / MuJoCo 仿真加周期性 `[Latency]`（约 2 s，`print_latency` / `latency_print_interval`）。

| 日志 | 含义 |
|------|------|
| VR `recv_to_pub` | PC 收到 Quest 文本到发出 ROS |
| VR `wrist_gap.{left,right}` | 同侧腕包间隔（跟踪率，不是 300 Hz 行率） |
| 臂 `vr_rx` / `vr_age` / `ik` / `loop` / `ik_fail` | 收腕、等到控制拍、IK、整圈、无解次数 |
| 仿真 `e2e.{side}` | VR 收包时间戳 → 仿真拿到 `joint_commands` |
| 仿真 `apply.{side}` | 回调入队 → 写入 `qpos` |
| `[VR Data Rate]` | 所有文本行（头+双腕+landmark）约 300–360 Hz |
| `[Sim Cmd FPS]` | 仿真实际收到的关节指令频率 |

Quest 头显内部时钟未对齐，设备→PC 的空中延迟测不到。`[VR Data Rate]` 不是臂跟踪率；臂跟踪看 `wrist_gap` ≈ 14 ms ≈ **72 Hz**。

- 遥操 `control_rate`：50 → **150 Hz**（yaml）。真机驱动 `control_rate` / `ctrl_hz` **仍为 50 Hz**，未改板卡发送。
- `max_joint_vel`：由「每拍 rad」改为 **rad/s**（默认 4.0，相当于旧 0.08 rad/tick @ 50 Hz）。`SafetyFilter` 每拍上限 = `max_joint_vel * dt`。
- `pos_smoothing` / `rot_smoothing`：仍为 0–1，按 50 Hz 的 α 标定成时间常数 τ，每拍 `α(dt)=exp(-dt/τ)`。改 `control_rate` 不再改变手感。
- 仿真循环原先每圈只 `spin_once` 一次，150 Hz 指令会被丢到 `[Sim Cmd FPS] ~50`。改为 `_spin_drain`（最多 24 次 `spin_once`），订阅 QoS depth 50。修好后 Cmd FPS ≈ **150 Hz**，`e2e` 约 10–12 ms。仿真只保留最新 `q` 再写入 `qpos`：收得比发得慢会跳过中间拍，看起来像冲一截；收得更快则零阶保持，不会因此冲。

有线 TCP + URDF 数值 IK 的仿真对比（150 Hz、drain 之后）：均值 `e2e` ~10–12 ms；DH 的 IK 约 +1 ms；UDP 主要伤尾部；**DH+UDP 左臂曾大量 `ik_fail`、指令中断**。

### 跟手调参曲线（`teleop_tune_plot`）

```bash
ros2 run astral_arm_teleop teleop_tune_plot --ros-args -p arm_side:=right
```

- 遥操默认发 `/teleop/{side}/tune/`：`ee_vr`（未平滑的 VR 映射）、`ee_filt`（滤波目标）、`ee_cmd`（指令 FK）、`xyz`（三条打成一条）。
- 窗口叠画 mm 曲线 + RMS / p95 / hold jitter / 互相关 lag。键位在**曲线窗口**（不是 launch 终端）：`z` 把当前末端设为图零点，`s` 存 CSV，`q` 退出。
- 滑动条热写 ROS 参数（默认左右同步）：手感 `pos_smoothing` / `rot_smoothing` / `motion_scale` / `max_joint_vel`；URDF LM：`ik_w_pos` / `ik_w_ori` / `ik_w_reg` / `ik_max_iter` / `ik_tol`。
- 左下角可热切换 `analytic_dh` ↔ `urdf_numerical`（重建求解器、按当前 `q` 重定 VR 零点）。DH 闭式忽略 LM 权重。仿真 launch 即使以 DH 启动也会带上 `urdf_path`，以便热切到 URDF。
- yaml 只是下次启动的初值；热改不写回文件。`solver_type` / `urdf_path` / `init_pose` / `control_rate` 等不可随意热改（`control_rate` 仍需重启）。

## 2026-08

### Wuji 手

- 包：`wuji_glove`、`wujihand_retargeting`、`wujihand_mujoco_sim`、`wujihand_control`（+ vendored `wujihandros2`）
- 统一话题：`hand_landmarks/{side}` → `/{side}_hand/joint_commands` → driver / MuJoCo
- 三后端：`official` / `wuji_retargeting` / `dexpilot`
- TuningViewer 三层骨架（橙/青/白）+ retarget yaml 热重载
- Quest3 专用 yaml：`retarget_wuji_lib_quest3_{left,right}.yaml`

#### Bug fixes

| ID | 现象 | 根因 | 修复 |
|----|------|------|------|
| Q1 | Quest3 小指呈 **Z 形** | `adaptive_retargeting_xhand` 顺序拉伸 | 默认 `enable_xhand_pinky_adapt:=false`；Wuji launch 强制关闭 |
| Q2 | Quest3 相对手套 **MCP / 掌部偏移** | Quest 已做腕帧+MANO，Retargeter 再变换一次 | `landmark_preprocess:=raw`；手套节点不做 MANO |
| Q3 | Quest3 调参橙/白 MCP 仍偏 | Adaptive 不约束 MCP | quest3 yaml：`snap_mcp_to_robot: true` |
| Q4 | 真机 **收不到** `joint_commands` | BEST_EFFORT vs RELIABLE | 对齐 SensorDataQoS BEST_EFFORT |
| Q5 | 真机未用上调参 yaml | real pipeline 仍指向手套 yaml | `input_source:=quest3` 自动传 quest3 yaml |
| Q6 | dexpilot/official 仿真握拳、不跟手 | MJCF `kp` 过小 | tuning/sim 写 `qpos`+`mj_forward` |

#### 操作提示

- 真机与 MuJoCo **二选一**，勿抢同一 `joint_commands`
- `*_serial` 可空（按 `hand_side` 连）
- USB：`0483` 需 udev `MODE="0666"`
