# MORLA 迁移进度

执行依据：SDK根目录 `docs/MORLA_MIGRATION_PLAN.md` 的校验基线副本 `docs/MIGRATION_PLAN.md`，以及 `docs/SOURCE_BASELINE.json`。当前运行进度以本文为准。
开始时间：2026-10-13（Asia/Shanghai）。
源基线：Astral `3a57d722d3c4257670d9f90526127bed77063e66`；XNero `e6a0100429024f04e18238f1a42ee968bb18967a`。

## 状态

| 阶段 | 状态 | 结果/证据 |
| --- | --- | --- |
| P0 独立副本与环境 | 完成 | 595 个 Astral Git 跟踪文件均存在且 SHA256 一致；迁移起点与冻结基线匹配；复核时 Astral 仍在基线提交，XNero 基线 `e6a0100` 是当前干净分支 `6f467bf` 的祖先；两源工作区均未被迁移修改；MORLA 独立提交 `f441a3e`。后续源提交复核见下节。 |
| P1 包路径与改名 | 完成 | 25 ROS 包可发现且全量 clean build 成功；五个共享包完成改名，入口脚本与Web launch路径适配；前端和指定回归通过。映射见 `docs/package_map.json`。 |
| P2 core/profile | 完成 | 新增`morla_core`共享schema/lease/资源/关节映射/profile registry；12个profile固定布局并从已安装ament share验证，见本次日志。消费者运行时接入仍在P7-P9完成。 |
| P3 description | 完成 | 新建`robot_description`（ament_cmake），集中27个model ID与358项资产；manifest含来源/目标SHA、关节映射、847个mesh闭包和授权信息。24包干净构建成功；普通安装、colcon symlink-install 均由Pinocchio加载12份URDF、MuJoCo加载5份MJCF；生成器可重跑，Astral FK和sim机械量、XNero sim机械量对照通过。 |
| P4 unified input | 完成 | `morla_input` 固定输出 robot-axis raw 21点；strict packet 默认拒绝legacy，session/坐标帧变化发epoch并由统一闸门停止策略/遥操、清缓存且要求松开后重新start；XHand旧MANO/小指局部适配和Nero新旧pose链等价验证通过。 |
| P5 Nero driver/teleop | 软件完成 | 新建标准 ROS driver/teleop 两包并按 profile 参数装配；fake 双侧 launch 和命名命令往返通过。真实 CAN/固件/安全路径未验收，见 P11。 |
| P6 XHand adapter/sim | 完成 | 31 ROS 包构建通过；adapter/retarget 59 项、XNero 仿真 29 项通过；profile 驱动 headless 整链路实测通过，触觉源约 50 Hz。 |
| P7 unified data collection | 完成 | profile数采/具名重排、XHand力/温度/valid raw sidecar、按源时钟离散对齐、repair同源选帧、source索引验证、LeRobot v2.1/v3 schema与触觉旁路、只读XNero导入均完成。端到端三项新增集成测试通过；数采测试128通过/1跳过。 |
| P8 policy/HITL | 软件完成 | policy、stub/remote handshake、Replay 统一接入 FrozenSchema；标准可靠 QoS、具名 writer 路由、所有权 freeze/ACK 与 Nero prepare/token；本轮含 raw profile block 映射、录制帧率节奏和 HITL 回放闭环，策略套件 156 项通过。19维实际仿真已有 stub→HITL takeover/release→LeRobot replay→pause/resume 证据；新增 raw 回放由合成数据节点流回归覆盖。没有真实硬件策略运行。 |
| P9 Web manager | 软件完成 | profile/API/操作互锁/三条泳道与动态 streams 已接通；本轮新增回放源只读浏览 API/网页选择器和大小写/空格路径保护，Web manager 测试 44 项通过；Vite 53 modules 构建通过；headless Chrome 既有记录显示 SPA 与 JS asset 均 HTTP 200。 |
| P10 software integration | 软件完成 | Astral8/35、Nero19/38已有profile仿真证据；8/35/38完成采集→对齐→校验→LeRobot v2.1，均218帧；Nero19完成277帧stub/HITL/replay/pause闭环。本轮验证统一 Astral/Nero 启动入口、Nero19采集到190帧并成功导出。profile CLI 12/12。 |
| P11 hardware/model acceptance | 待设备验收 | 等待 Astral/Nero/XHand/Wuji 实机、固件/SDK、严格Quest APK与38维策略模型。XNero 冻结点之后新增的 HUG grasp bridge 尚未纳入本次执行计划；其边界见下节。不得将软件仿真结果表述为真机通过。 |

## 源仓库后续提交复核

冻结基线 `docs/SOURCE_BASELINE.json` 记录 Astral `3a57d722d3c4257670d9f90526127bed77063e66` 和 XNero `e6a0100429024f04e18238f1a42ee968bb18967a`；基线文件保持只读，用于说明原始迁移起点。2026-10-15 重新检查 Astral WS，当前 HEAD 为 `38f9b458e80892c85db39f451ce68901e7c5dcbc`，较冻结点新增三项提交：`54ad233` 调整 Astral 双臂 HOME/init 工作位 J1 waypoint（−1.6→−1.2）、`41279f0` 在 HOME 回零途中单独归零 J4、`38f9b45` 将 Quest 时间协议从 UTC-ms envelope 改为单调时钟 challenge/response。已将对应功能同步到 MORLA 的 teleop 配置/逻辑与 `morla_input`，并保留 MORLA 的 profile/session epoch 和输入失效处理。受影响的两个 ROS 包定向构建成功；Quest 输入测试 10 项、Astral arm 回归 49 项通过，`PacketGuard` 定向 flake8/pydocstyle 检查通过。全包历史 lint 扫描仍报告既有格式问题，集中在原有 mocap/standalone 代码。Quest 端协议源码现已提交到本机 `astral-tracking` `a6e262c`，但该提交未构建或安装 APK，设备侧握手仍属待验收项。XNero 冻结点后的 `xnero_grasp_bridge` 边界说明仍适用。

随后复核 Astral WS HEAD `48642270361bf525d91b5b73f7bea893d9d918fe`（`feat: 支持 raw 遥操回放与回放源浏览`），新增更新限于数采/推理/Web 的 raw 回放与回放源浏览功能。本轮已同步到 `morla_policy_inference` 和 `morla_web_manager`，按 MORLA 冻结 schema 的 action/state block 映射实测流，同时保留 profile/layout/kinematics 校验和 SafeExecutor/HITL 路径；网页只读浏览配置的数据根目录并拦截越界路径/符号链接。`morla_policy_inference` 156 项、`morla_web_manager` 44 项测试通过，前端 Vite 53 modules 构建通过，两个 ROS 包定向构建通过。新增 raw 回放以合成 raw 数据和节点测试验证，尚未对真机录制段做回放验收；冻结基线文件未改。

XNero 冻结点后的17个新增文件全部属于独立的 `xnero_grasp_bridge` 研究实现及其 HUG 计划/设计文档：包含右手 MANO→retargeter 坐标候选、HUG WebSocket 模型服务和校准脚本；没有改动冻结基线中的 Nero driver/teleop、XHand retarget、MuJoCo sim 或旧数据采集源文件。该新实现没有 ROS package manifest，且计划基线未定义其接入契约，因此本次不把它并入 MORLA 的 robot graph，也不把它伪装成 `morla_policy_inference` 的标准 action writer。若后续要接入，需要单独定义它输出 MANO landmarks 如何通过冻结 schema、输入所有权和 XHand 坐标兼容校验进入 MORLA。MORLA 当前按用户指定的冻结源基线与迁移计划验收。

## 坐标与运动学专项实施（2026-10-14）

基于前一节冻结源码审查中发现的 Unity basis 分散、Nero J2 仿真零位、solver-tip/tool 语义和非零 TCP anchor 问题，已按 [优化计划](coordinate_kinematics_optimization_plan.md) 完成软件实施。原始修改前源码 SHA 和 FK 结果保留在 [审查与实施证据 JSON](coordinate_kinematics_audit_2026-10-14.json) 顶层历史字段；当前活动模型和实施后验证在 `implementation_validation`，两者不可混读。

实施结果：MORLA Core 新增严格 SO(3)/SE(3)、URDF 固定链与统一 coordinate contract；Quest 在线和 standalone 输入复用同一 Unity basis，并分别声明 tracking-world、operator-body 和 robot body；12 个 Profile 统一定义 input basis、身体参考系、左右安装派生、solver adapter、tip/tool 和 `kinematics_hash`。遥操 launch 从同一 Profile/URDF 解析左右输入变换和工具矩阵，清除了重复 YAML 矩阵。Astral/Nero 的 init/start/re-anchor/HITL 锚点现在保存 TCP pose 并在 IK 前反算 solver tip。活动 Nero URDF/MJCF 恢复硬件 q=0 与角限位，旧 J2 偏置模型移入 legacy，仿真垂下姿态由 sim-only 初始角表达。

TF 由 Profile-scoped `robot_state_publisher` 加测量 JointState bridge 提供；bridge 依 joint name 映射真实状态、保留每条来源时间戳、拒绝缺关节，不订阅命令或拼接异步状态。坐标 hash 已进入数采和 export、policy handshake/replay、Web Profile API/UI。Nero Profile 的活动裸臂控制末端保持 `*_link7`，与解析 IK solver tip 一致；`*_gripper_flange` 是 URDF 中独立的物理法兰帧，可供 TF、实测法兰位姿和显式工具选择使用。driver 规范实测接口为 `/flange_pose`，旧 `/tcp_pose` 同值兼容并标记 deprecated。SDK flange pose 到 URDF frame 对齐、实体安装尺寸和外接任务 TCP 仍待硬件资料/实测。

几何闭环以每侧501个 q_hw 随机/零位姿态核对。当前解析 solver tip `*_link7` 对 URDF 最大位置差约 `0.01002 mm`、姿态差 `2.18e-7 rad`；对 MJCF 最大位置差约 `0.01015 mm`、姿态差 `5.57e-7 rad`。普通安装加载检查通过 27 个模型/358 个资产；12 个 Profile 在 symlink 与普通安装均通过解析；状态桥/RSP action 能对全部12个 Profile 成功构造。Astral `astral_left_gripper` 统一 sim Profile 数据端到端通过：测量状态桥/RSP 与遥操启动后，208帧数据完成对齐、验证和 LeRobot v2.1 导出，报告为1 pass/0 warn/0 fail。`scripts/test_ros.sh`、`scripts/test_policy.sh`、`scripts/test_offline.sh`、Web `npm run build`、`scripts/test_sim_integration.sh` 及 `scripts/test_astral_sim_integration.sh` 均通过。Nero仿真整链路记录/对齐/校验/LeRobot v2.1导出190帧成功；唯一质量 warning 为左手状态约50.2Hz低于100Hz门槛，没有fail。P11设备验收未改变，仍待真实机器人和厂商/实体测量依据。

## 日志

### 2026-10-13

- 已确认源 commit 与迁移基线匹配；两个源工作区的 `git status --short` 为空。
- 已在 SDK 根目录创建新的 `MORLA/`，运行 `rsync -a` 从 Astral 复制源码，排除build/install/log、缓存、node_modules、egg-info和运行数据。
- 尚未修改ROS包、description、配置或机器人接口；尚未启动硬件。
- 本地环境已确认 ROS 2 Humble；后续按系统 Python 3.10构建ROS包。
- P0校验：`git -C astral_ws ls-files` 共595个文件；逐文件比较MORLA副本SHA256，缺失0、差异0；MORLA初始提交为`f441a3e`。
- P1：将原Astral包按`common/`、`robots/astral/`、`robots/wuji/`重组；`quest3_hand_mocap`→`morla_input`、`astral_data_collect`→`morla_data_collect`、`astral_policy_inference`→`morla_policy_inference`、`astral_web_monitor`→`morla_web_manager`、`astral_teleop`→`morla_bringup`。同步Python模块/resource/setup/import/launch/脚本路径；新增Astral/Nero/Wuji/XHand组件元包框架；`pi05_standalone`移入`tools/legacy/`并加`COLCON_IGNORE`。
- 新工作区包列表：`colcon list --base-paths MORLA/src --names-only`为25个包；单独只有一份`morla_input`与`morla_data_collect`。`docs/package_map.json`记录源/目标包、路径和可执行入口。
- P1 clean build：`source /opt/ros/humble/setup.bash; PATH=/usr/bin:/opt/ros/humble/bin:$PATH /usr/bin/colcon --log-base MORLA/log build --base-paths MORLA/src --build-base MORLA/build --install-base MORLA/install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3`，退出码0，25/25包完成。要求明确使用系统Python；默认shell的conda Python会导致ROS CMake解析找不到`catkin_pkg`。
- P1 launch解析：`web_manager.launch.py`、数采、推理和Astral full bringup均以`--show-args`加载成功；没有启动机器人或硬件节点。Python源码`compileall`通过。
- P1回归：`morla_input/test_packet_guard.py` 3通过；`morla_data_collect/test` 125通过/1跳过；`morla_policy_inference/tests` 143通过；`morla_web_manager/test` 25通过；`morla_bringup/test` 24通过。数采导出测试临时使用Pandas 2.3.3后通过；该测试依赖已清理，P7补齐包运行依赖。Quest全目录的flake8插件会递归扫描SDK根目录，故只执行了包保护逻辑测试，没有把该递归检查记作通过。
- Web前端`npm ci && npm run build`通过（TypeScript + Vite，52 modules）；npm审计提示锁定依赖有1项moderate和2项high advisories，尚未擅自升级依赖。
- 源`astral_ws`、`xnero_ws`提交和工作区状态复核无变化；迁移目录中运行时代码不再引用旧工作区路径或五个旧共享包名。尚未运行真机。
- P2：新增ROS-free `morla_core`，将schema/lease移出collector并将ROS lease bind放入惰性`ros_ownership`；policy、collector、Astral臂/夹爪、Wuji retargeting改依赖`morla_core`。严格关节映射、schema v2/hash/v1升级、资源路径保护和profile registry均有黄金/负例测试。
- P2 profiles：注册12种 Astral/Nero 布局：Astral 8/16/27/35/54，Nero 14/19/38；`nero_dual_xhand`严格按左臂、右臂、左手、右手 `[0,7),[7,14),[14,26),[26,38)`。从Astral URDF、Wuji/XHand URDF及XNero Nero IK提取limits/标准名字；保留原Astral臂与夹爪参数引用；增加左右组件模板、相机slot与XHand触觉sidecar定义。
- P2 回归：纯逻辑10通过；profile CLI从已安装ROS包索引解析12项并生成`resolved_profile.json`/`frozen_schema.json`；单独安装空间可见所有MORLA包。全量clean build 26/26通过。Input 3通过、policy 143通过、Web 25通过、bringup 24通过；统一采集125通过/1跳过。测试使用临时Pandas 2.3.3以满足v3导出；依赖已登记为`python3-pandas`，临时用户级包在最终清理时卸载。
- P2发现并修正5个新Python包的无效maintainer邮箱：否则colcon会将其识别为普通Python包，虽能编译但不会进入ROS包索引。重建后`colcon list`显示`ros.ament_python`，`ros2 pkg list`和已安装profile CLI验证通过。
- P2修复`data_pipeline.sh`在重命名后缺少`morla_core`的源码路径，并新增`MORLA_DATA_PYTHON`优先级与旧`ASTRAL_DATA_PYTHON`弃用提示；原兼容路径测试通过。

- P3：新建`src/robot_description`为独立`ament_cmake`资源包，集中Astral robot/arm/clean/legacy RobotMain、Wuji、Nero+XHand、独立XHand retarget URDF，及Astral/Wuji/XNero MuJoCo模型、完整mesh闭包、RViz/launch、触觉点输入、生成/校验脚本。删除MORLA内三个旧description ROS包以及teleop/retarget/sim里的URDF/MJCF/mesh运行时副本；源`astral_ws`/`xnero_ws`保留不变。`docs/package_map.json`已更新资产迁移映射。
- P3资源索引：manifest记录27个model ID、358个资源文件、847个mesh引用、source/target SHA256、base/end links、可动关节、profile ROS名→URDF名及符号/offset、用途、生成器和license来源。XNero retargeting原包没有声明license，manifest/docs明确标为未解决，不伪造授权；profile的`description_revision`统一固定manifest SHA256。
- P3路径：所有活动loader以`morla_core.resource_locator.resolve_robot_asset(model_id)`解析安装share+manifest；Astral IK、Astral/Wuji sim、Wuji DexPilot默认均不引用旧description包。`resolve_robot_asset`在普通安装与`colcon --symlink-install`均通过，并限制符号链接只回落到对应`robot_description`源码包。
- P3构建/安装：24 ROS包clean build通过；普通复制安装运行`check_assets.py --loaders --require-regular-install`通过，symlink安装运行同脚本及12个profile `validate-all`通过。两个安装空间均由Pinocchio实际加载12份URDF（含全部mesh），MuJoCo加载5份MJCF；847个mesh闭包与模型目标hash全通过。
- P3机械意义回归：7组迁移URDF对原Astral/XNero工作区做随机配置FK、关节名、限位比较均一致；Astral clean MJCF模型维度、body姿态/质量/惯量、joint axis/range、actuator limit完全相同；XNero生成器重跑后对应同一组模型量，`test_xhand_tactile_mujoco.py`验证1200/1200触觉测点、名称完整和测试压入力全部入传感，未启动机器人硬件。Astral Geometric IK、完整IK、VR映射三个原回归脚本均PASS。
- P3剩余验证限制：DexPilot实际retarget构造因系统Python没有Torch失败（当前只装了`dex_retargeting`代码但其import硬依赖Torch）；URDF本身由Pinocchio加载成功，实际backend将在依赖环境准备后补验。XNero XHand源包license需上游确认后方可外部分发。
- P4：将Quest ROS receiver统一为`landmark_preprocess=raw`：21点先按Astral Unity→robot矩阵转换、以腕为原点并应用现有EMA；移除全局MANO和小指stretch，Wuji现有raw路径不变。`quest3_mocap.yaml` 的`allow_legacy_packets`从true改为false；full teleop、Astral sim/standalone arm、Wuji sim/tuning/real入口均有显式`allow_legacy_packets:=true`旧APK兼容开关，启用时节点打印降级警告。
- P4输入epoch：PacketGuard在新session接替时接受首包并暴露线程内session变更事件，同时退役旧session；receiver在新session或已发布pose从world切到IOBT body frame时清空EMA/腕来源/帧锁并发布latched `quest3/input_epoch`。`controller_start_gate` 收到事件后发布latched `/teleop/disarm`、`/teleop/armed=false` 与policy `stop`，清除旧按钮边沿，并要求控制器先松开grip、再重新按下；新session、旧session重放、严格legacy拒绝均有逻辑测试。实际ROS多节点时序仍将在P10集成运行补验。
- P4坐标/手型：Nero左右`vr_to_arm_rot_after_unity_to_robot`沿用计划推导的`A_new=A_old*M^T`，flip_pitch=false；新增`xhand_retargeting.preprocess_xhand_landmarks()`，先按`F*M^T`恢复旧XNero手输入，再重放1.05缩放、腕心、SVD、左右OPERATOR2MANO及可选原小指顺序stretch。Wuji不再能被XHand全局变换污染；profile把XHand兼容模式显式写为`retarget_compatibility.preprocess_mode=legacy_xhand_pinky`。
- P4验证：全量增量构建25/25包通过；`morla_core.profile validate-all` 12/12 profiles通过（8/16/19/27/35/38/54维均保留）；输入/闸门/XHand新增与相关回归合计80 tests通过，其中Nero六方向、三轴旋转、500随机四元数与非零TCP各40随机样本/侧在float64容差内等价；XNero MANO+pinky 20组/侧及MANO-only左右参考对照共44用例通过。`colcon list`发现25个ROS包。未连接Quest、Nero或XHand硬件。
- P5：从 XNero `nero_quest_teleop` 拆出 `nero_robot_control` 与 `nero_arm_teleop`，保留供应商 `pyAgxArm` 源码/许可证于 `third_party/pyAgxArm`，新增仅供硬件部署显式运行的 editable 安装脚本；此迁移环境未安装供应商包。driver 以单一串行 worker 管理 SDK，connect 后设 streaming 模式，SDK 懒加载；具名7关节 `JointState` 严格校验、限位/有限值/新鲜实测反馈、rad/s限速、显式 enable/position/estop、一次测量态 watchdog hold 和超时后撤销软件运动资格。反馈独立于 teleop，按20Hz发布真实关节名和 flange Pose；没有自动 enable、HOME 或位置动作。默认 launch 只用 `FakeNeroSDK`，`hardware` 是明确覆盖项。
- P5 teleop 只订阅 Quest wrist 与真实 named state，输出 named command；保留 Nero IK/VR 算法和 `0.065 rad/step` 旧滤波语义，逐周期限速由 driver 的 `rad/s` 层另外处理。补入 core `PolicyLease`、动态 ownership ACK writer、`prepare_reanchor` 一次性token提交及手动 `reanchor`；策略/回放所有权下拒绝遥操，frame/session epoch 清锚并 disarm，不在退出/disarm 时回位。`nero_arm_stack.launch.py`从profile启用侧和组件参数路径装配 input/gate/driver/teleop，XHand本阶段仅报告尚未启动；未复制 `SimArmStub` 生产入口。
- P5 软件验证：完整 `colcon build --base-paths src` 27/27通过；12 profiles全通过；83项 driver/input/lease/XHand相关测试通过；Nero VR mapping 6/6、analytic IK 5/5。源IK solver SHA256与XNero完全相同；原压力用例的未设种子随机输入导致结果跨运行变化，本次仅在回归脚本固定 seed=2026（求解算法未变）。Fake双侧ROS launch实际跑通，双方SDK连接、实测反馈和125Hz teleop loop正常；隔离ROS domain显式enable左侧fake driver后，逆序JointState命名命令被正确重排，实测七关节各限速移动0.008rad。运行中没有发布Quest数据或自动运动；未连接CAN/实体手臂。
- P5 smoke同时发现并修正ROS全限定参数文件选择、空Python列表被推断为BYTE_ARRAY、连接future被重复当作未完成等三个启动/生命周期缺陷；回归运行后双侧节点保持存活且status反馈持续新鲜。实际状态发布保持20Hz，不把源配置里的20Hz写成125Hz。硬件 SDK ABI、CAN通道、位置/工作位机械安全仍属于P11。
- P6：迁入 XNero `xhand_control_interfaces`、`xhand_control_ros2` 和 `xnero_mujoco_sim` 到 `src/robots/xhand/`、`src/robots/nero/`；新增纯逻辑/ROS `xhand_adapter`。MORLA 具名 12 关节 `JointState` 是 retargeter 唯一标准输出；adapter 独占标准反馈和原生 XHand 命令；DexPilot 的 21 点 raw Quest 转旧 XHand MANO/pinky 坐标仅发生在 XHand 局部。旧手目标伪造 `/joint_states`、直接控制/HOME 入口均未安装为运行入口。
- P6 仿真装配：新增 `nero_xhand_stack.launch.py`，由 robot profile 选择左右 arm/hand；`mode=sim|real` 分支互斥，仿真接标准臂命令与原生手内部消息，真实分支必须显式硬件 backend 和真实串口配置。XNero MuJoCo MJCF/mesh 从 `robot_description` manifest 解析；1000 个触觉点/侧来自模型中的 1200 个 geom。未启动 real 分支、未连接物理设备。
- P6 验证：`colcon build --symlink-install --base-paths src` 31/31 成功；`morla_core.profile validate-all` 12/12；XHand adapter+retarget 59 项通过、XNero MuJoCo 29 项通过。profile `nero_left_xhand` headless launch 实测合成 21 点 `/hand_landmarks/left` → 95 对具名目标/标准命令 → 95 条原生 `XHandCommand`，adapter 发布 161 条具名 12 维状态，sim 原生反馈约 50.1Hz；原生命令与标准命令相同、hand_id/mode/gain/限位正确；每个触觉样本含5指×120点力及每指20温度值。未测相机和实体 Quest 输入。
- P6 调度修正：实测确认原 simulator 的“当前时刻+周期”调度会因 220Hz 物理步长产生约44Hz状态；改为累进固定周期后，独立订阅 5.98s 收到300条、50.01Hz，整链路测得50.1Hz。
- P6 依赖：安装 DexPilot runtime 缺失的 `torch==2.5.1+cpu` 到系统 Python 3.10 的用户目录，DexPilot + Pinocchio 现可实际构造/运行。该 wheel 仅验证 x86-64；ARM64/CUDA 依赖需按部署平台提供。供应商 ROS 包原始 license 字段仍为 TODO、XHand SDK 用户协议与外部分发授权待上游明确；SDK `.so` 为 x86-64，硬件分发/ARM64验收未完成。
- P7 profile collector：数据 writer/订阅由冻结 profile 建立；仅把 state 与当前动作语义需要的 command 标为必需，profile 中其它数值命令流仍订阅并落盘但在 `next_state` 旧示范中不伪造。XHand sidecar 按 profile `hand_id` 和 finger location 对应5指×120点力、每指20原始温度；错误 hand/finger、坏长度或非有限读数不记为有效，温度与力分别记 valid。sim 力标 N/link2 frame，sim 温度有效位为false并标 `synthetic_zero`。
- P7 离线触觉：raw `/sensors/<key>` 保存源/到达时间及 validity；align 只取 `max_gap_ms` 内最近源样本，写 `source_index/timestamps/offsets`，绝不插值；terminal hold 复制末次值但 source valid 置false。repair 按选中的源帧索引同步复制所有触觉字段；`verify-aligned` 逐元素比对 sensor sidecar。缺 required sensor 失败，optional missing/no-sample 给降级 warning。
- P7 导出：LeRobot v2.1 写完整 `meta/morla_schema.json`、profile/layout/schema hash 和 policy feature exclusions；非触觉模型将对齐触觉保存到 `meta/sensors/episode_*.h5`，不会拼入 state/action；v2.1→v3 继续复制契约、sidecar、质量和来源目录。`--model-capacity` 仅在显式选择目标模型时启用，通用导出不限制38/54维。
- P7 XNero import：统一入口 `data_pipeline import-xnero`，要求profile、活动侧、camera map、task、TCP frame、camera calibration和format version。验证旧 HDF5形状、共享严格递增host tick、TCP 7th列为零占位后仅导入xyz+rpy；不补command/tactile/ownership/validity。输出记录逐文件SHA256，导入前后hash一致；validator以`W10`警告 legacy cache validity未知，必须人工审查训练来源。
- P7 验证：新增profile触觉采集、raw→align→validate→repair→verify→LeRobot v2.1/v3，以及XNero只读导入三条合成集成测试，3/3通过；全量 `morla_data_collect/test` 为128 passed/1 skipped；ROS旧采集 smoke 为2/2；12 profile CLI 12/12；`import-xnero --help`可从安装入口调用；干净source `/opt/ros/humble/setup.bash` 后全workspace `colcon build --symlink-install --base-paths src` 为31/31通过。第一次构建只source了workspace overlay，缺ROS underlay导致依赖查找失败，按标准干净shell重跑通过。期间修复legacy JointState callback闭包变量未初始化导致 executor 线程退出的回归。

- P8：policy 与 replay 以 profile 解析出的 `FrozenSchema` 固定 state/action 布局；模型握手校验 action capacity、profile/layout hash 和具名命令块；stub 对当前观测输出绝对 hold target，policy 命令使用与机器人适配器匹配的 reliable QoS。LeRobot 外层 schema 会校验并解包内部 MORLA schema，避免 replay 将19/38维布局降级成 legacy gripper schema。新增绝对目标与 LeRobot replay 回归后，策略测试149项通过。
- P8协调：新增 `morla_control_coordinator` 和 ROS-free `coordinator_logic.py`；Web、手柄闸门与CLI共用 `/morla/control/*` 服务。start/pause/resume/release 和 HOME/workpos/driver mode 操作使用异步 operation_id/state；先冻结 policy/遥操 writer，要求全部活动 writer ACK/prepare 成功再路由，未知或过期状态 fail closed，急停可抢占。Nero19仿真实际测得协调 start完成、HITL takeover/release、playback、pause/resume；不连接硬件。
- P9：统一 Web profile 选择、sim/real模式、robot/collector/inference lanes；profile切换检查 web 与CLI launch、collector、policy、coordinator及ROS command publishers；页面 streams/joints 根据profile动态生成。修复 install symlink 下 Starlette拒绝 dist 资产路径导致首页资源404的问题：symlink install使用源码dist，普通安装使用share/dist。headless Chrome实测 dashboard 可显示，`/` 与 JS bundle 均为200；`npm run build`通过，52 modules。实际后端/前端/API/launch测试最终结果见本文件最后记录。
- P10 Nero19端到端：`nero_left_xhand`在fake MuJoCo仿真上经Web启动teleop、采集及stub policy；19维状态/动作、profile layout hash `c77a2284825a`。协调start与策略命令运行，采集两路camera、arm/XHand、tactile约277帧；raw align/validate/export LeRobot v2.1成功，策略绝对动作、HITL接管/释放、policy replay与暂停/恢复均实际运行。验证报告有可选对齐触觉缺样及约50Hz状态低于策略推荐100Hz warning，无fail。无真机动作。
- P10 Astral35端到端采集：`astral_dual_gripper_wuji`仿真Profile装配双臂、左夹爪、DexPilot右WuJi手及Wuji MJCF；新sim hand状态发布 `/right_hand/joint_states` 50Hz。合成21点Quest hand landmarks驱动retarget，Profile采集含35维state/action、3相机、arm/hand命令和仿真状态；7.27s原始段对齐218帧，校验0 fail/1个右手约46.8Hz warning，导出LeRobot v2.1成功。
- P10 Nero38端到端：`nero_dual_xhand` sim Profile真实启动双侧 retarget/adapter/XNero MuJoCo；修复发现的XHand输入QoS冲突（Quest `/hand_landmarks/{side}`是best-effort，retarget此前默认reliable，现用`qos_profile_sensor_data`）。标准21点输入驱动左右12维标准命令（各约30.1Hz），仿真反馈约50.1Hz；raw保存双臂14维、双手24维、3相机与命令，7.27s对齐218帧、导出LeRobot v2.1 state/action 38/38。validate 0 fail，报告两个手state约50.1Hz、可选tactile未对齐为warning。首次合成测试误发26点且得到0 hand command，未作为通过证据；修正为21点后以`smoke5`数据通过，保留失败诊断在`/tmp/morla-ner38/nero_dual_xhand/smoke*`供复查。
- P10 profile/Web/构建：安装态 `morla-profile validate-all` 12/12（覆盖8/16/19/27/35/38/54维）；本次全workspace增量构建31/31通过。
- P10分包回归（ROS Humble系统Python、`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`并隔离ROS domain）：Web manager 37 passed、policy 149 passed、bringup 33 passed、data collect 128 passed/1 skipped；XHand retarget+adapter+XNero sim 88 passed，另有2个ROS demo Node类不可收集的pytest warning。
- 环境/过程：仿真均置于隔离 ROS_DOMAIN_ID，采集输出只写 `/tmp`；session结束后用SIGINT关闭仿真和collector。由于data_collect节点收到SIGINT时writer join会被KeyboardInterrupt打断，launch记录进程退出码-2；episode已由显式`stop`保存并可读/对齐，不能把此收尾退出码当作episode写盘失败。

- P10 Astral8完整链路：`astral_left_gripper`仿真Profile接收21点手输入与Quest腕Pose，经Astral遥操和夹爪话题写入；录制7.27s，8维state/action、双相机218帧；align后validate为1 pass/0 warn/0 fail，LeRobot v2.1导出成功。
- P10 Astral35修订验证使用规范21点输入和真实遥操输出：双臂命令约150Hz、Wuji retarget命令约30Hz，仿真右手状态约46.8Hz；35维数据段218帧完成align/export，只有低于策略推荐100Hz的手状态warning。
- P10 Nero38最终`smoke5`数据包含双臂Quest遥操命令约125Hz、左右XHand标准命令约30Hz、仿真反馈约50Hz；校验无fail，LeRobot v2.1导出38/38与3路相机。此前`smoke4`为同一链路的手输入修复前样本，按错误诊断保留，不用作验收证据。
- P10 full colcon lint记录：`colcon test`汇总813项里49个失败归于静态检查；其中`morla_input` flake8/pep257，以及上游`xhand_control_ros2`文件缺版权头、CMake空白和uncrustify。功能pytest按包隔离ROS domain后全通过；不要格式化供应商代码或伪造授权头。
- P10补齐统一复验入口：新增 `scripts/build.sh`（symlink/regular两种完整工作区构建）、`test_ros.sh`、`test_offline.sh`、`test_policy.sh` 与 `test_sim_integration.sh`。集成脚本只在隔离ROS domain启动Nero+XHand MuJoCo和profile采集器，发布合成21点手输入、标准臂命令及两路合成相机帧，再执行raw→align→validate→LeRobot v2.1；输出与日志默认留在`/tmp`，不接真机。此处记录为实现项，实测状态在后续追加。

- P10 闭环复验：`scripts/build.sh --mode symlink` 31/31 包通过；`scripts/test_ros.sh` 341 passed，tactile 专项筛选为1 passed/2 deselected，且 Astral geometric IK、IK solver、Quest mapping 三个独立回归入口均通过；`scripts/test_policy.sh` 149 passed；`scripts/test_offline.sh` 97 passed/1 deselected；profile CLI 12/12。统一 `robot.launch.py` 的 Astral `astral_left_gripper mode=sim` 分支实际启动成功，Nero `nero_left_xhand mode=sim` 分支由端到端脚本启动成功。
- P10 本轮 Nero19 端到端产物：`/tmp/morla-sim-e2e.vLvZOy/nero_left_xhand/smoke/episode000000`，采集两路图像及 arm/hand 命令、状态和触觉；对齐190帧并成功导出 LeRobot v2.1，schema 为19/19。验证报告 `validation_report.json` 为0 fail、1 warn；唯一 warning 是仿真 `left_hand_state` 50.2Hz 低于100Hz推荐速率。触觉原始侧车300个有效样本/每指、对齐触觉组存在；此前 validator 查错 HDF5 根路径导致的 W9 已修复，并由触觉对齐/导出回归锁定；录制起始手势跳变 W3 通过录制前仿真预热消除。产物摘要 SHA256：episode `meta.json` `028ea03a6a7e3872f810f6dfa936b6b6960c65fff5496c3b7b9086d381e65215`；validation report `b7efd7c21439047c4b2db9a72e07c81937cd8af8cb64cb98d7dac0b456d5568f`；LeRobot `morla_schema.json` `0394182b03c61b644e77f84c21c2b1f8767721cfa47e02e619159e971724b2f1`。所有数据与日志留在 `/tmp`，不进入 Git。
- 最终基线复核：SDK 根迁移计划 SHA256 仍为 `2b8580fd3a1af2865297a5f60a36098351fa0e2ffd2c5e19447bd80415af82b5`；`docs/SOURCE_BASELINE.json` 与根目录冻结清单字节一致（SHA256 `8880299fead51c35619f8f7027d3a3231557acaac75d623174ff80e2122b5733`）。XNero 当前分支只比冻结点多出上一节描述的新独立组件；未修改任何源工作区文件。

### 2026-10-14：坐标与运动学实施复验

- 按坐标优化计划完成 runtime、Profiles、asset revision、data/policy/Web 和 TF 状态桥适配；修改前基线数值仅保留作审查证据，实施后数据单独记录在 JSON 的 `implementation_validation`。
- 修复 ROS 参数类型边界：`solver_tip_to_tcp_transform` 现在声明为明确的 16 元素 double array，并用 `use_solver_tip_to_tcp_transform` 选择该工具链；standalone `tcp_offset` 仍可作为既有 fallback。新增 Profile launch 对 true/false tool matrix 的一致注入。
- 改进 `scripts/build.sh`：symlink/regular install 的 build mode 或 install prefix 发生变化时使用 `--cmake-clean-cache` 重配；symlink 输出到 `install/`，普通安装独立输出到 `install_regular/`，避免交替构建时复用旧 CMake install prefix。两种模式各31个包构建通过。
- 12/12 Profile 在 symlink 与 regular install 空间验证通过；12个含机械臂的 Profile 均成功构造测量状态 bridge 和 `robot_state_publisher` 启动动作。普通安装检查 27 个模型、358 个资产及 URDF/MJCF 加载器和全部 SHA 通过。
- `scripts/test_ros.sh`、`scripts/test_policy.sh`、`scripts/test_offline.sh` 全部通过；Web `npm run build` 通过。新版本 `scripts/test_sim_integration.sh` 实际启动 `nero_left_xhand` 仿真、采集器和 TF 状态桥，得到 190 帧，raw→align→validate→LeRobot v2.1 export 成功；quality report 只有已知的左手状态约50.2Hz低于100Hz建议阈值 warning。
- Nero solver-vs-URDF/MJCF FK 核对两侧各501个姿态通过：URDF最大约 `1.002e-5 m / 2.183e-7 rad`，MJCF最大约 `1.015e-5 m / 5.565e-7 rad`。活动 URDF/MJCF/manifest SHA 见实施 JSON；设备未连接，真实安装、SDK flange、connector 与任务 TCP 仍待验收。
- Astral `astral_left_gripper` Profile 通过新建的 `scripts/test_astral_sim_integration.sh` 完成 teleop start、实测状态桥、双相机合成采集、对齐、校验和 LeRobot v2.1 导出；208帧、8/8 schema、校验1 pass/0 warn/0 fail。另有短时无动作 smoke 确认 RSP 识别身体和双臂 frame；本轮数据只使用软件仿真和合成操作者腕位。

### 2026-10-14：恢复 Nero 活动控制末端为 link7

- 按用户最新指示，撤回将 `*_gripper_flange` 设为 Nero 默认活动控制末端的改动；四个 Nero Profile 均改回 `legacy_solver_tip=*_link7`，与解析 DH IK 的 FK/目标帧一致。通用命名工具 frame 固定链能力仍保留，Profile 如显式选法兰或手部 frame 时仍可解析。
- Nero teleop 未显式指定 `tool_frame` 时回到 solver tip/link7；默认变换为单位阵。因此默认 anchor、workspace 检查和 IK 目标使用同一 link7 frame，不再把法兰目标隐式换算成 link7。
- `*_gripper_flange` 和 SDK `get_flange_pose()` 继续作为独立的物理法兰模型/测量接口；规范 `/flange_pose` 及旧 `/tcp_pose` 兼容别名保留，SDK 与 URDF frame 的实体对齐仍标记未验收。
- 同步修正坐标计划、TODO、Nero 接口说明、FK 审计断言及 VR→IK→FK 回归的 frame 语义；删除只对应已撤销默认法兰选择的临时审计 JSON。
- 验证：`scripts/build.sh --mode symlink` 31/31 包通过；坐标与 flange-topic 测试 8 passed；Profile 校验 12/12；Nero 501姿态/侧的解析 FK 与 URDF/MJCF 审计通过；VR 位置映射及 link7 FK→IK→FK 回归 8/8；`git diff --check` 通过。
