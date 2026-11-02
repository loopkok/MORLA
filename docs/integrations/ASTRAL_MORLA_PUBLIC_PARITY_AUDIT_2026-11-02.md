# astral_ws → MORLA 功能迁移与公共化检查（2026-11-02）

本次检查本地 `/Users/a1234/Desktop/astral/astral_ws` 与 `/Users/a1234/Desktop/MORLA` 当前工作区，MORLA 基线 HEAD 为 `1e90a89`，包含本次聊天中尚未提交的公共工作站/HITL/日志改动。不是只比较 Git HEAD，也不是按目录同名数量判断功能迁移。

主要结论：原项目的大部分代码已经进入 MORLA，缺口主要在“旧 Profile/专用节点的完整功能没有接入公共 Assembly/Runtime 工作站”。本清单记录 18 个需要补齐或公共化的能力组，其中 8 组旧路径保留、9 组公共能力部分接通、1 个并行独立工具未检出对应入口；另有 1 处文档一致性问题，列出 7 组已公共化能力作为对照。**这些是检查分组，不是功能覆盖率或完成百分比。**

## 判断口径

- 已公共化：当前公共工作站能通过组件/控制器接口复用；不等于所有机器人都做过实机验收。
- 旧路径保留：MORLA 有实现，部分还可跨 Profile 复用，但不是当前公共 Assembly/Runtime 路径自动具备。
- 部分接通：公共层有底层对象、子能力或 API，但原来的完整行为/页面/生命周期尚未接齐。
- 未检出：在本次本地 src/scripts 文件清单及相关入口中未见对应实现，限定于当前检查范围，不推断外部 SDK、GPU 主机或其他分支。

这次是代码与调用链审计，没有启动机器人、实际模型或改动产品行为。完成了证据文件/行号校验，以及公共引擎依赖、固定模型端点、任务文本、采集 API、Headless 回调和独立工具清单的静态断言。原项目 README 的“已测”表述只作为原项目证据，不能外推为当前 MORLA 现场验收。

## 优先处理顺序

建议第一批处理 G02 输出约束、G12 任务文本一致性、G13 采集质量状态；它们直接影响运动输出一致性或采集数据解释。第二批处理 G01 完整推理调度与 G09 模型服务入口。G03 是需要人体姿态复现的 7 自由度机器人特有能力，应通过可选接口接入；G06 多段回位和 G18 无界面宿主完整化按部署需求推进。P1 表示建议优先补齐，不表示本报告发现了已发生的实机事故。

## 待补齐或公共化的能力

### G01 · 完整分块推理引擎

状态：旧 Profile／专用路径保留；领域：推理；优先级：P1。

原项目：queue_sync / queue_async / RTC、guided RTC、时序融合、换块 anchor 混合和控制插值。

MORLA 当前：ActionEngine 与旧 policy_node 已迁入；公共 RuntimePolicy 使用单任务线程池、动作列表与超龄期检查。

差别：公共策略路径未调用 ActionEngine，不能把旧 Profile 的预取/RTC/平滑能力算作公共工作站已具备。

后续：在保持 Manifest/owner 网关的前提下抽取共享动作调度器，并验证模型延迟大于单周期时的续播。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_policy_inference/astral_policy_inference/engine.py:102)；[已保留的完整引擎](/Users/a1234/Desktop/MORLA/src/common/morla_policy_inference/morla_policy_inference/engine.py:102)；[公共策略执行路径](/Users/a1234/Desktop/MORLA/src/common/morla_policy_inference/morla_policy_inference/composable.py:84)。

### G02 · 统一的运动输出约束

状态：公共层有基础，尚未对齐；领域：运动控制；优先级：P1。

原项目：策略 SafeExecutor 和遥操 SafetyFilter 提供限位、限速、滤波及异常保持。

MORLA 当前：公共网关已校验控制权；求解器、CR5 控制器和原生驱动各有局部约束。action_commands 主要检查形状/有限值并拆分指令。

差别：公共策略/遥操路径没有统一接入上述输出处理层；所有权校验不等于全品牌统一的速度、步长与关节限位约束。不能因此说所有驱动都没有保护。

后续：定义按组件单位和能力声明的公共输出处理契约，分别核对公共策略、遥操和回放路径，避免只依赖某个品牌驱动。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_policy_inference/astral_policy_inference/executor.py:33)；[保留但未接入公共 RuntimePolicy 的执行器](/Users/a1234/Desktop/MORLA/src/common/morla_policy_inference/morla_policy_inference/executor.py:33)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_policy_inference/morla_policy_inference/composable.py:59)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/robots/native/morla_native_bridge/morla_native_bridge/native_ros.py:270)。

### G03 · 人体肘部先验与接管后的肘部保持

状态：旧 Profile／专用路径保留；领域：遥操；优先级：P1。

原项目：IOBT 肩/肘方向生成 psi_ref；软/硬跟肘、直肘可观测性门控、接管后保持肘角直到人臂明显移动。

MORLA 当前：Astral 专用节点和几何求解器仍保留；公共 CartesianController 只向 solve 传目标位姿，公共 Quest 事件桥没有 body_joints→冗余自由度策略接线。

差别：协议层 HITL 已公共化，但这项 7 自由度运动学策略没有因此自动通用；CR5 6 自由度也不应照搬肘角逻辑。

后续：抽象可选的冗余自由度先验/重锚状态接口，让具备对应能力的机器人声明支持，再接入身体输入。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_arm_teleop/astral_arm_teleop/astral_arm_teleop_node.py:1179)；[已迁入的 Astral 专用处理](/Users/a1234/Desktop/MORLA/src/robots/astral/astral_arm_teleop/astral_arm_teleop/astral_arm_teleop_node.py:1312)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_teleop/morla_teleop/kinematics.py:158)。

### G04 · Astral 多 IK 选择与在线调参

状态：旧 Profile／专用路径保留；领域：遥操；优先级：P2。

原项目：geometric / analytic_dh / urdf_numerical，可切换参数并使用曲线调参工具。

MORLA 当前：求解器和调参脚本存在；native bridge 的 Astral factory 固定 geometric，公共表单没有等价的三模式选择和参数热更新协议。

差别：不是缺少求解器代码，而是公共选择/参数能力声明与调试入口尚未接通。部分几何求解器自身避奇异能力仍可使用。

后续：先把 solver_type、可调参数、更新生效方式注册为能力；不要把 Astral 专有参数硬塞给全部机器人。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_arm_teleop/README.md:52)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/robots/native/morla_native_bridge/morla_native_bridge/kinematics.py:9)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/robots/astral/astral_arm_teleop/astral_arm_teleop/teleop_tune_plot.py:45)。

### G05 · 头部绝对跟随与零点/回中语义

状态：旧 Profile／专用路径保留；领域：遥操；优先级：P2。

原项目：右摇杆→头 yaw/pitch，启动记录 head_init、摇杆回中返回初始方向，并参与遥操门控。

MORLA 当前：head_teleop_node 由旧 full_teleop launch 启动；公共层可自行配置关节点动，但没有该绝对映射和生命周期的等价控制器。

差别：可手动映射头部关节不等于原来的头部控制行为已经公共化。

后续：增加可选的绝对轴映射/参考姿态控制器，并统一接管、暂停、输入超时和回中行为。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_teleop/README.md:164)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_bringup/launch/full_teleop.launch.py:270)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_teleop/morla_teleop/controllers.py:52)。

### G06 · 经途经点的工作位与 HOME 收回

状态：公共层有基础，尚未对齐；领域：设备操作；优先级：P1。

原项目：正序 init_waypoints→init_pose，HOME 反向途经点→零位，含实测到位和驻留防切角逻辑。

MORLA 当前：公共工作站已有逐设备直接关节插补回工作位；旧 Astral/Nero 轨迹保留，Runtime 有低层 home/workpos，公共 Service 没有通用多段 HOME 协调。

差别：直接回工作位已通用，但不能替代按已配置走廊回位，也不能把 driver home 与 teleop 多段 HOME 混为一谈。

后续：建立可配置的多段关节轨迹与到位驻留能力，区分直接回位、途经点回位和 HOME。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_arm_teleop/README.md:451)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/workstation_control.py:185)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_runtime/morla_runtime/session.py:575)。

### G07 · 阻尼拖动、位置保持和硬件模式切换

状态：旧 Profile／专用路径保留；领域：设备操作；优先级：P2。

原项目：ready / enable / damping / position / home / estop，切回位置模式时实测重写目标，防旧缓存回跳。

MORLA 当前：Astral 驱动和旧 SystemTab/v1 robot API 已保留；公共操作主要是 prepare/start/pause/reset/estop/stop，没有完整能力化的阻尼/位置模式入口。

差别：驱动支持不代表公共工作站可以按设备能力统一展示和执行这些操作。

后续：给 Adapter 描述操作能力、完成证据和缓存清理语义，再由公共页面生成对应操作。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_robot_control/astral_robot_control/driver_node.py:33)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/robots/astral/astral_robot_control/astral_robot_control/driver_node.py:33)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/web_server.py:1135)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/composable_service.py:245)。

### G08 · Quest WebRTC 视频回传的公共编排

状态：旧 Profile／专用路径保留；领域：视频与仿真；优先级：P2。

原项目：多相机推送到 Quest，推流开关、相机子集选择和 adb 信令端口编排。

MORLA 当前：quest3_video_streamer、旧 VideoCard 和 v1 video API 都在；公共工作站提供相机采集/MJPEG，但输入进程计划未编排视频回传。

差别：浏览器能看相机不等于 Quest 第一视角回传已与公共会话和相机资源共享。

后续：让采集、网页预览和 WebRTC 共用相机提供者及资源生命周期，防止分别打开同一硬件。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_teleop/README.md:175)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_bringup/launch/full_teleop.launch.py:355)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_input/morla_input/processes.py:2)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/web_server.py:1244)。

### G09 · 模型服务选择与部署界面

状态：公共层有基础，尚未对齐；领域：推理；优先级：P1。

原项目：可选择 remote/inproc/stub、GPU 地址和端口，旧 Web 控制启动/重启推理及模型/引擎参数。

MORLA 当前：公共 Manifest 校验和策略 start API 已有；Service 固定连接 127.0.0.1:8001，公共 ComposableTab 没有完整的模型服务选择/启动表单。旧 InferenceCard 属另一条节点路径。

差别：本地端口转发可以工作，但不是原先可选 GPU 服务地址与后端的等价公共入口；localhost 限制是现有设计，不应随意改成任意网络访问。

后续：接入受信任的模型服务注册与部署配置，将当前工作站、Manifest 和模型生命周期在页面关联。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_web_monitor/web/src/components/InferenceCard.tsx:47)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/composable_service.py:483)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/web/src/components/InferenceCard.tsx:53)。

### G10 · 从已保存数据直接选择并回放

状态：公共层有基础，尚未对齐；领域：数据；优先级：P2。

原项目：从 raw episode、aligned HDF5、LeRobot 数据集加载轨迹并回放。

MORLA 当前：旧 replay.py loader 和 ReplaySourcePicker 已迁入；公共 replay/start 接口接收 frozen_resolved + actions 数组，尚未调用文件 loader 形成一键公共工作站回放。

差别：连续步进和 HITL 偏移已通用，但数据源发现、SchemaV3 动作投影、文件加载和页面选择没有完整串联。

后续：基于冻结契约选择录制段，服务端加载并验证，再交给现有公共回放调度，避免手工拼 actions。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_policy_inference/astral_policy_inference/replay.py:151)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_policy_inference/morla_policy_inference/replay.py:331)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/workstation_control.py:173)。

### G11 · 采集暂停、继续、保存并开新段

状态：公共层有基础，尚未对齐；领域：数据；优先级：P2。

原项目：同一段 pause/resume、next 保存后续段，任务/session 管理与采集 FSM。

MORLA 当前：公共 recorder 内部已有 pause/resume；公共 action/API 只串联开始、停止保存和丢弃，未暴露同段暂停/继续和 next。

差别：底层 recorder 有方法，不等于手柄、键盘、Web 和状态显示已能使用。

后续：先明确采集暂停与运动暂停不同，再通过公共操作与状态协议接入，不擅自复用目前已指定的键位。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_data_collect/astral_data_collect/data_collect_node.py:438)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_data_collect/morla_data_collect/composable_recorder.py:164)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/workstation_control.py:107)。

### G12 · 任务文本在 Web/手柄/键盘间一致

状态：公共层有基础，尚未对齐；领域：数据；优先级：P1。

原项目：latched task 成为后续 episode 的任务文本，VR/键盘开始采集沿用同一 pending_task。

MORLA 当前：Web 开始记录会传页面 task；公共 A 键/键盘 A/Vader A 的录制分支硬编码 operator demonstration。

差别：同一工作站仅更换开始入口，就可能生成不同的训练任务文本。

后续：将当前任务文本纳入会话公共状态，所有录制入口使用同一任务，不在输入分支写固定文本。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_data_collect/astral_data_collect/data_collect_node.py:369)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/workstation_control.py:106)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/web/src/components/ComposableTab.tsx:315)。

### G13 · 采集在线质量告警与计数

状态：公共层有基础，尚未对齐；领域：数据；优先级：P1。

原项目：各流计数/实率、低 FPS、空录告警、丢帧和被忽略操作等状态提示。

MORLA 当前：公共 recorder 有状态/队列计数并会报告溢出失败；公共 session 主要暴露 recording 布尔值，没有旧采集卡同等的逐流实时质量状态。

差别：诊断日志正在写入，不能证明训练数据正在以正确频率采集；两种监控不能相互替代。

后续：从 recorder 统一导出采样率、计数、丢弃、写入失败和空录/必需流缺失告警，再接到工作站页面。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_data_collect/astral_data_collect/data_collect_node.py:118)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_data_collect/morla_data_collect/composable_recorder.py:229)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/composable_service.py:227)。

### G14 · 推理与相机的细分诊断/曲线工具接线

状态：公共层有基础，尚未对齐；领域：诊断；优先级：P2。

原项目：planner/换块 trace、server_timing、图像解码/观测龄期、命令尖峰与延迟拆分。

MORLA 当前：旧 node 的详细日志和 plot_inference_curves 等工具已保留；公共日志覆盖输入、映射、HITL、反馈和回执，ManifestRemotePredictor 主要返回 actions。

差别：公共日志不等于旧推理诊断字段已全覆盖，也未见旧曲线工具对公共 JSONL 的等价适配。

后续：统一输出模型端/网络端/调度端 timing、chunk 游标及相机阶段诊断，提供公共日志分析适配器。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_policy_inference/astral_policy_inference/node.py:1470)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_policy_inference/morla_policy_inference/node.py:1726)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_policy_inference/morla_policy_inference/composable_remote.py:30)。

### G15 · Wuji 调参视图与热重载

状态：旧 Profile／专用路径保留；领域：视频与仿真；优先级：P3。

原项目：TuningViewer 展示骨架/网格并热重载 retarget YAML。

MORLA 当前：wujihand_mujoco_sim 的 tuning_viewer_node 与 launch 已迁入，公共 MuJoCo/Web 控制台没有等价通用调参工作台。

差别：专用工具可以继续使用；不是公共仿真能渲染手模型就自动具备骨架对照和参数热重载。

后续：按需抽象重定向调参/可视化协议，优先保留专用工具可运行，不必为臂机器人强加手部界面。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/wujihand_mujoco_sim/README.md:9)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/robots/wuji/wujihand_mujoco_sim/wujihand_mujoco_sim/tuning_viewer_node.py:68)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_runtime/morla_runtime/sim_mujoco.py:29)。

### G16 · PiM-IK 学习臂角管线

状态：旧 Profile／专用路径保留；领域：遥操；优先级：P3。

原项目：网络预测 psi + Astral 几何 IK，含数据生成、训练、评估和遥操数据转换。

MORLA 当前：完整包已在 robots/astral/astral_pim_ik；公共 native factory 未选择 PiM predictor，也没有通用学习冗余策略插件入口。

差别：这是已迁入的研究工具，不是已接入生产遥操的公共能力；不应把原合成数据实验算作实机已验收。

后续：如确有需求，先定义可选先验插件和模型/几何契约，再做评估与现场验证，不直接替换当前默认 IK。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_pim_ik/README.md:5)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/robots/astral/astral_pim_ik/astral_pim_ik/pipeline.py:56)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/robots/native/morla_native_bridge/morla_native_bridge/kinematics.py:7)。

### G17 · 独立无 ROS pi0.5 分级试运行工具

状态：未检出对应迁移入口；领域：推理；优先级：P3。

原项目：pi05_standalone 提供 observe / infer / execute，直接 SDK 与相机运行，不依赖 Astral ROS 包。

MORLA 当前：本次文件名/包清单检查未检出该目录及 infer_astral_single.py / serve_pi05.py 的对应入口；MORLA 有公共策略、serve 和训练工具，不能视为完全等价。

差别：这是并行独立工具的迁移缺项，不是原主遥操链缺失；不建议为补齐而再造一个绕过工作站的命令源。

后续：如需要观测/影子推理验收，优先在公共模型路径增加不下发模式；保留独立工具时明确互斥和部署边界。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/pi05_standalone/README.md:54)；[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/pi05_standalone/infer_astral_single.py:445)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/composable_service.py:472)。

### G18 · 完整公共功能在 Headless 入口的等价性

状态：公共层有基础，尚未对齐；领域：编排；优先级：P1。

原项目：原 full_teleop CLI 与 Web 使用同一套 ROS 节点/操作话题，采集和推理可组合启动。

MORLA 当前：公共 Web 工作站已具确认/回滚/交还；独立 ControlHost 只有可选 operator_action 和录制回调，普通 runtime --inputs 不会自动装配完整协调器/recorder。

差别：多机器人通过公共 Web 可以复用完整 HITL；不能把所有无 Web 入口也宣传为自动拥有同等整套能力。

后续：将操作协调器的宿主依赖抽象成公共接口，提供具备同样状态与录制管理的 headless 启动入口。

依据：[Astral 原实现](/Users/a1234/Desktop/astral/astral_ws/src/astral_teleop/README.md:65)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_teleop/morla_teleop/session.py:7)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_runtime/README.md:41)。

## 已公共化，不应重复列为迁移缺项

### C01 · 声明式设备装配与 Web 配置

MORLA 已有组件目录、装配/标定/映射表单、编译快照与设备插件。 已公共化；不是迁移缺项。

依据：[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/web/src/components/ComposableTab.tsx:89)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/docs/device_integration_guide.md:9)。

### C02 · 基础遥操映射与统一操作键

公共 BindingEngine 与控制插件、Quest/Vader/键盘统一操作已实现。 基础链路已公共化，节点专有肘策略等另见 G03。

依据：[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_teleop/morla_teleop/engine.py:10)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/web/src/lib/operatorKeyboard.ts:3)。

### C03 · HITL 协议和回放偏移

公共工作站已对齐确认/回滚/取消/迟到隔离、首预测确认、暂停恢复和分发前 owner 检查。 协议层已公共化；不等于所有品牌已完成现场验收，也不涵盖 G03 的肘部语义。

依据：[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/handoff.py:8)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/docs/integrations/WORKSTATION_HITL_ASTRAL_ALIGNMENT_2026-11-02.md:51)。

### C04 · SchemaV3 采集与数据/模型契约

公共 recorder、独立 state/action 布局、来源标记、Manifest/几何/归一化校验已实现。 基础已公共化；采集控制和在线质量缺口见 G11–G13。

依据：[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_data_collect/morla_data_collect/composable_recorder.py:206)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_learning/morla_learning/contracts.py:203)。

### C05 · 离线清洗、导出与训练工具

代码已迁到 common pipeline/learning 和 scripts，另有 DataProcessTab；旧路径名称不同不代表功能缺失。 不应列为未迁入；公共录制目录与页面发现仍需按部署配置核对。

依据：[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_data_collect/morla_data_collect/pipeline/repair_aligned.py:67)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/web/src/components/DataProcessTab.tsx:73)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_learning/morla_learning/backends.py:90)。

### C06 · 公共 MuJoCo 与模型资源

公共仿真支持运动学/动力学、网页/本地窗口，CR5/Nero/Astral 共享后端；模型生成脚本已迁到 robot_description。 基础已公共化；专用调参视图见 G15，不能把模型移动算作代码丢失。

依据：[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_runtime/morla_runtime/sim_mujoco.py:29)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/robot_description/scripts/generate_clean_urdf.py:27)。

### C07 · 独立按次运行的诊断日志

公共 Runtime 现在默认按运行保存，独立数据采集，带限额、异步写入、错误计数和 Web 下载。 公共基础已实现；推理/相机细分字段与分析适配仍见 G14。

依据：[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_runtime/morla_runtime/diagnostic_log.py:81)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/docs/integrations/COMMON_TELEOP_DIAGNOSTICS_2026-11-02.md:21)。

## 附加发现：文档入口说明

### D01 · 入口与验收说明存在过时表述

公共 HITL 已加入网关检查，但 morla_policy_inference README 顶部仍写 Runtime 不执行互斥；后面又描述旧完整引擎。 按旧说明阅读容易误判公共功能已齐全，或误以为刚实现的网关检查不存在。

同步入口能力矩阵、文档版本和验收证据，明确 legacy/Profile 与公共 Assembly 的适用范围。

依据：[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_policy_inference/README.md:7)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/src/common/morla_web_manager/morla_web_manager/workstation_control.py:112)；[MORLA 当前实现](/Users/a1234/Desktop/MORLA/docs/integrations/WORKSTATION_HITL_ASTRAL_ALIGNMENT_2026-11-02.md:39)。

## 本次没有判成缺失的内容

PiM-IK 位于 `src/robots/astral/astral_pim_ik`；干净 URDF/MJCF/MDH 生成工具已移到 `src/robot_description/scripts`；原离线 align/validate/repair/export/Rerun 工具已进入 `morla_data_collect.pipeline`；旧推理引擎、Web 模型卡片、WebRTC 推流和 Wuji 调参工具也保留着。它们的主要问题是入口、协议和公共生命周期接线，不能因原路径不存在而断言代码丢失。

## 审计边界

已检查主要包、启动路径、公共 API、控制器、策略调度、采集、回放、仿真、视频、模型工具和相关说明。没有逐一证明每个旧参数和外部模型工程的运行效果；没有读取或复核 GPU 主机上部署代码、所有实机历史日志或外部 SDK 实现。本报告中的“没有接入”以本次实际调用链为依据，后续实现时应同时核对当前代码并更新清单。
