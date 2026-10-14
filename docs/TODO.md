# MORLA 后续 TODO

本文记录坐标运动学专项的完成状态，以及尚未排期的高自由度组合装配架构工作。组合装配不属于当前 P0–P11 软件迁移范围；真机坐标/TCP验收仍待设备条件。完整边界以 `migration_progress.md`、`MIGRATION_PLAN.md` 和源码为准。

## TODO：坐标、关节零位与工具链统一

**状态：Nero 裸臂活动控制末端保持 `*_link7`；SDK 实测法兰 pose 对齐、实体安装与外接任务 TCP 验收待设备条件。**

完整执行依据：[坐标与运动学优化计划](coordinate_kinematics_optimization_plan.md)。审查数值与源码基线：[证据 JSON](coordinate_kinematics_audit_2026-10-14.json)。

- [x] 统一 Unity basis、操作者输入对齐、机器人固定安装与求解器适配的解析入口；Profile 是唯一生效来源，重复 teleop 矩阵已清理。
- [x] 保留左右单臂基座，由共同身体/装配参考派生 solver-base 输入朝向；Quest operator-body 与 robot body 使用不同 frame 标识。
- [x] 恢复活动 Nero URDF/MJCF 的真实 J2 零位与硬件角限位；仿真下垂姿态作为 sim-only 初始关节值。
- [x] 实现 IK tip→URDF 命名工具 frame 固定链解析；修正 Astral/Nero 非零末端变换的 anchor/init/re-anchor/HITL 路径。Nero IK 与活动控制末端均使用 `*_link7`；`*_gripper_flange`、connector/hand/任务 TCP 保留为单独命名的物理或外接帧。
- [x] 将 frame、工具语义和 kinematics hash 接入测量状态→TF、采集/export、策略/replay 与 Web；软件几何闭环和 ROS 仿真数据整链路已通过。
- [ ] 用设备/CAD/厂商 SDK 依据核对 `get_flange_pose()` 是否与 URDF `*_gripper_flange` 原点、轴向及 RPY 顺序一致；核对外接 connector/手部安装尺寸，并对用户需要的抓取 TCP 单独标定。完成后做低速实机姿态验收。

本项先于高自由度组合装配的几何能力开放。已发现的模型零位修正属于显式的运动学版本变化，不能要求修正前后的错误模型几何仍完全相同。

## TODO：高自由度、声明式机器人与执行器组合装配

**状态：待规划；当前未实现。**

### 目标

把目前“Profile 登记 + 启动器按机器人类型分支”的有限装配方式，逐步演进为声明式组合架构。用户通过可编辑的机器人配置描述机器人本体、执行器、传感器、输入源、控制链路和运行后端之间的组合关系；MORLA 根据已安装组件的能力声明校验组合，并生成仿真或真机启动图。新增已实现的机器人或执行器时，常规接入工作应主要落在组件包、能力声明和机器人配置中，不应每次都修改公共启动器中的机器人专属分支。

### 当前限制

- `src/common/morla_bringup/config/robots.yaml` 是 Profile 索引，只把 `profile_id` 映射到 Profile 包、路径和显示标题；它不是完整的启动图或执行器注册表。
- `morla_bringup/morla_bringup/robot_profile_launch.py::select_robot_stack()` 按 `robot_id` 和已知组件类型选择固定 launch assembly。目前只分派 Astral 与 Nero，并分别识别有限的手臂、夹爪和手组件类型。
- Profile 配置描述的数据布局和组件，不代表对应任意组合已经有仿真、真机驱动、控制接口或安全实现。仿真与真机支持范围也不完全相同。
- `validate-all` 验证 Profile/Schema，不验证相机、CAN/串口、驱动反馈、模型兼容性或实体运动安全。

### 规划设计

1. **定义组件能力描述格式**：由机器人本体、机械臂、末端执行器、传感器、输入源和控制器包声明稳定 ID、版本、侧别、ROS 接口、关节/特征布局、单位与限位、资源模型、输入输出依赖，以及支持的仿真/真机后端。
2. **定义可组合的机器人配置**：配置引用已安装组件并声明实例、侧别、连接/绑定关系、数据流、控制所有权、描述资源和运行模式。先以版本化 YAML 为可审查来源；评估后再提供 Web 可视化编辑、预览和导出，避免 UI 状态成为另一份配置事实来源。
3. **建立组合兼容性校验器**：启动前检查机械与侧别约束、接口和消息类型、命名关节与维度、单位/坐标系、状态与动作布局、资源存在性、后端支持、输入/驱动依赖、控制权冲突和安全限制；拒绝无法证明兼容的组合，并给出指向具体组件/字段的错误。
4. **生成统一运行时装配图**：由验证后的配置确定 launch 节点、参数、topic/service remap、模型资源、驱动适配器和启动顺序；仿真、dry-run 与真机复用同一份机器人组成定义，只选择各组件已声明且已实现的后端。配置负责选择能力，不能凭空创建缺失的驱动或硬件支持。
5. **保持公共数据与控制契约统一**：从最终组合生成唯一的 FrozenSchema、`schema_hash`、`layout_hash` 和命名关节映射，并供 bringup、teleop、数据采集、策略推理、回放和 Web 共用。任何维度或布局变化都必须进入显式版本与模型握手校验。
6. **让 Web 管理可编辑且可预览**：展示可用组件、兼容关系和模式能力；允许用户构建/复制组合，预览将启动的组件与资源，运行 Profile 校验后保存版本化配置。当前阶段只规划这一能力，不把任意脚本或任意命令拼接进配置执行。
7. **建立迁移与验收路径**：先将现有 Astral/Nero 12 个 Profile 转为新格式并做到行为等价，再逐步开放新增组合；保留明确的真机确认流程、急停语义、单一写入者/租约、驱动限位和仿真/真机互斥检查。

### 分阶段实施建议

- **A. 盘点与规范**：整理现有 launch 参数、组件、topic、资源和硬件前置条件，确定组件描述与组合配置的 schema/version。
- **B. 纯逻辑解析与校验**：实现 ROS-free 配置解析、能力解析、兼容性诊断和确定性启动图生成；不触碰现有硬件控制路径。
- **C. 并行适配现有 Profile**：将 12 个 Profile 转为声明式组合，比较旧/新解析得到的 Schema、hash、命名映射和 launch 参数；在通过等价验证前保留旧入口。
- **D. 仿真装配**：优先支持已具备仿真组件的 Astral/Nero/XHand/Wuji 组合，覆盖 headless 和可视化启动、输入到状态闭环及失败回收。
- **E. Web 配置管理**：加入兼容性预览、配置保存/版本回滚和启动；UI 调用同一解析器与验证器，不重复实现兼容规则。
- **F. 真机逐组件开放**：按驱动、SDK、固件和设备验收结果声明 real 能力；未完成验收的组件或组合在真机选项中保持禁用。

### 完成标准

- 修改组件声明或组合配置即可接入一个已安装且能力完整的新组合；公共 bringup 不再需要为每个普通组合新增机器人 ID 分支。
- 对缺失驱动、未知执行器、错误侧别、不兼容关节/单位、重复控制权和不支持的运行后端，启动前给出稳定、具体的拒绝原因。
- 纯装配迁移保持现有 12 个组合的已验证布局与控制行为；模型零位/工具等有意修正按上述专项计划记录新运动学版本及预期差异，不能静默改变数据和控制语义。
- bringup、采集、策略、回放与 Web 对同一配置解析出相同 Profile 和 Schema；launch 记录配置版本及 hash。
- 仿真、dry-run、真机能力分别声明；选择真机时必须具备对应真实后端和设备配置，且不因仿真测试通过而标记为真机通过。
- Web 展示配置差异、兼容校验结果和待满足条件；保存的配置可在命令行和 CI 中复现。

### 实施入口参考

- Profile 索引：`src/common/morla_bringup/config/robots.yaml`
- 当前 launch 分派：`src/common/morla_bringup/morla_bringup/robot_profile_launch.py`
- 当前统一入口：`src/common/morla_bringup/launch/robot.launch.py`
- Profile 与布局基线：`docs/profile_reference.md`
- 迁移状态及硬件验收边界：`docs/migration_progress.md`
