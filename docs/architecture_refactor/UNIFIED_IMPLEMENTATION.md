# 统一组件与绑定：实现与复验记录

2026-10-30，基线 c1896c6。本轮提交/推送身份见 PUSH_LOG_UNIFIED_2026-10-30.md。执行计划见 UNIFIED_COMPOSITION_PLAN.md。

## 公共路径

1. morla_teleop.engine.BindingEngine 是目标生成内核。Web 的 _binding_controller/输入执行调用它；无 Web 的 ControlHost 调用同一内核。它处理实例/端口引用，不选择输入与机器人品牌组合。
2. morla-runtime --inputs 与 system.launch.py connect_inputs 使用 ControlHost；系统 ROS 入口可在同一 session 内启动输入与执行器，状态/操作仍走现有 Runtime 服务。仅启动不激活。
3. 输入 ROS 主机的内置设备订阅移入 builtin_sources.py。外部组件继续通过 morla.ros_inputs 提供桥。新增 morla.input.standard 提供公共 JSON 意图协议，可携带 pose/axes/scalar/joint_targets/hand_landmarks/button 和操作事件。设备特殊报文留在桥内部。
4. QuestRight 的 guarded 事件发布器/读取器同步增加 trigger/scalar；不只修改 descriptor。Vader、mobile 的 scalar 可以复用相同 range 映射。新增 pinch_scalar 把 21 点捏合距离转换成单自由度目标，可配置米制开口或比例目标；不直接写 Astral 话题。
5. ProcessSupervisor 根据 morla.processes 工厂自动托管已注册的输入进程，支持 external/managed/auto、环境参数、共享 key 去重、退出状态、失败回收；仅回收本对象创建的进程。SDK 内部进程仍由 adapter 的 configure/close 托管，原生设备由 RuntimeSession 创建/关闭其 adapter；其实现与启动计划已迁入独立 morla_native_bridge 包，并通过注册工厂加载。
6. QuestRight、旧 Quest、SDL gamepad/Xbox、Vader 注册进程工厂；Vader 整链 launch 和 CR5 VR launch 移除重复采集节点，使用公共 session 托管。保留单设备 launch 作为 external 运维入口。
7. Web 控制器匹配加入 scalar→joint_targets、pinch_scalar 和已安装 controller_types；轴名来自端口 axis_names，避免按 mobile 等品牌猜测。输入行新增进程模式。新增 controller 的 kinematics 可由 adapter 或 morla.kinematics entrypoint 提供。

## 配置示例

- examples/unified_composition/vr_epg.fake.yaml：VR 扳机→scalar_map→米制开口。
- examples/unified_composition/pinch_gripper.fake.yaml：骨架→pinch_filtered→Astral closure。换 EPG 时将 target/action 改为 position_target，output_range 改成 [0.05,0]。
- 双臂可由同一帧生成多个具名目标，或分别绑定不同输入。关节顺序与单位由组件描述决定。

通用无 Web 入口：先用 morla-system compile assembly.yaml --deployment deployment.yaml --output resolved.json 编译，再执行 morla-runtime resolved.json --inputs，或 ros2 launch morla_bringup system.launch.py resolved_path:=<绝对路径>。准备/激活使用 ROS runtime_node 服务；纯 CLI 不提供自动激活或隐式使能。

Web/HTTP session 继续使用 morla-composable-session --assembly ... --deployment ...；启动配置与同一 BindingEngine 复用。managed 来源所需 ROS 可执行文件与厂商 SDK 必须安装到运行环境。

## 实际证据

- pure：202 passed，4 skipped；其中厂商 SDK 缺失导致的跳过不能算通过。
- fake：23 passed；sim：7 passed。
- TypeScript 与 Vite build：通过，61 modules。
- Edge 实际浏览器：通过，pageErrors=[]。覆盖空配置建混合系统、能力绑定、数据键、版本启动、采集失败反馈、急停/复位/停止；浏览器并未连接真实设备。
- 新增扩展考试：临时安装两个未知机器人与一个输入组件的 distribution metadata，实际自动发现、编译、加载 adapter，使用公共 ControlHost 让双臂具名关节同步得到目标；未给公共层增加品牌分支。
- 新增进程测试：真实本机子进程启动/回收、external 不接管、启动失败清理、两个逻辑实例共享一个子进程。

原始输出见 evidence/unified-20261030/。当前主机是 macOS，无 ROS/rclpy，未运行 ROS 节点/launch、colcon 或实机测试。库测试不能证明 ROS topic/QoS 与现场驱动可用。

## 本轮代码检查与仍需完成的工作

检查中发现并修复：external 模式提前加载进程工厂；Vader/CR5 兼容 launch 与自动托管可能重复启动采集进程；Web scalar 控制器未接受米制 joint_targets；Web 按输入品牌猜轴名；原浏览器用例仍断言远端已删除的部署验证 flag。

最后一轮迁移已补齐：原生启动桥与 Nero/Astral 运动学注册下沉到设备包；CR5 注册通用运动学提供者；旧 pinch 自动范围、指数遗忘、EMA、扳机曲线和输出 slew 与通用控制器共用纯处理器；QuestRight 手部与扳机统一提供 scalar；输入动作分发在无 Web 与 Web 之间复用。

详细审查与反例见 [最终检查记录](reviews/UNIFIED_FINAL_REVIEW.md)。自动化代码检查、隔离安装与回归已执行；没有第二位 reviewer，不能宣称取得独立人工/agent 审查结论。当前未解决项属于目标环境验收：新版 ROS 构建/节点联调、完整硬件矩阵、实体 IK 与停止反馈。旧 full_teleop/pinch 节点保留为 legacy 兼容入口，新组合路径复用数值实现但不并行写同一设备。

## 文档核对新增开放项

UR11（源码核对，尚未执行 ROS）：Web stop 会关闭输入管理器，无 Web ROS operation(stop) 当前仅调用 Runtime.stop，输入进程需在宿主退出时回收。测试手册 U02/U04 与报告模板已区分这两个时点。若要求 stop 服务即完整释放输入，需要后续修改该入口生命周期并补 ROS 回归；当前不能声明这项已闭环。
