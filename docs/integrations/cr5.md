# CR5 独立组件接入记录

> 2026-10-30 更新：当前统一路径使用公共 BindingEngine/ControlHost；CR5 可通过 morla.kinematics 提供求解器，也保留原异步控制器插件。启动/输入托管、配置替换与验收见 [统一路径记录](../architecture_refactor/UNIFIED_IMPLEMENTATION.md) 和 [主机手册](../testing/host_build_and_hardware_acceptance.md)。

日期：2026-10-28；MORLA 基线 b91211a，当前工作区实现。依据 [设备接入指南](../device_integration_guide.md)。协议来源为桌面 cr5_loophand_graspnet/arm_CR，原项目未修改。

## 包边界

独立包：[morla_cr5_driver](../../src/robots/dobot/morla_cr5_driver/README.md)，组件 `morla.dobot.cr5@1.0.0`。拥有自己的 package.xml/setup.py、Python 模块、descriptor、ament manifest、adapter entry points、cr5_driver ROS 入口、单设备 launch、6 维 fake Assembly、real Deployment、许可证/来源记录和测试。

不依赖 LoopHand、手部 SDK 或 pyserial；可与其他品牌末端组合，也可只运行机械臂。通用缓存、时效、命令/操作和 ROS 反馈发布复用 morla_runtime。设备包不执行其他设备的生命周期。

## 数据与控制约定

canonical 为 joint1..joint6，位置 rad。TCP 控制/命令/反馈端口为 29999/30003/30004。1440 字节二进制 q_actual 是测量来源，校验包长、魔数、设备时钟；degree↔rad 转换在 transport 边界。使能/停止等待新的 enable_status；启动不使能、不清错、不回零、不自动 Continue。

JointState 发布实际采样时间且不重复刷新缓存样本时间；state/health 保留硬件/fake 来源与有效性。目标仍带 CommandEnvelope 的 session/token/epoch/sequence/TTL 字段，但 2026-10-30 回退后 runtime 只检查急停锁存、组件子集与 Adapter 接受，不校验租约或序号。Adapter 保留部署 joint_lower/joint_upper 与 URDF 交集限位；目标步长和主机命令 watchdog 已移除，上游停发时执行器保持最后目标。

## 部署与未验收项

### VR 与公共 IK 的实际关系

`morla_teleop/kinematics.py` 的公共 CartesianController 提供实测播种、VR 相对位移/旋转、重锚、TCP 偏置、求解耗时与结果跳变检查；它要求后端具备 sync_state(q)、fk(q)、solve(T_target)。实际数学求解仍在各机器人包中：Nero 使用专用七自由度解析/几何求解器；Astral 工厂支持 analytic_dh、geometric、urdf_numerical，新组合入口当前固定选 geometric。公共层不是已实现任意机器人的通用 IK 算法。

原 Profile 遥操节点和新组合 CartesianController 的输入处理、循环和过滤路径不同；复用同一求解器不推出完整链路验证等价。

2026-10-29 已在当前工作区补齐 CR5 URDF / Pinocchio 本地数值 FK/IK 后端、异步 controller 及选中加载、右手柄/右手来源感知输入、Quest 装配和控制页。完整流程见 [CR5 VR 遥操](cr5_vr.md)。复用公共增量映射和 TCP 逻辑，没有复制 Nero/Astral 几何参数。2026-11-02 起随包 Quest 装配不再声明 `binding.deadman`，但握把/捏合仍写入 `quest.right_hold`，公共工作站据此发布 `operator_hold`（TTL 250 ms）并门控运动：**必须按住右手 grip 才跟随，松开即停**（不撤权，会话保持 ACTIVE）。停止需显式 stop/revoke；输入断流超过 `max_age_ms` 时按当前臂位姿重新建锚，不产生跳变也不撤权。2026-11-03 更正了此前“持续跟随、松手/松开捏合不停止”的描述。

真实设备身份/IP、实际 joint_lower/joint_upper、零位/方向、物理停止/断网 watchdog、安装 TCP 和输入基座旋转仍需现场验证。真实模式的 kinematics_verified / binding calibration_verified 只是记录字段，回退后不再阻止激活；方向、零点和 TCP 必须现场核对后再运动。纯 CR5 本地 URDF 已接入统一 robot_description，未迁入夹爪/ros2_control/OCS2；fake/real 遥操均使用本地 IK，无控制器 IK 回退。物理仿真与实机尚未验收。

启动/构建/单包测试见包 README。验收状态：离线协议/worker/单组件 fake 测试已执行；ROS Humble colcon/IDL/运行、实体机械臂、标定、长稳、采集/模型效果 NOT_RUN。独立构建与安装验证结果集中记录于 [本轮拆包验证](cr5_loophand.md#拆包验证)。

### 2026-11-01 MuJoCo 调试环境

CR5 已增加统一 `sim` 后端和 Web 组合页模板/相机预览；`cr5_sim.launch.py` 可独立启动仿真与统一 Web。MJCF 由同一 IK URDF 生成，显式映射六轴并保留 Link6 末端坐标。使用方式、初始姿态和边界见 [包说明](../../src/robots/dobot/morla_cr5_driver/README.md#cr5-mujoco-仿真与-web)。碰撞、实机伺服等价性、实体 Quest 全链路与 IK 故障根因仍待现场验证。
