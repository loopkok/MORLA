# CR5 独立组件接入记录

日期：2026-10-28；MORLA 基线 b91211a，当前工作区实现。依据 [设备接入指南](../device_integration_guide.md)。协议来源为桌面 cr5_loophand_graspnet/arm_CR，原项目未修改。

## 包边界

独立包：[morla_cr5_driver](../../src/robots/dobot/morla_cr5_driver/README.md)，组件 `morla.dobot.cr5@1.0.0`。拥有自己的 package.xml/setup.py、Python 模块、descriptor、ament manifest、adapter entry points、cr5_driver ROS 入口、单设备 launch、6 维 fake Assembly、real Deployment、许可证/来源记录和测试。

不依赖 LoopHand、手部 SDK 或 pyserial；可与其他品牌末端组合，也可只运行机械臂。通用缓存、时效、命令/操作和 ROS 反馈发布复用 morla_runtime。设备包不执行其他设备的生命周期。

## 数据与控制约定

canonical 为 joint1..joint6，位置 rad。TCP 控制/命令/反馈端口为 29999/30003/30004。1440 字节二进制 q_actual 是测量来源，校验包长、魔数、设备时钟；degree↔rad 转换在 transport 边界。使能/停止等待新的 enable_status；启动不使能、不清错、不回零、不自动 Continue。

JointState 发布实际采样时间且不重复刷新缓存样本时间；state/health 保留硬件/fake 来源与有效性。所有目标经过 MORLA CommandEnvelope、租约、epoch/序号/TTL 及组件目标验证。Adapter 有部署限位、实测步长和主机命令 watchdog。

## 部署与未验收项

### VR 与公共 IK 的实际关系

`morla_teleop/kinematics.py` 的公共 CartesianController 提供实测播种、VR 相对位移/旋转、重锚、TCP 偏置、求解耗时与结果跳变检查；它要求后端具备 sync_state(q)、fk(q)、solve(T_target)。实际数学求解仍在各机器人包中：Nero 使用专用七自由度解析/几何求解器；Astral 工厂支持 analytic_dh、geometric、urdf_numerical，新组合入口当前固定选 geometric。公共层不是已实现任意机器人的通用 IK 算法。

原 Profile 遥操节点和新组合 CartesianController 的输入处理、循环和过滤路径不同；复用同一求解器不推出完整链路验证等价。

2026-10-29 已在当前工作区补齐 CR5 URDF / Pinocchio 本地数值 FK/IK 后端、异步 controller 及选中加载、右手柄/右手来源感知输入、握把/捏合 deadman、Quest 装配和控制页。完整流程见 [CR5 VR 遥操](cr5_vr.md)。复用公共增量映射和 TCP 逻辑，没有复制 Nero/Astral 几何参数。

真实设备身份/IP、实际 joint_lower/joint_upper、零位/方向、物理停止/断网 watchdog、安装 TCP 和输入基座旋转仍需现场验证。真实模式新增 kinematics_verified 和 binding calibration_verified 声明，未验证时不能激活该 controller。纯 CR5 本地 URDF 已接入统一 robot_description，未迁入夹爪/ros2_control/OCS2；fake/real 遥操均使用本地 IK，无控制器 IK 回退。物理仿真与实机尚未验收。

启动/构建/单包测试见包 README。验收状态：离线协议/worker/单组件 fake 测试已执行；ROS Humble colcon/IDL/运行、实体机械臂、标定、长稳、采集/模型效果 NOT_RUN。独立构建与安装验证结果集中记录于 [本轮拆包验证](cr5_loophand.md#拆包验证)。
