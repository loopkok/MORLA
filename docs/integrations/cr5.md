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

原 Profile 遥操节点和新组合 CartesianController 的输入处理、循环和过滤路径不同；复用同一求解器不能推出两条完整链路具有相同验证状态。CR5 当前只有关节级驱动；仍需 CR5 FK/IK 后端及选中加载接线、六关节契约与 base/TCP/VR 标定、Quest Assembly/controller 绑定和目标主机/实机验证。可复用公共增量控制逻辑，不应复制 Nero/Astral 几何参数或把 CR5 改名成已有机器人。当前没有可直接按 Nero 流程启动并完成 CR5 VR 跟随的已验证操作链。

真实设备身份/IP、6 个实际 joint_lower/joint_upper（rad）、零位/方向、物理停止、断网 watchdog、旧轨迹清理、安装/TCP 均需现场验证。descriptor ±2π 仅为外层保护上界。模板 verified/stop_verified=false 不表示可直接使能。CR5 URDF/IK、笛卡尔控制与组合仿真未实现，不能选其他品牌专用 IK。

启动/构建/单包测试见包 README。验收状态：离线协议/worker/单组件 fake 测试已执行；ROS Humble colcon/IDL/运行、实体机械臂、标定、长稳、采集/模型效果 NOT_RUN。独立构建与安装验证结果集中记录于 [本轮拆包验证](cr5_loophand.md#拆包验证)。
