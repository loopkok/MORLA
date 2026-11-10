# MMK2 SDK / MORLA ROS 2 driver

将用户提供的 `mmk2-sdk 0.0.4.dev5` 原生 gRPC 协议封装成 MORLA 组合组件。
本包是实机通信驱动，`fake` 是明确标记的协议合成数据；整机 MuJoCo 仿真仍由
`mmk2_mujoco_sim` 提供。源包模块版本与 distribution 版本不一致，见 PROVENANCE.json。

## 接口与边界

- 插件前缀 `morla.airbot.mmk2_sdk.`：left_arm、right_arm、left_gripper、right_gripper、lift、head、base、camera。
- 双臂各 6 个 SDK 具名关节，单位 rad；升降 slide_joint，m；头部 yaw/pitch，rad。
- 夹爪使用 SDK 原始 `[0,1]`、单位 `sdk_gripper`，没有假设哪端是闭合，也没有转换成仿真的单指米制量。
- 底盘原生目标 `[linear_x, angular_z]`，通道单位 m/s、rad/s，复合端口单位 `m_rad/s`，不接受仿真轮速。位置状态来自 base_state.pose；没有伪造轮角反馈。
- 七组执行器 19 维状态/动作（17 维位置 + 2 维底盘速度）。原始反馈按名字重排，缺关节、重复名字、错误码、非有限值、时钟倒退会报故障；重复设备时间戳不刷新新鲜度。
- 同一 session_id 共享连接、反馈和整机停止域。连接参数必须一致，运动门禁按执行器独立配置，相机不参与运动门禁。状态与命令、相机分别运行，RPC 有截止时间、状态流可取消、命令只有最新槽位。
- 相机选择 head_camera / left_camera / right_camera；RGB JPEG、原始深度 PNG16。深度为 `sdk_depth`，`depth_scale_m=0` 表示尚无厂商标定；不默认毫米。
- 不声明 SDK 未提供的 IMU、雷达、力反馈、碰撞模型或硬件看门狗。

## 写入与停止

默认 `allow_motion=false`，连接后只读。`activate/enable` 仅授权软件发送命令，
**不代表电机已使能**。SDK 没有可靠的电机使能/失能或急停接口；厂商 `stop_all()` 只有日志。
本包停止策略为向已写入过的组件发送最后发送的位置目标保持，以及已控制底盘的零 Twist；
清除所有待发目标、撤销整机软件授权。RPC ACK 仅表示请求被接受，不能证明实际物理停止。

实机写入必须显式设置 `allow_motion`、`mapping_verified`、`hold_stop_accepted` 三项为 true，
设备身份是公共编译器的硬检查；公共 `verified`/`stop_verified` 仅记录现场验收，不能替代以上三项驱动门禁。现场先由厂商流程准备控制器，确认急停、
使能、方向、零位、SDK/URDF 关节对应、头部轴顺序、夹爪极性与安全空间。
`joint_signs`/`joint_offsets` 用于部署映射，目标仍受 SDK 原始边界与部署收紧边界约束。
底盘不支持位置偏移。连接断开后软件无法保证控制器停止，需现场验证控制器侧超时。
故障不自动重连、不恢复旧目标；排除故障后重新启动会话。

双臂默认使用受限微分 IK（同 FR3/CR5 策略）：每帧一次阻尼更新，关节步长由驱动限速/频率与领先距离共同限制，笛卡尔目标领先截断 2cm/0.35rad，不宣称每帧精确收敛。领先与阻尼是实例参数 `position_lead_m`、`rotation_lead_rad`、`damping_min`、`damping_max`、`singularity_threshold`（默认 0.02 / 0.35 / 0.005 / 0.145 / 0.05，与仿真包一致；`rotation_lead_rad` 在设备页可见）；绑定侧另有 `smoothing_alpha`（输入低通，1=关闭）和 `rotation_scale`（旋转倍率，默认 1.0）。
有效目标每拍从原始目标重新推导（做领先截断后即用），不跨拍积分；积分写法在手静止时会把目标冻结在“上一拍关节位置 + 领先距离”，留下永久偏置。贴限位的关节按 active-set 锁住重解，其余关节承担步长，被拒绝的向外需求记入 `limit_pressure_rad`。
单拍关节步长同时受绑定参数 `max_joint_step` / `max_joint_velocity` 约束（`min(max_joint_step, max_joint_velocity·dt)`），模板 0.05 rad / 2.0 rad/s；低于这个量级时手一动臂就明显落后。
默认 trajectory 请求，缩放 0.05，主机限速/目标领先距离额外约束；此接口不宣称高频伺服。
`forward_position` 是显式可选 SDK 接口，需按固件验证后使用。底盘 250ms 主机命令超时会请求零速度，
不是经过验证的控制器看门狗。

## 安装与 ROS 2

在 Linux ROS 2 工作区安装 grpcio>=1.73.1、protobuf>=6.31,<7（生成代码的实际要求，
不能只遵循 SDK 中较低的 grpc 声明），再 `colcon build --packages-up-to morla_mmk2_driver`，source install/setup.bash。
IK 需要 scipy、mujoco>=3.2,<4 与 mmk2_description；`pip install -e '本包路径[ik]'` 可用于本地软件验证。

先通过公共编译器/Web 导出 resolved JSON，然后：

```bash
ros2 launch morla_mmk2_driver mmk2_driver.launch.py resolved_path:=/absolute/resolved.json
mmk2_check --resolved /absolute/resolved.json --duration 2
```

`mmk2_check` 强制只读且不发送目标。ROS 复用公共 CommandEnvelope/authority/RuntimeOperation
入口；双臂、头部、升降发布 `~/<instance>/joint_states`；底盘发布
`~/<instance>/velocity` (TwistStamped) 和 `~/<instance>/odometry`，不会把速度放进 JointState.position。
夹爪原始量保留在公共 `~/state`/ComponentHealth 及 Web，避免冒充 JointState 的弧度/米。
SDK 相机缓存经公共网关发布 `~/<instance>/rgb/compressed` (JPEG) 与
`~/<instance>/depth/compressed` (PNG16 原始深度，不是假定米制的深度图)。没有相机内参，未伪造 CameraInfo。
SDK 没有协方差，ROS 输出使用保守未知方差。裸 driver 不自动执行 Quest bindings；完整遥操由公共 Web/ControlHost 宿主管理。
**Web 和独立 driver 不要同时占用同一设备**，Deployment 使用真实序列号 resource_key。

## Web 逐步配置

1. 构建并加载本包后打开组合工作区，从预设选择 “AIRBOT MMK2 SDK 整机 / Quest（协议测试）”。组件、字段、控制器和预设均来自已安装 descriptor，无需前端品牌代码。
2. 先保持全部 fake，编译、启动、准备、激活、获取控制权；确认七组反馈及三路合成相机，录制后检查 19 维状态/动作和真实 sample_ns。fake 不是 MuJoCo 或实机测试。
3. 另建真实装配，选 “AIRBOT MMK2 SDK 整机（真实设备，默认只读）”。所有 SDK 组件使用同一 mmk2_sdk session_id。在部署中逐个填同一真实 device_id/resource_key/host/port/robot_name/domain_id。模板 deployment.real.example.yaml 展示结构；控制器地址已填 `172.25.14.1`，必须替换序列号占位符；本轮不连接该地址。
4. 保持三项写入开关 false，仅编译、启动，观察具名反馈/时间戳/健康与相机。缺任何执行器反馈不允许准备/激活；需要子集时从装配移除相应组件并调整 capture。
5. 完成现场核验后再配置 Quest：真实输入源双手、接收协议/端口/设备身份；不要同时运行测试数据注入器。quest.real.example.yaml 是完整双手绑定示例。
6. 双手位姿 → 各臂公共 Cartesian IK；双手 scalar → 各 SDK 夹爪；左 scalar 同时 → 升降（`morla.scalar_anchored`，按下 Grip 时锁存当前高度，只下发之后行程；绝对映射会让躯干在 Grip 生效那一拍瞬跳；`output_range` 必须显式填写为扳机满行程的位移，缺失时编译报 `ANCHORED_OUTPUT_RANGE`，不要依赖默认 `[0,1]`——那是 1 m 行程，`slide_joint` 正值朝下，会直接到底）；左摇杆 → 头；右摇杆 → 原生底盘 Twist。每条绑定都有 hold deadman 和 250ms 输入新鲜度。升降与夹爪共用左 scalar 是示例，可在 Web 改绑定。
7. 现场测量双臂输入坐标基、TCP 和方向；模板中的变换矩阵仅沿用模型参考，calibration_verified=false。夹爪 output_range=[0,1] 仅表示原始映射，确认极性后可反向。实机运动前，记录公共标定/停止 verified，并显式开启每个执行器组件的三项写入门禁。先单组低速，再整机。
8. 通过 Web 准备、激活、获取控制权、按住 deadman，松开后检查保持/零速度及诊断；再验证停止、故障、断网、物理急停和长稳。采集只选 SDK 实际提供的端口。结束采集先点“暂停遥操”，再点“结束并保存”，最后结束工作站。

本机浏览器实测：连续控制后直接结束录制，文件虽完整，但会话随后撤销了驱动授权并报故障；
原始回执为 ADAPTER_DISARMED，未能保留更早的驱动原因，故不能确认根因。已补充 device_fault 原因日志和故障回执；
“暂停→结束并保存→结束工作站”复验通过。同步收尾期间的调度时延仍需 Linux 现场测量，未放宽反馈期限。

相机资源是否需要厂商 enable_resources 参数尚无现场服务证据；本包不猜测配置。
若 get_image_once 返回空或错误，会报告 camera fault，先按厂商流程启用相机。

本机 macOS 无 ROS 2/rclpy、未连接真实 MMK2 或 Quest。Linux DDS、控制器版本、真实使能/停止、
时延与物理映射属于未完成现场验收项，不能以本地 mock 协议测试代替。
