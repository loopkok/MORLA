# FAIRINO FR3 ROS 2 / MORLA 驱动

组件 `morla.fairino.fr3@1.0.0`，包 `morla_fr3_driver`，对应**标准法奥 FR3 六轴机械臂**。
不是 Franka FR3，也不包含 FR3-WMS/WML/C/MT 的几何参数。

复用 MORLA 的组合编译、Runtime、ROS 网关、Web、输入绑定、数采与控制权流程。
厂商 SDK 以原样源码随包安装，摘要及许可证见 `PROVENANCE.json` / `LICENSE-FAIRINO`。
不需要 CR5、Nero、手部 SDK，也不导入桌面路径。fake/目录发现不加载厂商 SDK。

## 构建与独立 ROS 2 运行

Ubuntu 22.04 / ROS 2 Humble 主机，使用系统 Python（不能使用与 rclpy 不同 ABI 的 conda Python）：

```bash
source /opt/ros/humble/setup.bash
# Cartesian IK 需要 Pinocchio；ROS apt 安装（也可在兼容环境装 Python extra [ik]）。
sudo apt install ros-humble-pinocchio
# 真实驱动需要；SDK 即使不用 TLS 也会导入此依赖。
python3 -m pip install 'python-mbedtls>=2.9,<3'
colcon build --symlink-install --packages-up-to morla_fr3_driver morla_web_manager
source install/setup.bash
python3 -m morla_core.composition.cli compile \
  src/robots/fairino/morla_fr3_driver/config/assembly.fake.yaml \
  --output /tmp/fr3.resolved.json
ros2 launch morla_fr3_driver fr3_driver.launch.py resolved_path:=/tmp/fr3.resolved.json
```

默认 `/fr3_driver/arm/joint_states` 是六轴 `sensor_msgs/JointState`；`~/state`、`~/health`、
`~/commands`、`~/receipts`、`~/operation`、`~/authority` 复用公共网关。
只接收 `CommandEnvelope`，不新增绕过 Runtime 的 JointState 写入入口。
prepare 只检查反馈；显式 activate 才 `RobotEnable(1)`，确认新反馈中的使能位后进入 ServoMoveStart。
启动不回零、不清错、不切手自动模式、不执行历史目标。独立 ROS 节点和 Web 会话不能同时占用同一设备。

## 接入现有 Web

构建新版前端：`cd src/common/morla_web_manager/web && npm ci && npm run build`。
启动现有 Web 服务，或 `ros2 run morla_web_manager morla-composable-web`（以该包已安装入口为准）。
也可 `ros2 launch morla_fr3_driver fr3_web.launch.py` 直接启动默认 fake 工作站（默认端口 8080）。
在「机器人工作站」的设备预设选择 **FAIRINO FR3 / Quest（测试数据）**，载入、检查、保存、启动。
也可以直接从执行器目录添加 **FAIRINO FR3**。默认 fake，不连接硬件。

支持六轴状态、准备/激活/暂停/停止、公共 Quest 位姿控制、关节点动映射、采集/回放的六维契约。
Quest fake 只是输入测试源，必须注入输入才能跟随；真实 Quest 使用 `quest.real.example.yaml` 中的托管接收设置。
机械臂可 fake、Quest 可 real，分别配置。模板按住 right_hold 才输出，输入过期由公共控制层处理。
FR3 fake 是合成反馈，不是 MuJoCo；当前 `sim` 不可用，无碰撞和动力学验证。

真实部署时复制 `deployment.real.example.yaml`，填写序列号、唯一资源键和控制器 IPv4，
再 compile 加 `--deployment /path/to/deployment.yaml`；Web 中对应为“真实设备”及连接设置。
安装位置的输入基变换、法兰到 TCP、软限位必须按实际填写，示例只代表假定的操作员朝向。
`verified` / `kinematics_verified` 沿用现框架记录语义，不是自动验收或强制门禁。

## SDK、时间与停止语义

- 固定使用所提供 SDK 的 XML-RPC 20003、CNDE 20005；SDK 构造还启动 UDP 20007 接收线程。
  此版本新增 CNDE/TLS 协商；旧 20004 固件不能自动回退，本实现未启用 TLS。
- 每个设备独占一个 spawn 子进程，避免 SDK 类级全局状态污染其他机械臂；进程内只有一个 SDK 命令调用者。
  `rpc_timeout_s`、`startup_timeout_s`、`operation_timeout_s` 分别约束普通调用、构造和使能/停止。
  超时终止 SDK 进程并报告故障，不自动重连、不重放迟到目标。**进程退出不等于物理停止**。
- CNDE 完整帧复制后才记录主机 `monotonic_ns`。短帧、解码异常、NaN、重复帧、过期反馈均不能刷新有效性；
  静止关节值相同但新帧计数仍有效。状态保留错误码、使能、急停、保护停止、碰撞和帧序号。
- canonical `joint1..joint6`，位置 rad；SDK J1..J6 度，`q_sdk = q_rad * 180/pi`，无额外符号或零点偏移。
  实机零位/方向尚待确认。手册 FR3 软限位（度）为 `[-175,-265,-150,-265,-175,-175]` 到 `[175,85,150,85,175,175]`。
  部署只能收紧。最大速度默认 0.3 rad/s，目标与实测最大领先默认 0.1 rad；拒绝大幅绝对目标，使用增量目标。
- 伺服频率默认 125 Hz，`ServoJ.cmdT=1/rate_hz`，扩展轴固定四个 0（未接扩展轴）。
  单槽 latest target、按发送周期限速，迟到不补发历史队列。Python/网络不保证硬实时，实际频率需在目标机测量。
- 暂停保持最后**已发送**目标，不把尚未执行的目标作为保持点；恢复须显式激活。
  stop 依次尝试 `StopMotion`、`ServoMoveEnd`、`RobotEnable(0)`，再等新的失能反馈。
  所有失败可见；通信断开时物理停止标为未确认。关闭软暂停的会话也会清理伺服。
  从未激活的只读会话关闭时只断开通信，不发运动/失能指令。
- 沿用项目现状：没有基于上游命令 TTL 的统一 watchdog，停发目标时持续保持最后目标。
  设备自身断网停止机制仍需现场验收，不能用主机超时替代。

## 运动学与只读检查

本地 FK/IK 使用独立 `fr3_description/urdf/fairino3_v6.urdf`，末端为 `flange`，米/弧度；运行时不再使用旧 DH 求解器。
完整沿用 CR5 的 Pinocchio SE(3) DLS、微分目标吸收、自适应阻尼、指令积分种子和异步 worker 策略，使用 FR3 自身模型与限位。
`morla.fr3.cartesian_delta` / `morla.fr3.cartesian_jog` 默认 `ik_mode: differential`，可切换 `strict`；旧通用绑定通过 kinematics entry point 使用严格 URDF IK。
FR3 控制输出额外受 adapter 的速度/领先上限约束（默认 0.3 rad/s、0.1 rad，模板领先 0.08 rad）。
模型法兰补充、原模型精度、映射/限位、旧配置迁移和验证见 [URDF/IK 移植记录](../../../../docs/integrations/FR3_URDF_IK_MIGRATION.md)。
首次真机先运行只读检查，比较本地 FK 与 SDK GetForwardKin：

```bash
ros2 run morla_fr3_driver fr3_check --host ACTUAL_CONTROLLER_IP --seconds 5 --output /tmp/fr3-readonly.json
```

该命令没有使能、运动、模式切换和自动清错。PASS 只表示收到变化帧序号且本次关节位形的 FK 一致，
不代表各轴方向、整个工作空间、TCP 或停止已验收。需多组实测位形重复，再人工逐轴低速验证。

测试入口与本轮证据见 [FR3 接入记录](../../../../docs/integrations/fr3.md)。
