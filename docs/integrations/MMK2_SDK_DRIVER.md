# MMK2 真实 SDK → MORLA / ROS 2 / Web

2026-11-10。本轮基于用户提供的 `mmk2_sdk_src`，参考设备接入总指南、FR3 移植教程与 CR5/FR3 的 Adapter、SDK 边界、ROS 网关、微分 IK、descriptor/Web 接入经验。用户确认 SDK 没有实现使能/停止接口，控制器 IP 为 **172.25.14.1**，实体测试由用户后续完成；本轮没有连接该地址。

## 实现

新包 `src/robots/airbot/morla_mmk2_driver` 提供独立 ament_python/ROS 2 入口、launch、静态描述、fake/real 工厂、只读检查 CLI、真实部署与双手 Quest 模板。完整安装与 Web 操作步骤见[包说明](../../src/robots/airbot/morla_mmk2_driver/README.md)。Web 从已安装 descriptor 发现，不增加前后端品牌分支。

- SDK 七组执行器为双臂、双夹爪、升降、头和原生底盘速度，状态/动作各 19 维。三路 RGB-D 相机使用独立捕获线程与最新缓存。
- 新前缀 `morla.airbot.mmk2_sdk.*` 与现有 `morla.airbot.mmk2.*` 仿真分开：底盘是 m/s、rad/s 的 Twist3D；夹爪是 SDK 原始量 0..1，未假设闭合极性；不能把仿真轮速/米制夹爪直接送入实机。
- 同一整机 session 共用 gRPC 连接、具名原始反馈、目标槽位和停止域。缺失反馈不补零；重复时间戳不刷新新鲜度；错误码、非有限值、倒退设备时钟、过期反馈与拒绝目标均报告故障。RPC 有截止时间，关闭取消状态流并有界等待。
- 每帧受限微分 IK 采用共享 URDF/MJCF 几何，阻尼、目标领先吸收、步长和限位限制参考 FR3/CR5；零位附近不再用完整位姿求解导致的大腕部跳变。不是精确位姿收敛或碰撞验收。
- 默认只读；实机目标写入必须显式配置 allow_motion、mapping_verified、hold_stop_accepted。公共 verified/stop_verified 字段仅作现场记录。软件 activate 不冒充电机使能，未发明不存在的 SDK 急停/失能 RPC。
- SDK 的 stop_all 是无操作日志。本包请求保持最后发送的位置目标/底盘零速度，清除待发目标，不自动重连/重放；明确不保证实际物理停止或失能。
- 原始 SDK 高层对象有无限等待、缺关节补零和全局事件循环；本包保留未修改的厂商 protobuf/gRPC 生成文件，替换这些高层行为。依赖按生成代码要求 grpcio>=1.73.1、protobuf>=6.31,<7；许可和源版本差异保留在 PROVENANCE.json。

## 公共层

编译器和 BindingEngine 识别 `m_rad/s` 的原生底盘速度语义，目标单位来自目标端口。ROS 网关仅将 rad/m 的具名位置发布成 JointState；底盘输出 TwistStamped/Odometry；SDK 原始夹爪量保留在公共状态/健康与 Web，不冒充 SI 关节位置。缓存图像输出 CompressedImage，深度 PNG16 保留 SDK 原始单位，没有伪造深度尺度、相机内参或协方差实测值。

ROS 入口仍使用公共 CommandEnvelope、authority、RuntimeOperation。Web/ControlHost 才托管输入映射，裸 ROS driver 不自动运行 Quest 接收与 bindings；两条路径不可同时占用同一台机器人。

## 本地证据

- 驱动、公共 core/runtime/teleop 与 FR3/CR5 联合回归 **309 passed、1 skipped**，跳过的是可选 Hypothesis。另有 Web/采集/策略回归 **37 passed**，原 MMK2 MuJoCo 回归 **21 passed**。共 367 个通过案例，驱动测试 18 项包含在联合回归内。
- 驱动测试启动真实本地 gRPC server，使用厂商生成的协议序列化，并验证只读不写、目标格式、拒绝、RPC 截止时间、重复/损坏反馈、停止不重放、共享连接与相机关闭。它不是实体控制器测试。
- 从正常安装的 site-packages 加载真实 entry points、ament 静态资源与模板，确认 11 实例、单一 SDK 会话、19/19 维、READY_DISARMED→STOPPED；没有注入 Adapter 来替代安装验证。
- 实际浏览器载入预设、检查/保存、启动、七组 Quest 合成输入、三相机预览、两段录制、暂停、结束保存、停止释放。两段 meta 均 complete，七组状态/动作均有变化，JPEG 可解码，源时间戳递增。数据来自 fake SDK，不是 MuJoCo、真实 Quest 或实机。
- 首次完整位姿 IK 在零位产生腕部跳变并被领先距离检查拒绝，已改微分 IK并增加限步回归。另一段连续采集在直接保存后出现 ADAPTER_DISARMED，会话收尾缺少更早的内部原因；该现象保留为现场调度验证项，未虚构已定位根因。已补 device_fault 原始原因日志和故障回执。“暂停遥操→结束并保存→结束工作站”的复验正常；未放宽反馈超时。

测试输出、安装证据、录制逐流核验与画面见[证据目录](evidence/2026-11-10-mmk2-sdk/README.md)。完整数据和运行日志位于本机 `/tmp/morla-mmk2-sdk-web-20261110`，摘要已入库。

## 现场未验收

本机 macOS 无 rclpy/ROS 2，未运行 Linux ROS launch/DDS；包含 ROS 接收器的 test_teleop_events.py 无法在本机收集，联合回归明确选择可运行的公共层和设备包，没有通过打桩宣称 ROS 测试通过。未连接 MMK2/Quest。固件版本、SDK 原生/URDF 逐轴方向、头部轴顺序、夹爪极性、深度单位、相机启用、实际使能/急停、断网停止及长稳由现场验证。

## 复现

完成本包安装并 source ROS/ament 环境后：

```bash
MORLA_PYTHON=python3 bash scripts/test_mmk2_driver.sh
python3 scripts/run_mmk2_web_acceptance.py --target sdk-fake --port 8081 --store /tmp/mmk2-sdk-web
python3 scripts/check_mmk2_sdk_recordings.py --store /tmp/mmk2-sdk-web --output /tmp/sdk-audit.json
```

测试宿主拒绝向任何真实 SDK 后端注入 Quest 合成数据，不自动准备/激活。真实只读模板已填控制器 IP，但设备序列号和资源键仍须用户填写。运动写入开关默认 false。
