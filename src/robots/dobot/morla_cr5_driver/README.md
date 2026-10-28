# 独立 CR5 ROS 2 / MORLA 驱动

包：`morla_cr5_driver`；组件：`morla.dobot.cr5@1.0.0`。本包独立安装、注册、启动和测试，不依赖任何手部驱动，也不依赖 pyserial。来源是桌面 cr5_loophand_graspnet 的 CR5 协议；Dobot 二进制反馈布局来源、摘要和 MIT 许可见 PROVENANCE.json / LICENSE-DOBOT。

## 接口与实现

6 维 canonical 关节 joint1..joint6，位置单位 rad。29999 dashboard / 30003 ServoJ / 30004 二进制反馈分别连接；反馈校验 1440 字节包、魔数和设备时钟，q_actual 从 degree 转 rad。使能/停止等待新的 enable_status 确认。具备有界 I/O、latest target、命令 watchdog、实测位置步长和部署限位保护。启动不使能、不回零、不自动清错或继续历史动作。

通用 worker 和 ROS 网关复用 morla_runtime，无品牌分支。此包只提供自己的 transport、CR5Adapter、descriptor、部署参数、许可证及测试。MORLA 按已安装 morla_components 发现 descriptor，按 morla.adapters 选择本包 factory。

## 构建、fake 启动

在 ROS Humble 目标主机，从 MORLA 根目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to morla_cr5_driver
source install/setup.bash
python3 -m morla_core.composition.cli compile \
  src/robots/dobot/morla_cr5_driver/config/assembly.fake.yaml \
  --output /tmp/cr5.resolved.json
ros2 launch morla_cr5_driver cr5_driver.launch.py resolved_path:=/tmp/cr5.resolved.json
```

也可 `ros2 run morla_cr5_driver cr5_driver --ros-args -p resolved_path:=/tmp/cr5.resolved.json`。单设备入口只接受一台 CR5 的装配快照，其他组件请交给通用组合 runtime。fake state/action 均 6 维。

默认节点 `/cr5_driver`，私有接口：

- `~/arm/joint_states`：sensor_msgs/JointState，rad；`~/state` / `~/health` 同时提供真实来源、有效性、sample_ns 和操作状态。
- `~/commands` / `~/receipts`：CommandEnvelope / CommandReceipt；需要当前 session/token/epoch/sequence/TTL，不提供绕过网关的公开 JointState 命令入口。
- `~/operation` / `~/authority`：RuntimeOperation / ControlAuthority；显式 prepare、activate、stop 和 acquire/renew/revoke。异步操作按 request_id 在 state 中跟踪，接纳不等于物理完成。

## real 部署

配置模板为本包 `config/deployment.real.example.yaml`。填写实际设备身份、IP 与 6 个 joint_lower/joint_upper（rad），然后使用 compile 的 `--deployment` 参数编译。模板 verified/stop_verified=false 保留至现场证据齐全；descriptor 的 ±2π 仅是外层保护范围，不是认证物理限位。

真实 CR5、固件兼容、设备停止/断网 watchdog、旧动作缓存、零位/方向、安装 TCP 和长稳均未验收。主机超时保护不能替代设备 watchdog。CR5 URDF/IK 和笛卡尔 controller 尚未接入，可先用具名关节目标或 joint jog；不应选 Nero/Astral 专用 IK。

## 独立验证与组合

本包测试仅依赖自己的 Python 包及 MORLA 公共层：

```bash
PYTHONPATH=src/common/morla_core:src/common/morla_runtime:src/robots/dobot/morla_cr5_driver \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/robots/dobot/morla_cr5_driver/test
```

与其他末端的组合只在 Assembly/Deployment 中表达。双设备示例见 [examples/cr5_loophand](../../../../examples/cr5_loophand/README.md)，不要求安装额外的 CR5+手联合驱动包。详见 [CR5 接入记录](../../../../docs/integrations/cr5.md)。
