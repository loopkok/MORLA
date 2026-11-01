# 独立 CR5 ROS 2 / MORLA 驱动

> 2026-10-30 统一路径更新：CR5 保持独立 adapter/SDK/URDF IK，通过 morla.adapters 与 morla.kinematics 注册；VR/Vader 兼容整链由公共 session 托管输入，不再重复起接收节点。 当前接口、安装与测试见[设备指南](../../../../docs/device_integration_guide.md)和[实现记录](../../../../docs/architecture_refactor/UNIFIED_IMPLEMENTATION.md)。下文独立驱动入口仍可单用；不能与另一个会话同时占用设备。

包：`morla_cr5_driver`；组件：`morla.dobot.cr5@1.0.0`。本包独立安装、注册、启动和测试，不依赖任何手部驱动，也不依赖 pyserial。来源是桌面 cr5_loophand_graspnet 的 CR5 协议；Dobot 二进制反馈布局来源、摘要和 MIT 许可见 PROVENANCE.json / LICENSE-DOBOT。

## 接口与实现

6 维 canonical 关节 joint1..joint6，位置单位 rad。29999 dashboard / 30003 ServoJ / 30004 二进制反馈分别连接；反馈校验 1440 字节包、魔数和设备时钟，q_actual 从 degree 转 rad。使能/停止等待新的 enable_status 确认。具备有界 I/O、latest target，以及 6 关节部署限位校验（求解器/URDF 交集）。重构后主机命令 watchdog、目标步长和部署验证门禁已移除。启动不使能、不回零、不自动清错或继续历史动作。

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

也可 `ros2 run morla_cr5_driver cr5_driver --ros-args -p resolved_path:=/tmp/cr5.resolved.json`。单设备入口不再强制“只含一台 CR5”；多组件快照交给通用组合 runtime 仍更合适。fake state/action 均 6 维。

默认节点 `/cr5_driver`，私有接口：

- `~/arm/joint_states`：sensor_msgs/JointState，rad；`~/state` / `~/health` 同时提供真实来源、有效性、sample_ns 和操作状态。
- `~/commands` / `~/receipts`：CommandEnvelope / CommandReceipt；字段仍含 session/token/epoch/sequence/TTL，但当前runtime不校验授权、序号或时效，只检查急停锁存、组件子集和适配器接受。不提供绕过网关的公开 JointState 命令入口。
- `~/operation` / `~/authority`：RuntimeOperation / ControlAuthority；显式 prepare、activate、stop 和 acquire/renew/revoke。异步操作按 request_id 在 state 中跟踪，接纳不等于物理完成。

## real 部署

配置模板为本包 `config/deployment.real.example.yaml`。填写实际设备身份、IP 与 6 个 joint_lower/joint_upper（rad），然后使用 compile 的 `--deployment` 参数编译。`verified/stop_verified` 现在只是记录字段，不参与编译或运行门禁；descriptor 的 ±2π 仅是外层保护范围，不是认证物理限位。

真实 CR5、固件兼容、设备停止/断网 watchdog、旧动作缓存、零位/方向、安装 TCP 和长稳均未验收。主机命令超时/watchdog已移除，不能替代设备 watchdog；断链或上游停发命令时执行器保持最后目标。现已提供基于纯 CR5 URDF / Pinocchio 的本地数值 IK 和右侧 VR controller；不使用 Nero/Astral 参数或控制器内置 IK。动力学仿真尚未接入。

## 独立验证与组合

本包测试仅依赖自己的 Python 包及 MORLA 公共层：

```bash
PYTHONPATH=src/common/morla_core:src/common/morla_runtime:src/robots/dobot/morla_cr5_driver \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/robots/dobot/morla_cr5_driver/test/test_cr5_driver.py
```

与其他末端的组合只在 Assembly/Deployment 中表达。双设备示例见 [examples/cr5_loophand](../../../../examples/cr5_loophand/README.md)，不要求安装额外的 CR5+手联合驱动包。详见 [CR5 接入记录](../../../../docs/integrations/cr5.md)。

## 右手柄 / 右手追踪 VR 遥操（2026-10-29）

已补齐来源明确的 Quest 输入、右握把/右手捏合手势、相对建锚、姿态滤波、异步本地 URDF / Pinocchio FK/IK、统一 runtime 输出和本机控制页。手势仍作为输入意图记录，但runtime不再强制deadman或输入时效；来源切换/断流后需重新激活建锚。fake 与 real 共用实际 CR5 几何模型和 IK，fake 反馈仍为合成数据；真实跟随尚需现场验收。

在已构建本包与公共 morla_web_manager 的 Humble 环境，配置 ADB reverse TCP 8000 后运行：

```bash
ros2 launch morla_cr5_driver cr5_vr.launch.py
```

默认 fake；浏览器打开 http://127.0.0.1:8080/。真实参数、只读本地正逆解检查、控制手势和完整步骤见 [CR5 VR 操作指南](../../../../docs/integrations/cr5_vr.md)。本包新增 controller.json 与 morla.controllers 注册，需重新构建/source；旧冻结快照重新编译。单独关节驱动不需要启动 Web 宿主。

VR controller/输入及可选控制页的整链回归通过仓库 `scripts/test_composable.sh --layer pure` 执行，需要公共输入/遥操/学习/Web 源码及测试依赖。未安装可选 Web 模块时，对应整链/页面测试明确跳过；不能据此宣称该层已通过。

## CR5 MuJoCo 仿真与 Web

`sim` 复用 MORLA 统一的 MuJoCo 物理场景、状态缓存、BindingEngine 和 ControlHost；不连接 CR5 TCP 控制器。MJCF 由版本化的 CR5 IK URDF 生成，保留 joint1..joint6、Link6 末端、质量和惯量；生成脚本为 `robot_description/scripts/make_cr5_mjcf.py`。仿真控制器采用参考位置伺服，并非实机伺服参数。当前未启用机器人碰撞，不用于碰撞验收。

在 ROS Humble 主机安装 `mujoco`、`Pillow`、Pinocchio 和 Web 依赖后，构建并 source 工作区（包括 morla_cr5_driver、robot_description、morla_web_manager 及其 MORLA 依赖）：

```bash
ros2 launch morla_cr5_driver cr5_sim.launch.py
# 使用实体 Quest 输入，CR5 仍然是 MuJoCo 后端：
ros2 launch morla_cr5_driver cr5_sim.launch.py input_backend:=real
```

打开 `http://127.0.0.1:8080` →「组合配置」。页面自动连接启动的会话，展示 scene_camera 仿真画面和组件健康；点击「激活运动许可」后才能发送目标。也可在现有统一 Web 服务的「组合配置」中点击「加载 CR5 MuJoCo 仿真」→ 预览 → 保存 → 启动会话 → 连接输入设备 → 激活。模板默认 Quest 为 fake；使用实体手柄时，将 quest 的后端改为 real，再保存启动。`input_backend:=real` 托管输入接收进程，头显连接要求与现有 Quest 接入一致。

配置文件 `config/quest_right.sim.yaml` 设置六轴初始位置 `[0,-0.5,1,-0.5,1,0]` rad，避开全零伸直位；仿真相机位于世界坐标。arm 必须保留 sim，输入可以独立选 fake/real。局域网访问请使用已有统一 Web 服务的 HOST 配置；session launch 的默认地址仅限本机。不要与已运行的 Web 服务抢占同一端口。

仿真和真机遥操使用相同的本地 Pinocchio IK、TCP/增量映射和异步解算逻辑。仿真将物理关节反馈作为播种来源，状态标记 `source=simulation`、`synthetic=true`、`measured=false`。本次增加仿真调试环境，尚未确认或修复实体手柄测试中的 IK 故障根因。
