# 独立 LoopHand / Aero Hand Open ROS 2 / MORLA 驱动

包：`morla_loophand_driver`；组件：`morla.loophand.compact@1.0.0`。本包独立安装、注册、启动和测试，不依赖 CR5 或任何机械臂驱动。仅包含 Aero 手串口协议、耦合模型、手部 Adapter、descriptor、部署配置与测试。上游来源、SHA-256 及 Apache-2.0 许可见 PROVENANCE.json / LICENSE。

## 手型与反馈

左手为 7 维紧凑手型：thumb_cmc_abd、thumb_cmc_flex、thumb_flex、index_flex、middle_flex、ring_flex、pinky_flex。输入为 rad，经官方模型展开为 16 个运动学关节并换算成 7 个执行器；这不是 16 个独立主动关节。

串口使用 921600 和 `<2B7H`；要求稳定 `/dev/serial/by-id/...`。GET_POS 读取执行器编码器，反馈经模型换算，标 source=estimated、measured=false；JointState 没有来源字段，使用时同时查看 state/health。启动不自动 homing、使能或改变力矩。

**真实手当前只读，运动激活阻止。** 上游零力矩 CTRL_TOR 没有执行回执，也未提供可验证的通信 watchdog。stop 发请求后明确报告 STOP_UNCONFIRMED；设置 deployment verified=true 也无法绕过激活门禁。需补固件停止/禁能状态、watchdog 协议和实机证据后再开放 real 运动。fake 可验证全部组件控制协议。

## 独立构建、fake 启动

在 ROS Humble 目标主机，从 MORLA 根目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to morla_loophand_driver
source install/setup.bash
python3 -m morla_core.composition.cli compile \
  src/robots/loophand/morla_loophand_driver/config/assembly.fake.yaml \
  --output /tmp/loophand.resolved.json
ros2 launch morla_loophand_driver loophand_driver.launch.py \
  resolved_path:=/tmp/loophand.resolved.json
```

也可 `ros2 run morla_loophand_driver loophand_driver --ros-args -p resolved_path:=/tmp/loophand.resolved.json`。单设备入口只接受一只此类型手；其他组件通过通用组合 runtime 调度。fake state/action 均为 7 维。

默认节点 `/loophand_driver`；`~/hand/joint_states` 输出 rad，`~/state` / `~/health` 报告来源与有效性；`~/commands` / `~/receipts` 接收 MORLA CommandEnvelope 并反馈 CommandReceipt；`~/operation` / `~/authority` 为公共操作及租约服务。没有直接写关节的旁路话题。

本包独立的 `config/deployment.real.example.yaml` 只含手设备身份、串口及待验收标志。触觉总线、导纳抓取、Quest/手套重定向、手 URDF 和组合仿真尚未迁移，不要使用 XHand/Wuji 专用重定向冒充兼容。

## 验证与组合

```bash
PYTHONPATH=src/common/morla_core:src/common/morla_runtime:src/robots/loophand/morla_loophand_driver \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/robots/loophand/morla_loophand_driver/test
```

测试无需安装任何机械臂包。与 CR5、Nero 或其他臂的组合仅在 Assembly/Deployment 中引用手组件。示例见 [examples/cr5_loophand](../../../../examples/cr5_loophand/README.md)，接入详情见 [LoopHand 记录](../../../../docs/integrations/loophand.md)。当前 ROS Humble 构建与实体手验收均 NOT_RUN。
