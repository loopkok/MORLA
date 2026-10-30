# CR5 + LoopHand 装配示例（仅配置）

> 当前控制宿主、进程托管与测试见[统一示例](../unified_composition/README.md)。本例只是独立插件的装配；真实 LoopHand 只读限制按驱动文档保持。

此目录不提供 ROS 包、设备协议或联合设备驱动。它只通过 Assembly 引用两个独立组件：

- `morla.dobot.cr5` 来自 [morla_cr5_driver](../../src/robots/dobot/morla_cr5_driver/README.md)。
- `morla.loophand.compact` 来自 [morla_loophand_driver](../../src/robots/loophand/morla_loophand_driver/README.md)。

两包互不依赖，可以分别构建和用于其他装配。组合场景按实际需要安装两包，由一个公共 RuntimeSession 加载所选组件，管理锁、控制权、生命周期和采集契约。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to morla_cr5_driver morla_loophand_driver
source install/setup.bash
python3 -m morla_core.composition.cli compile examples/cr5_loophand/assembly.fake.yaml \
  --output /tmp/cr5_loophand.resolved.json
ros2 run morla_runtime joint_state_runtime_node --ros-args \
  -p resolved_path:=/tmp/cr5_loophand.resolved.json
```

组合 fake state/action=13 维，默认输出 `/morla_runtime/arm/joint_states` 和 `/morla_runtime/hand/joint_states`，其余接口仍为通用 runtime 的 commands/receipts/health/state/operation/authority。也可交给组合 Web 管理，无需另建品牌专用的组合节点。Web 与 ROS runtime 不应对同一装配同时启动两份会话。

`deployment.real.example.yaml` 仅为双设备部署模板，不是已通过现场验收的配置。真实手目前只读，整组 real 激活会受该门禁阻止。现场标定、连接器和 TCP 未填写，不能把本例当作已标定的物理组合。详见 [组合与验收记录](../../docs/integrations/cr5_loophand.md)。
