# cr5_mujoco_sim

CR5 独立仿真包，复用 MORLA 公共 MuJoCo 后端、CR5 本地 IK 和统一 Web。

```bash
ros2 launch cr5_mujoco_sim cr5_mujoco_sim.launch.py
ros2 launch cr5_mujoco_sim cr5_mujoco_sim.launch.py sim_mode:=physics sim_display:=both
```

默认 `kinematic/web`。`sim_mode` 支持 `kinematic/physics`，`sim_display` 支持 `web/local/both`；实体输入可设 `input_backend:=real`。打开 Web 的「组合配置」可在启动后调整选项。切模式前先撤销运动许可；本地窗口在服务主机打开。

本包不连接实体 CR5，不复制仿真循环。模型来自 robot_description 的 CR5 URDF/MJCF；物理模式为参考位置伺服动力学，尚不代表完成 sim2real 标定或碰撞验收。

完整参数、Nero/Astral 入口及验证边界见 [公共仿真说明](../../../../docs/integrations/SHARED_MUJOCO_2026-11-02.md)。
