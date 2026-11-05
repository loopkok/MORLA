# fr3_mujoco_sim

FAIRINO FR3（fairino3_v6）独立 MuJoCo 仿真包。沿用 CR5 的组织方式：本包拥有标准 ROS 2 包、launch、CLI 和默认装配，实际场景、步进、相机、显示与控制权均复用公共 MORLA Runtime，不另建仿真循环。

## 构建与运行

Ubuntu ROS 2 Humble，使用与 rclpy 兼容的系统 Python；先安装项目 Web/IK 依赖与 `mujoco>=3.2,<4`、Pillow。

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install 'mujoco>=3.2,<4' Pillow
colcon build --symlink-install --packages-up-to fr3_mujoco_sim
source install/setup.bash
ros2 launch fr3_mujoco_sim fr3_mujoco_sim.launch.py
```

默认 `kinematic/web`，Quest 输入为 fake，不连接 FR3 SDK、不使能真机。打开 `http://127.0.0.1:8080/`，选择已保存的 `fr3_quest_sim` 工作站查看运行；也可在普通 Web 的设备预设中选择 **FAIRINO FR3 / Quest（MuJoCo 仿真）**，检查、保存并启动。

```bash
# 参考位置伺服动力学，网页和服务主机本地窗口
ros2 launch fr3_mujoco_sim fr3_mujoco_sim.launch.py sim_mode:=physics sim_display:=both
# 真 Quest + 仿真机械臂；需确认 Quest 接收配置、输入坐标与 TCP
ros2 launch fr3_mujoco_sim fr3_mujoco_sim.launch.py input_backend:=real
# 独立 CLI / 驱动包兼容入口，同一个公共后端
ros2 run fr3_mujoco_sim fr3_mujoco_sim_node --sim-mode physics --sim-display web
ros2 launch morla_fr3_driver fr3_sim.launch.py
```

共用参数：`assembly`、`deployment`、`input_backend`、`sim_mode:=kinematic|physics`、`sim_display:=web|local|both`、`port`、`store`、`log_dir`。不要同时启动占用同一 Web 端口的入口。无显示服务器的 Linux 可用 `MUJOCO_GL=egl` 做网页离屏渲染；local/both 仍需可用图形桌面。macOS 本地窗口需要同环境的 mjpython。

需要单臂 ROS JointState 网关时，用驱动包 `config/assembly.sim.yaml` 编译 resolved，再运行 `fr3_driver.launch.py`，不要同时让另一会话拥有同一仿真设备。

## 行为与边界

- 资产来自独立 `fr3_description` 的同一份 IK URDF，MJCF 的 flange 与 Pinocchio 对照；七个 STL 仅用于显示。仅包含 fairino3_v6。
- `kinematic` 直接更新关节并 forward；`physics` 由位置伺服 ctrl、重力和 mj_step 积分产生反馈。参考增益沿用 CR5 的软件基线，未做 FR3 实机辨识。
- 模板保留右手 grip deadman、0.2 rad/s 速度、0.08 rad 指令领先、0.25 位移倍率；默认微分 IK，可切严格模式。fake Quest 不会自行生成手部动作；无新鲜输入不能开始遥操。
- 初始位置只在仿真初始化使用，不会触发真机 HOME。状态标记 simulation/synthetic、measured=false；相机为同一场景生成的 JPEG。
- 当前仿真使用原生轴映射（符号全 +1、零偏全 0）；非原生映射显式拒绝，防止 IK 和 qpos 意义不一致。部署软限位参与模型/执行器范围，越界命令在进入仿真线程前拒绝。
- 激活时禁止切换物理/运动学模式；停用后切换会清空待处理目标并保持当前姿态。local/web/both 共用同一个场景时钟和状态。
- 未启用机器人网格碰撞，不宣称避碰、接触动力学或 sim2real 标定已验收。原始模型许可仍按 fr3_description 的 NOTICE 记录，不能把仿真入口代码许可证套给网格。

实现与测试证据见 [FR3 仿真接入记录](../../../../docs/integrations/FR3_SIMULATION.md)。
