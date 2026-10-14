# astral_mujoco_sim

Astral 双臂 MuJoCo 仿真：订阅 teleop 的 `joint_commands`，驱动 MJCF（真机驱动替身）。

```text
quest3 (protocol: tcp_wired 默认; convert_to_robot:=true)
  Mixed → robot_world   IOBT → robot_body   # mocap 保证 frame_id 稳定
  → astral_arm_teleop_{left,right}
      geometric（yaml 默认）: SW base_link 系闭式解，输出即硬件约定，不 flip；
        use_human_elbow 默认开（Quest 肩肘方向作为臂角 psi_ref 先验）
      analytic_dh: R_baseᵀ + flip_q → 发布旧约定 q
      urdf_numerical: 无 flip，直接旧约定 q
  → /{side}_arm/joint_commands
  → astral_mujoco_sim_node (viewer)     # 旧 MJCF，不再 flip
  → /{side}_arm/joint_states (seed)

quest3 hand_landmarks/left
  → astral_gripper_teleop (with_gripper:=true)
  → /left_gripper/command | /left_gripper/joint_commands
  → astral_mujoco_sim_node              # 仅 echo 到 /left_gripper/joint_states
                                         # MJCF 暂无夹爪关节，viewer 里不动
```

**不要**与 `astral_robot_control` 同时跑同一 topic。

## 夹爪（sim 现状）

MJCF `astral_dual.xml` **没有夹爪关节**，所以 sim 节点对夹爪只做「订阅 + echo」：
- 订 `/left_gripper/command`（Float64 0=开 1=合，pinch_gripper_node 唯一发布流）和 `/left_gripper/joint_commands`（JointState 弧度，保留给直接发弧度的适配器，当前无人发布）
- 映射 0–1 → `left_gripper_open_rad`…`left_gripper_closed_rad`（sim 默认 2.5…0.0，仅用于 echo 显示；真机 rad 唯一权威在 driver `astral_robot.yaml`，当前同为 2.5 全开）
- 发 `/left_gripper/joint_states`，日志每秒打一次 `[Gripper][left] cmd rad=...`
- **不**驱动 MJCF qpos/ctrl（viewer 里夹爪不会动）

目的：在给 MJCF 加夹爪关节之前，先把 pinch→夹爪这条链在 sim 里跑通并可观。关夹爪 echo：`with_gripper:=false` 或 yaml `enable_gripper_cmd: false`。

## 手柄集成（sim）

sim 管线默认也起 `controller_start_gate` 与带 trigger 合并的 gripper，便于在仿真里测手柄遥操 + 外部启动开关：

- **左手柄 grip 键（中指，bit 5）→ `/teleop/start`**：`with_start_gate:=true`（默认）。臂节点 `require_start_signal:=true`（yaml 默认）时，启动后等外部信号——手摆好初始位姿，按左 grip 即记 `vr_init` 并 arm。
- **左 trigger 模拟量 → 左夹爪**：gripper 订 `quest3/left_controller_joy`，trigger（`axes[0]`，0=开 1=合）在手柄 Joy 新鲜时驱动夹爪，否则回退 pinch。sim 把 `/left_gripper/command` echo 到 joint_states（日志 `[gripper left] src=trigger|pinch`）。
- **右手柄摇杆 → 头 yaw/pitch**：`with_head_teleop:=true`（默认）起 `head_teleop_node`，受 `/teleop/start` 闸门。MJCF 无头关节，sim 节点只做「订阅 + echo」：订 `/head/joint_commands`，回显 `/head/joint_states` 并每秒打 `[Head] cmd yaw=.. pitch=..`（viewer 不动头）。便于在 sim 里验证摇杆→yaw/pitch 映射。

仿真验证外部开关（无真机）：`ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py`（viewer 关掉加 `enable_viewer:=false`）→ 手摆好 → 按左手柄 grip（或 web「开始遥操」/CLI `/teleop/start`）。

## 模型（运行时未改）

默认加载 **`assets/mjcf/astral_dual.xml`**（由 `astral_robot_description` 的 SW URDF 生成，14 铰链 + STL，**原轴符号**）。

| 文件 | 用途 |
|------|------|
| `astral_dual.xml` | **默认仿真**（`mjcf_path` 为空时的回退，`mjcf_path` 留空即用它）。与 `geometric`/`analytic_dh` 发布的 q 同约定 |
| `astral_dual_clean.xml` | 干净 MDH / 翻转限位的可选 MJCF。当前 teleop 默认链路下 **不接入**，误用会肘/肩符号对反 |

`mujoco_sim_node.py` 代码默认 `init_pose_left/right` 仍是 `[0.32, ±0.11, -0.53, -0.80, 0.28, 0, 0]`，
但 **`astral_mujoco_sim.yaml` 已覆盖为零位**：

```yaml
init_pose_left:  [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
init_pose_right: [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
```

零位让仿真开机就能看到「rest → via → init_pose」的 homing 过程
（teleop 启动时会沿 `init_waypoints` 慢速走到各 yaml 的 `init_pose`，
与 sim 的展示位姿无关）。想让模型启动即停在工作位姿，把 yaml 里这两行改回
代码默认值即可。

## 依赖

```bash
pip install mujoco
```

## 构建

```bash
cd /home/robot/loopkok/sdk/astral_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select astral_mujoco_sim astral_arm_teleop --symlink-install
source install/setup.bash
```

## 启动

```bash
# 仅仿真（另开终端发 joint_commands 或 keyboard_vr_sim + teleop）
ros2 launch astral_mujoco_sim astral_mujoco_sim.launch.py

# Quest 遥操 → MuJoCo（solver / protocol 在 yaml，默认 geometric + tcp_wired）
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py

# 一次性覆盖求解器或协议（不改 yaml）
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py solver_type:=urdf_numerical
```

键盘假 VR：

```bash
ros2 run astral_arm_teleop keyboard_vr_sim
```

IK / DH / flip 细节见 [`astral_arm_teleop/README.md`](../astral_arm_teleop/README.md)。
