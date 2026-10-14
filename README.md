# astral_ws

Quest3 → **Astral 双臂** + **Wuji 双手** 的 ROS 2 工作空间。  
从 `xnero_ws-main` 迁出，**不含** Nero / XHand / pyAgxArm。

```text
Quest3 (quest3_hand_mocap, convert_to_robot:=true)
  ├─ mixed: 一侧手柄 + 一侧手（controller 默认同写 wrist_pose）
  ├─ IOBT: hips 世界系；head/wrist/controller/body_joints 在 hips 系
  │        mocap 保证 frame_id 整段稳定（sticky latch + hold gate），
  │        下游按稳定系增量算 IK，不再跨系做 delta。
  ├─ quest3/{left,right}_wrist_pose
  │     → astral_arm_teleop → /{side}_arm/joint_commands
  │           → 仿真 astral_mujoco_sim    或  真机 astral_robot_control
  └─ hand_landmarks/{left,right}
        → wujihand_retargeting → /{side}_hand/joint_commands
              → 仿真 wujihand_mujoco_sim  或  真机 wujihand_control

Quest3 ← WebRTC（quest3_video_streamer，信令 :8765）
        PC 相机：D435i / USB 腕部 → 头显 3D 面板（第一视角）
```

臂与手两条链路独立；Quest 可同时喂两边。  
**仿真与真机不要抢同一条 `joint_commands`。**

## 包一览

### 共用

| 包 | 作用 |
|----|------|
| `quest3_hand_mocap` | Quest 腕/手柄/头/IOBT 身体（Wuji 用 `landmark_preprocess:=raw`） |
| `quest3_video_streamer` | PC 相机 WebRTC 推到 Quest（遥操第一视角）；与 mocap 独立 |
| `wuji_glove` | 手套 mocap（可选） |

### Astral 臂

| 包 | 作用 |
|----|------|
| `astral_arm_teleop` | 双臂遥操 IK（默认 `geometric` 臂角闭式 + 人臂肘先验；可选 `analytic_dh` / `urdf_numerical`）+ 安全滤波（原 `astral_quest_teleop`，纯臂） |
| `astral_teleop` | 整机遥操编排 launch：mocap + 双臂 + 左夹爪 + 右 Wuji（Quest 或手套） |
| `astral_gripper_teleop` | Quest3 左手捏合 → `/left_gripper/command`（可换硬件源） |
| `astral_robot_description` | 双臂 URDF（`astral_robot.pin.urdf`，SW 原约定） |
| `astral_robot_control` | 真机驱动（`astral_robot_sdk`） |
| `astral_mujoco_sim` | 双臂 MuJoCo（默认 `astral_dual.xml`） |
| `astral_arm_description` | 单臂 URDF（查看用） |
| `astral_arm_clean_description` | 可选干净 MDH URDF，**默认闭环不用** |

空目录 `astral_analytic_ik` / `astral_urdf_ik` 是旧 C++ IK 残留，无 `package.xml`，不参与编译。

### Wuji 手

| 包 | 作用 |
|----|------|
| `wujihand_retargeting` | landmark → 20 关节 |
| `wujihand_control` | 真机 launch 薄层 |
| `wujihandros2` | vendored 驱动：`wujihand_driver` / `wujihand_msgs` / `wujihand_bringup` |
| `wujihand_mujoco_sim` | 手仿真 + TuningViewer |

### 数据

| 包 | 作用 |
|----|------|
| `astral_data_collect` | VLA 数据采集：raw HDF5 录制（与遥操并行）→ 离线对齐 → 清洗校验 → LeRobot v2.1 导出（OpenPI pi0.5 可读）→ Rerun 回放 |
| `astral_policy_inference` | 策略部署/回放/HITL 推理节点：backend 抽象（openpi 远程 / ACT 进程内或远程 serve / stub）、三种引擎节奏、`~/cmd` 仲裁、绝对动作安全层。换模型=只改后端参数，换机器人=改与采集一致的 schema yaml |

开源 retarget 库仍可放在本仓库旁：`../wuji-retargeting`。

## 构建

```bash
cd /home/robot/loopkok/sdk/astral_ws
export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  quest3_hand_mocap quest3_video_streamer \
  astral_robot_description astral_arm_teleop astral_gripper_teleop astral_teleop \
  astral_robot_control astral_mujoco_sim \
  wuji_glove wujihand_retargeting wujihand_control \
  wujihand_driver wujihand_msgs wujihand_bringup \
  wujihand_mujoco_sim astral_data_collect
source install/setup.bash
```

真机臂还需：`pip install -e ../astral_robot_sdk`  
真机手还需：`wujihandcpp` deb（见 `wujihand_control/README.md`）。  
URDF 数值 IK：`pip install pin`。

## 启动

```bash
# 臂仿真（求解器 / 协议在 yaml，默认 geometric 臂角 IK + 人臂肘先验 + HTS 有线 TCP）
ros2 launch astral_mujoco_sim astral_sim_pipeline.launch.py

# 臂仿真改求解器：把 astral_arm_teleop_{left,right}.yaml 的 solver_type 改成 analytic_dh / urdf_numerical
# 臂仿真改 UDP：把 quest3_mocap.yaml 的 protocol 改成 udp

# 臂真机（默认左手捏合控左夹爪）
ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py \
  with_driver:=true control_board_ip:=192.168.10.2

# 整机真机：双臂 + 左夹爪 + 右 Wuji（右手 quest3 或 glove）
ros2 launch astral_teleop full_teleop.launch.py \
  with_arm_driver:=true with_hand_driver:=true \
  right_hand_source:=quest3


# 手仿真 / 调参
ros2 launch wujihand_mujoco_sim wujihand_sim_pipeline.launch.py \
  input_source:=quest3 retarget_backend:=wuji_retargeting
ros2 launch wujihand_mujoco_sim wujihand_tuning.launch.py \
  input_source:=quest3 retarget_backend:=wuji_retargeting

# 手真机
ros2 launch wujihand_control wujihand_real_pipeline.launch.py \
  input_source:=quest3 hand_side:=right retarget_backend:=wuji_retargeting
```

Quest 有线 HTS：`adb reverse tcp:8000 tcp:8000`。  
Quest 视频回传：`ros2 launch quest3_video_streamer multi_camera.launch.py`；有线可 `adb reverse tcp:8765 tcp:8765`。细节见 [`src/quest3_video_streamer/README.md`](src/quest3_video_streamer/README.md)。

数据采集（VLA 训练，遥操运行时并行）：

```bash
ros2 launch astral_data_collect data_collect.launch.py \
  session:=pick_place end_effector_right:=wuji
# 另一终端：ros2 run astral_data_collect keyboard_controller（s/q/d/n/p/t 热键）
# 离线：align_data → validate_data → convert_to_lerobot → replay_rerun
```

细节见 [`src/astral_data_collect/README.md`](src/astral_data_collect/README.md)。

## IK 约定（臂）

| `solver_type` | 位姿帧 | 发布的 q |
|---------------|--------|----------|
| `analytic_dh` | 干净 MDH 基座；yaml `vr_to_arm_rot=I` 再乘 \(R_\text{base}^\top\) | DH 约定求解后 `flip_q` 成 SW 约定 |
| `geometric`（yaml 默认） | SW `*_base_link`；`astral_robot.pin.urdf` | 已是硬件约定，不 flip |
| `urdf_numerical` | SW `*_base_link`；`astral_robot.pin.urdf` | 已是硬件约定，不 flip |

仿真 MJCF **未改**（`astral_dual.xml`）。细节见 [`astral_arm_teleop/README.md`](src/astral_arm_teleop/README.md)。

## 文档

| 文件 | 内容 |
|------|------|
| [`src/astral_arm_teleop/README.md`](src/astral_arm_teleop/README.md) | DH / flip / `vr_to_arm_rot` |
| [`src/astral_mujoco_sim/README.md`](src/astral_mujoco_sim/README.md) | 臂仿真 |
| [`src/astral_robot_control/README.md`](src/astral_robot_control/README.md) | 臂真机驱动 |
| [`src/wujihand_control/README.md`](src/wujihand_control/README.md) | 手真机 |
| [`src/wujihand_retargeting/README.md`](src/wujihand_retargeting/README.md) | 重定向 |
| [`src/wujihand_mujoco_sim/README.md`](src/wujihand_mujoco_sim/README.md) | 手仿真 / tuning |
| [`src/quest3_video_streamer/README.md`](src/quest3_video_streamer/README.md) | Quest 相机 WebRTC 回传 |
| [`src/astral_policy_inference/README.md`](src/astral_policy_inference/README.md) | 策略部署 / 数据回放 / 人在环路（HITL） |
| [`src/astral_policy_inference/CLAUDE.md`](src/astral_policy_inference/CLAUDE.md) | 推理包架构不变量 / 环境约束 / 已解决坑 / 测试验证清单 |
| [`CHANGELOG.md`](CHANGELOG.md) | 版本记录（按天演进史，每个问题 症状→根因→修法→数值） |

### 排查经验（`docs/`，症状→根因→修法 独立整理版）

| 文件 | 内容 |
|------|------|
| [`docs/README.md`](docs/README.md) | **排查经验索引**：主题 → 文档 → 一句话结论 + 跨主题方法论 |
| [`docs/act-inference-stutter-investigation.md`](docs/act-inference-stutter-investigation.md) | ACT 推理卡顿（四层叠加原因 + 排查 checklist） |
| [`docs/2026-09-21-pi05-inference-investigation.md`](docs/2026-09-21-pi05-inference-investigation.md) | pi0.5 真机推理问题调查（A/B/C/D 轮实验，进行中） |
| [`docs/inference-deploy-pitfalls.md`](docs/inference-deploy-pitfalls.md) | 推理部署环境/后端坑（版本墙/ACT 1 行 chunk/锁饥饿/陈旧队列/参数静默退化） |
| [`docs/teleop-stick-slip-investigation.md`](docs/teleop-stick-slip-investigation.md) | 遥操慢速"抖"（驱动层低速粘滑）排障全记录 |
| [`docs/motor-control-pitfalls.md`](docs/motor-control-pitfalls.md) | 驱动层/电机控制坑（"电机突然自己动"定责手册） |
| [`docs/data-quality-investigation.md`](docs/data-quality-investigation.md) | 数采数据质量排障（五层防线 + 卡点治本 + repair 演进） |
| [`docs/camera-video-performance-investigation.md`](docs/camera-video-performance-investigation.md) | 相机/视频回传性能排障（GIL/编码器线程/限流/设备指纹） |
| [`docs/camera-label-semanticization.md`](docs/camera-label-semanticization.md) | 相机 label 语义化（videoN→语义名，硬件指纹钉死） |
| [`docs/openpi-training-deploy.md`](docs/openpi-training-deploy.md) | openpi 训练部署（RAM≥32GB 等迁移清单） |

### 测试记录

| 位置 | 内容 |
|------|------|
| [`src/astral_arm_teleop/doc/2026-09-17-ik-solver-comparison.md`](src/astral_arm_teleop/doc/2026-09-17-ik-solver-comparison.md) | **逆解求解器对比**（geometric vs urdf_numerical）：慢速"一顿一顿"根因 = 电机死区(~1mrad)×geometric 最小关节速度优化；含现象/方法/指标/量化结论 + `cmd_deadband_mrad` 修复 |
| `astral_test_logs/`（sdk 根） | 大测试记录文件夹（独立于 git 仓库）：`2026-09-17_ik_solver_comparison/` 含遥操 JSONL 数据 + 可复用分析脚本 + README |
| [`inference_test_logs/`](inference_test_logs/) | 推理/遥操测试日志（`SUMMARY.md` 索引） |
