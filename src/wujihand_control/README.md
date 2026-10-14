# wujihand_control

真机控制薄包：launch + YAML，驱动源码 vendored 在 `src/wujihandros2/`（方案 B）。

## 契约（与 sim / retarget 一致）

```text
input (glove|quest3)
    → hand_landmarks/{side}
    → wujihand_retargeting → /{side}_hand/joint_commands
    → wujihand_driver      → 硬件
    → /{side}_hand/joint_states (~1000 Hz)
```

**不要**与 `wujihand_mujoco_sim` 同时占用同一条 `joint_commands`。

## 前置

1. **wujihandcpp**（编 `wujihand_driver` 需要）：

```bash
# deb（需 sudo）
wget https://github.com/wuji-technology/wujihandpy/releases/download/v1.8.0/wujihandcpp-1.8.0-amd64.deb
sudo apt install ./wujihandcpp-1.8.0-amd64.deb

# 或用户目录
bash src/wujihand_control/scripts/setup_wujihandcpp_user.sh 1.8.0
export WUJIHANDCPP_PREFIX=$HOME/.local/opt/wujihandcpp
export LD_LIBRARY_PATH=$WUJIHANDCPP_PREFIX/lib:$LD_LIBRARY_PATH
```

2. **构建**

```bash
cd /home/robot/loopkok/sdk/astral_ws
export PATH=/usr/bin:$PATH   # 避免 conda 盖掉系统 Python / catkin_pkg
source /opt/ros/humble/setup.bash
colcon build --packages-up-to wujihand_control --symlink-install
source install/setup.bash
```

3. **USB**：本机若已有 `/etc/udev/rules.d/95-wujihand.rules`（`0483` → `0666`）则无需再授权；否则见下方 Changelog。

4. **序列号（可选）**

```bash
ros2 run wujihand_driver wujihand_list
```

`right_serial` / `left_serial` **可留空**：驱动按 `hand_side` 连接；双手法或总线多设备时再填 SN。

## Launch

```bash
# 仅驱动冒烟
ros2 launch wujihand_control wujihand_drivers.launch.py hand_side:=right

# 手套 → 官方 RetargetSession → 真机
ros2 launch wujihand_control wujihand_real_pipeline.launch.py \
  input_source:=glove hand_side:=right retarget_backend:=official

# Quest3 → 与 tuning 相同的 quest3 yaml → 真机
ros2 launch wujihand_control wujihand_real_pipeline.launch.py \
  input_source:=quest3 hand_side:=right retarget_backend:=wuji_retargeting
```

`input_source:=quest3` 自动选用 `retarget_wuji_lib_quest3_{left,right}.yaml`。
覆盖：`wuji_lib_config_right:=/path/to.yaml`。

## 集成约定

| 项 | 约定 |
|----|------|
| Namespace | `/{left,right}_hand` |
| 指令 | `joint_commands`（`JointState.position` × 20） |
| QoS | **BEST_EFFORT** SensorDataQoS（driver / retarget / sim 对齐） |
| 滤波 | `filter_cutoff_freq`（默认 10 Hz） |

## Changelog / Bug fixes

| 日期 | 项 | 说明 |
|------|----|------|
| 2026-08 | 新建 | vendored `wujihandros2` + drivers / real_pipeline launch |
| 2026-08 | QoS | 与 retarget 对齐 BEST_EFFORT，修复真机收不到 `joint_commands` |
| 2026-08 | Quest3 yaml | real pipeline 与 tuning 同步自动加载 `retarget_wuji_lib_quest3_*.yaml` |
| 2026-08 | SN 可选 | serial 为空时按 `hand_side` 连接 |
| 2026-08 | USB | 需 udev：`SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"` |
