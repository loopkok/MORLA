# astral_gripper_teleop

Quest3 手部食指–拇指距离 → 夹爪开合。独立于 `morla_input` 和臂 IK。支持左/右任一侧（`hand_side` 参数）。

```text
hand_landmarks/{side}
        → pinch_gripper_node
              └── /{side}_gripper/command          Float64  0=开  1=合
                    → astral_robot_control → set_gripper_angle（CMD 0x97/0x98）
```

**只发闭合比，不发弧度**。`open_rad`/`closed_rad` 的唯一权威在 driver 的 `astral_robot_control/config/astral_robot.yaml`（`left/right_gripper_open_rad`，当前 **2.5** 实测全开、0.0 合拢；若开到最大又弹回说明超过机械行程，回调 ~2.0）。**切勿**同时往 `/{side}_gripper/joint_commands` 发 JointState——driver 两个话题都订，两路流若映射出不同 rad 会按控制频率互相覆盖，夹爪来回抽搐（2026-08-26 真机实测踩过）。该话题保留给以后需要直接发弧度的硬件适配器。

**上电瞬态（正常现象）**：启动时 driver `one_click_ready` 会把夹爪归零（0=合拢），遥操节点第一条指令随即把它开到 `open_rad`——看起来像"启动时抽一下"，随后稳定。无需处理。

以后换硬件：新节点发同一个 `/{side}_gripper/command`（0..1 闭合比），关掉本节点即可。臂仍走 `quest3/{side}_wrist_pose`。

```bash
# 左夹爪（默认）
ros2 launch astral_gripper_teleop gripper_teleop.launch.py
# 右夹爪
ros2 launch astral_gripper_teleop gripper_teleop.launch.py hand_side:=right
# 已包含在 morla_bringup/full_teleop.launch.py：
#   左夹爪 with_gripper:=true；右夹爪 right_hand_source:=gripper
```

调参：本包 `config/gripper_teleop.yaml` 只管**手部距离**（`open_dist_m`/`close_dist_m`，米）与滤波/超时；夹爪**电机角度**只改 driver 的 `astral_robot.yaml`（当前 2.5 全开 / 0.0 合拢）。真机方向：捏合→合、张开→开。

**自适应量程**（`auto_range:=true`，默认开）：跟踪你实际捏合距离的 min/max（带 `auto_range_forget_s` 遗忘，双向适应），把当前距离映射到这个真实范围，用满夹爪行程——避免 `open_dist_m/close_dist_m` 与你实际手部范围不符时夹爪只动一小段。`open_dist_m/close_dist_m` 退化为先验/回退；关掉则用固定 open/close 映射。`log_interval_s` 默认 **0**（关）：不再每 2 秒打 `dist=… close=…` 状态行；需要时 `log_interval_s:=2.0`。

**手柄 trigger 模拟量**（`controller_joy_topic` 非空时启用，`full_teleop` 默认设为 `quest3/{side}_controller_joy`）：订阅 Touch 手柄 `Joy`，取 `axes[trigger_axis]`（0=松开/张开 … 1=按下/合拢）作为夹爪闭合比。手柄 Joy 新鲜时优先用 trigger，否则回退到 pinch——同侧 Quest 手柄与裸手互斥，二者互补：握手柄时 trigger 控夹爪、裸手时捏合控夹爪。参数：`trigger_axis`(0)、`trigger_deadzone`(0.05)、`trigger_invert`(false)。日志 `src=trigger|pinch` 标明当前来源。

## 策略交接

收到策略 pending/活动状态后夹爪停止发送，并持续记录实际输出闭合比。交还 HUMAN 时先把本地输出种到该闭合比，
再等新鲜 trigger 或手部关键点追上当前值后才恢复控制；这避免策略抓住物体期间，操作员松开的 trigger 立即把夹爪打开。
Joy 和 PoseArray 按 ROS header 时间过滤，交接前缓存的输入不会触发夹爪运动。
