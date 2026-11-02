# XNero MuJoCo simulation

This package preserves the XNero hand physics, contact model, and 1200 tactile
sample sites. `robot_description` is the only installed source for its MJCF and
mesh assets. The arms remain kinematic overrides; they publish MORLA's standard
named arm state and consume standard named arm commands.

The canonical full stack is launched through `morla_bringup`, which selects one
profile and exactly one execution mode:

```bash
ros2 launch morla_bringup nero_xhand_stack.launch.py \
  profile_id:=nero_left_xhand mode:=sim enable_viewer:=false
```

In simulation, standard hand commands pass through `xhand_adapter` to the
internal vendor-shaped XHand command topic. The simulator publishes native
XHand feedback and tactile data; the adapter turns measured joint feedback
into `/{side}_hand/joint_states`. No retargeting or simulation node publishes
fake measured hand state. Real mode uses the vendor driver and Nero SDK driver;
the unified launch rejects a mixed sim/real selection.

Standalone simulation is useful for isolated contract testing:

```bash
ros2 launch xnero_mujoco_sim xnero_mujoco_sim.launch.py enable_viewer:=false
```

Its inputs are `/left_arm/joint_commands` and `/right_arm/joint_commands`
(`sensor_msgs/JointState`, exact side-qualified names), plus
`/{side}_hand/driver/xhand_command` (`xhand_control_interfaces/XHandCommand`).
It publishes standard arm states at 50 Hz and native hand state at 50 Hz under
`/{side}_hand/driver/xhand_state`. Each native hand state includes five finger
records, 120 force vectors per finger, and 20 temperature samples. Simulated
force is in N, each fingertip link2 frame; simulated temperatures are zero.
Those semantics remain distinct from real SDK sensor axes and units.

The model can be regenerated only through the centralized asset scripts under
`robot_description/scripts`; generated copies must not be placed back in this
package. See `robot_description/docs/xhand1-tactile-sim-params.md` for the
touch-site provenance.

## 公共后端（2026-11-02）

原有仿真入口与行为保留。新增 `ros2 launch xnero_mujoco_sim composable_sim.launch.py`，使用公共 MuJoCo 后端；支持 `sim_mode:=kinematic|physics` 和 `sim_display:=web|local|both`，默认运动学、网页。需要已安装统一 Web manager 和 native IK provider。公共模板为双臂、Quest 输入与场景相机，未默认装配手部/触觉/夹爪。详细说明见 [公共仿真文档](../../../../docs/integrations/SHARED_MUJOCO_2026-11-02.md)。
