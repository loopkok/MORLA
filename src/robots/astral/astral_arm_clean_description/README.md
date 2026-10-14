# astral_arm_clean_description

可选的「干净」Astral URDF：关节原点按正 MDH（\(\alpha=+90^\circ\)）放置，STL 用 visual/collision origin 重定位。DH 数值与 `astral_arm_teleop/ik/analytic.py` 的 `AstralParams` 一致。

**默认遥操 / 仿真不使用本包。** 控制闭环仍走 `astral_robot_description` + 旧 MJCF（架构 B：模型不动，teleop 层 `R_baseᵀ` + `flip_q`）。

| 文件 | 内容 |
|------|------|
| `urdf/astral_arm_clean.urdf` | 单左臂 |
| `urdf/astral_robot_clean.urdf` | 双臂 |

生成脚本：`python3 -m astral_arm_teleop.generate_clean_urdf`。  
对应可选 MJCF：`astral_mujoco_sim/assets/mjcf/astral_dual_clean.xml`。

```bash
ros2 launch astral_arm_clean_description display.launch.py
```
