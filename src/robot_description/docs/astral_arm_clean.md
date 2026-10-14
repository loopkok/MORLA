# Astral clean model

可选的「干净」Astral URDF：关节原点按正 MDH（\(\alpha=+90^\circ\)）放置，STL 用 visual/collision origin 重定位。DH 数值与 `astral_arm_teleop/ik/analytic.py` 的 `AstralParams` 一致。

**默认遥操 / 仿真使用的模型不变。** 此可选模型由 `robot_description` 集中管理，默认控制闭环仍走标准 Astral URDF/MJCF（架构 B：模型不动，teleop 层 `R_baseᵀ` + `flip_q`）。

| 文件 | 内容 |
|------|------|
| `urdf/astral/arm_clean/astral_arm_clean.urdf` | 单左臂 |
| `urdf/astral/arm_clean/astral_robot_clean.urdf` | 双臂 |

生成脚本：`python3 src/robot_description/scripts/generate_clean_urdf.py`。  
对应可选 MJCF：`mjcf/astral/astral_dual_clean.xml`。

```bash
ros2 launch robot_description display_astral_clean.launch.py
```
