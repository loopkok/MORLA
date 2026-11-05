# fr3_description

仅从用户提供的 `frcobot_ros2/fairino_description` 提取 **fairino3_v6**：一个 URDF、七个原始 STL。
不依赖原 fairino_description，不携带 MT、WMS、WML 或其他吨位机械臂。

安装后的 URDF：`share/fr3_description/urdf/fairino3_v6.urdf`；网格 URI 为 `package://fr3_description/meshes/fairino3_v6/...`。
保留源 `base_link`、`j1..j6`、关节原点/轴/限位、质量/惯量及 visual/collision 几何。
额外增加 `wrist3_link -> flange` 固定关节，沿腕 3 的 Z 轴 0.100 m，来源为标准 FR3 手册 4.0.1 的 DH d6。
该帧作为 URDF IK 的末端；任务 TCP 通过 MORLA 绑定单独配置。多位形软件核对不替代实体法兰/零位/TCP 验收。

源 URDF 把 forearm 长度写为 0.24001 m、角度约为 1.5708 rad，均原样保留。
URDF J3 约 ±162°，MORLA 原手册边界为 ±150°，驱动/求解器取双方与部署限位的交集，不自动放宽真机范围。

构建：`colcon build --symlink-install --packages-select fr3_description`。这是描述包，不会启动驱动、使能或运动。
模型含 visual/collision/inertial 元素，但本次未建立 MuJoCo 动力学场景，也未完成碰撞/动力学参数验收。

来源、所有文件摘要和本地修改见 PROVENANCE.json。上游 package.xml 的许可证为 TODO，随附仓库未找到 LICENSE；本包如实标记来源许可未明确，不能将 FAIRINO Python SDK 的 Apache-2.0 许可套用到这些模型资产。
