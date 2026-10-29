# CR5 纯本体描述

资源 ID `cr5_arm`，URDF `urdf/cr5/cr5.urdf`，mesh 根 `meshes/cr5`。

从用户指定的 `robot-descriptions-dobot/cr5_description` 提取 arm.xacro 的 CR5 宏（空前缀、convex 碰撞）及 tf_params.yaml，展开为不依赖 xacro 的固定六轴 URDF。只修改 mesh URI 为统一 `package://robot_description/meshes/cr5/…`，保留源原点的 1.5708 等数值精度、轴、限位及惯量。

没有引入夹爪、场景、ros2_control、OCS2、旧 launch/config 或外部包依赖。安装继续由统一 robot_description 包的 CMake 递归安装 urdf/meshes/config/docs 完成；没有在包内嵌套 ROS 包。

`config/asset_manifests/cr5.yaml` 为补充模型清单。公共资源查询和资产检查支持这些补充清单，原主清单保持逐字不变，因此已有 Nero/Astral profile 的 description_revision 不受新增 CR5 影响。

基座为 base_link，末端为 Link6，joint1..joint6 以 rad 定义；Link6 的安装面与实际任务 TCP 必须由部署显式对齐。第 3 关节限位 ±2.7925 rad，其余 ±2π。视觉/碰撞模型可加载不代表碰撞策略或物理模型已验收。

来源、每个源文件哈希与目标 URDF 哈希见 [PROVENANCE.json](PROVENANCE.json)，许可见 [LICENSE](LICENSE)。默认关节正号/零偏是待核验初始约定，不证明编码器与 URDF 相符。

本地求解器在 morla_cr5_driver/urdf_ik.py，使用此模型完成 FK/数值 IK；不调用控制器内置 IK。完整使用流程见仓库 docs/integrations/cr5_vr.md。
