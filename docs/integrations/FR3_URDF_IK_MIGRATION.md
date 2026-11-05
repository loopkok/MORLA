# FR3 fairino3_v6：独立模型与 CR5 同策略 IK 迁移

日期：2026-11-05。本文记录本轮 URDF/IK 更新；此前 DH 驱动的测试数字见历史验收报告，不能混作本轮结果。

## 交付边界

- `src/robots/fairino/fr3_description` 是独立 ament_cmake 描述包，仅含 `fairino3_v6.urdf` 与该款七个 STL。没有复制 fairino5/10 等其他型号，也不依赖原桌面路径。
- `morla_fr3_driver` 的 FK、严格 IK、微分 IK 使用此模型。SDK、设备生命周期、ROS 命令授权和数采接口继续沿用已有实现。
- Web 提供 FR3 位姿跟随和末端速度控制，默认微分模式，可选严格模式。fake 是合成关节反馈；此次没有提供物理仿真后端或完成真机验收。

## 从供应商包提取模型时做了什么

来源为用户提供的 `FR机械臂资料/frcobot_ros2/fairino_description`，型号仅 `fairino3_v6`。原提交、原/目标 URDF SHA256、逐个 STL 摘要和全部修改记录在描述包 `PROVENANCE.json`。

STL 原样复制；URDF 的 mesh URI 改为 `package://fr3_description/meshes/fairino3_v6/...`。保留原 j1..j6、轴向、零位、惯量、碰撞声明及几何数字，包括 240.01 mm 和 1.5708 rad，没有擅自按手册数值取整。

原 URDF 的最后一个 link 是 `wrist3_link`，其坐标位于 J6 转轴原点；原文件没有显式 flange。为保持既有 MORLA `base→flange` 与 SDK FK 比较口径，增加固定 `wrist3_link→flange` 的 Z 向 0.100 m 平移，依据本地 FR3 手册 4.0.1 表 2.4-1 的 d6。TCP 仍只写在绑定 `solver_tip_from_tool`，不能再重复加这段法兰长度。迁移测试对 62 组位形分别用 XML 变换乘积、Pinocchio FK 和历史手册 DH 进行对照；这证明软件几何一致，不代表现场法兰与轴方向已经标定。

供应商源包 `package.xml` 写的是 `TODO: License declaration`，所提供 checkout 未找到 LICENSE。描述包用 `LicenseRef-Upstream-Unspecified` 并保留 NOTICE；不能把驱动/SDK 的 Apache-2.0 自动套给 URDF/STL。

## 关节与坐标契约

对外维持 `joint1..joint6`、rad、SDK J1..J6 顺序；模型内部是 j1..j6。模型映射为 `q_model = urdf_joint_signs * q_hardware + urdf_joint_offsets`，默认全 +1 / 0。参数同时参与 Adapter 和 IK 的限位计算，FK/Jacobian/IK 全部应用同一映射。

有效限位取手册、映射后 URDF 与部署收紧范围的交集。特别是源模型 J3 约 ±162°，已有手册契约为 ±150°，本轮继续限制 ±150°。源模型其余轴的四位小数也参与交集。不能通过修改部署扩大手册边界。模型 SHA256 固定在 descriptor 与 solver 中，资产被改动会拒绝求解，重新发布模型时必须同步契约与测试。

## CR5 哪些代码与策略被移植

参考 MORLA 提交 `063168d` 中 CR5 的 `urdf_ik.py`、`differential_ik.py`、`vr_controller.py`，移入 FR3 包；运行时不 import CR5 包。

1. 严格 IK：Pinocchio SE(3) log 误差、Jlog6 修正的全雅可比、位置/姿态权重、DLS、单步限制、回溯线搜索、局部信任窗口重试、时间/迭代边界与最终 FK 验收。失败返回 None，目标不会变成迟到命令。
2. 微分 IK：每拍一次有界 DLS 更新；奇异值相关阻尼、平移/旋转目标领先吸收、反向时不累积回绕、关节范围裁剪与超时保持。返回 accepted 不等于原始绝对目标 converged，两类残差在诊断中分别报告。
3. 异步 controller：单槽最新输入、独立模型 Data/worker、释放/重锚代际隔离、worker FK 确认、输入滤波、上一拍实际输出作为积分种子、实测领先约束、输出速度限制与结果时效检查。

设备差异单独适配：FR3 模型与 frame/轴名、有效限位、fake/real 后端（不支持 sim），以及硬件参数的速度/领先上限。默认驱动 0.3 rad/s、0.1 rad；Quest 模板使用 0.2 rad/s、0.08 rad。即使绑定参数设置更高，controller 也会受设备上限约束。没有把 CR5 的反馈率、伺服参数或真机验证结论复制给 FR3。

## 框架与 Web 接线

- `morla.kinematics` 仍注册 `morla.fairino.fr3`，兼容旧通用 controller，但现在返回严格 URDF solver，不再加载旧生产 DH solver。
- 两个新 `morla.controllers` entry points 为 `morla.fr3.cartesian_delta` / `morla.fr3.cartesian_jog`；对应静态 descriptor 加入 ament 组件索引。目录发现不实例化 IK，不连接 SDK。
- `quest.fake.yaml` 改用 FR3 专属 controller 和差分默认值；真实 Deployment 仍可覆盖同一装配，不另建联名驱动。
- Web 的 controller 兼容性仅向 FR3 推荐这两个新插件；预设/自动选择、默认参数、IK 模式与诊断显示接入现有界面。编译后的 dist 同步更新。
- ROS package.xml 增加 `fr3_description` 与 `pinocchio` 依赖；资源解析优先安装后的 ament share，只有无安装包时才使用 MORLA 源码回退，不引用供应商桌面目录。

## 旧配置如何迁移

模型身份、摘要与默认有效限位已经变化，旧 resolved 冻结快照不能直接用于新模型：从原 Assembly/Deployment 重新检查、保存并编译。旧通用 controller 不会自动切换微分模式；要采用本轮完整策略，在 Web 重新选择 FR3 位姿/末端速度模式，或将绑定 controller 改为上述 FR3 专属 ID，设置 `ik_mode: differential`（或 `strict`）。保留现场 TCP、基变换、轴符号/零偏和部署收紧限位，重新校核后再使用。

构建：按驱动 README 安装 Pinocchio，使用 ROS 系统 Python，执行 `colcon build --symlink-install --packages-up-to morla_fr3_driver morla_web_manager`。独立描述包可 `colcon build --packages-select fr3_description`。

## 验证与仍需现场完成的内容

自动测试入口为 `scripts/test_fr3.sh`。覆盖资产闭包与摘要、独立 FK、严格 IK、微分跟踪/吸收/奇异/限位/超时、显式积分种子、异步诊断/重锚，以及真实 MORLA HTTP→Session→BindingEngine→fake Adapter→六维录制链。另有同一 FR3 模型上 FR3 与 CR5 严格/微分算法逐拍数值对照，避免拿不同几何的机器人关节值作错误比较。

本轮实测结果与主机日志见下节。实体轴方向/零位、固件 SDK 兼容性、多个实测位形的 SDK FK、TCP 标定、运动停止和断网、真实遥操质量、碰撞与动力学仍由现场完成。没有因引入 URDF 就宣称物理仿真可用。


## 本轮实际验收（2026-11-05 夜间至 11-06）

- Mac：Pinocchio 4.1.0，FR3 专项 **49 passed**；fake 集成 **28 passed**；前端 **19 passed**，TypeScript 与 Vite 构建通过。Mac 无 rclpy，完整 pure 在 ROS 输入测试收集阶段受限，改在目标 ROS 主机完成，没有把 Mac 结果当作 ROS 结果。
- ROS 主机：`loopkok@192.168.50.5`，Ubuntu 22.04 / Humble / 系统 Python 3.10 / Pinocchio 3.8.0。`colcon build --symlink-install --packages-up-to morla_fr3_driver morla_web_manager` **35 packages finished**，含 fr3_description。最终驱动/Web 增量重建 **2 packages finished**。
- 主机 FR3 专项 **49 passed**；扩充 pure 入口覆盖 FR3 全目录后的框架/CR5 回归 **355 passed**；fake 数据/策略/Web 集成 **28 passed**。保留 FastAPI 与 pandas 可选加速库警告。
- 已安装资源：solver 从 `install/fr3_description/share/fr3_description/urdf/fairino3_v6.urdf` 加载，摘要与模型契约一致；两个 FR3 controller entry points 均发现。
- 实际 fake DDS：六轴具名 JointState / 有限 rad / 非零时间戳；命令与回执、重复/未知 component 拒绝、急停锁存和复位后未使能拒绝均通过。DDS 脚本 exit 0，最终仅向 launch 父进程发送 SIGINT 后，其子驱动显示 finished cleanly，launch exit 0。初次向进程组发 SIGINT 又由 launch 转发，重复信号中断了诊断线程 join，子进程 exit -2；原始失败日志单独保留为 driver-double-sigint.log，未据父进程 exit 0 将其记为成功。
- 浏览器：载入 FR3/Quest 测试预设，FR3 专属位姿/速度控制选项可见，逆解在严格/微分间切换并恢复默认；配置检查、保存 v1、启动至 READY_DISARMED 通过。没有新输入时“开始遥操”被拒绝，提示 `重锚缺少新鲜输入：right_vr`，符合新鲜输入门控，没有伪造已完成 Quest 实机遥操。
- 浏览器停止确认框触发自动化超时，未将此记为浏览器停止成功；改用同一服务 operations/stop API 清理，返回 SUCCEEDED/errors=[]，state 确认 STOPPED，再终止本轮 Web 测试进程。

过程修复：首次 Mac tar 带出 AppleDouble `._` 元数据，主机资产闭包测试发现后清理；后续使用 `COPYFILE_DISABLE=1 tar --no-xattrs`。FR3/CR5 测试原同名导致合并收集冲突，FR3 测试改为 `test_fr3_urdf_ik.py` / `test_fr3_differential_ik.py` 后回归通过。目视检查发现 IK 模式表单仍仅显示 CR5，已改为 FR3/CR5 共用条件并重建前端。

证据位于 [本轮证据目录](evidence/2026-11-05-fr3-urdf-ik/)：build/final-build、fr3-tests/pure/fake、installed-model、DDS/driver/Web 日志、浏览器最终 STOPPED 状态和 [模式截图](evidence/2026-11-05-fr3-urdf-ik/ik-modes.png)。主机原始测试目录为 `/tmp/fr3-urdf-acceptance/`（临时目录）；需要长期保留的日志已复制入库。此前主机验收报告保留原 DH 阶段事实，不改写为本轮 URDF 证据。
