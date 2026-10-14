# MORLA 仓库发布日志

本文记录 MORLA 首次发布至 GitHub 的准备、基线、迁移内容、验证证据和远端结果。后续每次推送都应追加日期、提交范围、验证结果和远端提交号；硬件验收状态必须与仿真结果分开记录。

## 发布 1：首次推送

### 发布元数据

| 项目 | 值 |
| --- | --- |
| 操作时间 | 2026-10-13，Asia/Shanghai；准备记录 17:21 CST，推送结果在下方追加 |
| 目标仓库 | <https://github.com/loopkok/MORLA.git> |
| 分支 | `main` |
| 本地工作区 | `/home/robot/loopkok/sdk/MORLA` |
| 发布方式 | 使用已授权的 `loopkok` GitHub 连接器通过 Git Data API 写入；保留本地迁移提交顺序与文件树 |
| 原始本地集成 HEAD | `ef32ab174cb7f4caba875d96a79fee2ac97ea1b4` |
| 发布日志初始提交 | `cfd602a78550ba370d75aa4cfe95bca1c603c41c` |
| 推送前工作区 | 干净 |
| 连接器开始操作前的远端状态 | 仓库 size 0、没有 refs；GitHub API 报告 `Git Repository is empty` |

### 源码基线与边界

- Astral 冻结提交：`3a57d722d3c4257670d9f90526127bed77063e66`。
- XNero 冻结提交：`e6a0100429024f04e18238f1a42ee968bb18967a`。迁移复核时源仓库位于干净分支 `feat/hug-grasp-loop`，HEAD `6f467bf2a1131019a882399c0be21059464cd06e`；冻结提交仍为其祖先。
- 两个源仓库均未被迁移修改。XNero 冻结点之后的 17 个文件属于独立 `xnero_grasp_bridge` 研究实现，缺少本迁移计划定义的 ROS/action 接口，因此不纳入 MORLA 机器人控制图。
- 执行依据为仓库内 `docs/MIGRATION_PLAN.md`、`docs/SOURCE_BASELINE.json`、`docs/package_map.json` 与 `docs/migration_progress.md`。迁移进度记录根目录计划 SHA256 为 `2b8580fd3a1af2865297a5f60a36098351fa0e2ffd2c5e19447bd80415af82b5`；冻结清单 SHA256 为 `8880299fead51c35619f8f7027d3a3231557acaac75d623174ff80e2122b5733`。
- 发布内容只包含 MORLA 仓库。运行数据和集成测试产物位于 `/tmp`，未加入 Git；Astral、XNero 源工作区未纳入本仓库，也未改动。

### 本次迁移交付

- 在 Astral 基线副本上建立独立 MORLA 仓库，按 `src/common/`、`src/robots/{astral,nero,wuji,xhand}/` 和 `src/robot_description/` 组织共享包、机器人包及统一模型资源。
- 统一共享接口包：`quest3_hand_mocap` 改为 `morla_input`；`astral_data_collect` 改为 `morla_data_collect`；`astral_policy_inference` 改为 `morla_policy_inference`；`astral_web_monitor` 改为 `morla_web_manager`；`astral_teleop` 改为 `morla_bringup`。新增 `morla_core`、profile registry 和控制协调器。
- 以冻结 profile 和 `FrozenSchema` 连接输入、机器人状态/命令、在线采集、离线对齐/质量验证、LeRobot 导出、策略推理、HITL/回放及 Web 管理。首版支持 12 种 Astral/Nero 布局，维度覆盖 8、14、16、19、27、35、38、54。
- 将 Astral、Wuji、Nero/XHand 的 URDF、MJCF、mesh 和模型清单集中到 `robot_description`；manifest 记录 27 个 model ID、358 项资源和 847 个 mesh 引用，并由资产校验和 loader 测试验证。
- 将 Nero SDK 硬件 driver 与 teleop 分离；加入 XHand 标准关节接口适配和 XNero MuJoCo 仿真，保留硬件 SDK/native message 边界。硬件路径仍须设备验收。
- XNero 历史数据通过显式 `import-xnero` 只读导入；不会补造缺失的 command、tactile、ownership 或 validity 数据。
- 迁移提交链（从基线到推送前 HEAD）：
  - `f441a3e` 建立 MORLA/Astral 基线副本。
  - `9f933f1` 重组 MORLA 包目录。
  - `74d5537` 加入共享 core 和 robot profiles。
  - `f87a767` 集中机器人描述资源。
  - `9266430` 统一 Quest 输入与 XHand 预处理。
  - `2b8b4bc` 拆分 Nero SDK driver 与 teleop。
  - `be18e6f` 接入 XHand 与 Nero 仿真。
  - `1eff771` 统一 profile 数据采集与旧数据导入。
  - `d903c37` 统一策略所有权与控制协调。
  - `71717b1` 统一 profile 驱动的 Web 管理。
  - `ef32ab1` 完成统一机器人与数据链路集成（推送前 HEAD）。

### 验证记录

验证依据为 `docs/migration_progress.md` 中截至 2026-10-13 的逐阶段日志。汇总如下：

- ROS 2 Humble 全工作区 symlink 构建：31/31 包成功；profile CLI：12/12。
- `scripts/test_ros.sh`：341 passed；触觉专项 1 passed、2 deselected。Astral geometric IK、IK solver、Quest mapping 独立回归通过。
- `scripts/test_policy.sh`：149 passed。
- `scripts/test_offline.sh`：97 passed、1 deselected。
- 分包回归：Web manager 37 passed；bringup 33 passed；数据采集 128 passed、1 skipped；XHand retarget/adapter 与 XNero 仿真 88 passed（另有 2 个 ROS demo Node 类 pytest collection warning）。
- 机器人描述：Pinocchio 加载 12 份 URDF，MuJoCo 加载 5 份 MJCF；847 个 mesh 引用闭包及模型 SHA 校验通过。7 组 URDF 随机 FK/关节名/限位对照通过；Astral 与 XNero 仿真机械参数对照通过。
- Web：前端 TypeScript/Vite 构建通过（52 modules）；安装态页面及 JS bundle HTTP 200；profile/API/launch 回归 37 项通过。
- 数据与端到端：Astral 8/35 维及 Nero 19/38 维仿真采集、对齐和 LeRobot v2.1 导出均有通过证据；Nero19 完成策略/HITL/replay/pause/resume 软件闭环。最终 Nero19 样本为 190 帧、schema 19/19，0 fail、1 warning。
- 统一复验脚本 `scripts/build.sh`、`scripts/test_ros.sh`、`scripts/test_offline.sh`、`scripts/test_policy.sh`、`scripts/test_sim_integration.sh` 已加入仓库。集成脚本只运行隔离 ROS domain 中的仿真与合成输入，输出写入 `/tmp`。

### 已知限制与未完成验收

- P11 真机/硬件/模型验收未完成。Astral/Nero/XHand/Wuji 实机、固件和 SDK 安全路径、严格 Quest APK、38 维策略模型均需在对应设备和部署环境验证。本次不将仿真结果表述为硬件通过。
- Nero19 最终仿真报告有 1 个 warning：仿真手状态约 50.2 Hz，低于策略建议的 100 Hz；报告无 fail。Astral35 与 Nero38 仿真也记录了手状态频率或可选 tactile 对齐 warning，详见迁移进度日志。
- `colcon test` 汇总中有 49 项静态 lint 失败，归于 `morla_input` 的 flake8/pep257 和上游 `xhand_control_ros2` 文件的版权头、CMake 空白及 uncrustify 检查。功能 pytest 按包隔离 ROS domain 后通过；未格式化供应商文件或伪造授权头。
- XHand SDK 当前 `.so` 为 x86-64；ARM64/CUDA 运行依赖需按目标平台提供。XNero XHand/retarget 来源许可需上游确认后方可外部分发。
- Web npm audit 曾报告锁定依赖含 1 项 moderate、2 项 high advisories；本次未擅自升级依赖。
- DexPilot backend 在系统 Python 依赖准备前曾无法构造；迁移进度记录后续安装 Torch 并完成 backend 构造/运行验证。部署平台仍须核实匹配的 Torch/DexPilot ABI。

### 推送操作记录

| 时间（Asia/Shanghai） | 操作 | 结果 |
| --- | --- | --- |
| 2026-10-13 17:21 CST | 查询指定 GitHub 远端 refs | 命令成功；远端无 HEAD/main 引用 |
| 2026-10-13 17:22 CST | 查询 GitHub 仓库与分支 | `loopkok/MORLA` 为 public、size 0、无 branches；API 对 `main` 返回 `Git Repository is empty`（HTTP 409） |
| 2026-10-13 17:22 CST | 检查授权身份 | GitHub 连接器登录为 `loopkok`，仓库权限包含 admin/maintain/push；本机 GitHub CLI 未登录，Git credential store 无可用凭据 |
| 2026-10-13 17:22 CST | HTTPS 首次推送尝试 | 失败：Git 无法读取 GitHub Username，当前环境无可交互终端凭据 |
| 2026-10-13 17:22 CST | SSH 端口连通性与身份检查 | 22 端口超时；GitHub SSH over 443 可连接，但本机 key 对应账号 `zhongrenjiexing` |
| 2026-10-13 17:22 CST | SSH 推送尝试 | 失败：GitHub 返回 `Permission to loopkok/MORLA.git denied to zhongrenjiexing`；远端仍为空，未创建或修改任何分支 |
| 2026-10-13 17:34 CST | 切换到 `loopkok` GitHub API 身份 | 连接器返回登录用户 `loopkok`；仓库权限为 admin/maintain/push |
| 2026-10-13 17:34 CST | 初始化远端空仓库 | 以 MORLA `README.md` 创建 `main` 初始提交 `efc2de6a26e909cc25811e0380efe5973c4a4bac`；接下来写入完整 Git 对象和迁移提交树 |
| 待追加 | 写入 MORLA 完整对象、提交链并更新 `main` | 进行中 |
| 待追加 | 读取 GitHub `main` 并校验文件树与本地发布树一致 | 待执行 |

### 身份与传输说明

本机 HTTPS 没有可交互凭据；SSH over 443 的 key 登录为 `zhongrenjiexing`，对目标仓库无写权。按用户指示改用已认证且有管理员/push 权限的 GitHub `loopkok` 连接器执行 Git Data API 上传。API 创建的远端提交由 `loopkok` 签署身份、SHA 与本地对象不同；迁移提交标题和每个版本的完整文件树将按本地提交顺序保留。完成后读取远端 branch/tree 校验，再追加远端 SHA 与结果。
