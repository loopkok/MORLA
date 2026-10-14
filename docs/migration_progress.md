# MORLA 迁移进度

执行依据：`docs/MIGRATION_PLAN.md` 与 `docs/SOURCE_BASELINE.json`。
开始时间：2026-10-13（Asia/Shanghai）。
源基线：Astral `3a57d722d3c4257670d9f90526127bed77063e66`；XNero `e6a0100429024f04e18238f1a42ee968bb18967a`。

## 状态

| 阶段 | 状态 | 结果/证据 |
| --- | --- | --- |
| P0 独立副本与环境 | 完成 | 595 个 Astral Git 跟踪文件均存在且 SHA256 一致；源提交匹配 baseline；原工作区未修改；MORLA 独立提交 `f441a3e`。 |
| P1 包路径与改名 | 完成 | 25 ROS 包可发现且全量 clean build 成功；五个共享包完成改名，入口脚本与Web launch路径适配；前端和指定回归通过。映射见 `docs/package_map.json`。 |
| P2 core/profile | 完成 | 新增`morla_core`共享schema/lease/资源/关节映射/profile registry；12个profile固定布局并从已安装ament share验证，见本次日志。消费者运行时接入仍在P7-P9完成。 |
| P3 description | 完成 | 新建`robot_description`（ament_cmake），集中27个model ID与358项资产；manifest含来源/目标SHA、关节映射、847个mesh闭包和授权信息。24包干净构建成功；普通安装、colcon symlink-install 均由Pinocchio加载12份URDF、MuJoCo加载5份MJCF；生成器可重跑，Astral FK和sim机械量、XNero sim机械量对照通过。 |
| P4 unified input | 完成 | `morla_input` 固定输出 robot-axis raw 21点；strict packet 默认拒绝legacy，session/坐标帧变化发epoch并由统一闸门停止策略/遥操、清缓存且要求松开后重新start；XHand旧MANO/小指局部适配和Nero新旧pose链等价验证通过。 |
| P5 Nero driver/teleop | 未开始 | |
| P6 XHand adapter/sim | 未开始 | |
| P7 unified data collection | 未开始 | |
| P8 policy/HITL | 未开始 | |
| P9 Web manager | 未开始 | |
| P10 software integration | 未开始 | |
| P11 hardware/model acceptance | 未开始 | |

## 日志

### 2026-10-13

- 已确认源 commit 与迁移基线匹配；两个源工作区的 `git status --short` 为空。
- 已在 SDK 根目录创建新的 `MORLA/`，运行 `rsync -a` 从 Astral 复制源码，排除build/install/log、缓存、node_modules、egg-info和运行数据。
- 尚未修改ROS包、description、配置或机器人接口；尚未启动硬件。
- 本地环境已确认 ROS 2 Humble；后续按系统 Python 3.10构建ROS包。
- P0校验：`git -C astral_ws ls-files` 共595个文件；逐文件比较MORLA副本SHA256，缺失0、差异0；MORLA初始提交为`f441a3e`。
- P1：将原Astral包按`common/`、`robots/astral/`、`robots/wuji/`重组；`quest3_hand_mocap`→`morla_input`、`astral_data_collect`→`morla_data_collect`、`astral_policy_inference`→`morla_policy_inference`、`astral_web_monitor`→`morla_web_manager`、`astral_teleop`→`morla_bringup`。同步Python模块/resource/setup/import/launch/脚本路径；新增Astral/Nero/Wuji/XHand组件元包框架；`pi05_standalone`移入`tools/legacy/`并加`COLCON_IGNORE`。
- 新工作区包列表：`colcon list --base-paths MORLA/src --names-only`为25个包；单独只有一份`morla_input`与`morla_data_collect`。`docs/package_map.json`记录源/目标包、路径和可执行入口。
- P1 clean build：`source /opt/ros/humble/setup.bash; PATH=/usr/bin:/opt/ros/humble/bin:$PATH /usr/bin/colcon --log-base MORLA/log build --base-paths MORLA/src --build-base MORLA/build --install-base MORLA/install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3`，退出码0，25/25包完成。要求明确使用系统Python；默认shell的conda Python会导致ROS CMake解析找不到`catkin_pkg`。
- P1 launch解析：`web_manager.launch.py`、数采、推理和Astral full bringup均以`--show-args`加载成功；没有启动机器人或硬件节点。Python源码`compileall`通过。
- P1回归：`morla_input/test_packet_guard.py` 3通过；`morla_data_collect/test` 125通过/1跳过；`morla_policy_inference/tests` 143通过；`morla_web_manager/test` 25通过；`morla_bringup/test` 24通过。数采导出测试临时使用Pandas 2.3.3后通过；该测试依赖已清理，P7补齐包运行依赖。Quest全目录的flake8插件会递归扫描SDK根目录，故只执行了包保护逻辑测试，没有把该递归检查记作通过。
- Web前端`npm ci && npm run build`通过（TypeScript + Vite，52 modules）；npm审计提示锁定依赖有1项moderate和2项high advisories，尚未擅自升级依赖。
- 源`astral_ws`、`xnero_ws`提交和工作区状态复核无变化；迁移目录中运行时代码不再引用旧工作区路径或五个旧共享包名。尚未运行真机。
- P2：新增ROS-free `morla_core`，将schema/lease移出collector并将ROS lease bind放入惰性`ros_ownership`；policy、collector、Astral臂/夹爪、Wuji retargeting改依赖`morla_core`。严格关节映射、schema v2/hash/v1升级、资源路径保护和profile registry均有黄金/负例测试。
- P2 profiles：注册12种 Astral/Nero 布局：Astral 8/16/27/35/54，Nero 14/19/38；`nero_dual_xhand`严格按左臂、右臂、左手、右手 `[0,7),[7,14),[14,26),[26,38)`。从Astral URDF、Wuji/XHand URDF及XNero Nero IK提取limits/标准名字；保留原Astral臂与夹爪参数引用；增加左右组件模板、相机slot与XHand触觉sidecar定义。
- P2 回归：纯逻辑10通过；profile CLI从已安装ROS包索引解析12项并生成`resolved_profile.json`/`frozen_schema.json`；单独安装空间可见所有MORLA包。全量clean build 26/26通过。Input 3通过、policy 143通过、Web 25通过、bringup 24通过；统一采集125通过/1跳过。测试使用临时Pandas 2.3.3以满足v3导出；依赖已登记为`python3-pandas`，临时用户级包在最终清理时卸载。
- P2发现并修正5个新Python包的无效maintainer邮箱：否则colcon会将其识别为普通Python包，虽能编译但不会进入ROS包索引。重建后`colcon list`显示`ros.ament_python`，`ros2 pkg list`和已安装profile CLI验证通过。
- P2修复`data_pipeline.sh`在重命名后缺少`morla_core`的源码路径，并新增`MORLA_DATA_PYTHON`优先级与旧`ASTRAL_DATA_PYTHON`弃用提示；原兼容路径测试通过。

- P3：新建`src/robot_description`为独立`ament_cmake`资源包，集中Astral robot/arm/clean/legacy RobotMain、Wuji、Nero+XHand、独立XHand retarget URDF，及Astral/Wuji/XNero MuJoCo模型、完整mesh闭包、RViz/launch、触觉点输入、生成/校验脚本。删除MORLA内三个旧description ROS包以及teleop/retarget/sim里的URDF/MJCF/mesh运行时副本；源`astral_ws`/`xnero_ws`保留不变。`docs/package_map.json`已更新资产迁移映射。
- P3资源索引：manifest记录27个model ID、358个资源文件、847个mesh引用、source/target SHA256、base/end links、可动关节、profile ROS名→URDF名及符号/offset、用途、生成器和license来源。XNero retargeting原包没有声明license，manifest/docs明确标为未解决，不伪造授权；profile的`description_revision`统一固定manifest SHA256。
- P3路径：所有活动loader以`morla_core.resource_locator.resolve_robot_asset(model_id)`解析安装share+manifest；Astral IK、Astral/Wuji sim、Wuji DexPilot默认均不引用旧description包。`resolve_robot_asset`在普通安装与`colcon --symlink-install`均通过，并限制符号链接只回落到对应`robot_description`源码包。
- P3构建/安装：24 ROS包clean build通过；普通复制安装运行`check_assets.py --loaders --require-regular-install`通过，symlink安装运行同脚本及12个profile `validate-all`通过。两个安装空间均由Pinocchio实际加载12份URDF（含全部mesh），MuJoCo加载5份MJCF；847个mesh闭包与模型目标hash全通过。
- P3机械意义回归：7组迁移URDF对原Astral/XNero工作区做随机配置FK、关节名、限位比较均一致；Astral clean MJCF模型维度、body姿态/质量/惯量、joint axis/range、actuator limit完全相同；XNero生成器重跑后对应同一组模型量，`test_xhand_tactile_mujoco.py`验证1200/1200触觉测点、名称完整和测试压入力全部入传感，未启动机器人硬件。Astral Geometric IK、完整IK、VR映射三个原回归脚本均PASS。
- P3剩余验证限制：DexPilot实际retarget构造因系统Python没有Torch失败（当前只装了`dex_retargeting`代码但其import硬依赖Torch）；URDF本身由Pinocchio加载成功，实际backend将在依赖环境准备后补验。XNero XHand源包license需上游确认后方可外部分发。
- P4：将Quest ROS receiver统一为`landmark_preprocess=raw`：21点先按Astral Unity→robot矩阵转换、以腕为原点并应用现有EMA；移除全局MANO和小指stretch，Wuji现有raw路径不变。`quest3_mocap.yaml` 的`allow_legacy_packets`从true改为false；full teleop、Astral sim/standalone arm、Wuji sim/tuning/real入口均有显式`allow_legacy_packets:=true`旧APK兼容开关，启用时节点打印降级警告。
- P4输入epoch：PacketGuard在新session接替时接受首包并暴露线程内session变更事件，同时退役旧session；receiver在新session或已发布pose从world切到IOBT body frame时清空EMA/腕来源/帧锁并发布latched `quest3/input_epoch`。`controller_start_gate` 收到事件后发布latched `/teleop/disarm`、`/teleop/armed=false` 与policy `stop`，清除旧按钮边沿，并要求控制器先松开grip、再重新按下；新session、旧session重放、严格legacy拒绝均有逻辑测试。实际ROS多节点时序仍将在P10集成运行补验。
- P4坐标/手型：Nero左右`vr_to_arm_rot_after_unity_to_robot`沿用计划推导的`A_new=A_old*M^T`，flip_pitch=false；新增`xhand_retargeting.preprocess_xhand_landmarks()`，先按`F*M^T`恢复旧XNero手输入，再重放1.05缩放、腕心、SVD、左右OPERATOR2MANO及可选原小指顺序stretch。Wuji不再能被XHand全局变换污染；profile把XHand兼容模式显式写为`retarget_compatibility.preprocess_mode=legacy_xhand_pinky`。
- P4验证：全量增量构建25/25包通过；`morla_core.profile validate-all` 12/12 profiles通过（8/16/19/27/35/38/54维均保留）；输入/闸门/XHand新增与相关回归合计80 tests通过，其中Nero六方向、三轴旋转、500随机四元数与非零TCP各40随机样本/侧在float64容差内等价；XNero MANO+pinky 20组/侧及MANO-only左右参考对照共44用例通过。`colcon list`发现25个ROS包。未连接Quest、Nero或XHand硬件。
