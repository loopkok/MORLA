# CR5 右手柄 / 右手追踪 VR 遥操

> 2026-10-30 更新：当前统一路径：CR5 VR 兼容 launch 只启动公共 session，接收器由 morla.processes 工厂托管；纯 fake Quest 不启动实体接收器。真实 Quest＋fake CR5 必须使用仅输入为 real 的部署覆盖（device_id 与 process_mode），详见 [主机 Q01/U02–U04](../testing/host_build_and_hardware_acceptance.md)。QuestRight 现有 scalar/right_landmarks，通用控制/滤波不再仅服务 CR5。

更新：2026-10-29。当前使用 **CR5 本体 URDF + Pinocchio 本地数值 IK**；不再调用控制器 `InverseSolution` 来遥操。右手柄和右手追踪控制同一台独立 CR5，裸手捏合不驱动灵巧手。

软件及离线链路已验证；ROS Humble、Quest 与 CR5 实机验收仍未执行。fake 采用与 real 相同的 URDF/IK，但反馈由 FakeAdapter 合成，无动力学、碰撞或硬件实时性保证。

## 1. 实际数据链

```text
既有 Quest 应用 → 有线 TCP 8000 → 带会话/时效保护的 Quest 接收器
  → 原样输出 Unity 基准（事件 v2 携带 basis）
  → {left,right}_teleop_events（明确 controller / hand 来源）
  → QuestEventSource：按 side 选手的位姿 + 握把/捏合状态记录，逐端口源时效
  → BindingController：输入时效/序号/epoch 检查，来源切换清锚
  → CR5 异步 controller：滤波、相对位姿、输入旋转与 TCP 换算
  → CR5 IKSolver：本地 URDF / Pinocchio FK、SE(3) DLS、FK 硬回验
  → MORLA runtime：急停锁存、组件子集（不校验授权/序号/TTL）
  → CR5 Adapter：部署关节限位、ServoJ → 实测关节反馈
```

求解与设备通信分离，IK 不访问 Dashboard、不发送控制命令，也没有控制器 IK 回退。底层 ServoJ 仍沿用独立 CR5 驱动。Nero/Astral 的求解器与原 Quest 话题保留；事件流在接收器本身默认关闭，由公共输入进程工厂显式开启。

## 2. 模型、坐标与求解契约

纯本体模型：`src/robot_description/urdf/cr5/cr5.urdf`，资源 ID `cr5_arm`；网格位于 `meshes/cr5`。来源为用户提供的 `robot-descriptions-dobot/cr5_description` 的 arm 宏和 tf_params；保留六关节原点、轴、惯量、视觉/碰撞网格和数值精度。没有迁入夹爪、场景、ros2_control 或 OCS2。

- 固定基座 `base_link`；求解末端为 **Link6**。Link6 不自动等同于控制器 tool=0 或用户任务 TCP。
- 关节顺序 `joint1..joint6`，单位 rad；平移 m。URDF 第 3 关节限位 ±2.7925 rad，其余为 ±2π；IK 使用 URDF 与现场部署限位的交集。
- 关节对齐：`q_URDF = urdf_joint_signs * q_hardware + urdf_joint_offsets`。默认正号/零偏只是初始约定，真实机器人必须验证。FK、Jacobian、IK 和输出逆变换使用同一约定。
- `input_rotation` 把 Unity VR 的增量转到 URDF `base_link`（det −1 的基变换，见 2.1）；`solver_tip_from_tool` 是 **Link6 到任务 TCP** 的 4×4 刚体变换。后者的数值不是控制器 user/tool 的隐含配置。
- IK 接口与公共层一致：`sync_state(measured_q)`、`fk(q)`、`solve(T)`。每次由当前实测关节播种，只在局部信赖域内求解，不全局随机重启、不对多圈关节强行 wrap。
- 位姿模式（绝对目标）的局部窗口按 `retry_trust` 阶梯 `0.3 → 0.6 → 1.2` rad 逐级重试：目标每次从锚点重算，求解失败就是整帧丢弃、机械臂原地冻结，不能靠下一拍追回；模板位形下把工具绕某一轴转 30°，需要 0.6–1.2 rad 的关节行程（实测 x/z 轴要二级 0.6 rad、y 轴要三级 1.2 rad），单一 0.3 rad 窗口会直接判失败。摇杆（jog）仍只用一个 0.3 rad 窗口——被拒绝的摇杆方向必须保持被拒绝，否则 `jog_blocked` 的轴向屏蔽失效。命中 50 ms 预算即停止重试。
- 本地 SE(3) 对数误差、阻尼最小二乘、线搜索；每次迭代限制增量，检查关节交集限位。最多 100 次迭代、50 ms 计算预算；成功必须 FK 位置误差 ≤0.2 mm、旋转角误差 ≤0.002 rad。不可达、停滞、超时返回失败；回退后上层不再自动撤销授权，需要显式 stop/revoke。
- 外层仍按 `max_joint_velocity`（模板 `quest.sim.yaml` 为 1.0 rad/s；未配置时代码默认 0.3）裁剪输出关节速度，后台单槽 worker 隔离迟到解；完整任务 250 ms、结果年龄 300 ms 仍是拒绝门槛，不是硬实时保证。回退后不再限制相对实测关节步长（`max_joint_step` 参数仍被接受但已不生效）。
- URDF SHA-256 写入 device geometry 契约，求解器加载时校验模型身份；改变模型需同步契约、测试及重新编译 revision。

### 2.1 输入坐标系与 base_link 对齐

Quest 应用给出 Unity 左手系（X 右、Y 上、Z 前）。接收器**默认原样输出 Unity**（帧 id 以 `_unity` 结尾），Unity→各自机器人 base 的对齐由每台机器人自己声明。`M = [[-1,0,0],[0,0,-1],[0,1,0]]`（Unity→旧的 canonical 基，**X 左、Y 后、Z 上**）只是 Astral 的历史约定，不再是公共默认；从旧 canonical 矩阵换算一律**右乘** `A_new = A_old·M`。注意 `M` 是 det −1 的基变换（不是 TF），且 `M·M = diag(1,−1,−1) ≠ I`，逆是 `Mᵀ`。

Nero/Astral 由 `CoordinateContract.resolve_arm_input_rotation` 计算 `R_base_input = R_base_body @ R_body_input`，`R_body_input` 就是 Profile 里的 `input_rotation_to_robot_body`：

| 机器人 | 声明的 Unity→body（`input_rotation_to_robot_body` / binding `input_rotation`） | `robot_body_frame → solver base`（URDF 固定链） | 有效 Unity→solver base |
| --- | --- | --- | --- |
| Nero 左/右 | `[[0,-3.673205e-06,-0.999999999993],[1,0,0],[0,0.999999999993,-3.673205e-06]]` | `t_bracket_base → {left,right}_base_link`（左右差 180° yaw） | 左 `[[0,1,0],[0,0,1],[-1,0,0]]`；右 `[[0,1,0],[0,0,-1],[1,0,0]]` |
| Astral 左/右 | `[[-1,0,0],[0,0,-1],[0,1,0]]` | `body_link → {left,right}_base_link` = I | 同声明值 |
| CR5 | `[[0,0,-1],[1,0,0],[0,1,0]]`（binding 直接就是 base） | `base_link` 是 URDF 根，无安装链 | 同声明值 |

所有声明矩阵 det 均为 −1（Unity 左手系 → 右手 base 的必然结果）。控制器只把它作用于增量：`Δp_base = U·Δp_unity`、`ΔR_base = U·ΔR_unity·Uᵀ`（相似变换 det 仍为 +1），因此 U 从不被当作位姿使用。Nero 左右臂的有效矩阵**等于重构前 `vr_to_arm_rot` 的字面值**，这是"把基准还给输入层、行为不变"的直接证据（[test_nero_coordinate_equivalence.py](src/common/morla_input/test/test_nero_coordinate_equivalence.py)，4e-6）。CR5 以前只能用 canonical→base 的 Rz(−90°)；现在直接用真正的 Unity→base 矩阵，不需要再绕 canonical。

三者数学一致：`delta = R_curr R_anchorᵀ`，`R_target = R_base_input ΔR R_base_inputᵀ R_anchor_robot`，平移 `R_base_input Δp · position_scale`。差别只在 Nero 多一个 URDF 支架安装矩阵、Astral 没有，而 CR5 的 `base_link` 是根节点，没有可推导的安装链。

物理假设只有一条：**操作员站在基座 +X 一侧、面向机械臂**。于是操作员的后方 = base +X、右手 = base +Y、上方 = base +Z；矩阵把 Unity +X（右）映到 base +Y、+Y（上）映到 base +Z、+Z（前）映到 base −X。任何 roll/pitch 都会把"上"从 base +Z 拉偏，CR5 的落地安装没有这种自由度，所以只允许 yaw。若现场是操作员站在 −X 一侧（臂背对操作员），把 `input_rotation` 换成 Rz(+90°)·M = `[[0,0,-1],[-1,0,0],[0,-1,0]]`，其余参数不动。该矩阵由 `test_shipped_input_rotation_is_the_unity_to_base_alignment` 锁定（正交、det −1、三轴映射）。

`convert_to_robot` 仍保留为调试开关（`config/quest3_mocap.yaml` 默认 `false`）；显式 `true` 才恢复 canonical 输出，此时事件 `basis` 为 `quest3_canonical_legacy_v1`，而声明 `unity_left_handed_v1` 的绑定会在编译期/运行期拒绝这种帧。

实机核对只需三步：手向上抬 10 cm → 工具 +Z 上升；手向自己方向拉回 → 工具沿 base +X 朝操作员移动；手掌绕竖直轴顺时针（俯视）→ 工具同向自转。三向任一不符就先改 yaw，不要改 `position_scale` 或 `solver_tip_from_tool`。

算法依据 [Pinocchio 官方 IK 示例](https://github.com/stack-of-tasks/pinocchio/blob/master/examples/inverse-kinematics.py)，并增加部署限位、关节映射、局部连续性、迭代/时间边界和独立 FK 接受检查。

### 2.2 手柄与裸手的实测差异（2026-11-02）

操作员实测「手柄方向不对、裸手还可以」时，用 `scripts/quest3_pose_listener.py` 录制 261 s（手柄 / 裸手 / 手柄三段）后确认：坐标系与镜像没有问题（`{side}_wrist_pose` 与 `{side}_controller_pose` 498 个样本位置/四元数差为 0，帧 id 一致），偏差全部来自输入数据本身：

| 指标 | 手柄轮 | 裸手轮 |
| --- | --- | --- |
| 腕姿到达率 / 间隔 p90 / 最大 | 21.1 Hz / 144.8 ms / 466.4 ms | 88.6 Hz / 13.5 ms / 39.8 ms |
| 头/髋 | 89–90 Hz | 89–90 Hz |
| 超过 `max_age_ms`(250) 的空档 | 34 s 内 9 次 | 0 次 |
| 跟踪点相对腕部偏移 `r`（最小二乘） | `[-0.076, 0.075, 0.143]`，17.8 cm | `[-0.003, -0.01, 0.03]`，3.2 cm |
| 位姿路径中旋转引起的比例 | 4.70 / 7.23 m（65%） | 0.91 / 7.25 m（13%） |
| 静止姿态 | 俯仰约 −45°（握持角） | 约 0° |

也就是说：头显端的手柄位姿以约 21 Hz 发送（接收器与网络没有丢包，同轮 0 条拒绝），每次空档超过 `max_age_ms` 就会重锚；且手柄的跟踪点在腕部以外约 18 cm，手腕转动会直接变成最多约 7 cm 的工具位移（`position_scale 0.8`）。裸手给的是腕原点、约 89 Hz，所以同一套映射下手感明显不同。

若要缩小两者差异，可在接收器或绑定层加入手柄原点修正 `p_wrist ≈ p_ctrl − R_ctrl·r`（`r` 用上表拟合值作初值），并把该绑定的 `max_age_ms` 放宽到 400 覆盖手柄 p99 空档；手柄流的发送频率只能由头显 App 提高。三项都尚未改随包配置。

## 3. 构建与依赖

在 Linux ROS 2 Humble 主机、MORLA 根目录执行，使用 ROS Python 环境：

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-pinocchio
colcon build --symlink-install --packages-up-to morla_cr5_driver morla_web_manager
source install/setup.bash
python3 -c 'import rclpy, pinocchio, numpy, scipy, yaml, jsonschema, fastapi, uvicorn'
python3 -m morla_core.composition.cli catalog --json
ros2 run morla_cr5_driver cr5_kinematics_check --offline
```

ROS 发行包名为 pinocchio；独立非 ROS Python 测试环境用 PyPI `pin`（不是名为 pinocchio 的另一包）。本轮数学与软件回归实际使用 Pinocchio 4.1.0；Humble 主机上的发行版本和 colcon 待现场验证。缺依赖或模型加载失败时拒绝激活，不回退其他 IK。

目录应包含 `morla.dobot.cr5`、`morla.cr5.cartesian_delta`、`morla.input.quest`。CR5 包依赖公共模块与统一 robot_description，不依赖其他品牌臂或手。Web 宿主另用现有公共管理应用。

每个终端 source ROS 与工作区。退出旧 Nero 会话、其他 runtime 和原 CR5 写入脚本；不要同时启动独立 `cr5_driver`，此 VR 宿主已加载 Adapter。不要用 `nero_web_start.sh` 启动 CR5。

## 4. fake 演练与两种输入

沿用已验证的 Quest 应用、有线 TCP 模式，无需新增 Unity 报文。手柄模式打开控制器追踪；裸手模式打开手部追踪及 landmarks 推流。

```bash
adb devices
adb reverse tcp:8000 tcp:8000
ros2 launch morla_cr5_driver cr5_vr.launch.py
```

launch 启动公共 disarmed 会话，默认 `config/quest.fake.yaml`；实体接收器仅在输入绑定为 real 后由公共工厂启动，真实输入＋fake 机器人覆盖见主机 Q01。接收器启动后在 Quest Start Stream，浏览器打开 `http://127.0.0.1:8080/`，顶部应显示 FAKE。8000 只能有一个接收器。可加 `port:=8082` 或 `store:=/absolute/path`；HTTP 默认仅监听本机。

1. 页面出现“手柄”或“手追踪”及新鲜输入后，点击“准备”，确认完成，再点击“激活”。启动和刷新不会自动激活。
2. **右手柄**：激活后持续跟随，松手不会停止（与旧 Nero/Astral 节点一致）。随包配置的该绑定没有 `binding.deadman`；激活后的第一帧以当前手柄位姿和实测关节 FK 的 TCP 建锚，随后移动/旋转控制机械臂。需要停止时必须显式“暂停/撤权”。输入断流超过 250 ms 时绑定按当前臂位姿重新建锚，不产生跳变，也不撤权。
3. **右手追踪**：激活后与手柄一样持续跟随；捏合（原始 21 点距离比 <0.25）结果仍写入 `quest.right_hold`，仅作为采集辅助数据保留，不再门控运动。松开捏合不会停止机械臂。捏合不驱动灵巧手。
4. 点击“暂停 / 撤权”回到 `READY_DISARMED`；松手或松开捏合都不会停止运动，停止必须显式 stop/revoke。恢复跟随需再次点击激活并重新建锚。
5. 切换手柄与裸手、重定位、会话/frame 改变、断流会清除旧锚点，不能跨来源接续旧姿态；断流超过 250 ms 后第一帧按当前臂位姿重新建锚（不产生跳变，也不自动撤权）。
6. 结束时停止录制并检查结果，再停止会话、退出 launch。故障/急停复位不自动恢复跟随。

手部保持用原始 Unity 21 点（米）：拇指尖 4 到食指尖 8 的距离除以掌宽 5–17（距离在正交基变换下不变，判据不依赖坐标系）；<0.25 捏合，>0.45 释放，中间区保持已知状态。初始中间区不伪造释放；不合理掌宽、缺点和非有限值拒绝。pose 驱动运动，按钮/关键点更新保持状态；两端口分别检查 250 ms 时效，新手腕不能延长旧捏合状态。捏合结果写入 `right_hold`，仅作为采集辅助数据保留（旧版曾用作该绑定的 deadman），不再门控运动；停止必须显式 stop/revoke。

fake 可验证实际模型下可达/不可达及整链逻辑，但不模拟物理臂的跟踪误差或碰撞。默认零关节姿态可能处于奇异位置，不保证任意方向的平移/旋转都可解；不可达拒绝属于正常行为，不应关闭门槛强行运动。

## 5. 真实 CR5 部署

### 5.1 只读检查

```bash
ros2 run morla_cr5_driver cr5_kinematics_check \
  --host YOUR_CONTROLLER_IP --device-id YOUR_CR5_SERIAL
```

先占有独占锁，读取硬件关节，再在本地做 URDF FK→IK→FK，不发送 EnableRobot、ServoJ、回零或控制器 IK。报告 `physical_base_tcp_verified=false` 始终保留；模型内部数学闭环不能验证物理零位、方向、基座/TCP。

可选 `--compare-controller-fk` 仅额外请求控制器 FK 并报告差异，用于排查坐标约定；只有明确两套 base/tool 一致后该差值才可解释，程序不据此自动修改标定或宣告物理验收。

### 5.2 填写现场配置

复制 fake Assembly 和 `quest.real.example.yaml` 到现场配置目录，填写：

- arm 身份、独占资源 key、IP、六关节实际允许上下限 rad。
- `urdf_joint_signs` / `urdf_joint_offsets`，验证模型与编码器关节方向/零点一致。
- binding 的 `input_rotation`、`solver_tip_from_tool`，核验 base_link 安装方向及 Link6 到任务 TCP。
- Deployment `calibrations` 的实测记录、标定摘要；当前参数必须显式用于 binding，不会由标定名字自动生成。
- 验证后把 CR5 `kinematics_verified`、binding `calibration_verified` 和真实 Deployment 的 `verified/stop_verified` 置 true 作为记录；回退后这些字段不再参与门禁，不改开关也能激活。

真实模板保留占位和 false，不应只修改开关就当作标定完成。现场还需验证使能、停止、设备 watchdog 和断网/主机掉电行为；主机命令 watchdog 已移除，断链或主机掉电时 ServoJ 保持最后目标，必须依靠设备侧保护或外部断电。

### 5.3 启动与验收

```bash
ros2 launch morla_cr5_driver cr5_vr.launch.py \
  assembly:=/absolute/path/to/quest.assembly.yaml \
  deployment:=/absolute/path/to/quest.deployment.yaml
```

Deployment 将 arm/quest 覆盖为 real。页面必须显示 REAL，反馈 hardware / measured=true。runtime 不再检查标定声明或输入新鲜度；输入时效仍由 QuestEventSource 与 BindingController 把关，过期帧被丢弃并清锚，不会用陈旧位姿继续推进。按第 4 节逐步建锚，在现场限定范围验证方向、TCP、延迟与停止。

VR 默认驱动配置 50 Hz、位置比例 0.25、姿态平滑系数 0.35；实际频率/延迟需测量。主机命令 watchdog 已移除，输入中断时执行器保持最后目标，操作者应显式 stop/revoke 或切断外部电源。求解失败锁存至 reset，异常撤权后需显式重新激活。worker 按 generation 丢弃旧解，停止会话关闭线程。

## 6. 验证记录

本轮模型迁移、本地数学/整链测试、安装资源验证与边界见 [URDF / Pinocchio 实现记录](CR5_URDF_IK_2026-10-29.md)。此前控制器 IK 的 [实现记录](CR5_VR_IMPLEMENTATION_2026-10-29.md) 作为历史保留，已由本轮本地 IK 替代。

未执行 Linux Humble colcon/IDL、Quest/CR5 实体、物理标定、停止/watchdog、碰撞及长稳；软件通过不等于实机验收通过。

## 2026-11-02 连续微分跟随

当前 CR5 专用控制器默认选择完整六维微分 IK，增加自适应阻尼、目标领先限制和超量吸收；旧严格求解可通过绑定参数 `ik_mode: strict` 选择。Web 绑定提供模式选择，运行状态提供控制器诊断。位置/姿态残差与目标吸收是显式报告项，近似更新不代表精确到位。参数、行为变化及本轮验证见 [优化记录](CR5_DIFFERENTIAL_IK_2026-11-02.md)。
