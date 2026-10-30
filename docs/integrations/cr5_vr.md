# CR5 右手柄 / 右手追踪 VR 遥操

> 2026-10-30 更新：当前统一路径：CR5 VR 兼容 launch 只启动公共 session，接收器由 morla.processes 工厂托管；纯 fake Quest 不启动实体接收器。真实 Quest＋fake CR5 必须使用仅输入为 real 的部署覆盖（device_id 与 process_mode），详见 [主机 Q01/U02–U04](../testing/host_build_and_hardware_acceptance.md)。QuestRight 现有 scalar/right_landmarks，通用控制/滤波不再仅服务 CR5。

更新：2026-10-29。当前使用 **CR5 本体 URDF + Pinocchio 本地数值 IK**；不再调用控制器 `InverseSolution` 来遥操。右手柄和右手追踪控制同一台独立 CR5，裸手捏合不驱动灵巧手。

软件及离线链路已验证；ROS Humble、Quest 与 CR5 实机验收仍未执行。fake 采用与 real 相同的 URDF/IK，但反馈由 FakeAdapter 合成，无动力学、碰撞或硬件实时性保证。

## 1. 实际数据链

```text
既有 Quest 应用 → 有线 TCP 8000 → 带会话/时效保护的 Quest 接收器
  → Unity 到 canonical 坐标转换
  → right_teleop_events（明确 controller / hand 来源）
  → QuestRightSource：右位姿 + 握把/捏合状态记录，逐端口源时效
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
- `input_rotation` 把 canonical VR 的增量转到 URDF `base_link`；`solver_tip_from_tool` 是 **Link6 到任务 TCP** 的 4×4 刚体变换。不是沿用 Nero 朝向，也不是控制器 user/tool 的隐含配置。
- IK 接口与公共层一致：`sync_state(measured_q)`、`fk(q)`、`solve(T)`。每次由当前实测关节播种，只在局部信赖域内求解，不全局随机重启、不对多圈关节强行 wrap。
- 本地 SE(3) 对数误差、阻尼最小二乘、线搜索；每次迭代限制增量，检查关节交集限位。最多 100 次迭代、50 ms 计算预算；成功必须 FK 位置误差 ≤0.2 mm、旋转角误差 ≤0.002 rad。不可达、停滞、超时返回失败；回退后上层不再自动撤销授权，需要显式 stop/revoke。
- 外层仍按 `max_joint_velocity`（默认 0.3 rad/s）裁剪输出关节速度，后台单槽 worker 隔离迟到解；完整任务 250 ms、结果年龄 300 ms 仍是拒绝门槛，不是硬实时保证。回退后不再限制相对实测关节步长（`max_joint_step` 参数仍被接受但已不生效）。
- URDF SHA-256 写入 device geometry 契约，求解器加载时校验模型身份；改变模型需同步契约、测试及重新编译 revision。

算法依据 [Pinocchio 官方 IK 示例](https://github.com/stack-of-tasks/pinocchio/blob/master/examples/inverse-kinematics.py)，并增加部署限位、关节映射、局部连续性、迭代/时间边界和独立 FK 接受检查。

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

目录应包含 `morla.dobot.cr5`、`morla.cr5.cartesian_delta`、`morla.input.quest_right`。CR5 包依赖公共模块与统一 robot_description，不依赖其他品牌臂或手。Web 宿主另用现有公共管理应用。

每个终端 source ROS 与工作区。退出旧 Nero 会话、其他 runtime 和原 CR5 写入脚本；不要同时启动独立 `cr5_driver`，此 VR 宿主已加载 Adapter。不要用 `nero_web_start.sh` 启动 CR5。

## 4. fake 演练与两种输入

沿用已验证的 Quest 应用、有线 TCP 模式，无需新增 Unity 报文。手柄模式打开控制器追踪；裸手模式打开手部追踪及 landmarks 推流。

```bash
adb devices
adb reverse tcp:8000 tcp:8000
ros2 launch morla_cr5_driver cr5_vr.launch.py
```

launch 启动公共 disarmed 会话，默认 `config/quest_right.fake.yaml`；实体接收器仅在输入绑定为 real 后由公共工厂启动，真实输入＋fake 机器人覆盖见主机 Q01。接收器启动后在 Quest Start Stream，浏览器打开 `http://127.0.0.1:8080/`，顶部应显示 FAKE。8000 只能有一个接收器。可加 `port:=8082` 或 `store:=/absolute/path`；HTTP 默认仅监听本机。

1. 页面出现“手柄”或“手追踪”及新鲜输入后，点击“准备”，确认完成，再点击“激活”。启动和刷新不会自动激活。
2. **右手柄**：按住右握把移动。激活后第一帧以当前手柄位姿和实测关节 FK 的 TCP 建锚，随后移动/旋转控制机械臂。握把状态只作输入意图记录，回退后不再要求“先释放再按下”，松开也不会自动停止。
3. **右手追踪**：拇指/食指捏合后用手腕位置/姿态控制机械臂。捏合同样只记录为输入意图，不构成 runtime 的 deadman。
4. 点击“暂停 / 撤权”回到 `READY_DISARMED`；仅松开握把/手指不会撤权，需要显式 stop/revoke。恢复跟随需再次点击激活并重新建锚。
5. 切换手柄与裸手、重定位、会话/frame 改变、断流会清除旧锚点，不能跨来源接续旧姿态；但 runtime 不再自动撤权，操作者需自行停止。
6. 结束时停止录制并检查结果，再停止会话、退出 launch。故障/急停复位不自动恢复跟随。

手部保持用原始 canonical 21 点（米）：拇指尖 4 到食指尖 8 的距离除以掌宽 5–17；<0.25 捏合，>0.45 释放，中间区保持已知状态。初始中间区不伪造释放；不合理掌宽、缺点和非有限值拒绝。pose 驱动运动，按钮/关键点更新保持状态；两端口分别检查 250 ms 时效，新手腕不能延长旧捏合状态，但释放状态现在只记录在 `right_hold` 端口，不再触发撤权。

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

复制 fake Assembly 和 `quest_right.real.example.yaml` 到现场配置目录，填写：

- arm 身份、独占资源 key、IP、六关节实际允许上下限 rad。
- `urdf_joint_signs` / `urdf_joint_offsets`，验证模型与编码器关节方向/零点一致。
- binding 的 `input_rotation`、`solver_tip_from_tool`，核验 base_link 安装方向及 Link6 到任务 TCP。
- Deployment `calibrations` 的实测记录、标定摘要；当前参数必须显式用于 binding，不会由标定名字自动生成。
- 验证后把 CR5 `kinematics_verified`、binding `calibration_verified` 和真实 Deployment 的 `verified/stop_verified` 置 true 作为记录；回退后这些字段不再参与门禁，不改开关也能激活。

真实模板保留占位和 false，不应只修改开关就当作标定完成。现场还需验证使能、停止、设备 watchdog 和断网/主机掉电行为；主机命令 watchdog 已移除，断链或主机掉电时 ServoJ 保持最后目标，必须依靠设备侧保护或外部断电。

### 5.3 启动与验收

```bash
ros2 launch morla_cr5_driver cr5_vr.launch.py \
  assembly:=/absolute/path/to/quest_right.assembly.yaml \
  deployment:=/absolute/path/to/quest_right.deployment.yaml
```

Deployment 将 arm/quest 覆盖为 real。页面必须显示 REAL，反馈 hardware / measured=true。runtime 不再检查标定声明或输入新鲜度；输入时效仍由 QuestRightSource 与 BindingController 把关，过期帧被丢弃并清锚，不会用陈旧位姿继续推进。按第 4 节逐步建锚，在现场限定范围验证方向、TCP、延迟与停止。

VR 默认驱动配置 50 Hz、位置比例 0.25、姿态平滑系数 0.35；实际频率/延迟需测量。主机命令 watchdog 已移除，输入中断时执行器保持最后目标，操作者应显式 stop/revoke 或切断外部电源。求解失败锁存至 reset，异常撤权后需显式重新激活。worker 按 generation 丢弃旧解，停止会话关闭线程。

## 6. 验证记录

本轮模型迁移、本地数学/整链测试、安装资源验证与边界见 [URDF / Pinocchio 实现记录](CR5_URDF_IK_2026-10-29.md)。此前控制器 IK 的 [实现记录](CR5_VR_IMPLEMENTATION_2026-10-29.md) 作为历史保留，已由本轮本地 IK 替代。

未执行 Linux Humble colcon/IDL、Quest/CR5 实体、物理标定、停止/watchdog、碰撞及长稳；软件通过不等于实机验收通过。
