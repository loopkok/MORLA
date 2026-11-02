# 三种机械臂的公共 MuJoCo 后端与显示选择

日期：2026-11-02。基线 `1e90a89`。保留 Nero/Astral 专用 ROS 仿真入口，新增机器人包内的公共后端 launch；CR5 增加独立 `cr5_mujoco_sim` 包。

## 启动

先安装 MuJoCo、Pillow、Web 依赖及各机器人 IK 依赖，构建并 source 工作区。公共入口需要已安装 `morla_web_manager`；Nero/Astral 通过 `morla_native_bridge` 选择已有 IK provider。不要同时运行占用相同端口的多个 Web session。

```bash
ros2 launch cr5_mujoco_sim cr5_mujoco_sim.launch.py
ros2 launch xnero_mujoco_sim composable_sim.launch.py
ros2 launch astral_mujoco_sim composable_sim.launch.py
```

任选一个启动。三个公共 launch 都支持：

```bash
# 位置伺服动力学 + 网页和本地窗口同时显示
ros2 launch cr5_mujoco_sim cr5_mujoco_sim.launch.py sim_mode:=physics sim_display:=both
# 实体 Quest 输入；仿真仍为仿真后端
ros2 launch astral_mujoco_sim composable_sim.launch.py input_backend:=real sim_mode:=kinematic sim_display:=local
```

共享参数：`sim_mode:=kinematic|physics`、`sim_display:=web|local|both`、`input_backend:=fake|real`、`assembly`、`deployment`、`port`、`store`。默认运动学、网页、fake 输入。旧 CR5 `ros2 launch morla_cr5_driver cr5_sim.launch.py` 兼容入口也使用相同选项。

`ros2 run cr5_mujoco_sim cr5_mujoco_sim_node --sim-mode physics --sim-display local` 提供独立包的命令行入口。公共 session CLI 的 `--sim-mode/--sim-display` 覆盖配置中的默认值。

## Web

打开 `http://127.0.0.1:8080` 的「组合配置」，在「公共 MuJoCo 仿真」选择 CR5/Nero 双臂/Astral 双臂，加载模板；选择模式、显示方式，校验、保存并启动，然后连接输入并显式激活。模板中的 Quest 默认 fake，实体头显需将输入后端改为 real。

启动后可以切换显示方式。切换物理模式必须先撤销运动许可，且停止采集/策略。当前模式和 viewer 状态在 session state 中可见；本地窗口启动错误会显示在 viewer.error。运行时切换只影响本次会话；要保存默认值，在停止会话后编辑配置并保存下一版本。

本地窗口打开在 Web 服务所在主机，不会自动打开在远程浏览器所在电脑；需要图形桌面。macOS 使用同一 Python 环境中的 `mjpython`。关闭窗口不关闭仿真；切回网页、再选本地可重新打开。网页与本地显示同一场景，没有第二套控制器或独立物理时钟。本地 viewer 是只读状态观察窗口，鼠标可调整视角，界面中的模型扰动不会回写 runtime。

网页相机仍可作为采集源，`local` 只隐藏网页预览，不停止相机采集/渲染；这样显示选择不会改变冻结的数据契约。无相机的组合也可以打开本地 viewer。

## 模式与配置

```yaml
metadata:
  simulation:
    mode: kinematic
    display: web
```

未配置时公共 runtime 保持原来的 `physics/web`；新增的三种机器人模板明确选择 `kinematic/web`。

- `kinematic`：每步将最后目标写入映射后的 qpos，qvel 清零，执行 mj_forward，按 timestep 推进仿真时间。无重力积分、无动力学接触响应，适合验证 IK、姿态与映射。模型惯量不参与目标跟踪。
- `physics`：目标写入执行器 ctrl，经 mj_step 和 implicitfast 积分生成反馈；保留原有动力学行为，可能存在重力静差和跟踪延迟。
- 模式切换保持当前关节姿态、清除待处理目标和速度，将新模式的保持目标设为当前状态，避免沿用旧模式目标造成跳变。激活中的会话禁止改变模式；显示方式可在激活状态改变。
- 两种模式均标记 simulation/synthetic、measured=false。关节反馈包含 simulation_mode，运动学模式的 simulation_level 为 kinematic_collision_unverified。
- `POST /api/v2/sessions/{id}/simulation` 接受 mode/display；state 返回当前设置和本地 viewer 健康。选项修改写入 runtime 事件；开始采集时 provider_sources 事件记录实际运行模式，便于区分保存配置与本次会话覆盖值。

## 包与兼容性

`cr5_mujoco_sim` 拥有标准 ament 包、launch、装配配置和 CLI，复用公共模拟器，不复制物理循环。Nero/Astral 在现有包中新增 `composable_sim.launch.py` 和 `config/composable.sim.yaml`；保留 `xnero_mujoco_sim.launch.py`、`astral_mujoco_sim.launch.py`、`astral_sim_pipeline.launch.py` 等原有入口及原行为。

Nero/Astral 公共模板为双臂加 Quest 和场景相机，使用既有 Profile 的坐标合同推导输入基变换与工具变换；Nero 初始位置沿用 Profile 工作位。公共模板未自动包含旧专用入口中的 XHand/Wuji/夹爪/头部，也未迁移专用 XHand 触觉消息发布。需要这些组件时应显式装配，或继续使用原有专用入口。

本地 viewer 子进程只读取公共仿真的模型和状态快照，通过隔离的临时目录原子更新；不发送控制目标，不执行独立 mj_step。停止会话会终止子进程并清理临时文件。GUI 初始化/退出错误不停止物理线程。

## 边界

`physics` 表示执行动力学计算，并不代表已经完成真实机器人标定。CR5 当前没有启用碰撞，公共场景其他旧资源也未完成碰撞安全验证；参考伺服参数不是实机辨识参数。切到 physics 只是后续 sim2real 的仿真基础，不能替代碰撞模型、控制器参数、延时与实机验收。

本轮未进行 ROS Humble colcon/launch 实跑、实体 Quest/Vader 与三种实机测试。保留旧入口不代表它们已经切换到公共动力学实现；要使用新模式和 Web/local 选项请使用上述公共入口。

## 本轮验证

- Python 相关回归：53 passed、1 deselected，7.49 s。覆盖公共模式/显示、三种模板、CR5 IK/模型、Nero/Astral 拼接、Web 配置接口与原有公共仿真。
- 其中新增 7 项 runtime 测试验证三品牌运动学精确跟踪、零速度反馈、激活时拒绝切换、物理步进产生速度、显示切换复用模型与关闭 viewer，以及三种模板可编译；新增 Web 测试验证模板、模式切换、非法值与 ACTIVE 状态拒绝。
- macOS 本机真实 viewer 冒烟：通过 mjpython 打开窗口，3 s 后进程仍运行、公共场景时间正常推进；切回 web 后 viewer 已关闭且 runtime 持续运行，最后正常停止会话。未测试 Linux 远程桌面。
- TypeScript `tsc -b` 与 Vite build 通过，61 modules；生成并更新部署用 web/dist。
- 一项既有 `test_mobile_input_and_recording` 因 `release_or_reanchor_required` 失败；在隔离目录恢复 HEAD 版本的 Web API/service/CLI 后同样失败，确认基线已存在。本轮最终相关回归排除此项，不修改输入控制语义来规避。
- `git diff --check` 通过。ROS Humble colcon 与 launch 仍未实跑；网页完整交互未做浏览器自动化，Web 选项由 API 回归和前端类型检查覆盖。

可复现测试（在项目根目录，将 src 下各 setup.py 所在目录加入 PYTHONPATH，Python 环境需具备上述依赖）：

```text
python -m pytest \
  src/common/morla_runtime/test/test_sim_modes.py \
  src/common/morla_runtime/test/test_sim_mujoco.py \
  src/robot_description/test/test_composable_scene.py \
  src/robot_description/test/test_cr5_mujoco.py \
  src/common/morla_web_manager/test/test_composable_api.py \
  src/robots/dobot/morla_cr5_driver/test \
  -q -k 'not test_mobile_input_and_recording'
```
