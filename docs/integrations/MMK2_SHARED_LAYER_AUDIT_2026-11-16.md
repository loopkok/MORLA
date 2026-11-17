# MMK2 接入对公共层的影响：审计与回撤（2026-11-16）

审计范围：从接入 MMK2 之前的基线 `f2166a5`（2026-11-09 "docs: record workstation UI
release and push verification"）到 `781f2f2`。对象是公共层（`src/common/**`）以及
Web 公共工作站的共享代码，不看 MMK2 自己的驱动/遥操包内部实现。

结论分三部分：MMK2 的能力是否走了既有扩展点、共享默认值是否被悄悄改动、以及
是否存在只对 MMK2 成立的前端分支或打包缺口。**结论不是"MMK2 有问题"**：接入本身
没有把公共层 fork 成第二条路径；问题集中在"为让 MMK2 好用而改掉别的机器人也在用
的默认值"。

## 一、结论

1. **扩展点足够，没有形成 MMK2 专用路径。** MMK2 需要的东西全部通过既有机制进入：
   新增控制器插件 id、adapter 上的可选方法（`getattr` 探测）、entry point 组、
   descriptor 的可选字段与可选端口。公共 Runtime/Compiler/BindingEngine 没有出现
   `if mmk2` 之类的分支。
2. **有三处共享默认值被 MMK2 的需求改动**，会影响别的机器人；已在 `781f2f2` 回撤
   为"按绑定/按 descriptor 显式选择"。
3. **有一处打包缺口**：`morla_core.cadence_probe` 被 5 个公共模块无条件 import，
   却没有纳入版本管理；换一台机器就是 ImportError。已入库并改为默认关闭。
4. 其他机器人受影响面：astral/nero 只用共享 `morla.cartesian_delta_ik`，所以只受
   第 2 条第 1 项影响；FR3/CR5 用自己的包装控制器且显式 `step_cap=False`，未受影响；
   MMK2 全部走自己的包装/适配器。

## 二、公共层确实新增的能力（保留，非回撤对象）

这些是通用能力，别的机器人也能用，审计只确认它们没有改变旧行为：

- **控制器**：`morla.scalar_anchored`（相对锚点映射，见
  `src/common/morla_teleop`）；
  `morla.cartesian_delta_ik` 增加 `smoothing_alpha` 输入低通、`rotation_scale` 旋转
  倍率、`max_solve_ns`、`step_cap`（本次改为按绑定开启，见第三节）、jog 阻塞方向
  掩码、限步后"仍要改善笛卡尔误差"的回退检查。
- **Runtime**：`has_si_joint_position()` 把 JointState 收窄到 `rad`/`m` 单位；
  `TwistStamped`/`Odometry` 发布通道；`stop_wait_s`（adapter 可把停止等待延长到
  自己的 RPC 截止时间之后）；`device_scoped_stop`（同一 session 的设备共享停止域）；
  `morla.simulation_scenes` entry point 组（仿真场景由包自己注册）。
- **Compiler**：`_action_semantics()` 让采集动作端口按单位区分
  `velocity`/`closure_ratio`/`absolute_position`；`morla.scalar_anchored` 必须显式
  声明 `output_range`（缺省 1 m 行程曾把升降压到底）。
- **Input**：`right_ab` 只读端口、任一 grip 开始遥操、按装配计算的动作占用
  （`BindingEngine.reserves_input_action`）、operator hold 不再镜像 grip。
- **Web**：字段/控制器/预设/ik_modes 全部由 descriptor 驱动；新增
  `per_device_work_position` 与解算器上报的 `method_label` 两个数据字段，替代原先
  写在公共组件里的型号判断。

## 三、已回撤的四项（`781f2f2`）

### 1. `CartesianController.step_cap` 默认值

- 原状：为抑制 MMK2 全位姿求解的单拍跳变，共享控制器 `step_cap` 默认改为 `True`，
  于是**任何**声明了 `max_joint_step`/`max_joint_velocity` 的绑定都会被截断。
- 影响：astral / nero 的组合预设早已写了 `max_joint_step: 0.15`，那是设备描述，
  写在任何控制器读取它之前；默认截断等于静默改变两台已交付机器人的实际动作。
- 现状：默认 `False`；上限**声明即校验**（非法值照旧报错），只有构造时
  `step_cap=True` 或绑定参数 `step_cap: true` 才截断。非布尔值（`1`、`"true"`）拒绝。
- 不受影响：`morla_mmk2_teleop`、FR3、CR5 的包装控制器都显式传 `step_cap=False`
  （它们自己按上一条命令限步，重复截断会让指令速率减半）。

### 2. 共享 Web 组件里的 MMK2 分支

- `ReferenceWorkspace` 用 `morla.airbot.mmk2_sdk.left_arm/right_arm` 判断是否在设备
  页显示"直接回工作位"块 → 改为 descriptor 的 `web.per_device_work_position`
  （`make_descriptors.py` 对双臂置 `true`，`descriptors.json` 只多这两个字段）。
- `RunConsole` 用 `method==='mmk2_analytic_dh'` 等字符串判断 IK 报告标题 → 改为优先
  使用解算器自己上报的 `method_label`；三个 MMK2 解算器已带该字段，其余控制器仍走
  原有回退（`differential` → 连续微分跟随）。
- 现状：`src/common` 的非测试代码里搜不到 `mmk2`。

### 3. `cadence_probe` 的打包缺口

- 原状：模块未入库，却被 `morla_teleop`、`morla_input`、`morla_web_manager`、
  MMK2 遥操无条件 import；只有 MMK2 驱动自己写了 try/except 兜底。
- 现状：随 `morla_core` 提交；默认**关闭**，`MORLA_CADENCE_PROBE=1` 才启用。共享
  框架不应让每个进程都往用户家目录写文件。开启后行为不变：每进程一份
  `<wall_ns>-<pid>.jsonl`，逐秒写计数、时延分位与事件；文件不自动清理。
- 操作侧：MMK2 驱动 README 已注明采集 `dispatch` 前要带该环境变量启动工作站。

### 4. Quest 行为文档化

- "任一 grip 开始遥操"、"另一只手已按住时只作该手门控"、"`right_ab` 被点动绑定
  占用时 A/B 的录制操作不再派发"补进 `morla_input/README.md`、控制链里的
  「Quest 工作站快捷键」说明与 CHANGELOG。原先只有代码和实现者知道。

## 四、验证

- 合并运行 `morla_core` / `morla_teleop` / `morla_runtime` / `morla_input` /
  `morla_operator_control`：**333 passed, 1 skipped**。3 个失败为既有项：
  `test_flake8`/`test_pep257`（历史 lint 基线）与 `test_profiles` 的顺序依赖，
  在 `f2166a5` 的工作树副本上同样失败。
- `morla_mmk2_driver` + `morla_mmk2_teleop`：**59 passed**。
- `morla_web_manager`：**320 passed**；3 个 `test_piano_api` 失败因本机缺
  `~/Desktop/MORLA-piano`（`test_piano_home.py` 收集阶段即报错，已排除）。
- `npm run build`（tsc + vite）通过，`web/dist` 已随提交更新。
- 新增测试：`test_a_declared_bound_alone_does_not_clip`（只声明上限不再截断）、
  `test_invalid_step_cap_is_rejected`；原截断/回退用例改为显式 `step_cap: true`。

## 五、边界与未验证项

- 本次是代码与调用链审计加单元测试，**没有启动机器人、没有连接 MMK2 或 Quest**。
- 实机默认动作未变：MMK2 三个包装控制器本来就显式关闭共享限步，本次回撤不改
  MMK2 的实机行为；改的是"别的机器人不再被默认截断"。
- `cadence_probe` 改为可选后，历史记录目录不会被自动清理，也不会迁移。
- 未纳入本次范围：EPG/断链 watchdog（按操作员决定不作为测试门禁）、Linux DDS 时延、
  现场停止与使能验收——这些仍是独立验收项。
