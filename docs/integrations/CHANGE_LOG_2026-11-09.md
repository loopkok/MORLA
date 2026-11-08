# 遥操速度上限与设备参数清理 — 提交日志（2026-11-09）

记录时区 Asia/Shanghai；仓库 loopkok/MORLA，分支 main。本轮由操作员提出四件事：把
`fr3_quest_real` 的 arm `max_velocity_rad_s` 提到 1.0、检查还有没有 Web 改不了的参数、检查框架下
其他机器人是否有同类问题、修 CR5 的问题；随后追加“修复其他机器人检查出的同类问题”。

两个提交，均已普通推送至 origin/main：

| 提交 | 内容 |
| --- | --- |
| `e89c2e4` | FR3 实机速度上限 0.65→1.0；CR5 补齐设备级速度/领先上限；Web 暴露 `command_lead_rad` |
| `5121bfa` | 删除 CR5 / EPG / LoopHand 中没有任何代码消费的设备参数，记录字段加「仅记录」标签 |

## 1. FR3：0.65 从哪里来，为什么慢

- `0.65` 只存在于已保存装配 `fr3_quest_real`（rev1/rev2）的 arm 实例参数里；代码、descriptor
  默认值（0.3）和 `config/deployment.real.example.yaml` 都没有它。
- 控制器每拍执行 `speed = min(binding max_joint_velocity, instance max_velocity_rad_s)`，
  绑定的 `max_joint_velocity: 1` 被实例的 0.65 静默压掉：控制器每拍只放行约 7.2 mrad，
  而 IK 愿意走 30 mrad（`step = speed·dt`，dt 上限 0.05 s）。
- 异步 worker 的 `__call__` 先把请求投进 `self.pending`、唤醒线程，然后直接返回
  `self.result[0]`——也就是**上一拍**的解，恒定额外多等一个控制拍（实测 89.9 Hz，约 11.1 ms）。
  这与 0.65 是两个独立问题：0.65 是稳态速度上限，异步是固定一拍延迟。
- 修法：`fr3_quest_real` 新 revision 3，只把 `arm.max_velocity_rad_s` 改成 1.0（与绑定的 1 齐平），
  其余参数不动。

## 2. CR5：原本根本没有设备级上限

- 修改前 `AsyncCartesianController` 没有任何设备级速度/领先限制，绑定的 `max_joint_velocity`
  是唯一可调项；也就是说 FR3 的“实例压绑定”问题在 CR5 上不存在，反过来说 CR5 也不能通过设备
  参数先夹一层。
- 新增 `device_command_lead` / `device_joint_velocity` 构造参数，`__call__` 里与绑定值取 `min()`，
  `_create` 从实例参数读取；descriptor 新增 `max_velocity_rad_s`（默认 1.0）与
  `max_command_lead_rad`（默认 0.3）。
- 默认值取 1.0/0.3 是为了**保持改动前的有效行为**（此前无上限，等价于很松的上限），
  所以这次改动不改变现有装配的实机动作，只是把旋钮交给 Web。
- 仍存的差异（未改，记录在案）：FR3 在 `result = np.clip(result, measured-lead, measured+lead)`
  里对整拍结果做了硬夹；CR5 的 `lead` 只约束积分种子，没有同等的整拍 clip。

## 3. Web：`command_lead_rad` 此前改不了

- 绑定参数表单只渲染 descriptor schema 属性、`advanced` 列表和已存在的键。CR5 绑定没有
  `command_lead_rad` 时，Web 无法新增该字段，也就调不到领先上限。
- `ComposableTab.tsx` 的 `advanced` 列表加入 `command_lead_rad`，`workcell.ts` 的
  `parameterSchema` 补默认值 0.08。

## 4. 删除不生效的设备参数（“假旋钮”）

| 设备 | 删除的字段 | 依据 |
| --- | --- | --- |
| CR5 | `command_timeout_s` | 适配器不执行主机命令超时 |
| CR5 | 实例级 `max_step_rad` | 从未被读取；VR 步长只由绑定的 `differential_step_rad` / `max_joint_step` 决定（`vr_controller.py` 会覆盖该选项） |
| EPG | `max_step_m`、`command_timeout_s` | 重构后已不做目标步长限制、无主机命令超时；`command_timeout_s` 只剩 configure 里的范围检查 |
| LoopHand | `command_timeout_s`、`max_step_rad` | `CachedActuatorAdapter` 直接写目标，不消费这两个键 |

配套改动：

- 从三个 `descriptor.json` 的 `properties` 与 `defaults` 删除，Web 表单不再显示；
- 删除 EPG `adapter.py` 中针对 `command_timeout_s` 的范围检查（它只校验这个死键）；
- `morla_epg_driver/config/deployment.real.yaml` 删掉这两行显式记录；
- 文档 `docs/integrations/epg50_060.md`、`morla_epg_driver/README.md` 同步。

保留为**记录字段**、不删除：`kinematics_verified`、`verified`/`stop_verified`、
`calibration_verified`、`board_scope_verified`、EPG `activation_motion_verified`/
`position_feedback_verified`/`device_watchdog_verified`。这些字段不参与编译或运行门禁，
但已保存装配显式写过 `kinematics_verified: true`，删除会触发 `additionalProperties: false`
校验失败。Web 标签统一加「（仅记录）」，避免再被当成门禁开关。
设备 watchdog 相关按既定范围不处理。

## 5. 已保存装配的重编译

descriptor 摘要（`lockfile.plugins`）一改，所有引用该插件的 saved revision 都会在 start 时
以 “preview/installed dependency hash changed” 被拒，因此逐个重编译并保存新 revision：

| 装配 | revision | 最终 assembly_hash | 最终 resolved_hash |
| --- | --- | --- | --- |
| `cr5_quest_sim` | 8→9→10 | `67c99ffffa48ad4b` | `34e188fd589b2213` |
| `cr5_real_bench` | 3→4→5 | `2223a0b1a88c9a10` | `b162f9e434946752` |
| `cr5_real_epg` | 2→3 | `0e77d44ce6cf71d3` | `aa6a73742747abe9` |
| `cr5_real_joint_test` | 1→2→3 | `7a02b8a5f5cf4569` | `ffc82d14ea88cdf2` |
| `cr5_real_quest` | 3→4→5 | `0a4e9d4d2a11de73` | `929e4ce105d5783e` |
| `cr5_real_quest_epg` | 3→4→5 | `9738d0d1c1cae09f` | `5e9fa1a5ffe4094f` |
| `fr3_quest_real` | 2→3 | `8ae3ac34cccafeac` | `1ddd9ddb90dd3e4f` |
| `quest_controller_cr5_sim` | 1→2→3 | `495c224e6e00101d` | `4940f25e91d24c63` |
| `quest_right_cr5` | 2→3→4 | `cc88de8680213648` | `0fbdc2faed2ba994` |
| `test` | 2→3→4 | `ba08cef5cadac692` | `802b0407f0d66603` |
| `vader5_cr5_sim` | 1→2→3 | `9ed14c6ca232ea94` | `6d4d1380feb7676d` |

- 两次重存：先因新增 `max_velocity_rad_s`/`max_command_lead_rad`（`e89c2e4`），再因删除死参数
  （`5121bfa`）。两次都是同一装配的连续 revision，历史快照未被改写。
- `cr5_real_bench` 的 deployment 里显式写着 `max_step_rad: 0.1`，随字段删除一并去掉；
  它是死键，删掉不改变 resolved 的有效行为。
- 每次保存前先做“重编译 + 逐字段 diff”预演：除 `lockfile`/各类 hash 外，只允许
  `instances[*].parameters`、`capture_schema.metadata.assembly_revision` 和
  `cr5_real_bench.device_sessions` 变化。实际 diff 结果正是如此，被删的键就是上表那几个。

全部 16 个已保存装配的最终状态见
[assembly-state.txt](evidence/2026-11-09-teleop-speed-dead-params/assembly-state.txt)：
11 个与当前代码逐字节一致（可 start），5 个仍是本轮之前就不同步的旧 revision（见第 7 节）。

## 6. 验证

环境：`source /opt/ros/humble/setup.bash && source install/setup.bash`，`/usr/bin/python3`
（catalog 发现需要 ROS 环境，否则 `morla.fairino.fr3@1.0.0` 找不到）。

```bash
/usr/bin/python3 -m pytest src/robots/jodell/morla_epg_driver/test \
  src/robots/loophand/morla_loophand_driver/test \
  src/robots/dobot/morla_cr5_driver/test \
  src/robots/fairino/morla_fr3_driver/test -q
/usr/bin/python3 -m pytest src/common/morla_core/test -q
cd src/common/morla_web_manager/web && npm run build
```

- 设备驱动：**144 passed**（[device-tests.log](evidence/2026-11-09-teleop-speed-dead-params/device-tests.log)），
  一条既有 Starlette/httpx 弃用警告。
- 核心契约：**62 passed**（[core-tests.log](evidence/2026-11-09-teleop-speed-dead-params/core-tests.log)）。
- 前端：`tsc -b && vite build` 通过，产物 hash 与上一提交一致
  （[web-build.log](evidence/2026-11-09-teleop-speed-dead-params/web-build.log)）。
- 新增回归：`test_device_caps_are_taken_from_the_instance_not_only_the_binding`
  （实例 0.05 rad/s 压过绑定的 0.9，逐拍位移 ≤ 1 mrad）；`test_stale_feedback_does_not_reset_the_integration_base`
  改为显式构造 `device_joint_velocity=1.`，保持它原本只测“陈旧反馈”的语义。
- `morla_web_manager/test` 的收集失败是既有的（`ModuleNotFoundError: morla_piano`），与本轮无关；
  CR5 套件内的 ComposableService 端到端用例通过。
- 日志对齐分析（用运行日志推拍对齐）得到 0 条匹配，结论“多等一拍”来自代码路径，不是日志模型。

## 7. 边界与后续

- 本轮只改代码、descriptor、Web 与已保存 revision；**没有启动 Web、没有连接实机、没有做真机验收**。
  操作员自行启动 Web 时才会加载新 catalog；Web 进程不重启，设备参数表单仍是旧字段。
- 旧 deployment 里若仍写着被删的键：deployment 参数不与 descriptor schema 校验，
  这些键会被合并进 resolved 参数后被静默忽略——建议手工删掉。
- 仍未与当前代码同步（本轮之前就是如此，start 会要求重新保存）：`fr3_quest_sim` rev1、
  `fr3_real_test` rev5、`vader5_astral` rev1、`vader5_nero` rev1、`vader5_nero_mujoco` rev1。
- `fr3_quest_sim` 的 arm 实例仍是 `max_velocity_rad_s: 0.65`（本轮只改实机装配）。
- 保守说明：`max_velocity_rad_s` 只约束控制器每拍放行的目标步长，不等于机器人实际能达到的
  关节速度；CR5 的 `lead` 没有 FR3 那样的整拍硬夹，也未在本轮改动。
