# 门禁回退与 EPG/VR 收尾推送回执：2026-10-30

记录时区：Asia/Shanghai。仓库为 <https://github.com/loopkok/MORLA>，分支 `main`；未使用 force，未重写既有提交。起点为 `24a3baa`（`fix(core): accept colcon symlink-install resources inside their own package`）。

## 提交内容

| 提交 | 主题 | 范围 |
| --- | --- | --- |
| `9f98b67` | `refactor(runtime): remove redundant command gates and record EPG/VR acceptance` | 50 files changed、464 insertions、704 deletions |

### 运行时与设备回退

- 编译/契约：删除 `jsonschema` 参数校验、`resolved_hash` 校验、deadman/故障策略/混合后端限制，以及 `next_state` 必须有同名测量位置的编译期限制；`ResolvedSystem.from_dict` 不再复验哈希。`resolved` 快照仍由编译器生成并携带哈希，只作为产物记录。
- `RuntimeSession`：删除 `RESOLVED_HASH_REQUIRED`、activate/operation 状态机前置条件、authority lease/sequence/TTL/expired 检查、按健康状态 group-stop、提交时的名称/单位/有限值/关节限位校验。保留资源互斥、急停锁存、组件 ID 非空/唯一/属于本会话、adapter 接受，以及急停/reset/stop 的生命周期。
- `ComposableService`：删去“激活前必须有新鲜 pose/deadman”和输入过期自动撤权；输入在 ACTIVE 前仍返回 `explicit_activation_required`，租约只做登记。策略与人工之间不再有运行时互斥。
- 输入/控制器：`BindingController` 删除 deadman 与跨源检查，保留 250 ms 端口时效、source-id、epoch/frame 身份和递增序号；`scalar_map` 删除 `max_velocity` 裁剪；CR5 删除相对步长、IK deadline 与真实模式标定 flag 门禁，保留输出速度裁剪、250 ms 求解与 300 ms 结果年龄；Vader5 删除 START+SELECT 急停、按钮角色冲突和启动释放检查。
- 设备：EPG 删除 `max_step_m`、主机命令 watchdog 与验证 flag 门禁，保留未使能位置抑制、停止反馈确认和 `active` 拒绝；CR5/native/Astral 删除验证 flag 与 NERO 主机侧序号关联；LoopHand real 额外保留独立的 `activate` 只读阻断（固件无停止 ACK/watchdog），不随 flag 回退放开。
- 重新保留 `CachedActuatorAdapter` 与 `NativeRosAdapter` 的未激活提交拒绝（`ADAPTER_DISARMED`），使 reset/stop 后不会把目标继续写进传输层。

### EPG 台架验收与文档

- `CHANGELOG.md` 记录 `EPG-BENCH-01` 的只读、双向全行程、5 轮重复性、5 分钟/150 次连打、统一管理层 prepare/estop/reset/stop 与未使能位置抑制结果；断链为 **FAIL（操作者豁免）**，`device_watchdog_verified` 只是本地记录，不得写成已验证 watchdog。
- 同步更新运行时、CR5、CR5 VR、Vader5、EPG、设备接入指南、主机测试手册、示例与 Loophand README，删除对授权/序号/TTL、验证 flag、步长/主机 watchdog 等已删除门禁的描述；保留的资源互斥、急停、输入时效、设备参数/停止确认等边界逐项写明。

### VR/Vader5 到 EPG 的链路

- 核对结论：`morla.input.quest_right` 只有 `right_pose`/`right_hold`，没有扳机或 `scalar` 端口；Astral 的 VR 夹爪走 `pinch_gripper_node` → `/{side}_gripper/command` → `astral_robot_control` 设备专用旁路，不经过统一 Assembly/Binding。当前统一层能直接驱动 EPG 的连续输入是 Vader5 的 RT `scalar`。
- 台架配置位于仓库外 `~/morla-vader-epg/`：`vader_epg.assembly.yaml` 绑定 `pad.scalar → gripper.position_target`，`morla.scalar_map` 将 `[0,1]` 映射到 `[0.05,0]`；`vader_epg.deployment.yaml` 使用 `vader5-pad-01` 与 `EPG-BENCH-01`（站号 9、115200、稳定 by-id 串口）。已编译到 `/tmp/vader_epg.real.json`，两个实例均解析为 real。`deadman: pad.hold_to_control` 字段保留，但回退后不参与编译或运行。
- 整链入口为 `ros2 launch morla_vader5_input vader5_teleop.launch.py`。本机没有 `/usr/bin/python3` 的 `vader5_sdk`，也没有连接 Flydigi（vendor `37d7`）hidraw 设备，真实 VR 全流程 **NOT_RUN**；本轮只完成配置编译、Web/服务路径核对与 fake 回归，不能宣称实机通过。

### 测试对齐

- `docs/testing/fake_ros_gateway_smoke.py` 改为验证保留路径：重复组件、未知组件（`COMMAND_RESOURCES`）、急停锁存后提交（`RESET_REQUIRED`）、reset 后未重新激活（`ADAPTER_DISARMED`）；token/epoch/session/sequence/TTL/名称/单位/NaN 负例标为 `NOT_APPLICABLE`。
- 更新仍断言旧门禁的用例：LoopHand real 激活阻断、重复序号/撤销后提交、命令 TTL、移动端 deadman；改为验证回退后的实际行为（重复序号仍接受、撤销后由 adapter 拒绝、TTL 仅携带、移动输入全部转发）。

## 提交前实际验证

- `git diff --check` 无空白错误；全部改动的 Python 文件 `py_compile` 通过。
- `scripts/test_composable.sh --layer pure`（`/usr/bin/python3`）：**167 passed, 1 skipped**。
- `scripts/test_composable.sh --layer fake`：**23 passed**。
- `scripts/test_composable.sh --layer sim`：**7 passed**。
- Web manager 全量（ROS Humble 系统 Python，`MORLA_PIANO_PROJECT=/home/loopkok/MORLA-piano`）：**288 passed, 2 skipped**（约 119 s，`/tmp/morla-web-tests-final.txt`）。
- `colcon build --symlink-install`：**41 packages finished**，0 failure；6 个包仅有既有 stderr 告警（robot_description、wujihand_*、xhand_control_*）。

## 实际推送与核对

1. 推送前 `git fetch origin main`，`origin/main` 为 `24a3baa`；本地主线提交为 `9f98b67e3569aa2d405d9d677733f97e24277432`。
2. `git push origin main` 退出 0：`24a3baa..9f98b67  main -> main`（origin 为 SSH `git@github.com:loopkok/MORLA.git`）。
3. 推送后 `git rev-parse HEAD origin/main` 两行均为 `9f98b67e3569aa2d405d9d677733f97e24277432`。

本回执文件作为后续文档提交单独推送；其自身 SHA 由 Git 历史记录。CI 由推送触发，结果以 GitHub Actions 为准（本机没有 `gh`，未在此处粘贴 run id）。

## 边界

- EPG 断链结论不变：实测 **FAIL（操作者豁免）**。断链期间设备不停止，只有重连后的 `stop` 才停住；`device_watchdog_verified` 不得写成已验证事实，软件停止不替代物理急停/断电。
- 未完成：VR→EPG 真机全流程、Quest 扳机端口、EPG 夹持力/N 换算、小时级长稳/温漂、抱闸与位置计量学标定、CR5/Quest 实机停止与断网行为。
- 本轮删除的是重构新增的重复门禁，不是设备/固件层保护；操作者仍需自行确认外部急停、断电和机械限位。
