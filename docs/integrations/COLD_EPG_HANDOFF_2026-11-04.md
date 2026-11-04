# 冷机 EPG 无法“开始遥操”：接管前置校验改为设备健康

日期：2026-11-04。范围：Web 工作站（`morla_web_manager`）的接管事务 `Handoff`，只影响**使能前**的前置校验；真机 CR5 + EPG50 现场会话复测通过。

## 症状

真机装配（`cr5_real_quest_epg`，arm = `morla.dobot.cr5`，gripper = `morla.jodell.epg50_060`）刚上电、夹爪尚未使能时，在 Web 点「开始遥操」直接被拒：

```
接管需要新鲜且有效的设备反馈：gripper
```

先手动让夹爪使能一次，再点「开始遥操」就正常。冷机永远起不来，等于必须绕过 Web 先“预热”设备。

## 根因

`Handoff.begin()` 在**发起设备激活之前**调用了 `feedback()`，要求每个执行器都给出“与关节数一致、全部有限”的位置值（[handoff.py:25-36](../../src/common/morla_web_manager/morla_web_manager/handoff.py#L25-L36)）。

这条要求对 CR5 成立（上电即可回报关节角），对 EPG 不成立：EPG 只有在 `enabled and activation_state == 3`（gSTA=3）之后才回报夹爪宽度，而**“开始遥操”正是要发起这次使能**（`teleop_start` → `activate`）。于是请求在使能之前就被自己的前置校验拦下，永远走不到使能那一步。

EPG 适配器是**故意**不回传禁用态位置（[adapter.py](../../src/robots/jodell/morla_epg_driver/morla_epg_driver/adapter.py)）：禁用期间读到的宽度可能是 0 或 254 这类“看起来合理”的默认值，回传只会让上层拿一个假位置去建锚。因此修法不能是“让设备早点报位置”，只能是把“使能前”和“使能后”的校验要求分开。

## 修法

新增 `Handoff.health()`：只校验**可达性与新鲜度**——`ready is True`、无 `fault`、`sample_ns` 是整数且在 `max_age_ns` 内；`begin()` 改用它，`seeds` 留空。

位置校验并没有被放弃，只是挪到了它成立的位置：

| 阶段 | 校验 | 失败文案 |
|---|---|---|
| `begin()`（使能前） | `health()`：可达 + 新鲜 | 接管需要新鲜且健康的设备状态：`key` |
| `poll()` 激活完成后 `stage_prepared` | `feedback()`：可达 + 新鲜 + 位置向量合法 | 接管需要新鲜且有效的设备反馈：`key` |
| `_validate()` / `_commit()` 前两次 | `feedback()` + 重锚窗口内的姿态一致性 | 重锚期间设备姿态已改变，请重新开始：`key` |

因此没有放松安全性：故障设备（`ready=False` 或有 `fault`）仍在使能之前被拒（HTTP 422），不会出现“先使能、再回滚”；使能后位置不对或姿态已变，依旧拒绝接管。

## 测试日志

环境：Ubuntu、`/usr/bin/python3`（3.10）、`source /opt/ros/humble/setup.bash`、`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。

```bash
# 直接与本改动相关的用例
/usr/bin/python3 -m pytest -q src/common/morla_web_manager/test/test_workstation_operator.py
# 28 passed, 1 warning in 3.48s

# 组合层（含 Web API / 数采 / 策略）
bash scripts/test_composable.sh --layer fake
# 1 failed, 27 passed, 3 warnings in 3.89s

# 纯逻辑层（core/runtime/teleop/cr5/epg/input 等）
bash scripts/test_composable.sh --layer pure
# 302 passed, 1 skipped, 1 warning in 13.14s
```

新增两条用例（都在 `test_workstation_operator.py`）：

1. `test_disabled_device_without_position_can_be_enabled_by_teleop_start` —— 模拟“真实 EPG”：`rt.status != 'ACTIVE'` 时 `health()['values']` 为空。断言 `teleop_start` 返回 200、会话进入 `ACTIVE`、`operator.activity == 'human'`，并且**使能之后**的输入能产生目标（`grip.values == [0.4]`）。修复前这条用例在 `begin()` 就抛 “接管需要新鲜且有效的设备反馈”。
   - 注意用例里发了两次输入：`morla.input.quest` 在按下握持键的那一帧自带 `teleop_start`，该帧会重新建锚，因此**不产生指令**；从下一帧起才正常。这既是测试口径，也是现场“按下后第一帧不动”的解释。
2. `test_unhealthy_device_still_blocks_teleop_start_before_enabling` —— `ready=False` + `fault='bus down'` 的设备仍在 `teleop_start` 时返回 422（`detail` 含“设备状态”），且 `rt.status != 'ACTIVE'`，即故障设备不会被“先使能再回滚”。

### 既有失败基线（与本改动无关）

`test_composable_api.py::test_mobile_input_and_recording` 在 fake 层失败：

```
AssertionError: {'accepted': False, 'reason': 'release_or_reanchor_required'}
```

在 `HEAD`（`2ba91fd`）的独立 worktree 上以同样方式失败（`1 failed, 7 passed`），确认是既有基线，不是本次修复引入。原因属于数采录制在“已激活但未重新建锚”状态下的准入判定，另案处理。

## 真机证据

会话 `~/.local/share/morla/logs/20261104T085804Z_cr5_real_quest_epg_991c0720ec76456286567176ad561cda`（83 s，含 CR5 + EPG 真机）：

| 时刻（相对 run 起点） | 事件 |
|---|---|
| 20.7 s | `prepare` SUCCEEDED（arm、gripper） |
| 27.4 s | handoff `requested`（source=paused → destination=human），`authority_granted`（arm、gripper） |
| 32.6 s | `activate` SUCCEEDED（臂松闸 + 夹爪使能，约 5.2 s） |
| 32.6 s | `binding_ack`：`control_2` 回报夹爪种子 `[0.05]`，`control_1` 回报 `current_tcp` |
| 33.5 s | handoff `committed`，`human_mobile` 获得授权，开始下发指令 |

即：**同一次「开始遥操」里完成“使能 → 采样真实位置 → 建锚 → 提交”**，全程没有再手动使能夹爪。

需要说明的边界：trace 只记录激活**之后**的种子值，不记录使能前的健康状况，因此上述真机记录证明的是“修后链路端到端可用”，不能单独证明“那一拍夹爪确实处于禁用态”。冷机语义由上面的单元测试覆盖（禁用态 `values` 为空仍可启动）。

## 现场注意事项

- 按下握持键的第一帧不产生指令（该帧自带 `teleop_start` 并重新建锚），从第二帧开始跟手。这是 `quest_events.actions` 的既有设计，不是本次改动引入。
- 本次修复只动 `begin()` 的前置校验，**不涉及**停止路径。既有问题“回到工作位之后 EPG `stop()` 1 s 内未确认 gGTO 清零 → `STOP_TIMEOUT` → 会话进入 `FAULTED`、后续动作全部被拒”仍然存在，另案处理。
- 该修复改变了 `begin()` 的失败文案：设备不可达/不新鲜时提示“接管需要新鲜且健康的设备状态：`key`”，不再是“…有效的设备反馈”。排查时按文案区分是“使能前”还是“使能后”的失败。

## 回滚

`git revert` 本提交即可。回滚后冷机 EPG 恢复“必须先手动使能才能开始遥操”的行为；其它路径（CR5 单臂、仿真、fake）不受影响。
