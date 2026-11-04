# 冷机 EPG 接管修复提交与推送回执

日期：2026-11-04，Asia/Shanghai。仓库：[loopkok/MORLA](https://github.com/loopkok/MORLA)，分支 `fix/cr5-teleop-smoothness-and-web-errors`（非 `main`）。

## 提交与范围

- 提交：[2329e0fc57f005039fe047a6f6d732e52a4e8fcb](https://github.com/loopkok/MORLA/commit/2329e0fc57f005039fe047a6f6d732e52a4e8fcb)。
- 主题：`fix(workstation): validate device health, not positions, before enabling`。
- 基线：`2ba91fd`（`fix(cr5): integrate differential IK from the last command and expose ServoJ tuning`）。
- 范围：4 个文件，153 行新增、1 行删除——[handoff.py](../../src/common/morla_web_manager/morla_web_manager/handoff.py) 新增 `Handoff.health()` 并改由 `begin()` 调用，[test_workstation_operator.py](../../src/common/morla_web_manager/test/test_workstation_operator.py) 新增两条用例，`CHANGELOG.md` 与 [详细修复日志](COLD_EPG_HANDOFF_2026-11-04.md)。
- 只改“使能前”的前置校验；停止路径、`solver_tip_from_tool`、ServoJ 参数与前端代码均未改动，因此不需要重建 `web/dist`。

## 验证记录

环境：`/usr/bin/python3` 3.10、`source /opt/ros/humble/setup.bash`、`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。

| 门禁 | 命令 | 结果 |
|---|---|---|
| 相关单测 | `pytest -q src/common/morla_web_manager/test/test_workstation_operator.py` | **28 passed** |
| 组合层 | `scripts/test_composable.sh --layer fake` | **27 passed, 1 failed** |
| 纯逻辑层 | `scripts/test_composable.sh --layer pure` | **302 passed, 1 skipped** |

唯一失败项 `test_composable_api.py::test_mobile_input_and_recording`（`release_or_reanchor_required`）在 `HEAD=2ba91fd` 的独立 worktree 上同样失败（`1 failed, 7 passed`），确认是既有基线，不是本次改动引入。

真机会话 `~/.local/share/morla/logs/20261104T085804Z_cr5_real_quest_epg_991c0720ec76456286567176ad561cda` 显示：`prepare` → `teleop_start` → `activate`（约 5.2 s，臂松闸 + 夹爪使能）→ `committed` 在同一次请求内完成，全程未手动使能夹爪。现场手感验收（旋转灵敏度、平移跟手）未包含在本次提交内，仍待调参复测。

## 实际推送

```bash
git push origin fix/cr5-teleop-smoothness-and-web-errors
```

推送退出 0，返回：

```text
To github.com:loopkok/MORLA.git
   2ba91fd..2329e0f  fix/cr5-teleop-smoothness-and-web-errors -> fix/cr5-teleop-smoothness-and-web-errors
```

推送后本地 `HEAD` 与 `origin/fix/cr5-teleop-smoothness-and-web-errors` 均为 `2329e0fc57f005039fe047a6f6d732e52a4e8fcb`，工作区干净（本回执按仓库惯例作为后续文档提交推送，其自身 SHA 以 Git 历史为准）。未使用 force push，未改动 remote 配置。远端 CI 未检查。

## 后续：合入 main（2026-11-04 19:1x CST）

操作员指示「后续都直接推送到main分支」。据此把 `main` 从 `29e4ca6` **fast-forward** 到 `e8ec17a`（3 个提交，无 merge commit、无冲突），并推送：

```text
To github.com:loopkok/MORLA.git
   29e4ca6..e8ec17a  main -> main
```

第一次推送被 GitHub 拒绝（`remote rejected main -> main (Internal Server Error)`），同一时刻 `ssh -T git@github.com` 报 `Permission denied (publickey)`，而 [githubstatus.com](https://www.githubstatus.com) 显示 **Minor Service Outage**、Git Operations 为 `degraded_performance`。属远端故障，不是密钥或权限问题（`ssh -v` 显示 `Server accepts key`，签名阶段失败）；重试第二次成功。此后 `main`、`origin/main` 与功能分支均为 `e8ec17a`，工作区干净。

