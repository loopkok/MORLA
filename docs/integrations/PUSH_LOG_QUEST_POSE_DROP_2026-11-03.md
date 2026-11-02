# Quest 位姿事件丢失 BUG 记录推送回执

日期：2026-11-03，Asia/Shanghai。仓库：[loopkok/MORLA](https://github.com/loopkok/MORLA)，分支 `main`。

## 远端同步

同步前本地 HEAD 与 `origin/main` 均为 `accf9c1`（`fix(input): keep the Quest controller event
JSON-serializable`），`git rev-list --left-right --count HEAD...origin/main` 为 `0 0`，
无远端新提交，无需合并。

## 本地更新

- 提交：[fd546b3f95d6cba09f61dd42c63d4ce8c6feea99](https://github.com/loopkok/MORLA/commit/fd546b3f95d6cba09f61dd42c63d4ce8c6feea99)
- 主题：`docs: document the Quest pose-event drop on the input side`
- 父提交：`accf9c1`，保留线性历史，未使用 rebase / force push。
- 范围：新增 [BUG.md](../../BUG.md)，131 行，无代码改动。

### 内容边界

只记录问题与证据，不包含任何修复：

- 现象：开始遥操报 `重锚失败 control_1: reanchor requires fresh bound input`。
- 现场 trace 统计：`right_pose` ≈14 Hz（16.7 s 内 241 次，17 段 >250 ms 空档，最大 1.022 s），
  同期 `right_hold` 90 Hz、`right_axes` ≈190 Hz；5 次重锚全部失败回滚。
- 头显与链路实测（已安装构建 `com.loopkok.morlaxr` `versionCode=8`、`1.1.0`、
  `lastUpdateTime=2026-11-02 15:41:35`）：独立 TCP 探针监听 `8000`，不经 ROS，
  六路数据全部 90.1 Hz，间隔中位 11.2 ms、p90 12.1 ms、最大 40.3 ms，
  无 >100 ms 空档；959 条右手柄位姿与 959 条按键行在 2 ms 内成对到达（100%）。
  结论：输入侧与 USB 链路正常。
- 根因：接收器每帧向同一话题连发 `pose` 与 `controller_state`，而输入层订阅为
  `BEST_EFFORT + KEEP_LAST depth=1`，先发的 `pose` 恒被覆盖；
  `QuestEventSource` 在收不到 `pose` 时模式为 `None`，后续 `controller_state` 一并丢弃。
- 复现（不依赖头显）：合成 90 Hz × 2 条事件，depth 1 订阅者测得 `pose` 0.1/s、
  `controller_state` 64.2/s；depth 20 两者均 64.2/s。真实 `RosInputManager` +
  `QuestEventSource` 下 depth 1 收到 0 帧、depth 16 收到 1292 帧且 100% 带 `right_pose`。
- 候选方向 5 条均标注为"未采用"，其中 depth 1 → 16 已在此前被否决；
  同时写明约束：任何缓冲的积压必须显著小于 `max_age_ns`（250 ms），
  否则缓冲事件会变成 `Queued/stale Quest event` 并触发断连。

## 推送结果

沿用既有 SSH remote（`git@github.com:loopkok/MORLA.git`），未做 URL 重写：

```bash
GIT_SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=10' git push origin main
```

推送退出 0，返回：

```text
To github.com:loopkok/MORLA.git
   accf9c1..fd546b3  main -> main
```

推送后 `git rev-parse HEAD` 与 `git ls-remote origin refs/heads/main` 均为
`fd546b3f95d6cba09f61dd42c63d4ce8c6feea99`。本回执作为后续独立文档提交推送，
其自身 SHA 以 Git 历史为准。

## 验证范围

已做：暂存区空白检查（`git diff --cached --check`）、HEAD 与远端引用一致性核对。

未做：未运行测试套件（本次只新增文档，不改代码）；未重复真机抓包；
未执行任何修复方案，`src/` 下文件与 `accf9c1` 逐字节相同。

## 未决事项

修复方向仍待决定，BUG.md §6 列出 5 个候选；按既定范围，本次不触碰
watchdog / 断链。
