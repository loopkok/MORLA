# Quest depth=1 快照修复提交与推送回执

日期：2026-11-03，Asia/Shanghai。仓库：[loopkok/MORLA](https://github.com/loopkok/MORLA)，分支 `main`。

## 提交与范围

- 提交前 fetch origin/main 成功，远端与本地基线均为 `884719f460d9abda00295c62cf52e9007153bff4`，无新增远端更新，无冲突。
- 功能提交：[95e94318afe10529163658c8db9e14db475defad](https://github.com/loopkok/MORLA/commit/95e94318afe10529163658c8db9e14db475defad)。
- 主题：`fix(input): preserve Quest state under depth-one delivery`。
- 范围：20 个文件，759 行新增、76 行删除；包含 v3 快照协议、接收器/解析器、字段样本去重、操作 journal/ACK、测试迁移与新增、README、CHANGELOG、BUG 状态和 [详细修复日志](QUEST_SNAPSHOT_DEPTH1_2026-11-03.md)。
- 保留 BEST_EFFORT/depth=1、现有 Web 端口与保存配置；接收器和 Web 需要配套更新重启，头显 APK 不因此改变。现场验收未关闭。

## 验证记录

实现完成后的最终软件回归为 336 passed、3 skipped、1 deselected；3 项跳过来自 hypothesis/Jodell SDK 缺失，排除项为已知手机输入基线失败。另有 4 项 ROS 消息替身边界检查通过。消息替身结果不表述为真实 ROS/DDS 或头显验收。

提交准备阶段只修订归档文字，没有继续改功能代码，未重复相同回归。15 个修改/新增 Python 文件的 AST 检查、测试脚本 bash 语法与暂存区 whitespace 检查通过。未提交临时替身/基准脚本、虚拟环境、数据和运行日志。远端 CI 未检查。

## 实际推送

沿用已可用的 SSH 通道，仅针对命令做 URL 重写，不修改持久 remote，不使用 force push：

```bash
GIT_SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=10' \
  git -c url.git@github.com:.insteadOf=https://github.com/ push origin main
```

推送退出 0，返回：

```text
To github.com:loopkok/MORLA
   884719f..95e9431  main -> main
```

推送后 HEAD 和 origin/main 均为 `95e94318afe10529163658c8db9e14db475defad`，工作区干净。本回执按仓库惯例作为后续文档提交推送，其自身 SHA 以 Git 历史为准；完成后另用 ls-remote 核对最终远端分支。
