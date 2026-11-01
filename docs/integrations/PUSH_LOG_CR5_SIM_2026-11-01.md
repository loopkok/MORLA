# CR5 MuJoCo / Web 提交与推送回执

日期：2026-11-01，Asia/Shanghai。仓库 [loopkok/MORLA](https://github.com/loopkok/MORLA)，分支 `main`。未 force push，未重写已有提交。

## 功能与实施日志提交

- 起点：`9272b7f3805f92d05787530e655b3969b61a93d4`。
- 提交：`e222e3545ce2209dfefc9bbbe6d6c64401d664cc`。
- 主题：`feat(cr5): add MuJoCo simulation and web integration`。
- 范围：23 个文件，597 行新增、62 行删除；包含模型生成器与 MJCF、统一物理后端接入、sim 装配与 launch、Web 模板/预览/会话恢复、测试、前端构建产物、README、CHANGELOG 和实施日志。
- 验证：相关 Python 回归 43 passed；TypeScript 检查与 Vite 构建通过；实际离屏渲染成功；提交前 `git diff --check` 无输出。详细测试范围、复现方式与未执行项见 [实施日志](CR5_MUJOCO_2026-11-01.md)。本轮补充文档后未重复运行相同代码测试。

## 远程同步

本机到 GitHub HTTPS 443 连接超时，初始 fetch 未完成。停止本轮挂起的抓取后，改用 SSH；仅对命令设置 URL 重写，没有修改仓库持久 remote 配置：

```bash
GIT_SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=10' \
  git -c url.git@github.com:.insteadOf=https://github.com/ fetch origin main
GIT_SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=10' \
  git -c url.git@github.com:.insteadOf=https://github.com/ push origin main
```

抓取成功，远程无新增提交。推送退出 0，实际返回：

```text
To github.com:loopkok/MORLA
   9272b7f..e222e35  main -> main
```

推送后 `git rev-parse HEAD origin/main` 两行均为 `e222e3545ce2209dfefc9bbbe6d6c64401d664cc`。

本回执沿用仓库惯例，作为后续文档提交独立提交并推送；回执自身的 SHA 以 Git 历史为准。推送触发的远程 CI 未在本轮检查，不将本地测试结果表述为 CI 通过。

## 保留边界

ROS Humble colcon/launch、实体 Quest/Vader、真实 CR5、长稳和碰撞验收未执行。现有 IK 失败导致会话撤权及输入 reader 退出感知问题未修复。本次提供仿真诊断环境，不代表原实体手柄故障已关闭。
