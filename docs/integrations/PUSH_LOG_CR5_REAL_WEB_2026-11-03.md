# CR5 真机操作手册与跟随重锚修复推送回执

日期：2026-11-03，Asia/Shanghai。仓库：[loopkok/MORLA](https://github.com/loopkok/MORLA)，分支 main。

提交前 fetch 成功，远端与本地基线均为 e183d36，没有新增远端提交或冲突；未修改持久 remote，未重写历史。

## 功能和文档提交

- 提交：[d0a596ccd2c73a42ff37ffd0854b2ba9e97257cf](https://github.com/loopkok/MORLA/commit/d0a596ccd2c73a42ff37ffd0854b2ba9e97257cf)。
- 主题：fix(workstation): honor continuous follow and stage reanchors after builds。
- 范围：9 个文件，363 行新增、23 行删除。
- 包含 Web 空 deadman 持续跟随修复、所有控制器构造完成后统一重锚、回归测试、CR5 文档更正、CHANGELOG、[修复日志](WORKSTATION_FOLLOW_REANCHOR_2026-11-03.md)及 [CR5 真机完整操作手册](CR5_REAL_WEB_OPERATION_2026-11-03.md)。
- 操作手册按反馈/使能/失能、小幅关节插补、Quest 右手柄三阶段编写；实际设备限位、零点、TCP、负载与停止能力必须现场核对。

## 验证与边界

实现结束时的最终回归为 343 passed、3 skipped、1 deselected；依赖跳过为 hypothesis 和两项 Jodell SDK，排除项是既有手机输入基线失败。此后提交准备只补操作文档及日志，未重跑同一功能套件。再次执行 3 个修改 Python 文件的 AST、操作手册链接和暂存区 whitespace 检查，均通过。

没有执行真机使能/失能、运动、Quest 实体遥操或完整 Linux ROS 验收。未修改前端源码或 web/dist。没有提交临时测试工具、依赖环境和运行数据，也没有把推送成功表述为 CI 通过。

## 推送

通过已可用的 SSH 通道正常推送 origin/main，命令级 URL 重写，退出 0：

```text
To github.com:loopkok/MORLA
   e183d36..d0a596c  main -> main
```

功能推送后 HEAD 与 origin/main 均为 d0a596ccd2c73a42ff37ffd0854b2ba9e97257cf，工作区干净。本回执另作独立文档提交推送，自身 SHA 以 Git 历史为准；最终分支再通过 ls-remote 核对。
