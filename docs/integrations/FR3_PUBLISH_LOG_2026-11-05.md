# FR3 实施与发布日志 — 2026-11-05

本轮按用户要求提交并推送本地 FR3 新增内容。仓库为 `https://github.com/loopkok/MORLA`，目标为当前 `main` 分支。
提交前基线为 `70ee93fd45b3983992f0eb438c80cf060066dcdd`；fetch 后本地与 `origin/main` 无领先或落后。

## 变更范围

- 新增独立 `morla_fr3_driver` ROS 2 / MORLA 包、厂商 SDK 原样源码、许可证与来源摘要。
- 实现 SDK 子进程超时隔离、CNDE 反馈采样时间、六轴命令校验/限位/限速、显式使能与停止、标准 FR3 本地运动学。
- 增加 fake / real 示例、独立 driver / Web launch、只读检查命令和专项测试入口。
- 接入公共 Web 组件选择、FR3/Quest fake 预设及控制器匹配，同步前端 dist。
- 更新根 README、文档索引、CHANGELOG 与 FR3 接入说明；修正公共 Web 测试中与现行显式 deadman 行为不符的旧断言。

## 验证记录

开发轮已完成：FR3 34 项；相关 core/runtime/teleop/CR5/FR3 回归 252 passed / 1 skipped；公共 Web 10 项；前端 17 项和 TypeScript/Vite 构建；Python wheel 安装、ament 静态发现及两个配置的 Runtime 生命周期；实际 localhost HTTP 冒烟。
本次提交前另行复验 FR3 专项，并执行 `git diff --check`。验证计数有包含关系，不能相加。

当前未接机械臂，用户将自行进行后续真机测试。Mac 无 ROS 2/colcon，ROS 构建/IDL/DDS、真实 SDK 网络、实体 FR3/Quest、物理停止/断网、长稳、碰撞及模型效果均未验收。浏览器目视检查此前因锁屏和初始化超时受阻。详细依据、命令及限制见 [接入与验收记录](fr3.md)。

## 发布记录

功能提交和推送后追加实际 SHA 与远端核验结果。本日志首次随功能提交保存，当前段落不预先宣称推送成功。
