# MMK2 整机仿真更新与发布日志

日期：2026-11-10（Asia/Shanghai）。本轮更新起点及 fetch 后的远端基线均为 `f2166a56475b980f791f1ea9998f6fd92445daa2`，当前分支为 `main`，仓库为 `https://github.com/loopkok/MORLA`。用户已明确授权提交和推送本轮本地更新。

## 更新范围

- 新增机器人专属 `mmk2_description`、`mmk2_mujoco_sim`，覆盖整机模型、静态组件注册、场景工厂、双臂运动学、底盘差速控制器、ROS 包/launch/CLI 与 Web 模板。
- 从用户提供的完整 URDF/网格构建运行模型；保留原始 URDF、模型许可证及 SHA-256。SDK 提供的 MuJoCo XML 缺少网格，且与 URDF 的部分关节约定不同，原样保留用于对照，不作为可运行的 SDK 仿真冒充验收。
- 公共层增加 descriptor 选择的可信场景工厂；修正速度动作语义、输入绑定输出的单位/模式与逐端口采样时间，保留无 ports 的旧测试/嵌入式夹具兼容。
- 七组执行组件、四个 RGB-D 视角与综合传感器共用一个 MuJoCo 场景。底盘有独立轮速刷新超时，位置组停止保持末次目标；修正镜头外参及完整机器人安装旋转链。
- 新增 Web 测试输入宿主、录制只读核验脚本、21 项 MMK2 测试、模型/插件接入说明及实际浏览器证据。
- 更新根 CHANGELOG、文档索引与本发布日志。

## 验证与证据

- 联合回归：**355 passed、1 skipped**，跳过项因未安装可选 Hypothesis。范围为 MMK2、FR3/CR5、公共 runtime、core contracts、teleop 和 Web composable/operator/plugin。日志见 [regression.log](evidence/2026-11-09-mmk2-web/regression.log)。代码验证完成后，本轮发布准备只补充文档和提交记录，不重复运行同一测试集。
- 实际浏览器：从空工作空间逐步配置，检查保存、启动、七组遥操、录制保存、Grip 释放、暂停、急停/复位、模式切换和停止均完成。最终工作空间为 `mmk2_quest_whole_sim` v2。详情见 [Web 验收报告](MMK2_WEB_ACCEPTANCE_2026-11-10.md)。
- 运动学片段约 26.4 秒、动力学片段约 46.7 秒，均为 complete；七组执行状态有变化，19 维状态/动作、三路 JPEG 与辅助传感器均经只读核验。最终动力学会话有 1560 次完整七组命令回执，停止状态为 STOPPED、清理错误为空。
- 发布前检查工作树、文件范围、来源/许可和差异格式；模型资产合计约 30 MB，原始 HDF5、约 89 MB 的单次运行日志、临时测试存储及进程不纳入提交。提交精简后的 JSON 摘要、冻结配置、截图及测试输出。
- 本机图像实际录制约 6–8 fps，低于配置 15 fps。真实 Quest/机器人、ROS 主机 colcon/DDS、碰撞及轮地接触未验收；深度数据集导出与整机速度动作策略/回放尚未支持。

## 发布过程

1. 已执行 `git fetch origin`，本地 HEAD 与 origin/main 相同，无需合并远端更新。
2. 将本日志、CHANGELOG、实现和验收证据作为实现提交；使用普通推送更新 `origin/main`，不改写既有历史。
3. 推送完成后读取远端 ref 核验，并以单独文档提交记录实现提交号和实际推送结果，避免日志自引用。

## 实际发布结果

- 实现与验收证据提交：`7a5aa55833004b494656cde100b53be689f66c0f`，标题 `feat(mmk2): integrate whole-body MuJoCo simulation and Web validation`。共 105 个文件；大量新增行来自约 30 MB 的文本 OBJ 模型资源，不是原始运行数据。
- 首次 `git push origin HEAD:main` 因 HTTPS 终端缺少可用凭据失败：`could not read Username for 'https://github.com': Device not configured`，未因此修改用户 Git 配置。
- 随后使用已有 SSH 认证执行普通推送 `git push git@github.com:loopkok/MORLA.git HEAD:main`，Git 返回 `f2166a5..7a5aa55 HEAD -> main`，成功更新同一个仓库的 main 分支。
- HTTPS `git ls-remote origin refs/heads/main` 返回完整实现提交号 `7a5aa55833004b494656cde100b53be689f66c0f`；随后 `git fetch origin` 确认 origin/main 相同。
- 本节作为独立文档收尾提交再次推送。它的最终提交号以 Git 历史及远端 main 为准，不在自身文件中自引用；最终远端核验结果在交付回复中报告。
