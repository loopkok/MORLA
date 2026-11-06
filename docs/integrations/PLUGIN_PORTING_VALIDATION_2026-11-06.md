# 插件接入框架改造验收记录

日期：2026-11-06；基于 `71a13de`。本轮仅验证软件、fake、共享仿真和 ROS 主机安装，不连接实体机器人。实现与升级步骤见 [接入规范](PLUGIN_DRIVEN_PORTING.md)。

## 最终结果

- macOS 相关 Python 回归：252 passed、1 skipped（可选依赖），覆盖 FR3、核心契约、运行时、控制器、Web 和场景模型。
- TypeScript/Vite 构建通过；前端 21 项通过，包含陌生设备/控制器的声明驱动表单、默认值、能力及后端筛选。
- Ubuntu 22.04 / ROS 2 Humble 主机：`colcon build --symlink-install --packages-up-to fr3_mujoco_sim morla_cr5_driver morla_loophand_driver morla_epg_driver`，39 packages finished；最终 Web 包另构建通过。
- 主机 pure：366 passed；fake：34 passed；sim：21 passed。fake 层纳入新增 `test_plugin_discovery.py`，其中使用非品牌表机器人构建实际 MuJoCo 模型；三个层级有覆盖交集，不应简单相加当独立测试总数。
- 浏览器连接主机安装后的 Web：五个动态预设可见，FR3 控制器、速度/倍率及按住跟随正确显示；保存、重开并再次保存为 v2，预设仍正确指向 FR3。截图见证据目录。
- 浏览器验收发现保存同名装配会清空预设选择，已修复并在最终构建复验。初轮主机仅一项测试断言旧错误文案失败，更新为通用维度错误后完整 pure 通过；不是放宽拒绝规则。
- 原始 macOS pure 脚本因本机未安装 rclpy 无法收集 ROS 输入测试，因此完整 pure 在目标 ROS 主机执行。没有用跳过 ROS 测试冒充主机验收。

## 证据与范围

[证据目录](evidence/2026-11-06-plugin-porting/) 保存最终构建、pure/fake/sim、前端、本机相关测试和浏览器服务器日志，以及 [保存后截图](evidence/2026-11-06-plugin-porting/web-saved.png)。测试服务器已停止；浏览器仅编辑/编译测试装配，未启动实体设备。

本轮未重复执行上一轮实测 DDS 冒烟，也未做真机、实物 Quest、长期稳定性、碰撞安全或动力学参数识别。已有描述符更新后需要重新保存配置 revision；不修改历史数据。机械臂脚手架、完整插件安装自检命令、多独立会话及力控等不属于本轮已完成能力。

## 发布

本轮代码、文档、changelog 和证据在同一提交交付。提交哈希以 `git log` 中 `refactor(plugins): drive Web and simulation from descriptors` 为准；推送完成后的回执由交付消息给出，避免在提交中写自身哈希。
