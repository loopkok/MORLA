# MORLA 迁移进度

执行依据：`docs/MIGRATION_PLAN.md` 与 `docs/SOURCE_BASELINE.json`。
开始时间：2026-10-13（Asia/Shanghai）。
源基线：Astral `3a57d722d3c4257670d9f90526127bed77063e66`；XNero `e6a0100429024f04e18238f1a42ee968bb18967a`。

## 状态

| 阶段 | 状态 | 结果/证据 |
| --- | --- | --- |
| P0 独立副本与环境 | 进行中 | 新建 MORLA；从 Astral rsync 源码。待哈希与路径检查、进度初始化完成。 |
| P1 包路径与改名 | 未开始 | |
| P2 core/profile | 未开始 | |
| P3 description | 未开始 | |
| P4 unified input | 未开始 | |
| P5 Nero driver/teleop | 未开始 | |
| P6 XHand adapter/sim | 未开始 | |
| P7 unified data collection | 未开始 | |
| P8 policy/HITL | 未开始 | |
| P9 Web manager | 未开始 | |
| P10 software integration | 未开始 | |
| P11 hardware/model acceptance | 未开始 | |

## 日志

### 2026-10-13

- 已确认源 commit 与迁移基线匹配；两个源工作区的 `git status --short` 为空。
- 已在 SDK 根目录创建新的 `MORLA/`，运行 `rsync -a` 从 Astral 复制源码，排除build/install/log、缓存、node_modules、egg-info和运行数据。
- 尚未修改ROS包、description、配置或机器人接口；尚未启动硬件。
- 本地环境已确认 ROS 2 Humble；后续按系统 Python 3.10构建ROS包。
