# Astral 离线数据处理

> 2026-10-30：新组合采集冻结 Schema v3，state/action 分开、相机/辅助输入/传感器具名；旧 v2 数据与下文处理步骤继续兼容。输入滤波或设备组合改变后，须核对实际 dispatch 标签、单位与端口时间，不能靠相同维度猜映射。见[接入与数据契约](../../../../../docs/device_integration_guide.md)和[主机数据测试](../../../../../docs/testing/host_build_and_hardware_acceptance.md)。

所有离线处理实现集中在本目录。采集节点、raw 写盘、schema 留在父包；视频采集仍属于 quest3_video_streamer。旧数据脚本、父包离线模块及旧 ROS 离线命令已删除，统一使用 data_pipeline。

从 sdk 目录使用统一入口；可在任意 cwd 通过绝对路径调用：

```bash
bash MORLA/scripts/data_pipeline.sh --help
bash MORLA/scripts/data_pipeline.sh repair --help
```

默认使用 sdk/VLA/openpi/.venv/bin/python，路径按脚本位置推导。其他环境：

```bash
ASTRAL_DATA_PYTHON=/path/to/python bash MORLA/scripts/data_pipeline.sh --help
```

安装或 colcon 重建并 source 后，也可以使用 `data_pipeline <command>` / `ros2 run morla_data_collect data_pipeline <command>`。离线 ROS 命令只保留 `data_pipeline`。离线命令需要相应环境的 numpy/h5py/cv2/pyarrow/av；回放另外需要 rerun-sdk。顶层帮助不加载这些可选依赖。

## 文件与命令

| 命令 | 实现文件 | 用途 |
| --- | --- | --- |
| prepare | prepare_session.py | 原始数据复制到新工作目录，可选段号与相机映射 |
| preview | preview_session.py | 生成浏览器图像抽查与状态曲线 |
| quantify | quantify_cmd_state.py | raw cmd/state 诊断；--aligned 模式仅按 next_state 解释 |
| align | align_data.py | raw → aligned_data.h5 |
| validate | validate_data.py | 数据体检，写报告；--apply 移动失败段 |
| repair | repair_aligned.py | 可选保守清洗，默认保护意图；先 --dry-run |
| replay | replay_rerun.py | 人工检查任务、标签、同步与动作 |
| export-openpi | convert_to_lerobot.py | aligned session → LeRobot v2.1 |
| export-act | convert_to_act.py | session 或 v2.1 → v3 + 自检 |
| upgrade-v3 | convert_to_lerobot_v3.py | 单独升级已有 v2.1 |
| verify-aligned | verify_aligned.py | 按来源索引逐帧核对 state/图像/时间戳/action |
| verify-existing | verify_existing_data_pipeline.py | raw 只读检查 + 独立样本对齐/清洗/导出验证 |
| migrate-cameras | migrate_camera_labels.py | 历史原地迁移工具；先 --dry-run，不替代 schema 校验 |
| process-openpi / process-act | process.sh | 共用一键编排：对齐 → 校验/隔离 → 导出 |

cli.py / __main__.py 是统一命令分发入口；artifacts.py 提供暂存提交、路径隔离和来源签名。扩展新命令时把实现加到本目录并注册 cli.COMMANDS，不再往仓库 scripts/ 添加独立业务实现。

## 从 raw 到训练输入

在工作副本处理，保留原始 raw。基础校验 → 可选 quantify → align → validate + 回放 → 可选 repair → 再校验/回放/verify-aligned → 导出 → 实际训练环境加载验证 → 配置数据集并计算 norm stats → 训练。

复用已有 `aligned_data.h5` 时必须有与 raw 匹配的 `raw_signature`；旧版无签名对齐缓存需在工作副本执行 `align --force` 重建，不能直接用于训练导出。

```bash
# 下列示例从 sdk 目录运行，pick_place 是已准备好的工作副本。
bash MORLA/scripts/data_pipeline.sh validate --session astral_data/work/pick_place
bash MORLA/scripts/data_pipeline.sh quantify --h5 astral_data/work/pick_place --out astral_data/qa/cmd_state.json
bash MORLA/scripts/data_pipeline.sh align --session astral_data/work/pick_place
bash MORLA/scripts/data_pipeline.sh replay --session astral_data/work/pick_place --episode 0

# 可选清洗：输出目录必须不存在；输入必须已对齐、校验。
bash MORLA/scripts/data_pipeline.sh repair --session astral_data/work/pick_place --dry-run
bash MORLA/scripts/data_pipeline.sh repair --session astral_data/work/pick_place --out-session astral_data/clean/pick_place
bash MORLA/scripts/data_pipeline.sh validate --session astral_data/clean/pick_place
bash MORLA/scripts/data_pipeline.sh verify-aligned --source astral_data/work/pick_place --session astral_data/clean/pick_place

# 不需要清洗时，把下面输入改成工作 session。
bash MORLA/scripts/data_pipeline.sh process-openpi astral_data/clean/pick_place astral_data/pi/pick_place
bash MORLA/scripts/data_pipeline.sh process-act astral_data/clean/pick_place astral_data/act/pick_place \
  --check-python /home/robot/miniconda3/envs/lerobot/bin/python
```

一键流程不自动执行 quantify/repair，也不代替人工任务评审。默认只报告失败段，不移动输入数据；明确指定 `--quarantine` 才会把失败段移动到输入 session/quarantine/。校验失败会中止导出，失败段仍留在原目录。对齐失败会立即中止，需处理后重跑。清洗后不要 --force-align，否则从 raw 重建会覆盖清洗结果。

输入 raw 文件保持不变，但 align 在输入 episode 内新建 aligned，validate 在 session 写报告。repair 新建 session；导出新建或按既有覆盖规则替换输出目录。历史相机命名/schema 不一致应在副本显式修正并重建；旧 migration 工具只处理名称，不可作为完整修复入口。

## 入口约定

- 源码运行：`bash MORLA/scripts/data_pipeline.sh <command>`。
- 安装后：`data_pipeline <command>` 或 `ros2 run morla_data_collect data_pipeline <command>`。
- Python 实现只位于 `morla_data_collect.pipeline`；旧父包模块及 scripts/ 数据工具已删除。
- 来源验证须显式给 `--source` 和 `--session`；空 session 会失败。
- scripts/openpi_train.sh / act_train.sh 属于下游训练工具，继续留在仓库 scripts/。

Web 入口：morla_web_manager 的「数据处理」Tab 调用本 CLI，在独立工作副本执行完整流程或单步，显示报告与浏览器预览。prepare 不修改原始源，preview 是每段首/中/末帧抽查，并非完整视频回放。

## 人在环路标签

采集节点把 `/policy_inference/state` 的所有权事件写入 episode `meta.json`；`align` 将 `intervention`、
`intervention_valid`、`control_owner` 对齐到帧，`repair` 按来源帧索引同步筛选，v2.1 与 v3 导出都把
三列保留在 Parquet 中。历史 episode 没有所有权事件时标成 `control_owner=unknown`、
`intervention_valid=false`，不把未知片段误当作策略示范或人工接管。合成 hold 帧也不会标成人工动作。

`control_owner` 编码：0 unknown、1 policy、2 human、3 playback、4 pending、5 idle；`intervention=true`
只表示已确认的 HUMAN 帧。未知标签应在新数据采集时补上事件，不要根据旧动作轨迹推断。
