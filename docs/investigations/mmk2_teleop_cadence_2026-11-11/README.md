# MMK2 遥操节奏核查（2026-11-11）

操作员问题：MMK2 遥操"一卡一卡"，FR3/CR5 同样是 6 轴却不这样，差别在哪。

本目录是当时的实测证据。原始日志（每轮约 450 MB 的 `trace-*.jsonl`）在
`~/.local/share/morla/logs/`，**不在仓库里**；本目录存的是可复核的压缩数据
（`data/`，共 2.7 MB）、测量脚本和脚本输出原文。换一台电脑不需要 trace 也能
读结论、看曲线、核对每个数字（见文末"在另一台电脑上怎么看"）。

## 结论

1. **头显一侧没有降频。** 接收器每解析到一只手的位姿或 landmarks 就把该手的
   `sequence` 加一（`src/common/morla_input/morla_input/quest_snapshot.py:131`）。
   四轮运行里，左/右手的 `sequence` 都稳定在 **180/s**，换算就是每只手
   **90 个位姿样本/秒**，卡顿那一轮也不例外。
2. **卡住的是引擎摄取。** 每个新样本在 `_ingest_frame` 的锁内同步跑一次 IK
   （`composable_service.py:424` → `engine.targets` → `controller.step`），
   MMK2 的单次解算 p50 是 **12.4 / 13.4 ms**，串行起来上限约
   **1 / 12.8 ms ≈ 78 条/秒**；两条流分摊后引擎只拿到 **39.5 / 33.5** 个位姿/秒。
3. **丢掉的部分没人告警。** 订阅用 BEST_EFFORT + depth=1
   （`builtin_sources.py:47`），处理不过来时 DDS 直接只留最新一条；
   于是 `seq/pose` 从 2.00 涨到 **4.56 / 5.37**——每收到一个位姿，接收端其实已经更新过 4–5 次。
4. **FR3 没有这个问题，因为它的 IK 快。** FR3 解算 p50 **2.8 ms**，
   90 个样本/秒只占 0.25 s/s，`seq/pose` 保持 **2.00**，指令 89.9 Hz。
5. 结果就是手感：MMK2 指令 35.6 / 29.3 Hz、每拍关节 0.015 rad；
   FR3 89.9 Hz、每拍 0.0070 rad。两者平均速度接近（0.53 vs 0.63 rad/s），
   **台阶粗 5–6 倍**，所以是"一格一格"，不是"慢"。

## 证据一：接收端交付 vs 引擎消费（`results/rates.txt`）

`sequence` 由接收器在解析完 APK 数据后自增（位姿一次、landmarks 一次），
所以它衡量"APK 交给 MORLA 多少"，与引擎快慢无关。`seq/pose` 才是
"接收器发布 → 引擎拿到"这一跳有没有丢：2.00 = 全收，>2 = 有丢。

| 运行 | 手的位姿进引擎 | 接收端 sequence | seq/pose | 下发指令 |
|---|---|---|---|---|
| `20261111T052027Z` MMK2 `side: both`（报卡的那轮） | 左 39.5 Hz / 右 33.5 Hz | 180.0 / 180.0 | **4.56 / 5.37** | 35.6 / 29.3 Hz |
| `20261111T053601Z` MMK2 `side: right`（空跑） | 右 89.4 Hz | 180.1 | **2.01** | 无 |
| `20261106T090825Z` FR3 `side: right` | 右 89.9 Hz | 180.0 | **2.00** | 89.9 Hz |
| `20261106T091451Z` FR3 `side: right` | 右 89.9 Hz | 180.0 | **2.00** | 90.0 Hz |

同一轮 MMK2 里的三段对照（`data/<run>/rates_per_second.csv`，按秒统计）：

| 时段 | 引擎收到的左/右位姿 | 指令 |
|---|---|---|
| 0–4 s（未按 deadman，引擎空转） | 90 / 90 | 0 |
| 6–89 s（正常遥操） | 36 / 29 | 与样本 1:1 |
| 90–91 s（松手，指令归零） | 58 / 52 → 50 / 50，仍在回升 | 24 / 13 → 0 |

不跑 IK 时能全速收（261 帧/s），一开始解算就掉到 ~78 帧/s，松手又回升——
这与"摄取被解算耗时卡住"一致，而不是头显发得慢。

## 证据二：控制级手感（`results/control.txt`）

| 指标 | MMK2 左 | MMK2 右 | FR3 |
|---|---|---|---|
| 指令率 | 35.6 Hz | 29.3 Hz | 89.9 Hz |
| 指令间隔 p50 / p95 | 28.4 / 45.4 ms | 33.9 / 54.9 ms | 11.1 / 12.8 ms |
| 每拍关节步长 p50 / p95 | 0.0149 / 0.0321 | 0.0156 / 0.0378 | 0.0070 / 0.0082 |
| 每拍末端跳变 p50 / p95 | 2.3 / 8.7 mm | 2.2 / 9.9 mm | — |
| 解算耗时 p50 / p95 | 12.4 / 24.4 ms | 13.4 / 26.7 ms | 2.8 / 3.6 ms |
| candidate age p50 / p95 | 35.3 / 55.8 ms | 39.6 / 60.5 ms | 11.5 / 13.2 ms |
| σ_min p50 | 0.031 | 0.032 | 0.111 |
| 阻尼 p50 / p95 | 0.025 / 0.104 | 0.024 / 0.115 | 0.005 / 0.005 |

读法：

- 每拍步长 ÷ 间隔 = 0.53 / 0.46 rad/s（MMK2）对 0.63 rad/s（FR3）：平均速度相当，
  台阶粗 5–6 倍。
- 解算 12–13 ms 是整条链的节拍器：它既是丢弃率的来源（1/12.8 ms ≈ 78 条/s），
  也是 candidate age 的来源（MMK2 35–40 ms，FR3 11.5 ms）。
- σ_min 0.031、阻尼 0.025：MMK2 的初值位形 `[0,-0.6,1,±1.6,∓0.6,0]` 在奇异附近，
  会让部分拍被回溯砍步，是叠加因素。FR3 的阻尼一直钉在 0.005 下限。
- `raw pos err` 两边算法不同（不同 solver 的定义），**不要横向比较**。

## 证据三：配置差异（`results/config.txt`）

同一批 trace 对应的 manifest：

| | FR3 | MMK2 |
|---|---|---|
| Quest 实例 `side` | `right` | `both` |
| Quest 实例 `rate_hz` | 100 | 100 |
| position_scale | 0.8 | 0.8 |
| smoothing_alpha | 0.6 | 0.6 |
| max_joint_step | 0.03 | 0.15 |
| max_joint_velocity | 1（设备上限 `max_velocity_rad_s` 0.65 生效） | 0.8（仿真设备默认 3.14，不生效） |
| command_lead_rad | 0.08（设备上限 `max_command_lead_rad` 0.3） | 0.15（设备默认 0.15） |
| IK | 内置解析求解 2.8 ms | `morla.mmk2.cartesian_delta` 数值解 12.4 ms |

即：输入侧的绑定参数几乎一样，真正的差别是**解算器**和它带来的摄取上限。
`smoothing_alpha = 0.6` 的时间常数是 2.5 个控制周期——90 Hz 下 28 ms，
35 Hz 下 71 ms，同样的数值在 MMK2 上多滞后约 40 ms，是次生效应。

## 已排除

- **不是头显降频/丢包**：接收端 `sequence` 四轮都是 180/s，与是否遥操无关。
- **不是引擎在门控**：`control_gate` 只解释指令比样本少的那部分（deadman/来源缺席），
  不解释每拍之间丢掉的样本；丢发生在到达 `targets()` 之前。
- **不是 CR5/FR3 的绑定参数更宽容**：两边的 position_scale / smoothing 相同，
  差别在解算耗时。

## 证据边界

- `sequence` 是接收器自增的计数，不是头显时钟。它能证明"APK 交给接收器多少"，
  不能证明 APK 内部有没有再丢采样（那需要头显侧日志）。本轮的结论只需要前者。
- 丢弃率是由"发布速率 vs 摄取速率"推出来的；DDS 层没有计数接口，
  所以具体丢了多少条无法直接读出，只能由 `seq/pose` 反推。
- `20261111T053601Z` 是 17 秒空跑（未按 deadman、无指令），只用它做"无解算负担"对照。
- 本轮只覆盖仿真 + 上述几条运行；MMK2 真机（`mmk2_sdk`）不在其中。
- 真实的操作手感仍需现场复测；本目录只解释节奏。

## 在另一台电脑上怎么看

不需要 trace，也不需要 Python：

1. **打开 `report.html`**（双击即可，纯静态、无外部依赖）：结论、曲线、
   逐秒数据都在里面。
2. **用表格软件打开 `data/<run>/` 下的 CSV**：
   `rates_per_second.csv` 逐秒的帧/样本/指令数，`snapshots.csv` 每个位姿样本，
   `commands.csv` 每条指令的步长与解算耗时，`summary.json` 汇总百分位。
3. **想重跑报告**：`/usr/bin/python3 scripts/report.py` —— 只读 `data/`。

## 复现（需要原始 trace，只在录制的机器上跑）

```bash
cd <repo>
D=docs/investigations/mmk2_teleop_cadence_2026-11-11
L=~/.local/share/morla/logs
R1=$L/20261111T052027Z_mmk2_quest_sim_7ef3737a672a4e589b26db511a4dc0dd
R2=$L/20261111T053601Z_mmk2_quest_sim_8f7f6c0e1c994fdda6739f1752b0a62d
R3=$L/20261106T090825Z_fr3_quest_real_27073bfb455d45aaa69c338c6520a879
R4=$L/20261106T091451Z_fr3_quest_real_bdb268bc79d44e8f8c413e840b86d036

/usr/bin/python3 $D/scripts/rates.py  $R1 $R2 $R3 $R4          # → results/rates.txt
/usr/bin/python3 $D/scripts/config.py $R1 $R2 $R3              # → results/config.txt
PYTHONPATH=src/common/morla_core:src/common/morla_teleop:src/robots/airbot/morla_mmk2_teleop \
  /usr/bin/python3 $D/scripts/control.py --tcp $R1 $R3         # → results/control.txt

# 重新生成 data/（压缩证据）和 report.html
for r in $R1 $R2 $R3 $R4; do /usr/bin/python3 $D/scripts/export.py $r $D/data/$(basename $r); done
/usr/bin/python3 $D/scripts/report.py
```

`--tcp` 需要上面的 PYTHONPATH（用 MMK2 URDF 把关节指令正解成末端跳变）；缺了只是少一行。

## 文件

```
README.md                       本文
runs.json                       证据用到的四次运行（id / 路径 / quest side / 用途）
report.html                     自包含报告（另一台电脑直接打开）
data/<run_id>/                  压缩证据：snapshots / commands / rates_per_second / summary
scripts/rates.py                接收端 sequence、引擎摄取率、seq/pose、指令率
scripts/control.py              指令间隔、每拍步长、IK 延迟、奇异值
scripts/config.py               从 manifest 取 side / 绑定参数 / 设备参数
scripts/export.py               trace → data/（需要 trace）
scripts/report.py               data/ → report.html（不需要 trace）
results/rates.txt               本次 rates.py 输出原文
results/control.txt             本次 control.py 输出原文
results/config.txt              本次 config.py 输出原文
```
