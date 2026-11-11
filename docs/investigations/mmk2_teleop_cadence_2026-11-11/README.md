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
2. **丢发生在摄取线程，不是头显、也不是 IK。** 同一轮运行里，没有指令的秒帧间隔
   p50 **2.17 ms**（≈260 帧/秒），一有指令就变成 **15.11 ms**（≈66 帧/秒）。
   订阅是 BEST_EFFORT + depth=1（`builtin_sources.py:47`），处理不过来的消息被
   DDS 直接只留最新一条，于是每只手只拿到 **39.5 / 33.5** 个位姿/秒，
   `seq/pose` 从 2.00 涨到 **4.56 / 5.37**。
3. **每帧贵在两处，都在摄取线程的锁内。** 见证据二：
   `controller.step` **1.23 ms**（MMK2 的输出整形里同步算了一次 FK 和误差指标，
   FR3 0.09 ms）＋ `adapter submit` **6.15 ms / p95 20.6 ms**（要等 MMK2 仿真的
   步进线程：场景线程整段持锁跑 `mj_step` 和 `_snapshot`，而命令只能排队，
   FR3 真机 0.10 ms）。
4. **IK 不在这一跳上，但也不便宜。** MMK2 的控制器是异步的
   （`AsyncCartesianController`，求解在 worker 线程），所以摄取线程测到的是
   1.23 ms 而不是 12 ms。求解本身单次 **12.4 / 13.4 ms**；每秒 65 次就占
   **0.8 s/s**。它是纯 Python + numpy + scipy，**GIL 串行**——1/2/4 个线程的
   总吞吐是 195/184/219 次/s，不随核数增长；而且 6×6 的小矩阵也让 OpenBLAS
   拉满线程，每个求解线程白烧 **4 个核**（`OPENBLAS_NUM_THREADS=1` 后 1 个核，
   吞吐不变）。
5. **FR3 三项都小，所以一条不丢。** 解算 2.8 ms（Pinocchio/C++）、
   摄取线程 `step` 0.09 ms、`submit` 0.10 ms，一帧 **1.28 ms**，
   整轮 **268 帧/秒**，`seq/pose` 保持 **2.00**。
6. 手感层面：MMK2 指令 35.6 / 29.3 Hz、每拍关节 0.015 rad；FR3 89.9 Hz、
   每拍 0.0070 rad。两者平均速度接近（0.53 vs 0.63 rad/s），**台阶粗 5–6 倍**，
   所以是"一格一格"，不是"慢"。

## 责任怎么分

| 层 | 问题 | 证据 |
|---|---|---|
| 仿真后端（`mmk2_mujoco_sim`） | 步进线程整段持有 `command_lock` 跑 `mj_step` + `_snapshot`，命令提交只能等它 | `submit` p50 6.15 ms，FR3 真机 0.10 ms |
| MMK2 控制器 | 在调用线程（摄取线程）上做输出整形，含一次 FK 与 scipy 旋转 | `step` p50 1.23 ms，FR3 0.09 ms |
| MMK2 求解器 | 单次 12.4 ms，纯 Python、GIL 串行、BLAS 空转 | 每拍 21 次 `log6`、8 次 `command_metrics`；1/2/4 线程总吞吐不变 |
| MORLA 框架 | 命令节奏与输入样本 1:1；DDS depth=1 丢弃不计数、不告警 | `seq/pose` 只能反推丢弃率 |

真机上第 1 项不存在（`mmk2_sdk` 走自己的会话），但第 2、3 项照旧；按实测，
MMK2 两臂同时 90 Hz 需要每秒 180 次解算，而当前求解器在单进程里最多约 200 次/秒
——真机即便不丢帧也只剩不到 10% 余量。

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

不跑 IK 时能全速收（261 帧/s），一开始遥操就掉到 ~66 帧/s，松手又回升——
丢发生在消息到达引擎之前，具体贵在哪见证据二。

## 证据二：一帧输入花在哪（`results/frame_cost.txt`）

三段都在同一条摄取线程上，时间取自 trace 的 `monotonic_ns`：

| 项 | MMK2 仿真（双手） | FR3 真机 |
|---|---|---|
| 帧间隔，该秒**无**指令 p50 | 2.17 ms（≈260 帧/s） | 1.30 ms |
| 帧间隔，该秒**有**指令 p50 | **15.11 ms**（≈66 帧/s） | 1.28 ms |
| `controller.step` p50 / p95 | **1.23 / 6.24 ms** | 0.09 / 0.11 ms |
| `adapter submit` p50 / p95 | **6.15 / 20.58 ms** | 0.10 / 0.13 ms |
| 整轮输入 | 78.5 Hz | 268 Hz |

读法：

- `controller.step` 是 engine 同步调用控制器的耗时（trace 里的 `elapsed_ns`）。
  MMK2 的 IK 在 worker 线程，但输出整形 `_output` 在调用线程上又算了一次 FK
  和误差指标（`command_metrics`，含 scipy 旋转），于是这一跳 1.23 ms。
- `adapter submit` 是 `session.submit` 里 `command_request → command_receipt`
  的间隔，即命令写进设备的那一跳。MMK2 仿真的场景线程在 `_step()` 里整段持锁
  跑 `mj_step` + `_snapshot`（`mmk2_mujoco_sim/scene.py:106`），`submit` 只能
  排队（`scene.py:194`），于是 6.15 ms、p95 20.6 ms。
- 两项相加约 7.4 ms，实测帧间隔 15.1 ms：一帧里常有两条指令（左右手各一），
  再加上 trace 与锁的固定开销。

`scripts/ik_bench.py` 的对照（同一台机器，热身后固定目标；原文 `results/ik_bench.txt`）：

| 求解线程数 | 总吞吐（默认 BLAS） | CPU/墙钟 | 总吞吐（`OPENBLAS_NUM_THREADS=1`） | CPU/墙钟 |
|---|---|---|---|---|
| 1 | 195 次/s | **4.00 核** | 214 次/s | **1.00 核** |
| 2 | 184 次/s | 4.15 核 | 205 次/s | 1.13 核 |
| 4 | 219 次/s | 4.16 核 | 232 次/s | 1.15 核 |

两件事由此坐实：吞吐不随线程数增长（GIL 串行，多开 worker 不增加容量）；
6×6 的小矩阵也让 OpenBLAS 拉满线程，白烧 3 个核，限制后吞吐不变。

## 证据三：控制级手感（`results/control.txt`）

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
- 解算 12–13 ms 不是丢弃的直接原因（它不在摄取线程上），但它是 candidate age
  35–40 ms 的主要来源，也把每个 worker 线程钉在 ~50% 的负载上：两臂同时 90 Hz
  需要 180 次/s，而整套求解器在单进程里只有 ~200 次/s。
- σ_min 0.031、阻尼 0.025：MMK2 的初值位形 `[0,-0.6,1,±1.6,∓0.6,0]` 在奇异附近，
  会让部分拍被回溯砍步，是叠加因素。FR3 的阻尼一直钉在 0.005 下限。
- `raw pos err` 两边算法不同（不同 solver 的定义），**不要横向比较**。

## 证据四：配置差异（`results/config.txt`）

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
- **不是绑定参数更宽容**：MMK2/FR3 两边的 position_scale / smoothing 相同。
- **不是 IK 在摄取线程上阻塞**：MMK2 的求解在 worker 线程，摄取线程测到的是
  1.23 ms，不是 12 ms（见证据二）。IK 的成本体现在 CPU/GIL 与 candidate age 上。

## 证据边界

- `sequence` 是接收器自增的计数，不是头显时钟。它能证明"APK 交给接收器多少"，
  不能证明 APK 内部有没有再丢采样（那需要头显侧日志）。本轮的结论只需要前者。
- 丢弃率是由"发布速率 vs 摄取速率"推出来的；DDS 层没有计数接口，
  所以具体丢了多少条无法直接读出，只能由 `seq/pose` 反推。
- `adapter submit` 的 6.15 ms 是**仿真后端**的锁等待；真机（`mmk2_sdk`）不共用
  这把锁，但本轮没有真机 trace，真机的提交耗时未测。
- 每帧的耗时分解是实测（`step` / `submit` / 帧间隔），但"哪一项占多少"在同一帧里
  无法逐段对齐，所以表格给的是各自的 p50，不是逐帧相加。
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
/usr/bin/python3 $D/scripts/frame_cost.py $R1 $R2 $R3          # → results/frame_cost.txt

# 求解器成本画像（不需要 trace，跑两遍对比 BLAS 线程数）
/usr/bin/python3 $D/scripts/ik_bench.py                        # → results/ik_bench.txt（默认）
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 $D/scripts/ik_bench.py # → 追加对照组

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
scripts/frame_cost.py           trace → 一帧的 step / submit / 帧间隔（需要 trace）
scripts/ik_bench.py             求解器单次耗时、线程扩展、CPU 占用（不需要 trace）
results/rates.txt               本次 rates.py 输出原文
results/control.txt             本次 control.py 输出原文
results/config.txt              本次 config.py 输出原文
results/frame_cost.txt          本次 frame_cost.py 输出原文
results/ik_bench.txt            本次 ik_bench.py 输出原文（默认 / BLAS=1 两组）
```
