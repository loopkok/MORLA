# MMK2 遥操节奏核查（2026-11-11）

操作员问题：MMK2 遥操"一卡一卡"，FR3/CR5 同样是 6 轴却不这样，差别在哪；并追问
"上位机每帧只发一只手"是不是 MORLA 的问题。

本目录是当时的实测证据。日志本体在 `~/.local/share/morla/logs/`，不在仓库里；本目录存的是
**测量脚本 + 脚本输出原文 + 运行清单**，任何一次都能重跑。

## 结论

1. 双手同时在线时，头显 APK 的每个快照只装一只手，快照约 73 份/秒、左右轮流，
   每只手 **39.5 / 33.5 Hz**。这是**上游 APK 的发送节奏**，MORLA 的接收/解析/引擎各层都是等速透传。
2. 引擎只在收到新样本时下发一次指令（`src/common/morla_teleop/morla_teleop/engine.py:121`），
   所以 MMK2 的指令节奏 = **35.6 / 29.3 Hz**，每拍关节步长 0.015 rad、末端 2–10 mm。
3. FR3 单手那轮是 **89.9 Hz**、每拍 0.0070 rad；`20261111T053601Z` 那次 MMK2（`side: right`）
   同样是 **89.4 Hz**。同一个 APK、同一份 MORLA 代码，差别只在发几只手的位姿。
4. 因此"一卡一卡"的主因是**台阶粒度**（频率低 2.5 倍、每拍大 2.1 倍），
   不是 MMK2 的 IK 或限幅把速度卡住了——平均速度两边已接近（0.53 vs 0.63 rad/s）。

## 证据一：节奏（`results/rates.txt`）

| 运行 | 输入帧 | 左手 APK → 收到 → 指令 | 右手 APK → 收到 → 指令 |
|---|---|---|---|
| `20261111T052027Z` MMK2 `side: both` | 78.5 Hz | 39.5 → 39.5 → 35.6 Hz | 33.5 → 33.5 → 29.3 Hz |
| `20261111T053601Z` MMK2 `side: right` | 233.9 Hz | 无 | **89.4 → 89.4** Hz |
| `20261106T090825Z` FR3 `side: right` | 268.0 Hz | 无 | 89.9 → 89.9 → 89.9 Hz |
| `20261106T091451Z` FR3 `side: right` | 268.4 Hz | 无 | 89.9 → 89.9 → 90.0 Hz |

三个字段分别来自：

- `port_metadata.<port>.source_time_ns` —— 头显 APK 自己写的时间戳
  （`src/common/morla_input/morla_input/quest_events.py:342` 把 `sample['sent_ns']` 写进端口元数据）；
- `port_metadata.<port>.received_ns` —— MORLA 收到的时刻，与上一行逐项相等 ⇒ 链路没有降频；
- `kind=control` 的 `source_port` —— 真正下发的指令，比样本少的部分是门控
  （该轮 1692 条 `control_gate`，deadman/来源缺席），不是丢样本。

双手那一轮的交错形态（同文件）：

```
pose snapshots   73.0 Hz   interval p50 13.4  p95 29.8 ms
interleave       Lx1=2529  Lx2=527  Lx3=8  Rx1=3065
```

即：快照严格左右轮流（右手从不连发），右手偶尔缺位由左手顶替；每个快照只带一个 `side`
（协议本身就是一侧一份快照，`src/common/morla_input/morla_input/quest_events.py:322`）。

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

- `candidate age` = 本拍下发的解对应的是多久以前的那次请求。两边都等于"一个请求间隔"，
  FR3 的间隔小所以总滞后小；MMK2 的滞后主要是**节奏低**带来的，其次是解算慢 4 倍。
- `每拍关节步长 ÷ 间隔` = 0.53 / 0.46 rad/s（MMK2）对 0.63 rad/s（FR3）：**平均速度相当，
  台阶粗 5–6 倍**。
- σ_min 0.031 对 0.111、阻尼 0.025 对 0.005：MMK2 的初值位形 `[0,-0.6,1,±1.6,∓0.6,0]`
  在奇异附近，解算"发软"、更容易被回溯砍步，是叠加因素不是主因。
- `raw pos err` 两边的算法不同（不同 solver 的 raw 误差定义），**不要横向比较**。

## 证据三：配置差异（`results/config.txt`）

同一批 trace 对应的 manifest：

| | FR3 | MMK2 |
|---|---|---|
| Quest 实例 `side` | `right` | **`both`** |
| Quest 实例 `rate_hz` | 100 | 100 |
| position_scale | 0.8 | 0.8 |
| smoothing_alpha | 0.6 | 0.6 |
| max_joint_step | 0.03 | 0.15 |
| max_joint_velocity | 1（设备上限 `max_velocity_rad_s` 0.65 生效） | 0.8（仿真设备默认 3.14，不生效） |
| command_lead_rad | 0.08（设备上限 `max_command_lead_rad` 0.3） | 0.15（设备默认 0.15） |
| 臂设备参数 | `max_velocity_rad_s 0.65`、`max_command_lead_rad 0.3` | `position_lead_m 0.02`、`rotation_lead_rad 0.35`、`damping_min 0.005`、`damping_max 0.145`、`singularity_threshold 0.05` |

即：除了 `side` 和两台机器人各自的设备上限，绑定参数几乎一样。
`smoothing_alpha = 0.6` 的时间常数是 2.5 个控制周期——90 Hz 下 28 ms，35 Hz 下 71 ms，
同样的数值在 MMK2 上多滞后约 40 ms。

## 已排除

- **不是接收/解析循环的吞吐**：同一份代码在单手时帧率 78.5 → 233.9 Hz，随输入涨。
- **不是引擎丢节奏**：`engine.py:121` 只在 `sample_key` 变化时调控制器；cmd 与样本的差
  由 `control_gate` 解释。
- **不是接收器拒收**：无 `InputError`、`sample_sequence` 无回退/重放，且 received == sent。

## 证据边界

- 只能证明"MORLA 收到的样本就是这个节奏"。**APK 是否采样了却没发**，需要头显侧日志，trace 看不出来。
- `20261111T053601Z` 是 17 秒空跑（未按 deadman，无 `control` 记录），只用它的位姿节奏做对照。
- 本轮只覆盖仿真 + 上述几条运行；MMK2 真机（`mmk2_sdk`）不在其中。

## 复现

```bash
cd <repo>
D=docs/investigations/mmk2_teleop_cadence_2026-11-11
L=~/.local/share/morla/logs
R1=$L/20261111T052027Z_mmk2_quest_sim_7ef3737a672a4e589b26db511a4dc0dd
R2=$L/20261111T053601Z_mmk2_quest_sim_8f7f6c0e1c994fdda6739f1752b0a62d
R3=$L/20261106T090825Z_fr3_quest_real_27073bfb455d45aaa69c338c6520a879
R4=$L/20261106T091451Z_fr3_quest_real_bdb268bc79d44e8f8c413e840b86d036

/usr/bin/python3 $D/scripts/rates.py  $R1 $R2 $R3 $R4
/usr/bin/python3 $D/scripts/config.py $R1 $R2 $R3
PYTHONPATH=src/common/morla_core:src/common/morla_teleop:src/robots/airbot/morla_mmk2_teleop \
  /usr/bin/python3 $D/scripts/control.py --tcp $R1 $R3
```

`--tcp` 需要上面的 PYTHONPATH（用 MMK2 URDF 把关节指令正解成末端跳变）；缺了只是少一行。

## 文件

```
README.md                       本文
runs.json                       证据用到的四次运行（id / 路径 / quest side / 用途）
scripts/rates.py                快照 → 指令每一跳的频率、交错形态
scripts/control.py              指令间隔、每拍步长、IK 延迟、奇异值
scripts/config.py               从 manifest 取 side / 绑定参数 / 设备参数
results/rates.txt               本次 rates.py 输出原文
results/control.txt             本次 control.py 输出原文
results/config.txt              本次 config.py 输出原文
```
