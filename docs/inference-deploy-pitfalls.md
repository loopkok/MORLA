# 推理部署环境/后端坑（"模型部署不响、回传不对"手册）

> 2026-09 部署 pi0.5/ACT 全链路反复踩，2026-09-22 整理归档。
> 给新会话/接手者：推理**卡顿**类问题（四层原因 + pi0.5 A/B/C/D 轮）见
> [act-inference-stutter-investigation.md](act-inference-stutter-investigation.md) 与
> [2026-09-21-pi05-inference-investigation.md](2026-09-21-pi05-inference-investigation.md)；
> 本文件收**环境/后端/协议/部署**这一类"跑不起来、回传不对、静默退化"的坑。
> 母本：`src/astral_policy_inference/CLAUDE.md`（坑表 + 环境约束 + 验证清单）+ CHANGELOG 09-03~09-22。

## 一、环境硬墙（先确认能跑起来）

| 坑 | 症状 | 根因 | 修法 |
|---|---|---|---|
| lerobot 0.6.2 vs rclpy 版本墙 | 单进程跑 ACT 节点不可行 | lerobot 0.6.2 要求 py3.12（代码用 PEP 695 泛型 `def f[T]`，py3.10 解析期 SyntaxError，shim 无法绕过）；rclpy（Humble）只有 py3.10 | **远程 serve 化**：`scripts/serve.py`（py3.12 lerobot 环境）+ policy_node（py3.10 + ROS）远程连；协议自包含（client.py 自带 vendored msgpack_numpy） |
| vendored msgpack_numpy 不兼容 | 服务端解出 dict / 字段错 | openpi_client **自带 vendored msgpack_numpy**（键 `__ndarray__`），与 pip 版（键 `nd`）线上不兼容；server 必须用与 client 同源的 vendored 版 | 统一用包内 `protocol.py`（vendored `__ndarray__`）；Jetson 无需再装 openpi_client；编辑残留重复 `_make_packer` 定义会覆盖新版 |
| `PreTrainedPolicy` 抽象基类 | 加载真实 checkpoint 崩 "Can't instantiate abstract class" | lerobot 0.6.2 里 `PreTrainedPolicy` 是抽象基类，`from_pretrained` 直接调用无法实例化（两个 checkout 行为一致）；`test_backend.py` 从没覆盖过 `open()` | `open()` 读 `config.json["type"]` → `get_policy_class` 解析具体类（ACTPolicy）再 from_pretrained；补 2 例 hermetic 回归（sys.modules 伪造 lerobot，无 torch） |
| `msgpack_numpy.Packer` 无 unpackb | server 报错 | Packer 是打包器，unpackb 是模块级函数 | 用模块级 `unpackb` |
| `ros2 run`/`ros2 launch` 报 "No executable found" | 找不到可执行文件 | 缺 `setup.cfg`，console script 装进 `bin/` 而非 ament 的 `lib/<pkg>/` | 补 `setup.cfg`（`[install] install_scripts=$base/lib/<pkg>`） |

## 二、ACT 后端/引擎机制坑（"控制率不对、动作不对"）

| 坑 | 症状 | 根因 | 修法 |
|---|---|---|---|
| **ACT 一次 infer 只回 1 行** | 部署控制率只有 13Hz（应 30Hz） | `InprocBackend.infer` 走 `select_action`（内部 50 行队列逐行吐），引擎分块/预取/网络全浪费 | **方案 A**：`predict_action_chunk` 一次返回完整 chunk（`temporal_ensemble_coeff` 非 None 回退 select_action）；图像上传从每行一次变每 chunk 一次（-96%），queue_async 达 30Hz，loop 7.9→1.26ms |
| **queue_async 锁饥饿** | 瞬时推理也只有 ~15Hz；真实 ACT 0.5Hz | `_planner_loop` 把 `self._stop.wait(0.005)` 写在 `with self._lock` 内——planner 空闲时几乎 100% 持锁，`tick()` 在锁上饿死；此前被 select_action 1 行 chunk（planner 一直重填）掩盖 | `Event.wait` 移出锁外（空闲判定在锁内、等待在锁外）；回归 `test_queue_async_control_thread_not_starved_by_planner`（0.5s >200 pops） |
| **陈旧队列跨会话** | HITL 接管/重启后重新进 POLICY 重放 ~1.7s 陈旧动作（危险跳变） | serve 的 ACT 后端常驻，`select_action` 50 行队列跨客户端连接残留 | serve 每个新连接 `backend.reset()`（openpi 协议无 reset 消息）；节点每次 POLICY 会话正好一个新连接 |
| **逐帧评估被缓存行污染** | 正确性 MAE 高达 0.20 rad（实际模型只有 0.004） | `select_action` 队列：后续 infer 只吐上一 chunk 缓存行、不重新推理 | 逐帧评估每帧先 `backend.reset()`（**节点运行时此行为是正确**=ACT 自管节奏，勿"修"）；正确性脚本已内置 |
| **后端把 delta 当绝对返回** | Jetson 静默跳到错误目标 | 节点信任 backend「返回绝对」契约，无语义断言；openpi server 漏 AbsoluteActions/错 checkpoint 会静默返回 delta | **绝对语义守卫** `abs_action_min_scale`（默认 0.5，≤0 关）：机器人离开零位（\|state\|>1 臂维）时 chunk 首行量级不得塌缩，否则 PolicyError→安全 stop；只查 \|state\|>1 的维（夹爪 [0,1] 自动排除） |
| 正确性脚本把整段视频解码进内存 | OOM 卡死 | 单个 `file-*.mp4` 含整个 chunk 所有 episode（~1 万帧 ≈15GB/相机） | PyAV `seek` 按需解码目标帧附近（O(1) 内存） |

## 三、参数/配置坑（"改了没生效、静默退化"）

| 坑 | 症状 | 根因 | 修法 |
|---|---|---|---|
| **yaml 顶层键 ≠ 节点名** | 改了 coeff/tol/interp/jpeg 运动却不见变化（连续 4 轮真机测试被吞） | rclpy 按节点名匹配 `--params-file` 段，键不匹配**整份参数静默丢弃**（Humble 无单键回退） | yaml 顶层键 `astral_policy_inference:` → `policy_node:`；节点启动行**自报生效参数**（ctrl/coeff/anchor_tol/.../jpeg） |
| **launch 默认值盖 yaml** | 改了 yaml host/port 仍连 127.0.0.1:8000 | launch 默认值非空且无条件 append 整包参数 | host/port/checkpoint_dir 默认空串，只 append 显式传入键（与 data_collect 不变量 1 同反模式） |
| rclpy 无 struct 参数 | `camera_map` 声明 dict 报 "not one of allowed types" | Humble rclpy 无 struct 参数类型 | `camera_map` 改 JSON 字符串参数（`_parse_camera_map`） |
| `BEST_EFFORT` 话题 echo 收 0 条 | `ros2 topic echo` 收不到节点指令 | 节点发布 cmd/gripper 是 BEST_EFFORT depth=1，默认 RELIABLE echo 匹配不上 | 加 `--qos-reliability best_effort --qos-depth 1`；测速用 `scripts/hz_best_effort.py` |
| launch 不透传 `camera_image_size` | ACT 部署静默用 224，与模型 480 不匹配直接崩 | launch 缺透传，回退 yaml 默认 | launch 补 `camera_image_size` 透传（ACT 必传 480，pi05=224） |
| `HF_LEROBOT_HOME` 不生效 | pinned lerobot 0.1.0 找不到数据集 | import 时把 `HF_LEROBOT_HOME` 固化为模块常量，且只认新名（设旧名 `LEROBOT_HOME` 直接 raise） | 在 import openpi **之前**设定环境变量 |
| `OPENPI_DATA_HOME` 错误指向 | 每次重下 11.6GB 权重（慢网络卡 1h+） | 环境变量指向错误位置，`maybe_download` 不命中本地缓存 | 正确权重在 `~/.cache/openpi`（12GB 完整）；建议 unset |
| serve `SLOT_MAP` 值误改 | 图喂到错误的 `observation.images.<键>` | serve 把图喂到 `observation.images.<值>`，值必须是**模型 `input_features` 图像键**（不是 collect label） | 恢复 `{base_0_rgb: video8, left_wrist_0_rgb: video0}` 这类模型键；迁移时勿把 collect label 写进 SLOT_MAP |

## 四、web / 协议坑（快速过一遍）

| 坑 | 症状 | 根因 | 修法 |
|---|---|---|---|
| web 推理白屏 | 一跑就白屏、刷新仍白屏 | 节点一跑 WS 遥测 `engine` 出现嵌套对象 `server_timing`，`InferenceCard` 渲染 `<b>{v}</b>` 把对象当 React child → 抛错整树卸载 | `fmtStat` 安全格式化（对象压平为 `{k1:v1 k2:v2}`）+ 顶层 `ErrorBoundary` |
| 启动节点后立刻"开始策略"偶发无反应 | 命令静默丢失（节点保持 IDLE ~30s） | `/policy_inference/cmd` 是 VOLATILE 一次性，web 不等 DDS 发现订阅者就返回成功 | 发送前最多等 2 秒发现订阅者，超时返回 503；state 离线时前端禁用按钮（VOLATILE 语义保留，防重启后重放旧 `policy` 自动运动） |
| web 推理参数命令注入 | host 含 shell 元字符 | `LaunchManager` 用 `bash -c` 拼 launch 命令 | `policy_launch_args` 白名单清洗（`[^A-Za-z0-9_.:\-]` 剔除，port/尺寸先 int 化） |
| 接管显示 ~1.1s 延迟 | 点接管后 web 还显示 POLICY | reanchor 响应/提交 + teardown join 阻塞 | 三处时间戳日志 + `_commit_takeover` 在拆引擎**前**立即 publish state |

## 五、部署/运维速查

- **换模型 = 只改后端参数**：`backend_type=remote\|inproc\|stub`（传输方式）+ `model=act\|pi05`（模型族，
  决定 serve/inproc 加载路径），机器人端零改动。
- **真机 launch 必传**：`backend_type:=remote host:=<gpu> port:=8001 camera_image_size:=<模型输入>`；
  ACT 另传 `camera_image_size:=480`；`engine_mode` 默认 queue_async 即 30Hz（方案 A 后无需改）。
- **部署前冒烟**：`run_act_integration.py`（模型侧）→ `run_act_e2e.py --check-server`（节点全链路）
  → `run_act_correctness.py`（真实数据 MAE ~0.005 rad）→ `run_act_benchmark.py`（链路基准）。
  实测：控制率 29.6Hz、稳态 RTT 4.3ms、吞吐 198-207 req/s、GPU ~957MiB-1GiB。
- **pi05 部署**：serve 需 **RAM≥32GB** 机器（权重 11.6GB 必须整体进 RAM，本机 3 次 OOM）；
  首次推理 XLA 编译 2-5 分钟——serve 内 `_warmup_policy` + `jax_compilation_cache_dir` 跨进程复用。
- **每类坑的现有防护/参数**：完整坑表见 `src/astral_policy_inference/CLAUDE.md`「已解决的坑」。
