# Web 相机实时画面的成本核查（2026-11-18）

操作员问题：Web 可视化时的相机实时画面，数据从哪来？会不会影响遥操链路的延迟等性能指标？
开启数据录制之后，Web 画面会不会降低数采帧率？之前在 MMK2 仿真里开三个相机画面遥操就"非常卡"，
是不是同一个原因？真机相机链路有没有同样的问题？

结论先行：

1. **Web 画面不参与遥操链路。** 它是一个只读取"最新一帧缓存"的独立推流路径，遥操的输入帧、
   解算、下发里都没有它的环节。它唯一能影响遥操的方式是同进程的 CPU / 解释器时间竞争，实测约 **1%**。
2. **开启录制后，Web 画面不会降低数采帧率。** 录制和画面读同一份缓存；画面不触发编码、不写盘、
   也不拿录制那一把锁。
3. **MMK2"开三个相机就非常卡"里，相机不是主因。** 2026-11-11 那三轮复测本身就是 4 路仿真相机
   （320×240@15）全开跑的：双臂输入 66 Hz、单臂 130 Hz——同样是 4 路相机在渲，单臂能到 130 Hz。
   相机是常数税：一次完整渲染 8–10 ms，按 15 Hz 约 **单核 13–15%**，解释器时间只占 **~1%**。
4. 顺手发现两处结构性浪费：**仿真相机的深度端口没有任何消费者，却每帧都渲**；以及运行页会
   **自动为每个仿真相机挂一路预览**（看的成本可忽略，但容易让人把卡顿归因到它）。

本目录存测量脚本与实测数字，便于换机器复核。

## 一、两条互不相关的画面链路

| 场景 | 前端 | 服务端 | 帧的来源 | 帧的产生位置 |
|---|---|---|---|---|
| 工作站页（composable，MMK2/FR3/CR5 仿真与真机） | `<img src=/api/v2/sessions/{id}/cameras/{id}/stream>`（[shared.tsx:72](../../../src/common/morla_web_manager/web/src/components/workstation/shared.tsx#L72)） | MJPEG 长连接，60 Hz 轮询 `adapter.read()`，`(source_ns, sample_ns)` 变了才推（[composable_api.py:107](../../../src/common/morla_web_manager/morla_web_manager/composable_api.py#L107)） | adapter 的"最新一帧"缓存 | 仿真：渲染线程；UVC：子进程；`ros_image`：订阅回调 |
| 监控页（老链路，`quest3_video_streamer`） | `<img src=/api/v1/...>` | 15 Hz MJPEG 转发 [`video_feed`](../../../src/common/morla_web_manager/morla_web_manager/web_server.py#L1277) | monitor 节点对 `~/preview/{label}` 的订阅（[monitor_node.py:388](../../../src/common/morla_web_manager/morla_web_manager/monitor_node.py#L388)，对所有上报的 label 无条件订阅） | streamer 的 `PreviewPublisher`：10 fps / 640 宽 / q65，独立线程+最大长度 1 的队列，忙时丢（[preview.py:72](../../../src/common/quest3_video_streamer/quest3_video_streamer/preview.py#L72)） |

关键点：`PreviewPublisher.submit()` 只检查**推送门控**（`StreamGate.is_enabled(label)`），
**不看订阅者数量**——所以这条预览编码是"门控开着就常驻"，跟有没有浏览器无关。
另一条 `/frame` 端点（一次请求一张图，约 10 fps）曾因为把感知延迟压在这一跳上而被 `/stream` 取代，
代码注释里保留了这段缘由。

## 二、成本分两类，别混

**（a）不看也在跑的无条件成本**

- 仿真：装配里只要存在相机实例，渲染线程就启动，并且**按实例数无条件渲染**，与观众无关。
  公共后端见 [sim_mujoco.py:245](../../../src/common/morla_runtime/morla_runtime/sim_mujoco.py#L245)
  （每轮渲染所有配置相机，循环周期 `1/min(120, max fps)`）；MMK2 自己的版本更重：
  固定 5 ms 一轮，每轮 `mj_forward`，每台相机到 15 Hz 时**先渲彩色、再渲深度**
  （[scene.py:238-275](../../../src/robots/airbot/mmk2_mujoco_sim/mmk2_mujoco_sim/scene.py#L238)）。
- 老链路：预览编码常驻（上节）；采集抽头则是**有订阅者才编码**，全分辨率 q90，最多 30 fps
  （[collect_tap.py](../../../src/common/quest3_video_streamer/quest3_video_streamer/collect_tap.py)）——"录"才付这笔钱，且付在 streamer 进程里。
- `morla.camera.ros_image` 的默认配置（`compressed: false`）订阅裸 `Image`，**全分辨率 JPEG 编码发生在
  Web 进程的 rclpy 回调里**（[cameras.py:222-224](../../../src/common/morla_runtime/morla_runtime/cameras.py#L222)）——
  这是真机上唯一一条把相机编码放进遥操同进程的路径。

**（b）只有打开页面才多出来的成本**

每路一条 60 Hz 的 `adapter.read()`（`deepcopy` 缓存，实测 **0.007 ms/次**）＋ 一次 HTTP 推送。
仿真 JPEG 约 4–7 KB，真机 JPEG 视分辨率几十到几百 KB。可以忽略。

## 三、实测（本机 16 核，真实 MMK2 模型 33 body / 30 geom，320×240，q90）

| 项 | p50 |
|---|---|
| `mj_step` | 0.06 ms |
| `mj_forward` | 0.02 ms |
| 彩色渲染 / 相机 | 1.47 ms |
| 深度渲染 / 相机 | 0.56 ms |
| JPEG q90 + 深度拷贝 / 相机 | 0.27 ms |
| **4 相机完整一轮** | **8.2–10.4 ms** |
| 按 15 Hz 折算 | **单核 12–15%** |
| 深度缓冲 / 帧 | 307,200 B（无人消费） |
| JPEG / 帧 | 4.3–7.0 KB |

解释器时间的实测更关键：在另一个线程里跑完整的 4 相机渲染（彩色+深度+JPEG）时，纯 Python 循环线程的
`time.thread_time()/wall` 从 **0.999–1.000 降到 0.988–0.991**——渲染线程只拿走约 **1%**
的解释器时间（`mj_step` 500 Hz 时是 0.3%）。也就是说它主要在别的核上花自己的 CPU，不会把遥操的
`engine.targets`/`submit` 挤成两倍慢。

复现：

```bash
cd MORLA
PYTHONPATH=src/common/morla_core:src/robots/airbot/mmk2_mujoco_sim:src/common/morla_runtime \
  /usr/bin/python3 docs/investigations/camera_preview_2026-11-18/scripts/bench_camera_render.py
```

## 四、录制与预览为什么互不影响

- **composable**：录制在 [`tick()`](../../../src/common/morla_web_manager/morla_web_manager/composable_service.py#L481) 里、持有 `service.lock` 调用
  [`capture_tick()`](../../../src/common/morla_web_manager/morla_web_manager/composable_service.py#L572)，按 `dataset_fps` 读**同一份 adapter 缓存**，
  只有 `source_ns` 前进才 `append_camera`；写盘进 1024 长队列，由独立的 `morla-record-writer` 线程完成。
  预览走 `adapter.read()`，**不拿这把锁**，也不触发编码。两者唯一的耦合是读同一份缓存。
- **老链路**：录制订阅 `~/collect/{label}`（有订阅者才编码），画面订阅 `~/preview/{label}`（常驻）。
  两个话题互不门控。
- 真正限制录制帧率的是：`tick()` 的 20 ms 节拍、`dataset_fps`（默认 30）、ingest 线程在同一把
  `service.lock` 里的 `targets + submit`（实测每帧 p50 约 10 ms 量级，录制 tick 要排队），
  以及生产端速率（仿真相机 15 Hz、真机相机自身帧率）。**都不是画面造成的。**

## 五、"开了三个相机就非常卡"应该怎么归因

- 2026-11-11 的三轮复测（`20261111T162107Z_mmk2_quest_sim` 等）manifest 里就是 4 路相机
  （320×240@15）全开；其中单臂那轮输入仍有 130 Hz。相机在，单臂也能跑满。
- 当时真正让遥操卡的是已经定位过的那条链：ingest 单线程里 `targets + submit`、两道 IK worker 抢
  CPU/GIL、20 ms 求解 deadline 被超时后 `deadline_hold` 原样返回（`deadline_hold` 29%/21%，
  零关节步长 30%/28%），加上 2 cm / 0.35 rad 的前瞻窗口与 22–40 Hz 的有效更新率。
  详见 [MMK2 遥操节奏核查（2026-11-11）](../mmk2_teleop_cadence_2026-11-11/README.md)。
- 相机的角色：常数税 8–10 ms/轮 ≈ 单核 13–15%，约占解释器时间 1%。在 16 核机器上不会造成 2 倍的
  频率差；但在核数少、或另开 `display: local` 窗口、或更多/更大相机的机器上，这笔税会放大。

## 六、可以省但本次没改的两处

1. **深度渲染无人消费**：`rgb`/`depth` 两个端口里，全仓只有生产者，没有任何读取方
   （采集只 append `rgb`，预览只推 `rgb`）。深度占每相机约 23% 的渲染成本，还让每帧多 307 KB
   数据进缓存、被每次 `read` 的 `deepcopy` 再拷一遍。
2. **相机实例数就是渲染成本**：不需要看/录的相机不要建实例——渲染线程按实例数工作，与观众无关。
   `mmk2_quest_sim` 的 revision-16 之后已经没有相机实例，正好可以作为 A/B 基线。

另外运行页会为每个仿真相机自动挂一路预览（[RunConsole.tsx:29](../../../src/common/morla_web_manager/web/src/components/workstation/RunConsole.tsx#L29)），
"看的成本"可忽略，但会让人误以为卡顿来自预览。

## 七、真机三种相机后端

| 后端 | 编码位置 | 与遥操同进程？ | 与录制的关系 |
|---|---|---|---|
| `morla.camera.uvc` | 子进程 `_uvc_worker`（cv2，q90） | 否 | 同缓存，无额外编码 |
| `morla.camera.ros_image` + `compressed: true` | 驱动侧 | 否 | 同缓存 |
| `morla.camera.ros_image`（默认，裸 `Image`） | **Web 进程 rclpy 回调** | **是** | 同缓存 |

老链路（`quest3_video_streamer`）里预览、WebRTC、采集抽头都在 streamer 自己的进程，遥操臂节点是
独立进程，链路不串；共享的只有整机 CPU（Jetson 上要当回事，关闭「视频回传」可省掉常驻预览编码）。

## 八、未验证边界

- 没有在**活动会话**下读 Web 进程的逐线程 CPU（核查时工作站无活动会话），所以"13–15% 单核"
  来自同模型离屏基准，不是现场 `/proc` 采样。
- 没有测浏览器侧解码/绘制开销；它不影响机器人进程。
- 老链路（preview/collect 的 10 fps、q65、30 fps、q90）来自代码与 streamer 文档，本轮未实测。
- 全部数字来自本机 16 核、离屏渲染走本机 GL 的路径；换机器（尤其无 GL 加速的 Jetson）需要重跑脚本。
