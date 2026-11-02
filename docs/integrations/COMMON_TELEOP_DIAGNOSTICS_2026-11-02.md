# 公共遥操运行诊断日志（2026-11-02）

公共 `RuntimeSession` 每次启动默认创建独立日志目录，停止后保留。不依赖数据采集任务、采集通道或 recorder；没有启动数据记录也会保存。公共 Runtime、BindingEngine 和 ControlHost 共用此机制，当前及后续按公共接口接入的机器人自动获得基础日志。旧的独立 ROS/Profile 遥操入口仍保留其原有日志，不因此自动迁移。

## 使用与保存位置

Web「工作站 → 检查与运行 → 运行诊断日志」可设置是否启用、反馈采样上限、单次容量上限，保存配置后下次启动生效。运行时显示写入、丢弃、排队数量与错误；可选择历史运行、查看最近记录和逐文件下载，刷新页面后仍能查看。

- Web 服务：默认 `<store>/logs/`，与 `<store>/recordings/` 分开。
- 直接 Runtime：默认 `${XDG_STATE_HOME:-~/.local/state}/morla/teleop/`。
- 两种 CLI 支持 `--log-dir`；公共 sim launch 支持 `log_dir:=...`；Runtime ROS 节点支持参数 `diagnostic_log_root`。环境变量 `MORLA_DIAGNOSTIC_LOG_ROOT` 也可指定根目录；显式参数优先。

每次运行目录名为 `UTC时间_工作站名_完整session_id`，目录权限 0700。配置预检不创建运行目录；实际启动（包括随后启动失败）才创建。

- `manifest.json`：schema 版本、时间基准、日志选项、脱敏后的解析配置快照。
- `trace-00000.jsonl` 等：逐条结构化记录，按容量分段。
- `summary.json`：结束状态、各类写入/丢弃数量、错误、运行结果和未完成操作快照，临时文件写完后原子替换。

装配配置中的 `metadata.diagnostic_logging` 支持 `enabled`（默认 true）、`feedback_hz`（50，范围 1–200）、`queue_size`（4096，范围 16–65536）、`segment_mb`（16，范围 1–256）及 `max_run_mb`（1024，范围 1–8192）。分段上限不能大于单次上限。容量限制针对 JSONL，另有配置和摘要文件；不会自动删除历史运行目录。

## 记录内容与关联

基础记录包括运行开始/结束、生命周期事件、操作开始/结束、规范化输入、控制门禁原因、映射输出、计算耗时、命令请求及各 Adapter 回执、反馈状态与异常。相机图像等二进制仅留下省略标记，不重复写入日志。

每条记录包含 `schema_version`、`session_id`、`record_sequence`、`kind`、`monotonic_ns`、`wall_time_ns` 和 `data`；适用时还包含 `component_id`、`binding_id`。输入与控制记录保留源、epoch、帧序号和接收时间。单调时间用于同一运行内计算间隔，墙上时间用于对应外部日志，设备原始时间保留在各自载荷中。

公共笛卡尔控制器提供已有计算中的当前 TCP、目标 TCP/末端、反馈种子、候选关节与求解报告，不为记录额外进行 FK。CR5 异步 IK 在工作线程完成时另写 `controller` 记录，携带真正对应的输入帧、原始/滤波输入、目标、候选与求解报告，区分完成、重锚后丢弃和异常。异步计算完成不表示该候选已经下发或执行。

命令分发成功与设备执行确认分开解释，执行语义以 Adapter 回执为准。反馈来自 Runtime 读取的缓存健康样本，按组件去重并限制采样频率，故障/就绪变化绕过限频；它不是硬件每一帧原始反馈的完整抓包。显式接入 `trace_feedback` 的 Adapter 也可补充更新。

## 新设备与控制器接入

基础日志位于公共链路，无需按品牌添加日志分支。Adapter 保持现有 `read/health/command` 契约即可；如需补充驱动事件，可使用 factory/configure 上下文中的可选 `diagnostics(kind, data, component_id=..., binding_id=...)` 回调，或 `trace_feedback(component_id, sample)`。回调只负责观测，不参与控制许可。

同步控制插件可提供 `diagnostics()` 或 `last_report`。异步插件可提供 `set_diagnostic_sink(callback)` 接收完成报告回调，并通过 `set_trace_context(context)` 保存当前输入身份；报告必须携带实际参与计算的帧上下文，不能把旧缓存报告当作当前帧结果。未提供这些可选接口时，基础输入、输出、回执和反馈仍会记录。

## 性能和完整性边界

控制路径只做有界快照和非阻塞入队，单独写线程进行磁盘 I/O。默认队列最多 4096 条，同时限制估算载荷为 16 MiB；单条快照、深度和向量长度也有限制。饱和时丢弃日志并累计计数，序号可能有空洞；磁盘错误、容量耗尽或停止排空超时会显示 degraded，不改变已有运动许可。敏感字段名含 token/password/secret/credential/authorization 时脱敏，超长、非有限值和二进制显式标记；这不保证识别任意自由文本中的秘密。

写线程约每 0.5 秒 flush；未提供断电级 fsync 保证。正常停止先处理设备停止和清理，再在控制锁外等待日志排空（默认最多 2 秒）。硬退出可能丢失缓冲尾部且没有摘要，历史列表标为 incomplete。`clean_shutdown=true` 仅表示日志正常排空且没有已知丢弃/错误；设备停止是否成功必须检查 `outcome`，不能从日志标记推断物理停止。

只读 Web API：`GET /api/v2/diagnostics/runs`、`GET /api/v2/diagnostics/runs/{session_id}/tail?limit=50`、`GET /api/v2/diagnostics/runs/{session_id}/files/{filename}`。文件名白名单与根目录边界检查限制访问；tail 最多 200 条、读取窗口 1 MiB，截断用 partial 标记；下载正在增长的分段时只发送请求时已存在的字节。

## 验证

171 项相关 Python 测试通过，排除 1 项此前已复现的基线失败 `test_mobile_input_and_recording`（release_or_reanchor_required）。新增覆盖无 recorder、未来第三方机器人、慢盘与队列满、不可写目录、轮转与容量耗尽、写入异常、脱敏、反馈采样、CR5 异步输入关联、历史 API/下载和路径约束。前端 8 项逻辑测试、TypeScript/Vite 构建通过；Edge 验证默认开启、采样配置、无采集自动保存、停止摘要、文件下载及页面重载后的历史访问。未进行实体输入或实机长时运行验收。
