# 单人操作：语音、Quest 手柄与 USB 脚踏

统一操作入口位于 `src/common/morla_operator_control`，不绑定 Nero、Astral 或末端类型。输入适配器只发操作意图；机器人动作仍通过原有控制协调器，采集仍通过原有采集状态机，不直接发送关节命令。

```text
Ubuntu 麦克风 → 离线中文识别 ─┐
Quest 麦克风 → TCP 8001 → 识别 ├→ /morla/operator/command
USB 踏板 → 按下/释放/长踩 ───┤      ↓ 统一去重、状态检查、请求关联
原 Quest 手柄按键 ──────────┘      ├→ /morla/control/{start,pause,release}
                                  └→ /data_collect/control
结果 ← /morla/operator/events ← 最终控制状态 / 采集执行回执
```

原手柄在统一入口在线时自动走新路由；没有统一入口时保留旧路径。左手柄启动/右 A 在 HUMAN 时交还控制、采集 A/B/摇杆按键语义保留。摇杆丢弃维持原有立即丢弃行为；语音丢弃需要二次确认，脚踏丢弃需要长踩后释放。Web、键盘和原来的纯字符串采集命令继续兼容。

## 先准备一次

以下在 Ubuntu 机器人主机运行。软件部署不自动打开麦克风，也不启动实体 Profile 或使能电机。

```bash
cd /home/loopkok/MORLA
source /opt/ros/humble/setup.bash
colcon build --packages-select morla_operator_control morla_bringup morla_data_collect --symlink-install
source install/setup.bash

# 首次装离线识别依赖及中文模型；本次部署已经安装到 loopkok 用户目录。
bash scripts/setup_operator_audio.sh
```

依赖隔离存放于 `~/.local/share/morla/operator-python`，模型在 `~/.local/share/morla/models/vosk-model-small-cn-0.22`，不更新系统 ROS 的 Python 包。主机可使用 sounddevice/PortAudio，缺少 PortAudio 时可用系统 `arecord`。确认设备：

```bash
arecord -l
```

当前部署检查时 Ubuntu 没有可用的物理输入麦克风，Quest 也未连接；接上麦克风后再启动输入。只有 `auto_null.monitor` 不表示具备可用麦克风。

## 主机和 Quest 两路语音

先通过 Web 启动选定的机器人 Profile；新版 `morla_bringup robot.launch.py` 默认自动启动一个命令路由器。Web 中完成硬件准备与工作位，工作位后再次准备并使能切回遥操模式；预先启动采集节点，但不点击开始录制。

新版一键脚本 `bash scripts/nero_web_start.sh` 已默认同时启动 Quest 语音接收，含 8001 映射；`--voice-source both` 可开启双路，`--pedal-device PATH` 可接脚踏，`--operator-dry-run` 可先预览。使用一键脚本时不要再运行下面的独立语音 launch，详细参数见 [一键启动说明](nero_web_start.md)。

如果没有通过一键脚本启动输入，另开终端启动语音输入，首次建议只识别不执行：

```bash
source /opt/ros/humble/setup.bash
source /home/loopkok/MORLA/install/setup.bash
adb reverse tcp:8001 tcp:8001

ros2 launch morla_operator_control operator_control.launch.py \
  start_router:=false voice_source:=both dry_run:=true
```

`voice_source` 支持 `host`、`quest`、`both`、`off`。本机只有一个输入时选对应来源，避免启动无设备的一路。`start_router:=false` 是因为 Profile 已启动路由器；旧启动入口未启动路由器时使用 true。不要同时运行两份路由器。输入节点与录制节点生命周期独立，退出语音输入不会关闭已有录制。

Quest 需要先在 Windows 用 Unity 6000.0.65f1 编译更新后的 `/home/loopkok/morla-xr` 源码，安装新 APK；旧 APK 不包含麦克风功能。在应用中打开 **Voice Mic**，允许麦克风并 Start Stream。默认有线连接使用与追踪相同的主机地址、独立端口 8001；启动脚本 `nero_web_start.sh` 已加入该 reverse 映射。只配置 reverse 不会替代 APK 更新。

**新版一键脚本已经补齐 8000 追踪测试接收服务。** 使用 Quest 时，Web 自动启动仅接收 Quest 数据的节点，配合 8001 语音即可在没有机器人 Profile、没有机械臂驱动时点击 Start Stream 测试。`--operator-dry-run` 仍只识别不执行操作。`--no-quest` 默认不启动追踪测试接收器。

点击 Web「启动 Profile」时，会先停止测试接收器并等待其退出，再由 Profile 启动带正式坐标契约的接收器。切换连接时 Quest 可能报告断开，**重新点击 Start Stream 即可**；不会自动使能或自动恢复遥操。Profile 停止后，Web 会恢复测试接收器，退出 Web 会一并关闭它。不要另开独立接收器；如果 8000 被外部接收器占用，Web 不会擅自终止该进程，启动 Profile 会提示先停止它。

只运行旧的 Web 入口时，需加 `quest_preview:=true` 才启用这一功能；默认直接 `ros2 launch morla_web_manager web_manager.launch.py` 仍保持原行为。可检查当前模式：

```bash
curl -s http://127.0.0.1:8080/api/v1/state | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"]["quest_preview"])'
```

`listening` 表示测试接收器已监听，`profile` 表示端口已交给机器人栈，`external_port_owner` 表示存在外部占用。测试接收器提供通用输入格式，不能替代正式 Profile 中的坐标契约配置。

检查识别、设备和操作结果：

```bash
ros2 topic echo /morla/operator/sources
ros2 topic echo /morla/operator/transcripts
ros2 topic echo /morla/operator/events
```

确认识别文本与指令匹配后，停止上述语音 launch，去掉 `dry_run:=true` 再启动，双手摆好后说完整指令。`dry_run` 禁止本次启动的语音和脚踏发出操作请求；它不拦截原有 Web／手柄操作。指定 ALSA 麦克风示例（设备名按 `arecord -l` 的实际结果替换）：

```bash
ros2 launch morla_operator_control operator_control.launch.py \
  start_router:=false voice_source:=host \
  microphone_device:=alsa:plughw:CARD=Device,DEV=0
```

不指定设备则使用默认输入。修改唤醒词、置信度、Quest 监听地址及端口可编辑 `config/operator_control.yaml` 或用 `config:=/absolute/path/custom.yaml`。无线 Quest 需要将 `quest_bind` 配置为主机可接入地址；默认是 127.0.0.1，仅用于 ADB reverse。

## 说什么与什么时候说

- “机器人开始跟随”：在新鲜实测状态上重锚定并开始遥操。听到确认提示前保持起始手势；已在跟随时语音重复此命令不重新锚定。
- “机器人暂停跟随”：冻结跟随，保留电机保持；它不是物理急停，也不会自动关闭录制。
- “机器人开始采集”：遥操已确认跟随后，从 IDLE 开始录制新段。
- “机器人停止采集”：结束并保存当前段，实际保存完成后才反馈“本段已保存”。
- “机器人暂停采集”、“机器人继续采集”：沿用原采集状态机的 pause/resume；resume 的段切分语义保持原实现。
- “机器人丢弃本段”：先提出请求，五秒内另说“机器人确认丢弃”才删除当前段；也可说“机器人取消操作”取消待确认的丢弃。取消不会撤销已经执行的动作或保存。
- “机器人交还控制”：沿用 HITL 的 release，只有 HUMAN 等有效状态下执行。

`confirmed` 是系统返回的最终操作回执，不需要另说“确认”才能开始跟随。控制协调器先下发启动信号，再等待写入者状态确认，因此机器人可能在提示音或 HUD 显示 `confirmed` 之前已经开始跟随。应在说口令之前摆好起始姿态，说完保持，收到确认后再明显移动；若操作失败，先检查 Web 或 `/morla/operator/events` 的原因。

“开始遥操／开始录制／结束采集／结束保存”也支持，完整别名见 `commands.py`。没有归零、移至工作位、使能或任意自由文本执行功能。必须有唤醒词且整句匹配；否定句、低置信度、部分识别结果不执行。两麦克风相同命令两秒内去重；不要根据收到请求就开始操作，等待最终确认提示。语音识别是普通控制入口，物理急停保持独立。

Quest HUD 显示最终结果并响提示音；主机安装 `espeak-ng` 时读出中文，否则使用 `aplay` 短提示音。没有物理扬声器时仍可看 `/morla/operator/events` 和 Web 状态。原始音频只在内存中用于识别，不写入数据集；操作来源及请求编号记录在元数据事件里，不增加 action/state 维度。

## 之后接 USB 踏板

选择明确的一只设备，通过其稳定的 `/dev/input/by-id/...-event-kbd` 路径接入，不扫描或抓取其他键盘：

```bash
ls -l /dev/input/by-id/
# 确认设备读取权限和实际按键码后，替换下面路径。
ros2 launch morla_operator_control operator_control.launch.py \
  start_router:=false voice_source:=both \
  pedal_device:=/dev/input/by-id/usb-YOUR_PEDAL-event-kbd
```

默认键码 F13=开始/暂停跟随、F14=开始/结束保存、F15=长踩 1.5 秒后释放丢弃。设备若输出其他键码，在 YAML 的 `bindings` 中修改。长按重复事件不执行；启动/重连时已按下的键必须释放后重新按下。断开已经连接的踏板会提出暂停跟随请求；保存过程中仍可以暂停跟随。脚踏设备未连接或无读取权限会显示 disconnected/error，不会自动启用任何机械臂。

## 新增输入源的接口

来源只需发布 `std_msgs/String` JSON 到 `/morla/operator/command`：

```json
{"version":1,"request_id":"unique-id","source":"your_input","command":"teleop_start","sent_at":1792500000.0}
```

`sent_at` 必须是发送时的主机 Unix 时间，示例时间不能直接复用。Python 适配器可直接使用 `morla_operator_control.commands.request(command, source)` 生成。消息不持久化；两秒外的旧命令、重复请求编号、未知动作会被拒绝。事件阶段包括 submitted、confirmed、failed、rejected、confirmation_required；submitted 不代表操作成功。

控制协调器的 operation_id 与最终状态关联；采集侧支持带 request_id、期望状态、期望段号和 Profile 的结构化命令，并从 `/data_collect/control_ack` 回执实际执行结果。重放请求不重复创建或删除段，错误段号不能删除新段，保存异常不能播报保存成功。

Quest 传输协议及 Windows 编译说明位于 morla-xr 的 `docs/VOICE_CONTROL.md`。识别依赖参考 [Vosk 官方模型](https://alphacephei.com/vosk/models)，USB 输入接口参考 [python-evdev](https://python-evdev.readthedocs.io/en/latest/tutorial.html)。

## 本次验证记录（2026-10-20）

- MORLA 三个受影响包已在 Ubuntu 编译；全部 12 个 Profile 校验通过。
- 统一控制、手柄桥接、音频传输及遥操协调器：133 项通过；采集回归：132 项通过、1 项跳过。
- 使用实际 Vosk 中文模型和合成语音检查开始/暂停跟随、开始/结束采集、丢弃/确认丢弃；否定句不执行。TCP 分片、重放、过期帧及断连后的识别结果也已验证。
- Quest 源码已推送到 [morla-xr 提交 da6fd0a](https://github.com/loopkok/morla-xr/commit/da6fd0a52a97eca71e38d2d2e0ee480bf4838f5d)。Windows 先 `git pull --ff-only origin main`，再用项目版本 Unity 编译安装 APK。
- Quest C# 已做语法解析、Android 清单 XML 检查；未在这里编译 APK。物理麦克风、Quest 真机拾音和 USB 踏板尚需接入后验收，不能把合成音频测试视为真机验收。
