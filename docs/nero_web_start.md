# Nero 双臂 + 双 XHand 一键启动 Web

在连接设备的 Ubuntu / ROS 2 Humble 主机上运行。先完成 ROS 工作区及 Web 前端编译；脚本不编译代码。默认工作区为脚本所在仓库，可由 `MORLA_WS` 或 `--ws PATH` 覆盖。

```bash
cd /home/loopkok/MORLA

# 使用 Quest 3（默认）：Web + Quest 追踪测试接收 + 语音接收，含 reverse 8000/8001/8765
bash scripts/nero_web_start.sh

# 不使用 Quest 3，也可以启动 Web 做连接检查、工作位、HOME 等操作
bash scripts/nero_web_start.sh --no-quest

# 同时使用主机与 Quest 麦克风
bash scripts/nero_web_start.sh --voice-source both

# 不连接 Quest，使用主机麦克风
bash scripts/nero_web_start.sh --no-quest --voice-source host

# 保留 Quest 追踪，但关闭语音
bash scripts/nero_web_start.sh --voice-source off

# 首次检查识别：会启动 Web 和识别，但语音/脚踏不执行操作
bash scripts/nero_web_start.sh --operator-dry-run

# 后续接脚踏：路径换成实际设备，不接脚踏时无需此参数
bash scripts/nero_web_start.sh --pedal-device /dev/input/by-id/usb-YOUR_PEDAL-event-kbd

# 多个 ADB 设备时，显式选中 Quest
adb devices -l
bash scripts/nero_web_start.sh --quest-serial YOUR_QUEST_SERIAL

# 只预览：不更改 CAN/权限，不停止进程，不操作 ADB，不启动 Web
bash scripts/nero_web_start.sh --dry-run
bash scripts/nero_web_start.sh --no-quest --dry-run
```

脚本按以下顺序执行，任一必需步骤失败即退出：

1. 检查至少两路 CAN 接口和两个 `/dev/ttyUSB*` 字符设备，显示串口及 `/dev/serial/by-id/` 映射；执行 `bash scripts/find_all_can_port.sh`。若根目录缺少该发现脚本，使用仓库内 pyAgxArm 的只读发现脚本。
2. 执行主机已配置的 `bash scripts/can_muti_activate.sh`。需要正确设置当前 USB 口映射；本机为右 `3-2.1:1.0`、左 `3-2.2:1.0`。激活后检查 `can_nero_left`、`can_nero_right` 均 UP、1000000 bit/s 且非 BUS-OFF。不会回退到厂商未配置的激活脚本。
3. `sudo chmod a+rw /dev/ttyUSB*`，仅为当前已插入的串口赋予读写权限。拔插后设备权限可能重置，重跑脚本即可。不会把串口改写成固定 USB0/USB1，驱动仍使用原有串口配置。
4. 加载 `/opt/ros/humble/setup.bash`、`$MORLA_WS/install/setup.bash`，检查 Web ROS 包已安装；启用语音/脚踏时，也检查统一输入包、所需依赖和默认中文模型，指定脚踏时检查设备可读。缺少依赖会在停止旧服务之前报错。优先使用 `/usr/bin` 的系统运行时，避免 conda Python 混入 ROS。
5. 识别并打印旧 MORLA Web（任意端口）、ROS Profile/launch、驱动与子进程，同时终止 TCP 8080 的监听进程。先 SIGINT 正常退出，最多给 35 秒保存/清理，再 TERM/KILL 收尾；对识别到的 MORLA 系统服务停止自动重启单元，并复查残留和端口占用。停止当前环境的 ROS 2 daemon。保留 SSH、远程桌面、其他工程和其他端口的无关服务。无法释放端口或服务重新出现时退出，不带冲突继续启动。
6. 默认执行 `adb devices -l`，检查设备为已授权的 `device`；多设备需 `--quest-serial`，也可用 `ANDROID_SERIAL`。发送 `shell input keyevent KEYCODE_WAKEUP`，建立并校验 `reverse tcp:8000 tcp:8000`、`reverse tcp:8001 tcp:8001`、`reverse tcp:8765 tcp:8765`。8001 是新加入的 Quest 语音回传端口，只替换这三条映射，保留其他 reverse 端口。`--no-quest` 完全跳过本步，不需要安装 adb。
7. 设置 `MORLA_WS`、`MORLA_DATA_ROOT`（默认 `~/morla_data`）、`MORLA_WEB_MANAGER_DIST`，在一个前台 ROS launch 中启动 Web 和所选语音/脚踏输入；使用 Quest 时由 Web 管理 8000 追踪测试接收器：

```bash
ros2 launch morla_web_manager web_manager.launch.py \
  web_host:=0.0.0.0 web_port:=8080 voice_source:=quest quest_preview:=true
```

脚本通过 sudo 完成设备授权和进程清理；Web 使用普通用户运行，不保存密码。请保持终端开启，通过 Ctrl+C 一并停止 Web 和本次启动的语音/脚踏；Web 进程退出时也会关闭这些输入。浏览器访问 `http://127.0.0.1:8080`，其他电脑使用机器人主机 IP，例如 `http://192.168.0.231:8080`。

之后在 Web 中选择 `nero_dual_xhand` / 真机 → 启动 Profile → 检查连接 → 准备并使能 → 按需执行工作位、HOME 或开始遥操。脚本本身不启动 Profile、不下发电机使能或运动。接口存在和 CAN 激活只证明主机侧设备就绪，机械臂控制器的新鲜反馈仍需在 Web 中确认。

`--no-quest` 跳过本脚本的 ADB 步骤，语音默认也改为 off；可显式加 `--voice-source host` 使用主机麦克风。它不改变 Web 后续启动 Profile 的位姿输入配置，也不提供离线 Quest 遥操。使用 Quest 时，还需戴上头显允许 USB 调试，并在 HTS 内连接 `localhost:8000`、手动 Start Stream；唤醒指令不自动启动 HTS 应用。

语音默认使用 Quest 麦克风，不会打开主机麦克风；`--voice-source both` 才同时开启两路。Quest 必须安装新增语音功能的 APK，在应用中打开 **Voice Mic**、允许麦克风并 **Start Stream**。端口映射不等于应用已开始录音。

8000 的追踪测试接收器也随 Web 启动，因此不必先启动机器人 Profile 就能在 Quest 点击 Start Stream、测试语音。点击 Web「启动 Profile」时会先停止测试接收器并释放端口，再启动正式接收器；切换时 Quest 可能断开，重新点 Start Stream。Profile 停止后恢复测试接收器，退出 Web 后一起停止。8000 有外部占用时不会杀掉该进程，需操作者先停止它。

输入节点随脚本启动，统一命令路由器随 Web 中选定的 Profile 启动，避免重复路由。启动 Profile 并准备好机器人后，双手摆好说“机器人开始跟随”，等待确认，再说“机器人开始采集”。采集节点仍需先在 Web 启动；脚本不自动录制。**使用本脚本后，不要再单独启动一份 `operator_control.launch.py`。**

默认脚踏 F13=开始/暂停跟随、F14=开始/结束保存、F15=长踩 1.5 秒后释放丢弃。可通过 `--operator-config /absolute/path/custom.yaml` 自定义按键；主机麦克风可用 `--microphone-device alsa:plughw:CARD=Device,DEV=0` 指定。首次依赖/模型安装仍为 `bash scripts/setup_operator_audio.sh`，不在每次启动时自动下载。完整语音和脚踏说明见 [operator_control.md](operator_control.md)。

`--operator-dry-run` 会打开指定输入设备但只预览识别，不发送语音/脚踏操作；原有 Web/手柄照常工作。`--dry-run` 是整个启动脚本的只读预览，不启动任何节点，二者用途不同。

主机依赖：`iproute2`、`ethtool`、`can-utils`、`sudo`、系统 Python 3，使用 Quest 时另需 `adb`。

清理助手为 [`scripts/morla_stop_processes.py`](../scripts/morla_stop_processes.py)。软件验证：

```bash
bash -n scripts/nero_web_start.sh
python3 -m unittest discover -s scripts/tests -p 'test_morla_stop_processes.py' -v
```

测试只终止临时目录中的虚拟服务，使用随机空闲端口，不触及真实机器人或 8080。完整实机启动流程须由操作者运行上述启动命令。
