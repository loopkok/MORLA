#!/usr/bin/env bash
# Nero 双臂 + 双 XHand：USB 检查、CAN、串口权限、环境、清场、Quest、前台 Web。
set -Eeuo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS=${MORLA_WS:-$(dirname -- "$SCRIPT_DIR")}
USE_QUEST=true
QUEST_SERIAL=${ANDROID_SERIAL:-}
DRY_RUN=false
VOICE_SOURCE=auto
MICROPHONE_DEVICE=
PEDAL_DEVICE=
OPERATOR_CONFIG=
OPERATOR_DRY_RUN=false
log() { printf '\n[Nero] %s\n' "$*"; }
die() { printf '\n[Nero] 错误：%s\n' "$*" >&2; exit 1; }
usage() {
    cat <<'EOF'
用法：bash scripts/nero_web_start.sh [选项]
  --no-quest          跳过全部 ADB/Quest 步骤；仍启动 Web
  --quest-serial ID   选择 adb devices 中的 Quest 序列号（多个设备时必填）
  --voice-source SRC  quest / host / both / off；默认 quest，--no-quest 时默认 off
  --microphone-device DEV  主机麦克风，如 alsa:plughw:CARD=Device,DEV=0
  --pedal-device PATH USB 脚踏的 /dev/input/by-id/...-event-kbd 路径，默认不接入
  --operator-config PATH  可选语音/脚踏 YAML 配置
  --operator-dry-run  启动识别/脚踏预览，但不发送操作命令（Web 正常启动）
  --ws PATH          工作区路径，默认 MORLA_WS 或脚本所在仓库
  --dry-run          只检查和打印计划，不激活、不授权、不停止、不启动任何服务
  -h, --help         显示帮助
默认顺序：检查两路 USB-CAN/两只串口手 → find CAN → 激活 CAN → ttyUSB 权限
          → source Humble/工作区 → 终止旧 MORLA 栈及 8080 占用者
          → Quest 唤醒及 reverse 8000/8001/8765 → 启动 Web :8080 和所选语音/脚踏。
Quest 应用需安装新版 APK，打开 Voice Mic、允许麦克风并 Start Stream。
统一路由器随 Web 中的 Profile 启动；本脚本仅启动输入，未启动 Profile 时不会执行命令。
Ctrl+C 一并停止 Web 与本次启动的输入节点，不留下语音后台服务。
使用 Quest 时同时启动 8000 追踪测试接收器；Web 启动 Profile 时自动释放端口。
本脚本不启动机器人 Profile、不使能电机；之后在 Web 中操作。
EOF
}
while (($#)); do
    case "$1" in
        --no-quest) USE_QUEST=false; shift ;;
        --quest-serial) (($# >= 2)) || die '--quest-serial 缺少序列号'; QUEST_SERIAL=$2; shift 2 ;;
        --voice-source) (($# >= 2)) || die '--voice-source 缺少来源'; VOICE_SOURCE=$2; shift 2 ;;
        --microphone-device) (($# >= 2)) || die '--microphone-device 缺少设备'; MICROPHONE_DEVICE=$2; shift 2 ;;
        --pedal-device) (($# >= 2)) || die '--pedal-device 缺少路径'; [[ -n "$2" ]] || die '--pedal-device 不能为空'; PEDAL_DEVICE=$2; shift 2 ;;
        --operator-config) (($# >= 2)) || die '--operator-config 缺少路径'; [[ -n "$2" ]] || die '--operator-config 不能为空'; OPERATOR_CONFIG=$2; shift 2 ;;
        --operator-dry-run) OPERATOR_DRY_RUN=true; shift ;;
        --ws) (($# >= 2)) || die '--ws 缺少路径'; WS=$2; shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "未知选项：$1（使用 --help 查看用法）" ;;
    esac
done
if [[ "$VOICE_SOURCE" == auto ]]; then
    if $USE_QUEST; then VOICE_SOURCE=quest; else VOICE_SOURCE=off; fi
fi
case "$VOICE_SOURCE" in quest|host|both|off) ;; *) die '--voice-source 只能是 quest、host、both 或 off' ;; esac
if [[ -n "$OPERATOR_CONFIG" ]]; then
    [[ -f "$OPERATOR_CONFIG" ]] || die "配置不存在：$OPERATOR_CONFIG"
    OPERATOR_CONFIG=$(cd -- "$(dirname -- "$OPERATOR_CONFIG")" && pwd)/$(basename -- "$OPERATOR_CONFIG")
fi
[[ $(uname -s) == Linux ]] || die '本脚本在连接硬件的 Ubuntu/ROS 主机上运行'
[[ -d "$WS" ]] || die "工作区不存在：$WS"
WS=$(cd -- "$WS" && pwd)
export PATH="/usr/bin:/bin:$PATH"
FIND_CAN="$WS/scripts/find_all_can_port.sh"
# Vendor discovery is read-only; never fall back to its unconfigured activation mapping.
[[ -f "$FIND_CAN" ]] || FIND_CAN="$WS/third_party/pyAgxArm/pyAgxArm/scripts/linux/find_all_can_port.sh"
ACTIVATE_CAN="$WS/scripts/can_muti_activate.sh"
STOP_HELPER="$SCRIPT_DIR/morla_stop_processes.py"
for file in "$FIND_CAN" "$ACTIVATE_CAN" "$STOP_HELPER" /opt/ros/humble/setup.bash "$WS/install/setup.bash" "$WS/src/common/morla_web_manager/web/dist/index.html"; do
    [[ -f "$file" ]] || die "缺少文件：$file。请先编译工作区/前端，并配置主机 CAN 激活脚本。"
done
for cmd in ip ethtool candump python3 ss sudo timeout; do
    command -v "$cmd" >/dev/null || die "缺少命令：$cmd"
done
if $USE_QUEST; then command -v adb >/dev/null || die '缺少 adb；安装后重试，或使用 --no-quest'; fi
cd -- "$WS"

log '1/7 检查 USB-CAN 和串口设备'
mapfile -t CAN_DEVICES < <(ip -j link show type can | /usr/bin/python3 -c 'import json,sys; print("\n".join(row["ifname"] for row in json.load(sys.stdin)))')
((${#CAN_DEVICES[@]} >= 2)) || die '未检测到两路 CAN 接口；检查双臂 USB-CAN 接线、电源和 gs_usb 驱动'
printf 'CAN：%s\n' "${CAN_DEVICES[*]}"
shopt -s nullglob
SERIAL_DEVICES=(/dev/ttyUSB*)
((${#SERIAL_DEVICES[@]} >= 2)) || die '未检测到两个 ttyUSB 串口；检查双 XHand USB-RS485 连接'
for dev in "${SERIAL_DEVICES[@]}"; do [[ -c "$dev" ]] || die "不是串口设备：$dev"; done
ls -l -- "${SERIAL_DEVICES[@]}"
if [[ -d /dev/serial/by-id ]]; then ls -l /dev/serial/by-id/; fi
log "find CAN：bash $FIND_CAN"
if ! $DRY_RUN; then sudo -v; bash "$FIND_CAN"; fi

log "2/7 激活 CAN：bash $ACTIVATE_CAN"
if ! $DRY_RUN; then
    bash "$ACTIVATE_CAN"
    /usr/bin/python3 - <<'PY'
import json, subprocess, sys
rows=json.loads(subprocess.check_output(['ip','-details','-json','link','show','type','can']))
rows={r['ifname']:r for r in rows}
for name in ('can_nero_left','can_nero_right'):
    row=rows.get(name,{})
    details=row.get('linkinfo',{}).get('info_data',{})
    bitrate=details.get('bittiming',{}).get('bitrate')
    if 'UP' not in row.get('flags',[]) or bitrate!=1000000 or details.get('state')=='BUS-OFF':
        sys.exit(f'{name} 未就绪：需要 UP、1000000 bit/s 且非 BUS-OFF；实际：{row}')
    print(f'{name}: UP, bitrate={bitrate}, CAN state={details.get("state")}')
PY
fi

log '3/7 为当前插入的 /dev/ttyUSB* 赋予读写权限（不加执行权限）'
if ! $DRY_RUN; then
    sudo chmod a+rw -- "${SERIAL_DEVICES[@]}"
    for dev in "${SERIAL_DEVICES[@]}"; do [[ -r "$dev" && -w "$dev" ]] || die "串口仍不可读写：$dev"; done
fi

log '4/7 加载 /opt/ros/humble/setup.bash 和工作区 install/setup.bash'
if ! $DRY_RUN; then
    # ROS setup scripts may read unset variables.
    set +u
    source /opt/ros/humble/setup.bash
    source "$WS/install/setup.bash"
    set -u
    command -v ros2 >/dev/null || die 'source 后仍找不到 ros2'
    ros2 pkg prefix morla_web_manager >/dev/null || die '未安装 morla_web_manager，请先编译'
    if $USE_QUEST; then
        ros2 pkg prefix morla_input >/dev/null || die '未安装 morla_input，请先编译'
    fi
    if [[ "$VOICE_SOURCE" != off || -n "$PEDAL_DEVICE" ]]; then
        ros2 pkg prefix morla_operator_control >/dev/null || die '未安装 morla_operator_control，请先编译'
        # Fail before stopping the user's stack; this import opens no audio devices.
        if [[ "$VOICE_SOURCE" != off ]]; then
            /usr/bin/python3 - <<'PY' || die '语音依赖/模型未准备好；先运行 bash scripts/setup_operator_audio.sh，或 --voice-source off'
from morla_operator_control.asr import load_optional_dependencies
load_optional_dependencies()
import vosk
PY
            if [[ -z "$OPERATOR_CONFIG" ]]; then
                [[ -d "$HOME/.local/share/morla/models/vosk-model-small-cn-0.22/am" ]] || die '缺少中文模型；先运行 bash scripts/setup_operator_audio.sh'
            fi
        fi
        if [[ -n "$PEDAL_DEVICE" ]]; then
            /usr/bin/python3 -c 'import evdev' || die '脚踏依赖缺失：安装 python3-evdev'
            [[ -c "$PEDAL_DEVICE" && -r "$PEDAL_DEVICE" ]] || die "脚踏不是可读字符设备：$PEDAL_DEVICE；确认设备路径和读取权限"
        fi
    fi
fi

log '5/7 清理旧 MORLA Web（所有端口）、Profile/ROS launch 及子进程，并释放 8080'
if $DRY_RUN; then
    /usr/bin/python3 "$STOP_HELPER" --workspace "$WS" --web-port 8080 --dry-run
else
    sudo /usr/bin/python3 "$STOP_HELPER" --workspace "$WS" --web-port 8080 --exclude-pid "$$"
    timeout 8s ros2 daemon stop >/dev/null 2>&1 || true
fi

log '6/7 Quest 3'
if ! $USE_QUEST; then
    printf '%s\n' '--no-quest：跳过 adb devices、唤醒和 reverse；Web 可独立运行。'
elif $DRY_RUN; then
    printf '%s\n' '计划：adb devices -l → 选择唯一已授权设备 → adb [-s ID] shell input keyevent KEYCODE_WAKEUP'
    printf '%s\n' '计划：adb [-s ID] reverse tcp:8000 tcp:8000；reverse tcp:8001 tcp:8001；reverse tcp:8765 tcp:8765'
else
    adb start-server >/dev/null
    adb devices -l
    if [[ -z "$QUEST_SERIAL" ]]; then
        mapfile -t ADB_DEVICES < <(adb devices | awk 'NR > 1 && NF >= 2 {print $1}')
        ((${#ADB_DEVICES[@]} == 1)) || die '需要且只能有一个 ADB 设备；多个设备请用 --quest-serial ID，无 Quest 请用 --no-quest'
        QUEST_SERIAL=${ADB_DEVICES[0]}
    fi
    ADB=(adb -s "$QUEST_SERIAL")
    [[ $("${ADB[@]}" get-state 2>/dev/null) == device ]] || die 'Quest 未授权或离线；戴上头显允许 USB 调试后重试，或使用 --no-quest'
    "${ADB[@]}" shell input keyevent KEYCODE_WAKEUP
    # Replace only MORLA mappings; preserve unrelated reverse ports.
    "${ADB[@]}" reverse tcp:8000 tcp:8000
    "${ADB[@]}" reverse tcp:8001 tcp:8001
    "${ADB[@]}" reverse tcp:8765 tcp:8765
    MAPPINGS=$("${ADB[@]}" reverse --list)
    printf '%s\n' "$MAPPINGS"
    for port in 8000 8001 8765; do
        awk -v port="tcp:$port" '$2 == port && $3 == port {found=1} END {exit !found}' <<< "$MAPPINGS" || die "ADB reverse $port 校验失败"
    done
    export ANDROID_SERIAL="$QUEST_SERIAL"
    printf '%s\n' '请在 Quest 的 HTS 中选择 localhost:8000 并手动 Start Stream；本步骤只唤醒设备。'
fi

log '7/7 前台启动 Web 与操作输入：http://127.0.0.1:8080（同局域网用本机 IP）'
export MORLA_WS="$WS"
export MORLA_DATA_ROOT=${MORLA_DATA_ROOT:-$HOME/morla_data}
export MORLA_WEB_MANAGER_DIST="$WS/src/common/morla_web_manager/web/dist"
WEB_COMMAND=(ros2 launch morla_web_manager web_manager.launch.py
    web_host:=0.0.0.0 web_port:=8080 "voice_source:=$VOICE_SOURCE"
    "operator_dry_run:=$OPERATOR_DRY_RUN" "quest_preview:=$USE_QUEST")
[[ -z "$MICROPHONE_DEVICE" ]] || WEB_COMMAND+=("microphone_device:=$MICROPHONE_DEVICE")
[[ -z "$PEDAL_DEVICE" ]] || WEB_COMMAND+=("pedal_device:=$PEDAL_DEVICE")
[[ -z "$OPERATOR_CONFIG" ]] || WEB_COMMAND+=("operator_config:=$OPERATOR_CONFIG")
printf '语音来源：%s；脚踏：%s；输入只预览：%s\n' "$VOICE_SOURCE" "${PEDAL_DEVICE:-未启用}" "$OPERATOR_DRY_RUN"
if $DRY_RUN; then
    printf '计划：'; printf '%q ' "${WEB_COMMAND[@]}"; printf '\n'
    log '只读预览结束；未激活 CAN、未改权限、未清场、未操作 ADB、未启动 Web。'
    exit 0
fi
mkdir -p -- "$MORLA_DATA_ROOT"
printf '%s\n' '保持本终端开启，Ctrl+C 一并停止 Web 与语音/脚踏；在 Web 中选择并启动 Profile。'
if [[ "$VOICE_SOURCE" == quest || "$VOICE_SOURCE" == both ]]; then
    printf '%s\n' 'Quest：安装新版 APK，打开 Voice Mic、允许麦克风并 Start Stream；不要另开一份语音 launch。'
fi
if $USE_QUEST; then
    printf '%s\n' '8000 追踪测试接收器随 Web 启动；启动 Profile 时自动切换，Quest 若断开请重新 Start Stream。'
fi
printf '%s\n' 'USB-CAN/串口存在不代表控制器反馈正常；启动 Profile 后再检查连接、准备并使能。'
exec "${WEB_COMMAND[@]}"
