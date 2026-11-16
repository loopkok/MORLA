#!/usr/bin/env bash
# One-click MORLA web console.
#
# Starts the manager web server only. Sessions are built and started in the
# browser's 工作站 tab, so this script needs no assembly/deployment file.
# Run on the Linux ROS 2 Humble host; a local viewer needs its desktop session.
#
#   tools/start_morla_web.sh                # start + open browser; a console
#                                           #   already on the port is stopped
#   OPEN_BROWSER=0 tools/start_morla_web.sh # skip opening the page
#   PORT=9090 tools/start_morla_web.sh      # different port
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PORT="${PORT:-8080}"
if ! [[ "$PORT" =~ ^[0-9]{1,5}$ ]] || (( 10#$PORT < 1 || 10#$PORT > 65535 )); then
  echo "PORT 必须为 1–65535 的端口号。" >&2
  exit 1
fi
# Normalize leading zeros for both the URL and the Python server port.
PORT=$((10#$PORT))
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}" ROS_LOCALHOST_ONLY=1
export MORLA_WEB_MANAGER_PORT="$PORT"

if ! command -v curl >/dev/null 2>&1; then
  echo "缺少 curl，请先安装后重试。" >&2
  exit 1
fi
if ! command -v ss >/dev/null 2>&1; then
  echo "缺少 ss（iproute2），无法检测端口占用。" >&2
  exit 1
fi
URL="http://127.0.0.1:$PORT/"
probe() { curl --noproxy 127.0.0.1 --connect-timeout 1 --max-time 2 -sf "$1" >/dev/null 2>&1; }
open_browser() {
  if [ "${OPEN_BROWSER:-1}" = 1 ]; then
    if command -v xdg-open >/dev/null 2>&1; then
      xdg-open "$URL" >/dev/null 2>&1 || true
    else
      echo "请在浏览器中手动打开: $URL"
    fi
  fi
}

# PIDs listening on $PORT (only processes owned by this user are visible).
listener_pids() {
  ss -H -tlnp "sport = :$PORT" 2>/dev/null |
    grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u || true
}

proc_cmdline() {
  tr '\0' ' ' <"/proc/$1/cmdline" 2>/dev/null || true
}

# Send a signal to the process, and to the children it spawned when it leads
# its own process group, so a stopped console leaves nothing behind.
signal_pid() {
  local sig=$1 pid=$2 pgid
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
  if [ -n "$pgid" ] && [ "$pgid" = "$pid" ]; then
    kill "-$sig" -- "-$pgid" 2>/dev/null || true
  else
    kill "-$sig" "$pid" 2>/dev/null || true
  fi
}

# A console left over from a previous run keeps serving the old build, so stop
# it before starting; this is a restart, not a reuse.
stop_stale_console() {
  local pids pid cmd
  local -a morla_pids=() other_pids=()
  mapfile -t pids < <(listener_pids)
  [ "${#pids[@]}" -gt 0 ] || return 0

  for pid in "${pids[@]}"; do
    cmd=$(proc_cmdline "$pid")
    if [[ "$cmd" == *morla_web_manager* ]]; then
      morla_pids+=("$pid")
    else
      other_pids+=("$pid")
    fi
  done

  if [ "${#other_pids[@]}" -gt 0 ]; then
    echo "端口 $PORT 被其他程序占用，未自动结束：" >&2
    for pid in "${other_pids[@]}"; do
      echo "  PID $pid: $(proc_cmdline "$pid")" >&2
    done
    echo "请先停止该程序，或用 PORT=9090 指定其他端口。" >&2
    exit 1
  fi

  echo "检测到旧的 MORLA 控制台（PID ${morla_pids[*]}），正在停止…"
  for pid in "${morla_pids[@]}"; do signal_pid TERM "$pid"; done
  for _ in $(seq 1 40); do
    [ -z "$(listener_pids)" ] && break
    sleep 0.25
  done
  if [ -n "$(listener_pids)" ]; then
    echo "旧控制台未在 10 秒内退出，强制结束。" >&2
    for pid in "${morla_pids[@]}"; do signal_pid KILL "$pid"; done
    for _ in $(seq 1 20); do
      [ -z "$(listener_pids)" ] && break
      sleep 0.25
    done
  fi
  if [ -n "$(listener_pids)" ]; then
    echo "端口 $PORT 仍被占用，请手动检查。" >&2
    exit 1
  fi
  echo "旧控制台已停止。"
}

stop_stale_console

# Anything else still holding the port is not ours to kill.
if probe "$URL"; then
  echo "$URL 已被其他 HTTP 服务占用。请确认占用进程，或用 PORT=9090 指定其他端口。" >&2
  exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "未找到 /opt/ros/humble/setup.bash。此脚本用于已安装 ROS 2 Humble 的 Linux 主机。" >&2
  exit 1
fi
if [ ! -f "$REPO/install/setup.bash" ]; then
  echo "未找到 $REPO/install/setup.bash，请先在 MORLA 根目录完成 colcon build。" >&2
  exit 1
fi

# ROS setup scripts read unset variables; keep `set -u` away from them.
set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "加载环境后仍找不到 ros2，请检查 ROS 安装。" >&2
  exit 1
fi
BIN="$(ros2 pkg prefix morla_web_manager)/lib/morla_web_manager/morla_web_manager"
if [ ! -x "$BIN" ]; then
  echo "未找到可执行的 Web 入口 $BIN，请重新构建 morla_web_manager。" >&2
  exit 1
fi
echo "启动 MORLA 网页控制台…"
"$BIN" &
WEB_PID=$!
trap 'kill "$WEB_PID" 2>/dev/null || true' EXIT INT TERM HUP

READY=0
for _ in $(seq 1 60); do
  if probe "${URL}api/v2/components"; then READY=1; break; fi
  if ! kill -0 "$WEB_PID" 2>/dev/null; then echo "控制台进程已退出，见上方日志。" >&2; exit 1; fi
  sleep 1
done
if [ "$READY" != 1 ]; then echo "等待控制台超时。" >&2; exit 1; fi

echo
echo "网页: $URL"
echo "  工作站 → 载入仿真预设，或通过表单添加设备，无需提前编写配置文件。"
echo "  设备选型 → 安装与标定 → 遥操作设置 → 数据记录 → 检查与运行"
echo "  检查配置 → 检查运行依赖 → 保存配置 → 启动工作站"
echo "  连接输入 → 准备设备 → 开始遥操（Vader M3 / Quest 右 grip / Web / 键盘 S）"
echo "  网页与 MuJoCo 本地窗口在‘仿真显示’选择；本地窗口打开在服务主机桌面。"
echo "  裸手模式用 Web 或启用本页键盘后按 S 开始；暂停使用页面按钮。"
echo "先在页面停止工作站，再按 Ctrl-C 结束控制台。"
echo
open_browser

wait "$WEB_PID"
