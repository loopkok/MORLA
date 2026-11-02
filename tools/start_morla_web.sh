#!/usr/bin/env bash
# One-click MORLA web console.
#
# Starts the manager web server only. Sessions are built and started in the
# browser's 工作站 tab, so this script needs no assembly/deployment file.
# Run on the Linux ROS 2 Humble host; a local viewer needs its desktop session.
#
#   tools/start_morla_web.sh                # start + open browser
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

# Check the public MORLA API, not just an arbitrary HTTP page on this port.
if probe "${URL}api/v2/components"; then
  echo "MORLA 控制台已在运行，打开 工作站: $URL"
  echo "已有服务会继续使用原来的代码；更新后请先停止旧服务再重新启动。"
  open_browser
  exit 0
fi
if probe "$URL"; then
  echo "$URL 已有 HTTP 服务，但没有 MORLA 公共 API。请确认占用进程，或用 PORT=9090 指定其他端口。" >&2
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
trap 'kill "$WEB_PID" 2>/dev/null || true' EXIT INT TERM

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
