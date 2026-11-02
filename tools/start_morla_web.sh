#!/usr/bin/env bash
# One-click MORLA web console.
#
# Starts the manager web server only. Sessions are built and started in the
# browser's 组合配置 tab, so this script needs no arguments.
#
#   tools/start_morla_web.sh                # start + open browser
#   OPEN_BROWSER=0 tools/start_morla_web.sh # skip opening the page
#   PORT=9090 tools/start_morla_web.sh      # different port
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PORT="${PORT:-8080}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}" ROS_LOCALHOST_ONLY=1
export MORLA_WEB_MANAGER_PORT="$PORT"

# ROS setup scripts read unset variables; keep `set -u` away from them.
set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

URL="http://127.0.0.1:$PORT/"
if curl -sf "$URL" >/dev/null 2>&1; then
  echo "MORLA 控制台已在运行，直接打开网页: $URL"
  if [ "${OPEN_BROWSER:-1}" = 1 ]; then xdg-open "$URL" >/dev/null 2>&1 || true; fi
  exit 0
fi

BIN="$(ros2 pkg prefix morla_web_manager)/lib/morla_web_manager/morla_web_manager"
echo "启动 MORLA 网页控制台…"
"$BIN" &
WEB_PID=$!
trap 'kill "$WEB_PID" 2>/dev/null || true' EXIT INT TERM

READY=0
for _ in $(seq 1 60); do
  if curl -sf "http://127.0.0.1:$PORT/api/v2/components" >/dev/null 2>&1; then READY=1; break; fi
  if ! kill -0 "$WEB_PID" 2>/dev/null; then echo "控制台进程已退出，见上方日志。" >&2; exit 1; fi
  sleep 1
done
if [ "$READY" != 1 ]; then echo "等待控制台超时。" >&2; exit 1; fi

echo
echo "网页: $URL"
echo "  组合配置 → 添加组件与输入绑定 → 保存并启动会话 → 连接输入 → 激活运动许可"
echo "Ctrl-C 结束控制台（同时结束进程内的会话）。"
echo
if [ "${OPEN_BROWSER:-1}" = 1 ]; then xdg-open "$URL" >/dev/null 2>&1 || true; fi

wait "$WEB_PID"
