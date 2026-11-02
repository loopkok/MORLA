#!/usr/bin/env bash
# One-click CR5 + Vader 5 Pro simulation bench.
#
# Starts the composable session host (which also serves the web UI), opens the
# web page, and mirrors the scene in a native MuJoCo window. No arguments.
#
#   tools/start_cr5_pad_sim.sh             # full bench
#   OPEN_BROWSER=0 tools/start_cr5_pad_sim.sh   # skip opening the page
#   NO_VIEWER=1 tools/start_cr5_pad_sim.sh      # skip the MuJoCo window
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ASSEMBLY="$REPO/src/common/morla_vader5_input/config/cr5.sim.yaml"
DEPLOYMENT="$REPO/src/common/morla_vader5_input/config/input.real.yaml"
STORE="$HOME/.local/share/morla-cr5-sim"
PORT="${PORT:-8080}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}" ROS_LOCALHOST_ONLY=1
export DISPLAY="${DISPLAY:-:1}"

# ROS setup scripts read unset variables; keep `set -u` away from them.
set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

# ROS Humble is built against the system 3.10 interpreter; a uv/conda python
# earlier in PATH cannot import the workspace packages.
PY=/usr/bin/python3

if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
  # An earlier session is still up: reuse it instead of failing on the port.
  echo "端口 $PORT 上已有会话在运行，直接打开网页: http://127.0.0.1:$PORT/"
  if [ "${OPEN_BROWSER:-1}" = 1 ]; then xdg-open "http://127.0.0.1:$PORT/" >/dev/null 2>&1 || true; fi
  if [ "${NO_VIEWER:-0}" != 1 ]; then
    "$PY" "$REPO/tools/cr5_live_viewer.py" --hz 50 >/dev/null 2>&1 || true
  fi
  exit 0
fi

# The Vader's vendor (config) hidraw node changes across replugs; resolve it
# now instead of relying on the device path recorded in the deployment.
mapfile -t FOUND < <("$PY" - <<'PY'
from vader5_sdk._vendor.device import find_config_hidraws
for path, description in find_config_hidraws():
    print(f'{path}\t{description}')
PY
)
DEVICE_ARGS=()
if [ "${#FOUND[@]}" -gt 1 ]; then
  echo "找到多个 Flydigi 连接，请只接一个（或一个 dongle）后重试:" >&2
  printf '  %s\n' "${FOUND[@]}" >&2
  exit 1
elif [ "${#FOUND[@]}" -eq 1 ]; then
  DEVICE="${FOUND[0]%%$'\t'*}"
  echo "手柄: ${FOUND[0]}"
  DEVICE_ARGS=(--input-device "$DEVICE")
else
  echo "警告: 未检测到 Flydigi 手柄，沿用部署文件里的 device；插好后重开本脚本。" >&2
fi

VIEWER_PID=""
cleanup() { if [ -n "$VIEWER_PID" ]; then kill "$VIEWER_PID" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM

HOST="$(ros2 pkg prefix morla_web_manager)/lib/morla_web_manager/morla-composable-session"
ARGS=(--assembly "$ASSEMBLY" --deployment "$DEPLOYMENT" --input-backend real
      --port "$PORT" --store "$STORE")
if [ "${#DEVICE_ARGS[@]}" -gt 0 ]; then ARGS+=("${DEVICE_ARGS[@]}"); fi

echo "启动 CR5 MuJoCo 会话…"
"$HOST" "${ARGS[@]}" &
HOST_PID=$!

READY=0
for _ in $(seq 1 60); do
  if curl -sf "http://127.0.0.1:$PORT/api/v2/active-session" >/dev/null 2>&1; then READY=1; break; fi
  if ! kill -0 "$HOST_PID" 2>/dev/null; then echo "会话主机已退出，见上方日志。" >&2; exit 1; fi
  sleep 1
done
if [ "$READY" != 1 ]; then echo "等待会话超时。" >&2; exit 1; fi

echo
echo "网页: http://127.0.0.1:$PORT/"
echo "  组合配置 → 连接输入设备 → 激活运动许可"
echo "Ctrl-C 结束会话。"
echo
if [ "${OPEN_BROWSER:-1}" = 1 ]; then xdg-open "http://127.0.0.1:$PORT/" >/dev/null 2>&1 || true; fi
if [ "${NO_VIEWER:-0}" != 1 ]; then
  "$PY" "$REPO/tools/cr5_live_viewer.py" --hz 50 >/dev/null 2>&1 &
  VIEWER_PID=$!
fi

wait "$HOST_PID"
