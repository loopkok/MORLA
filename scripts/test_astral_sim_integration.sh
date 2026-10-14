#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="$(dirname "$WS")"
[[ -r /opt/ros/humble/setup.bash ]] || { echo "ROS Humble is required" >&2; exit 1; }
[[ -r "$WS/install/setup.bash" ]] || { echo "Run scripts/build.sh first" >&2; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PATH="/usr/bin:/opt/ros/humble/bin:/bin:/usr/sbin:/sbin:${PATH:-}"
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$WS/src/common/morla_core:$WS/src/common/morla_data_collect${PYTHONPATH:+:$PYTHONPATH}"

DOMAIN_ID="${MORLA_ASTRAL_TEST_ROS_DOMAIN_ID:-98}"
[[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] || { echo "MORLA_ASTRAL_TEST_ROS_DOMAIN_ID must be numeric" >&2; exit 2; }
((DOMAIN_ID <= 220)) || { echo "MORLA_ASTRAL_TEST_ROS_DOMAIN_ID must be <= 220" >&2; exit 2; }
export ROS_DOMAIN_ID="$DOMAIN_ID"

OUTPUT_ROOT="${MORLA_ASTRAL_TEST_OUTPUT_ROOT:-$(mktemp -d /tmp/morla-astral-sim-e2e.XXXXXX)}"
if [[ -n "${MORLA_ASTRAL_TEST_OUTPUT_ROOT:-}" ]]; then
  mkdir -p "$OUTPUT_ROOT"
  [[ -z "$(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -print -quit)" ]] || {
    echo "MORLA_ASTRAL_TEST_OUTPUT_ROOT must be empty: $OUTPUT_ROOT" >&2
    exit 2
  }
fi
OUTPUT_ROOT="$(cd "$OUTPUT_ROOT" && pwd)"
SESSION="$OUTPUT_ROOT/astral_left_gripper/smoke"
mkdir -p "$OUTPUT_ROOT/logs"
STACK_LOG="$OUTPUT_ROOT/logs/astral_sim_stack.log"
COLLECT_LOG="$OUTPUT_ROOT/logs/data_collect.log"
STACK_PID=""
COLLECT_PID=""

stop_launch() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  else
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  stop_launch "$COLLECT_PID"
  stop_launch "$STACK_PID"
}
trap cleanup EXIT INT TERM

echo "Starting Astral Profile simulation on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
setsid ros2 launch morla_bringup robot.launch.py \
  profile_id:=astral_left_gripper mode:=sim sdk_backend:=fake \
  with_input:=false with_video:=false with_head_teleop:=false \
  enable_viewer:=false >"$STACK_LOG" 2>&1 &
STACK_PID=$!
for _ in $(seq 1 100); do
  kill -0 "$STACK_PID" 2>/dev/null || { tail -80 "$STACK_LOG" >&2; exit 1; }
  if rg -q 'Astral MuJoCo sim ready' "$STACK_LOG"; then break; fi
  sleep 0.2
done
rg -q 'profile_joint_state_bridge|robot_state_publisher' "$STACK_LOG" || {
  tail -100 "$STACK_LOG" >&2
  echo "Astral profile did not launch measured-state TF nodes" >&2
  exit 1
}

setsid ros2 launch morla_data_collect data_collect.launch.py \
  profile_id:=astral_left_gripper runtime_mode:=sim session:=smoke \
  save_root:="$OUTPUT_ROOT" keyboard:=false vr_control:=false >"$COLLECT_LOG" 2>&1 &
COLLECT_PID=$!

if ! /usr/bin/python3 "$WS/scripts/test_astral_sim.py" \
    --output-root "$OUTPUT_ROOT" --duration "${MORLA_ASTRAL_TEST_DURATION_S:-6}"; then
  echo "Astral ROS simulation smoke failed; launch logs follow" >&2
  tail -100 "$STACK_LOG" >&2 || true
  tail -100 "$COLLECT_LOG" >&2 || true
  exit 1
fi

cleanup
trap - EXIT INT TERM

export MORLA_DATA_PYTHON="${MORLA_DATA_PYTHON:-$SDK_ROOT/VLA/openpi/.venv/bin/python}"
"$WS/scripts/data_pipeline.sh" align --session "$SESSION"
"$WS/scripts/data_pipeline.sh" validate --session "$SESSION" --min-duration 4
"$WS/scripts/data_pipeline.sh" export-openpi \
  --session "$SESSION" --output "$OUTPUT_ROOT/lerobot-v21" \
  --codec h264 --image-size 64

"$MORLA_DATA_PYTHON" - "$OUTPUT_ROOT/lerobot-v21/meta/morla_schema.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    schema = json.load(stream)
if schema.get("profile_id") != "astral_left_gripper":
    raise SystemExit(f"unexpected exported profile: {schema.get('profile_id')!r}")
if schema.get("state_dim") != 8 or schema.get("action_dim") != 8:
    raise SystemExit(f"unexpected LeRobot dimensions: {schema.get('state_dim')}/{schema.get('action_dim')}")
print(f"LeRobot v2.1 schema verified: profile={schema['profile_id']} state/action=8/8")
PY

echo "Astral simulation, collection, alignment, validation and LeRobot export passed. Artifacts: $OUTPUT_ROOT"
