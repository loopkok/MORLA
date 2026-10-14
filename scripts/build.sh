#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE=symlink

while (($#)); do
  case "$1" in
    --mode)
      [[ $# -ge 2 ]] || { echo "--mode requires symlink or regular" >&2; exit 2; }
      MODE="$2"
      shift 2
      ;;
    --help|-h)
      cat <<'EOF'
Usage: scripts/build.sh [--mode symlink|regular] [colcon build options...]

Build the complete MORLA workspace with the ROS Humble system Python.
The default symlink mode is convenient for development; regular mode checks
that installed assets do not depend on source-tree symlinks.
EOF
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

case "$MODE" in
  symlink|regular) ;;
  *) echo "unsupported build mode: $MODE (expected symlink or regular)" >&2; exit 2 ;;
esac

ROS_SETUP=/opt/ros/humble/setup.bash
[[ -r "$ROS_SETUP" ]] || { echo "ROS Humble setup not found: $ROS_SETUP" >&2; exit 1; }
set +u
source "$ROS_SETUP"
set -u
export PATH="/usr/bin:/opt/ros/humble/bin:/bin:/usr/sbin:/sbin:${PATH:-}"

COLCON_ARGS=(
  build
  --base-paths "$WS/src"
  --build-base "$WS/build"
  --install-base "$WS/install"
  --event-handlers console_direct+
  --cmake-args
  -DPython3_EXECUTABLE=/usr/bin/python3
  -DPYTHON_EXECUTABLE=/usr/bin/python3
)
if [[ "$MODE" == symlink ]]; then
  COLCON_ARGS+=(--symlink-install)
fi

printf 'MORLA workspace: %s\nBuild mode: %s\n' "$WS" "$MODE"
exec /usr/bin/colcon "${COLCON_ARGS[@]}" "$@"
