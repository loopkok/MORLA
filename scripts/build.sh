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
The default symlink mode installs to install/ for development; regular mode
installs to install_regular/ so its resource tree stays independent of an
existing symlink install.
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

# CMake caches the install prefix and AMENT_CMAKE_SYMLINK_INSTALL, so changing
# colcon flags alone does not switch an existing build tree between modes.
# Reconfigure CMake when either the mode or install base changes.
INSTALL_BASE="$WS/install"
if [[ "$MODE" == regular ]]; then
  INSTALL_BASE="$WS/install_regular"
fi
MODE_STAMP="$WS/build/.morla_build_mode"
STAMP_VALUE="$MODE:$INSTALL_BASE"
CLEAN_CMAKE_CACHE=()
if [[ ! -f "$MODE_STAMP" ]] || [[ "$(cat "$MODE_STAMP")" != "$STAMP_VALUE" ]]; then
  CLEAN_CMAKE_CACHE+=(--cmake-clean-cache)
fi
mkdir -p "$WS/build"
printf '%s\n' "$STAMP_VALUE" > "$MODE_STAMP"

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
  --install-base "$INSTALL_BASE"
  --event-handlers console_direct+
  "${CLEAN_CMAKE_CACHE[@]}"
  --cmake-args
  -DPython3_EXECUTABLE=/usr/bin/python3
  -DPYTHON_EXECUTABLE=/usr/bin/python3
)
if [[ "$MODE" == symlink ]]; then
  COLCON_ARGS+=(--symlink-install)
fi

printf 'MORLA workspace: %s\nBuild mode: %s\n' "$WS" "$MODE"
exec /usr/bin/colcon "${COLCON_ARGS[@]}" "$@"
