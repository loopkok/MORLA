#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -r /opt/ros/humble/setup.bash ]] || { echo "ROS Humble is required" >&2; exit 1; }
[[ -r "$WS/install/setup.bash" ]] || { echo "Run scripts/build.sh first" >&2; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PATH="/usr/bin:/opt/ros/humble/bin:/bin:/usr/sbin:/sbin:${PATH:-}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_LOCALHOST_ONLY=1
exec bash "$WS/src/common/morla_policy_inference/scripts/run_policy_inference_tests.sh" "$@"
