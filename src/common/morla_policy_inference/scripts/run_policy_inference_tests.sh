#!/usr/bin/env bash
# Run morla_policy_inference tests, including ROS node-flow and HITL writer
# checks. Requires ROS Humble and the affected workspace package sources.
set -euo pipefail
# ROS setup.bash touches AMENT_* env vars that trip 'set -u'; keep -e only here.
set +u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
# WS == MORLA
source /opt/ros/humble/setup.bash
PYTHONPATH="$WS/src/robots/astral/astral_arm_teleop:$WS/src/robots/astral/astral_gripper_teleop:$WS/src/common/morla_bringup:$WS/src/common/morla_input:$WS/src/robots/wuji/wujihand_retargeting:$WS/src/common/morla_data_collect:$WS/src/common/morla_policy_inference:${PYTHONPATH:-}"
export PYTHONPATH

fail=0
for t in "$WS"/src/common/morla_policy_inference/tests/test_*.py; do
  echo "== $(basename "$t")"
  /usr/bin/python3 -m pytest "$t" -q -p no:anyio "$@" || fail=1
done
exit "$fail"
