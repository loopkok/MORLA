#!/usr/bin/env bash
# Run astral_policy_inference tests, including ROS node-flow and HITL writer
# checks. Requires ROS Humble and the affected workspace package sources.
set -euo pipefail
# ROS setup.bash touches AMENT_* env vars that trip 'set -u'; keep -e only here.
set +u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
# WS == astral_ws
source /opt/ros/humble/setup.bash
PYTHONPATH="$WS/src/astral_arm_teleop:$WS/src/astral_gripper_teleop:$WS/src/astral_teleop:$WS/src/quest3_hand_mocap:$WS/src/wujihand_retargeting:$WS/src/astral_data_collect:$WS/src/astral_policy_inference:${PYTHONPATH:-}"
export PYTHONPATH

fail=0
for t in "$WS"/src/astral_policy_inference/tests/test_*.py; do
  echo "== $(basename "$t")"
  /usr/bin/python3 -m pytest "$t" -q -p no:anyio "$@" || fail=1
done
exit "$fail"
