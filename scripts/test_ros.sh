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

DOMAIN_BASE="${MORLA_TEST_ROS_DOMAIN_ID:-91}"
[[ "$DOMAIN_BASE" =~ ^[0-9]+$ ]] || { echo "MORLA_TEST_ROS_DOMAIN_ID must be numeric" >&2; exit 2; }
((DOMAIN_BASE <= 220)) || { echo "MORLA_TEST_ROS_DOMAIN_ID must be <= 220" >&2; exit 2; }

domain_offset=0
run_suite() {
  local label="$1"
  local path="$2"
  shift 2
  if [[ ! -e "$WS/$path" ]]; then
    echo "Missing test path: $path" >&2
    return 1
  fi
  local domain=$((DOMAIN_BASE + domain_offset))
  domain_offset=$((domain_offset + 1))
  echo "== $label (ROS_DOMAIN_ID=$domain)"
  ROS_DOMAIN_ID="$domain" /usr/bin/python3 -m pytest -q -p no:cacheprovider "$WS/$path" "$@"
}

run_module() {
  local label="$1"
  local module="$2"
  local domain=$((DOMAIN_BASE + domain_offset))
  domain_offset=$((domain_offset + 1))
  echo "== $label (ROS_DOMAIN_ID=$domain)"
  ROS_DOMAIN_ID="$domain" /usr/bin/python3 -m "$module"
}

# Keep each ROS test group on its own DDS domain. The standalone Quest test
# directory contains source-tree-wide style scanners, so run only behavior tests.
run_suite "core profiles" src/common/morla_core/test
run_suite "Quest input behavior" src/common/morla_input/test \
  --ignore="$WS/src/common/morla_input/test/test_copyright.py" \
  --ignore="$WS/src/common/morla_input/test/test_flake8.py" \
  --ignore="$WS/src/common/morla_input/test/test_pep257.py"
run_suite "bringup and coordinator" src/common/morla_bringup/test
run_suite "camera streamer" src/common/quest3_video_streamer/test
run_suite "collector recording smoke" src/common/morla_data_collect/test/test_collect_smoke.py
run_suite "collector node guards" src/common/morla_data_collect/test/test_node_guards.py
run_suite "collector VR control logic" src/common/morla_data_collect/test/test_vr_collect_control.py
run_suite "collector VR ROS node" src/common/morla_data_collect/test/test_vr_collect_node.py
run_suite "collector tactile subscriber" src/common/morla_data_collect/test/test_profile_tactile_import.py \
  -k profile_collector_writes_hand_id_selected_sensor_with_sim_validity
run_suite "web manager" src/common/morla_web_manager/test
run_suite "Astral driver" src/robots/astral/astral_robot_control/test
run_suite "Astral arm regression" src/robots/astral/astral_arm_teleop/astral_arm_teleop \
  --ignore="$WS/src/robots/astral/astral_arm_teleop/astral_arm_teleop/test_geometric_ik.py" \
  --ignore="$WS/src/robots/astral/astral_arm_teleop/astral_arm_teleop/test_ik_solver.py" \
  --ignore="$WS/src/robots/astral/astral_arm_teleop/astral_arm_teleop/test_vr_mapping.py"
run_module "Astral geometric IK" astral_arm_teleop.test_geometric_ik
run_module "Astral IK solvers" astral_arm_teleop.test_ik_solver
run_module "Astral Quest mapping" astral_arm_teleop.test_vr_mapping
run_suite "Nero driver" src/robots/nero/nero_robot_control/test
run_suite "Nero MuJoCo simulation" src/robots/nero/xnero_mujoco_sim/test
run_suite "XHand adapter" src/robots/xhand/xhand_adapter/test
run_suite "XHand retargeting" src/robots/xhand/xhand_retargeting/test

echo "ROS behavior suites passed. Policy and offline suites have dedicated entry points."
