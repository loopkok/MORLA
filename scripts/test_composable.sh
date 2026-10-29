#!/usr/bin/env bash
set -euo pipefail
MORLA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MORLA_PYTHON="${MORLA_PYTHON:-python3}"
MORLA_LAYER=pure
while [[ $# -gt 0 ]]; do
  case "$1" in
    --layer) MORLA_LAYER="$2"; shift 2 ;;
    --help) echo 'test_composable.sh --layer pure|fake|sim|web|learning [pytest arguments]'; exit 0 ;;
    *) break ;;
  esac
done
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
for package_dir in "$MORLA_ROOT"/src/common/* "$MORLA_ROOT"/src/robots/*/*; do
  [[ -f "$package_dir/package.xml" ]] && export PYTHONPATH="$package_dir${PYTHONPATH:+:$PYTHONPATH}"
done
case "$MORLA_LAYER" in
  pure) exec "$MORLA_PYTHON" -m pytest -q "$MORLA_ROOT/src/common/morla_core/test/contracts" "$MORLA_ROOT/src/common/morla_runtime/test" "$MORLA_ROOT/src/common/morla_teleop/test" "$MORLA_ROOT/src/common/morla_learning/test" "$MORLA_ROOT/src/robots/dobot/morla_cr5_driver/test" "$MORLA_ROOT/src/robots/loophand/morla_loophand_driver/test" "$MORLA_ROOT/src/robots/jodell/morla_epg_driver/test" "$MORLA_ROOT/examples/cr5_loophand/test" "$MORLA_ROOT/src/common/morla_input/test/test_quest_right.py" "$@" ;;
  fake) exec "$MORLA_PYTHON" -m pytest -q "$MORLA_ROOT/src/common/morla_data_collect/test/test_composable_data.py" "$MORLA_ROOT/src/common/morla_policy_inference/tests/test_composable.py" "$MORLA_ROOT/src/common/morla_web_manager/test/test_composable_api.py" "$@" ;;
  sim) exec "$MORLA_PYTHON" -m pytest -q "$MORLA_ROOT/src/robot_description/test" "$@" ;;
  web) exec "$MORLA_PYTHON" -m pytest -q "$MORLA_ROOT/src/common/morla_web_manager/test/test_composable_api.py" "$@" ;;
  learning) exec "$MORLA_PYTHON" -m pytest -q "$MORLA_ROOT/src/common/morla_learning/test" "$MORLA_ROOT/src/common/morla_policy_inference/tests/test_composable.py" "$@" ;;
  *) echo "Unsupported layer: $MORLA_LAYER" >&2; exit 2 ;;
esac
