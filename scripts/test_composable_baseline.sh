#!/usr/bin/env bash
set -euo pipefail
MORLA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MORLA_PYTHON="${MORLA_PYTHON:-python3}"
export PYTHONPATH="$MORLA_ROOT/src/common/morla_core${PYTHONPATH:+:$PYTHONPATH}"
exec "$MORLA_PYTHON" -m pytest "$MORLA_ROOT/src/common/morla_core/test/contracts" "$MORLA_ROOT/src/robot_description/test/test_composable_scene.py" "$@"
