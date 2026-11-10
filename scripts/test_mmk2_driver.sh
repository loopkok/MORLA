#!/usr/bin/env bash
set -euo pipefail
MORLA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MORLA_PYTHON="${MORLA_PYTHON:-python3}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  export AMENT_PREFIX_PATH="$("$MORLA_PYTHON" -c 'import sys; print(sys.prefix)')"
fi
for package_dir in "$MORLA_ROOT"/src/common/* "$MORLA_ROOT"/src/robots/*/*; do
  [[ -f "$package_dir/package.xml" ]] && export PYTHONPATH="$package_dir${PYTHONPATH:+:$PYTHONPATH}"
done
exec "$MORLA_PYTHON" -m pytest -q "$MORLA_ROOT/src/robots/airbot/morla_mmk2_driver/test" "$@"
