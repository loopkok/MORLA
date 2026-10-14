#!/usr/bin/env bash
# Source-checkout launcher. Implementation and help live in the pipeline package.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MORLA_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
SDK_ROOT="$(dirname "$MORLA_WS")"
PY="${MORLA_DATA_PYTHON:-${ASTRAL_DATA_PYTHON:-$SDK_ROOT/VLA/openpi/.venv/bin/python}}"
[ -x "$PY" ] || { echo "Python 不存在：$PY；可用 MORLA_DATA_PYTHON 指定环境" >&2; exit 1; }
export PYTHONPATH="$MORLA_WS/src/common/morla_data_collect${PYTHONPATH:+:$PYTHONPATH}"
exec "$PY" -m morla_data_collect.pipeline "$@"
