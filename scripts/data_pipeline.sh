#!/usr/bin/env bash
# Source-checkout launcher. Implementation and help live in the pipeline package.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PY="${ASTRAL_DATA_PYTHON:-$SDK_ROOT/VLA/openpi/.venv/bin/python}"
[ -x "$PY" ] || { echo "Python 不存在：$PY；可用 ASTRAL_DATA_PYTHON 指定环境" >&2; exit 1; }
export PYTHONPATH="$SCRIPT_DIR/../src/astral_data_collect${PYTHONPATH:+:$PYTHONPATH}"
exec "$PY" -m astral_data_collect.pipeline "$@"
