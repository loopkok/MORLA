#!/usr/bin/env bash
# Source-checkout launcher. Implementation and help live in the pipeline package.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MORLA_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
SDK_ROOT="$(dirname "$MORLA_WS")"
PY="${MORLA_DATA_PYTHON:-${ASTRAL_DATA_PYTHON:-$SDK_ROOT/VLA/openpi/.venv/bin/python}}"
[ -x "$PY" ] || { echo "Python 不存在：$PY；可用 MORLA_DATA_PYTHON 指定环境" >&2; exit 1; }
if [ -n "${ASTRAL_DATA_PYTHON:-}" ] && [ -z "${MORLA_DATA_PYTHON:-}" ]; then
    echo "ASTRAL_DATA_PYTHON 已弃用；当前使用其兼容值：$PY" >&2
fi
export PYTHONPATH="$MORLA_WS/src/common/morla_core:$MORLA_WS/src/common/morla_data_collect${PYTHONPATH:+:$PYTHONPATH}"
exec "$PY" -m morla_data_collect.pipeline "$@"
