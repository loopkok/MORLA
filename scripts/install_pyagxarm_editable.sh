#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MORLA_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SDK_DIR="${MORLA_ROOT}/third_party/pyAgxArm"

if [[ ! -f "${SDK_DIR}/pyproject.toml" ]]; then
  echo "pyAgxArm source tree is missing: ${SDK_DIR}" >&2
  exit 2
fi

echo "Installing pyAgxArm editable for: $(python3 -c 'import sys; print(sys.executable)')"
python3 -m pip install --editable "${SDK_DIR}"
