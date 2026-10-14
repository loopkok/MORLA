#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="$(dirname "$WS")"
DATA_PYTHON="${MORLA_DATA_PYTHON:-${ASTRAL_DATA_PYTHON:-$SDK_ROOT/VLA/openpi/.venv/bin/python}}"
if [[ ! -x "$DATA_PYTHON" ]]; then
  DATA_PYTHON=/usr/bin/python3
  echo "Data Python unavailable; falling back to system Python: $DATA_PYTHON" >&2
fi
if [[ -n "${ASTRAL_DATA_PYTHON:-}" && -z "${MORLA_DATA_PYTHON:-}" ]]; then
  echo "ASTRAL_DATA_PYTHON is deprecated; using it as MORLA_DATA_PYTHON" >&2
fi

export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$WS/src/common/morla_core:$WS/src/common/morla_data_collect${PYTHONPATH:+:$PYTHONPATH}"

TEST_DIR="$WS/src/common/morla_data_collect/test"
PYTEST=("$DATA_PYTHON" -m pytest -q -p no:cacheprovider)
FILES=(
  test_adversarial_configs.py
  test_align.py
  test_convert_act.py
  test_convert_lerobot.py
  test_convert_lerobot_v3.py
  test_intervention.py
  test_pipeline_entrypoints.py
  test_pipeline_quality.py
  test_replay.py
  test_schema.py
  test_validate.py
)

echo "Offline data pipeline Python: $DATA_PYTHON"
for file in "${FILES[@]}"; do
  echo "== $file"
  "${PYTEST[@]}" "$TEST_DIR/$file" "$@"
done

# This file also contains ROS subscriber tests. Keep its pure pipeline and
# read-only legacy import checks in the non-ROS environment.
echo "== profile tactile and legacy import contracts"
"${PYTEST[@]}" "$TEST_DIR/test_profile_tactile_import.py" \
  -k 'not profile_collector_writes_hand_id_selected_sensor_with_sim_validity' "$@"

echo "== installed-source data pipeline entrypoint"
"$WS/scripts/data_pipeline.sh" --help >/dev/null
echo "Offline data pipeline suites passed."
