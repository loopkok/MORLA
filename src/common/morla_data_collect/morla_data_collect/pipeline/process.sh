#!/usr/bin/env bash
# Both exports use the same orchestration. Python modules own data processing.
set -euo pipefail
TARGET="${1:?missing target}"
shift
usage() {
    echo "usage: data_pipeline $TARGET <session_dir> <output_dir> [--force-align] [--quarantine] [--no-quarantine] [--image-size N]"
    if [ "$TARGET" = process-act ]; then
        echo "       [--check-python PY] [--keep-v21 DIR] [--overwrite]"
    fi
    echo "默认不移动失败段；--quarantine 会把失败段移动到输入 session/quarantine/。"
    echo "不自动执行 quantify 或 repair；在工作副本上处理。"
}
if [ "${1:-}" = --help ] || [ "${1:-}" = -h ]; then usage; exit 0; fi
if [ "$#" -lt 2 ]; then usage >&2; exit 2; fi
SESSION="$1" OUTPUT="$2"
shift 2
PY="${MORLA_DATA_PYTHON:-${ASTRAL_DATA_PYTHON:-python3}}"
if [ -n "${ASTRAL_DATA_PYTHON:-}" ] && [ -z "${MORLA_DATA_PYTHON:-}" ]; then
    echo "ASTRAL_DATA_PYTHON 已弃用；当前使用其兼容值：$PY" >&2
fi
ALIGN_ARGS=() VALIDATE_ARGS=() EXPORT_ARGS=() FORCE_ACT=()
IMAGE_SIZE=224
while [ "$#" -gt 0 ]; do
    case "$1" in
        --force-align) ALIGN_ARGS=(--force); FORCE_ACT=(--force-align); shift;;
        --quarantine) VALIDATE_ARGS=(--apply); shift;;
        --no-quarantine) VALIDATE_ARGS=(); shift;;
        --image-size|--check-python|--keep-v21)
            [ "$#" -ge 2 ] || { echo "missing value: $1" >&2; exit 2; }
            if [ "$1" = --image-size ]; then IMAGE_SIZE="$2";
            elif [ "$TARGET" = process-act ]; then EXPORT_ARGS+=("$1" "$2");
            else echo "unsupported for $TARGET: $1" >&2; exit 2; fi
            shift 2;;
        --overwrite)
            [ "$TARGET" = process-act ] || { echo "unsupported: $1" >&2; exit 2; }
            EXPORT_ARGS+=(--overwrite); shift;;
        *) echo "unknown arg: $1" >&2; exit 2;;
    esac
done
echo "[1/3] 对齐：$SESSION"
"$PY" -m morla_data_collect.pipeline align --session "$SESSION" "${ALIGN_ARGS[@]}"
echo "[2/3] 校验：$SESSION"
if ! "$PY" -m morla_data_collect.pipeline validate --session "$SESSION" "${VALIDATE_ARGS[@]}"; then
    echo "校验未通过，已终止导出；请处理失败项后重试" >&2
    exit 1
fi
echo "[3/3] 导出：$OUTPUT"
case "$TARGET" in
    process-openpi)
        "$PY" -m morla_data_collect.pipeline export-openpi \
            --session "$SESSION" --output "$OUTPUT" --image-size "$IMAGE_SIZE"
        echo "数据更新后，使用对应 OpenPI 配置重新计算 norm stats 再训练。";;
    process-act)
        "$PY" -m morla_data_collect.pipeline export-act \
            --session "$SESSION" --output "$OUTPUT" --image-size "$IMAGE_SIZE" \
            "${EXPORT_ARGS[@]}" "${FORCE_ACT[@]}";;
    *) echo "unknown target: $TARGET" >&2; exit 2;;
esac
