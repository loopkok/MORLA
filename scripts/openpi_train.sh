#!/usr/bin/env bash
# openpi_train.sh — pi0.5 训练入口（norm stats → 训练），在 VLA/openpi 的 uv 环境内执行
#
# 用法:
#   ./openpi_train.sh <lora|full> <exp_name> [--stats-only] [-- extra train.py 参数...]
#
# 示例:
#   ./openpi_train.sh lora run1                    # LoRA（4090D 24GB 推荐），含 norm stats 重算
#   ./openpi_train.sh full run1 -- --batch-size 4  # 全量微调并透传参数
#   ./openpi_train.sh lora run1 --stats-only       # 只重算 norm stats 不训练
set -euo pipefail

if [[ "${1:-}" == "--spec" ]]; then
    shift
    exec "${MORLA_LEARNING_PYTHON:-python3}" -m morla_learning.cli "$@"
fi

MODE="${1:?usage: openpi_train.sh <lora|full> <exp_name> [--stats-only] [-- extra args...]}"
EXP="${2:?missing exp_name}"
shift 2
STATS_ONLY=0
EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        --stats-only) STATS_ONLY=1;;
        --) shift; EXTRA=("$@"); break;;
        *) EXTRA+=("$1");;
    esac
    shift
done

case "$MODE" in
    lora) CONFIG="pi05_astral_lora";;
    full) CONFIG="pi05_astral";;
    *) echo "mode 只能是 lora|full" >&2; exit 2;;
esac

OPENPI="/home/robot/loopkok/sdk/VLA/openpi"
CKPT_CACHE="${OPENPI_DATA_HOME:-$HOME/.cache/openpi}/openpi-assets/checkpoints/pi05_base"
DATASET_LINK="$HOME/.cache/huggingface/lerobot/astral/morla_bringup"

cd "$OPENPI"

# 前置检查 1: 数据集就位（软链或实体目录均可）
if [ ! -e "$DATASET_LINK/meta/info.json" ]; then
    echo "缺少数据集: $DATASET_LINK" >&2
    echo "  先跑 data_pipeline.sh process-openpi，或手动软链:" >&2
    echo "  mkdir -p ~/.cache/huggingface/lerobot/astral && \\" >&2
    echo "  ln -sfn <astral_data_lerobot 路径> $DATASET_LINK" >&2
    exit 1
fi

# 前置检查 2: pi05_base 权重（训练时才需要；stats-only 可跳过）
if [ "$STATS_ONLY" = 0 ] && [ ! -d "$CKPT_CACHE/params" ]; then
    echo "pi05_base 权重未就绪 ($CKPT_CACHE/params 不存在)" >&2
    echo "  手动下载: cd $OPENPI && uv run python -c \\" >&2
    echo "    \"from openpi.shared import download; download.maybe_download('gs://openpi-assets/checkpoints/pi05_base')\"" >&2
    exit 1
fi

echo "=== [1/2] 重算 norm stats ($CONFIG)"
uv run scripts/compute_norm_stats.py --config-name "$CONFIG"

[ "$STATS_ONLY" = 1 ] && { echo "=== stats-only，结束"; exit 0; }

echo "=== [2/2] 启动训练 $CONFIG / exp=$EXP"
export XLA_PYTHON_CLIENT_MEM_FRACTION=0.9
exec uv run scripts/train.py "$CONFIG" --exp-name="$EXP" ${EXTRA[@]+"${EXTRA[@]}"}
