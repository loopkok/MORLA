#!/usr/bin/env bash
# act_train.sh — ACT 三种训练模式一键直训（from_scratch / resume / finetune）
#
# 环境: conda lerobot (py3.12 + torch) + GPU；数据 = act/<session> 的 v3 数据集（480 等）。
#
# 模式:
#   from_scratch   从头训（无 checkpoint，新 output_dir）
#   resume         接着训（--resume + 同一 output_dir，续步数；lr/bs 沿用原 run）
#   finetune       已有模型上加数据训（--policy.path=<checkpoint> + 新/合并数据集 + 新 output_dir）
#
# 用法:
#   ./act_train.sh --mode from_scratch --dataset <root> --output-dir <dir> \
#                  [--repo-id NAME] [--steps N] [--batch-size N] [--lr X] \
#                  [--chunk-size N] [--n-action-steps N] [--kl-weight X] \
#                  [--num-workers N] [--job-name NAME] [--no-wandb] [--wandb-project P] [--dry-run]
#
#   # 示例
#   # 从头训（合并集 480）
#   ./act_train.sh --mode from_scratch \
#       --dataset /home/robot/loopkok/sdk/astral_data/act/pick_place_merged \
#       --output-dir /home/robot/loopkok/sdk/astral_ckpt/pick_place_merged_fs \
#       --steps 80000 --job-name astral_act_merged_fs
#   # 接着训（同数据续步数：从 80000 续到 120000；batch 必须与原 run 一致）
#   ./act_train.sh --mode resume \
#       --dataset /home/robot/loopkok/sdk/astral_data/act/pick up and place_480 \
#       --output-dir /home/robot/loopkok/sdk/astral_ckpt/pickup_act_480 \
#       --steps 120000
#   # 已有模型上加数据训（在 080000 基础上学红绿两任务）
#   ./act_train.sh --mode finetune \
#       --dataset /home/robot/loopkok/sdk/astral_data/act/pick_place_merged \
#       --checkpoint /home/robot/loopkok/sdk/astral_ckpt/pickup_act_480/checkpoints/080000/pretrained_model \
#       --output-dir /home/robot/loopkok/sdk/astral_ckpt/pick_place_merged_ft \
#       --steps 30000 --job-name astral_act_merged_ft
#
# 说明:
#   - 微调 lr 默认 1e-5（比首训 3e-5 小，保已有特征）；--lr / --lr-backbone 可覆盖。
#   - resume 不改 lr/bs（lerobot 从 checkpoint 恢复优化器，CLI lr 不生效；bs 改会破坏采样 offset）。
#   - --dry-run 只打印要执行的命令，不真跑。
set -euo pipefail

LEROBOT_TRAIN="${LEROBOT_TRAIN:-/home/robot/miniconda3/envs/lerobot/bin/lerobot-train}"

MODE="" DATASET="" REPO_ID="" CHECKPOINT="" OUTPUT_DIR="" STEPS="" JOB_NAME=""
BATCH_SIZE=32 NUM_WORKERS=4 LR="" LR_BACKBONE="" CHUNK=50 N_ACTION=50 KL=10.0
WANDB=1 WANDB_PROJECT="astral_act" DRY_RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --mode) MODE="$2"; shift 2;;
        --dataset) DATASET="$2"; shift 2;;
        --repo-id) REPO_ID="$2"; shift 2;;
        --checkpoint) CHECKPOINT="$2"; shift 2;;
        --output-dir) OUTPUT_DIR="$2"; shift 2;;
        --steps) STEPS="$2"; shift 2;;
        --batch-size) BATCH_SIZE="$2"; shift 2;;
        --lr) LR="$2"; shift 2;;
        --lr-backbone) LR_BACKBONE="$2"; shift 2;;
        --chunk-size) CHUNK="$2"; shift 2;;
        --n-action-steps) N_ACTION="$2"; shift 2;;
        --kl-weight) KL="$2"; shift 2;;
        --num-workers) NUM_WORKERS="$2"; shift 2;;
        --job-name) JOB_NAME="$2"; shift 2;;
        --wandb-project) WANDB_PROJECT="$2"; shift 2;;
        --no-wandb) WANDB=0; shift;;
        --dry-run) DRY_RUN=1; shift;;
        *) echo "unknown arg: $1" >&2; exit 2;;
    esac
done

# ---------- 校验 ----------
case "$MODE" in
    from_scratch|resume|finetune) :;;
    *) echo "usage: act_train.sh --mode from_scratch|resume|finetune ...（见文件头）" >&2; exit 2;;
esac
[ -n "$DATASET" ]    || { echo "缺 --dataset（act/<session> 的 v3 数据集目录）" >&2; exit 2; }
[ -n "$OUTPUT_DIR" ] || { echo "缺 --output-dir" >&2; exit 2; }
[ -d "$DATASET" ]    || { echo "数据集目录不存在: $DATASET" >&2; exit 2; }
[ -x "$LEROBOT_TRAIN" ] || { echo "缺少 lerobot-train: $LEROBOT_TRAIN（设 LEROBOT_TRAIN 覆盖）" >&2; exit 2; }
if [ "$MODE" = from_scratch ] && [ -n "$CHECKPOINT" ]; then
    echo "from_scratch 不能带 --checkpoint（那是 finetune 用的）" >&2; exit 2
fi
if [ "$MODE" = finetune ] && [ -z "$CHECKPOINT" ]; then
    echo "finetune 必须带 --checkpoint（已有模型的 pretrained_model 目录）" >&2; exit 2
fi
if [ "$MODE" = resume ]; then
    [ -n "$STEPS" ] || { echo "resume 必须带 --steps（续到的新的总步数，如 120000）" >&2; exit 2; }
    [ -d "$OUTPUT_DIR/checkpoints" ] || { echo "resume 的 --output-dir 必须是原 run 目录（含 checkpoints/）: $OUTPUT_DIR" >&2; exit 2; }
fi

# ---------- 组装 lerobot-train 参数 ----------
ARGS=(--dataset.root="$DATASET")
[ -n "$REPO_ID" ] && ARGS+=(--dataset.repo_id="$REPO_ID")

case "$MODE" in
    from_scratch)
        ARGS+=(--policy.type=act --policy.push_to_hub=false)
        ARGS+=(--policy.chunk_size="$CHUNK" --policy.n_action_steps="$N_ACTION" --policy.kl_weight="$KL")
        LR="${LR:-3e-5}"
        ARGS+=(--policy.optimizer_lr="$LR" --policy.optimizer_lr_backbone="${LR_BACKBONE:-$LR}")
        ;;
    finetune)
        ARGS+=(--policy.path="$CHECKPOINT" --policy.push_to_hub=false)
        ARGS+=(--policy.chunk_size="$CHUNK" --policy.n_action_steps="$N_ACTION" --policy.kl_weight="$KL")
        LR="${LR:-1e-5}"
        ARGS+=(--policy.optimizer_lr="$LR" --policy.optimizer_lr_backbone="${LR_BACKBONE:-$LR}")
        ;;
    resume)
        # lr/bs 由 checkpoint 恢复，这里不传（传了 resume 也不会生效）
        ARGS+=(--policy.type=act --policy.push_to_hub=false --resume)
        ;;
esac

ARGS+=(--output_dir="$OUTPUT_DIR")
[ -n "$STEPS" ] && ARGS+=(--steps="$STEPS")
ARGS+=(--batch_size="$BATCH_SIZE" --num_workers="$NUM_WORKERS")
[ -n "$JOB_NAME" ] && ARGS+=(--job_name="$JOB_NAME")
if [ "$WANDB" = 1 ]; then
    ARGS+=(--wandb.enable=true --wandb.mode=offline --wandb.project="$WANDB_PROJECT")
fi

echo "=== ACT $MODE: dataset=$DATASET output=$OUTPUT_DIR steps=${STEPS:-<沿用 checkpoint>} batch=$BATCH_SIZE ==="
if [ "$DRY_RUN" = 1 ]; then
    printf '%s ' "$LEROBOT_TRAIN" "${ARGS[@]}"; echo
    exit 0
fi
exec "$LEROBOT_TRAIN" "${ARGS[@]}"
