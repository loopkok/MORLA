# openpi（pi0.5）训练部署交接清单

> 用途：在 **RAM ≥ 32GB** 的机器上训练 openpi（pi0.5），喂 astral 采集/转换的 v2.1 数据集。
> 本机（4090D，24GB VRAM / **15GB RAM**）**RAM 装不下 pi05_base 权重（11.6GB）**，训练必须换机器；
> ACT（0.2GB 模型）可留本机训。数据侧（转换、norm stats、语义金标准、对抗回归 8/8）已在本机全部验证通过。

## 0. 目标机器硬性要求（不满足别开工）

| 资源 | 要求 | 原因 |
|---|---|---|
| **RAM** | **≥ 32GB**（关键） | pi05_base 权重 11.6GB 必须整体进 RAM + JAX 2-4GB ≈ 需 14-16GB 可用；15GB 机实测 3 次 OOM |
| **VRAM** | ≥ 24GB（4090 系） | 权重 11.6GB + lora 激活值；`pi05_astral` 全量 24G 会 OOM，用 `pi05_astral_lora` |
| 驱动/CUDA | NVIDIA 驱动支持 CUDA 12.x（jax-cuda12 插件要求） | |
| 系统 | Linux + Python 3.11 + `uv` | openpi 用 uv 管理 venv |
| 磁盘 | 至少 ~30GB 空闲 | 权重 12G + 代码/venv ~8G + 数据集 + 训练产物 |

## 1. 要迁移的东西（三件 + 环境）

| 项 | 来源路径（本机） | 体积 | 目标机器动作 |
|---|---|---|---|
| **① 代码** | `sdk/VLA/openpi/`（**排除 `.venv`/`assets`**） | ~几百 MB（源码） | 拷贝源码 → `uv sync` 重建 venv（**不要拷 .venv，路径绑定会坏**） |
| **② 数据集** | `astral_data/pi/pick_place_merged`（v2.1, 100 段 21321 帧, 红/绿双任务） | **58MB** | 拷贝 → 软链到 `HF_LEROBOT_HOME/astral/astral_teleop` |
| **③ pi05_base 权重** | `~/.cache/openpi/openpi-assets/checkpoints/pi05_base` | **12GB** | 拷贝到目标机 `~/.cache/openpi/openpi-assets/checkpoints/pi05_base`（**避免 11.6GB 重新下载**；别设 `OPENPI_DATA_HOME` 到别处，见 §3 坑） |
| 环境 | Python 3.11 + uv + NVIDIA 驱动 | — | 目标机装好 |

**数据传输建议**：
```bash
# ① 代码（排除 venv/assets，到目标机后 uv sync）
tar czf openpi_src.tar.gz -C sdk/VLA openpi --exclude='.venv' --exclude='assets'
# ② 数据集
tar czf pi_pick_place_merged.tar.gz -C astral_data/pi pick_place_merged
# ③ 权重（12GB，走 rsync/scp 都可）
rsync -aP ~/.cache/openpi/openpi-assets/checkpoints/pi05_base/ <目标机>:/home/<user>/.cache/openpi/openpi-assets/checkpoints/pi05_base/
```

## 2. 目标机准备步骤（按序）

```bash
# ① 环境
curl -LsSf https://astral.sh/uv/install.sh | sh     # uv
# ② 代码 + venv
cd VLA/openpi && uv sync                            # 拉 jax-cuda12/torch 等（需网络）
# ③ 数据集软链（pinned lerobot 0.1.0 经 HF_LEROBOT_HOME 解析 repo_id=astral/astral_teleop）
mkdir -p ~/.cache/huggingface/lerobot/astral
ln -sfn <你放的路径>/pi/pick_place_merged \
        ~/.cache/huggingface/lerobot/astral/astral_teleop
# ④ 权重（已 rsync）→ 验证命中缓存不下载：
uv run python -c "from openpi.shared import download; print(download.maybe_download('gs://openpi-assets/checkpoints/pi05_base'))"
#    ↑ 应秒级返回并指向 ~/.cache/openpi/...，若开始下载说明路径不对
# ⑤ 重算 norm stats（1 分钟，CPU 即可；不依赖 GPU）
uv run scripts/compute_norm_stats.py --config-name pi05_astral_lora
```

## 3. 训练（核心命令）

```bash
cd VLA/openpi
uv run scripts/train.py pi05_astral_lora \
  --exp-name=run1 \
  --num_train_steps=30000 \
  --no-wandb-enabled        # 本机未配 wandb 登录；要曲线用 --wandb-enabled + wandb login
```

**推荐先用冒烟验证**（30 步，确认不 OOM、loss 下降、checkpoint 落盘）：
```bash
uv run scripts/train.py pi05_astral_lora --exp-name=smoke \
  --num_train_steps=30 --batch_size=8 --no-wandb-enabled --overwrite
```

训练参数（`pi05_astral_lora` config 默认）：LoRA（paligemma 2b + action expert 300m）、
batch 32、action_horizon 50、lr 1e-4→1e-5 cosine、clip 1.0。可 `--batch_size=N` 覆盖（RAM/VRAM 允许可加大）。

**产出**：checkpoint 在 `VLA/openpi/checkpoints/pi05_astral_lora/<exp-name>/`，推理用
`serve.py --model pi05 --checkpoint-dir <该目录>` 部署（见 astral_policy_inference）。

### 训练参数（`pi05_astral_lora` 配置，源码 config.py 为准）

| 参数 | 值 | 说明 |
|---|---|---|
| **模型** | pi05_base 初始化 + **LoRA**（`paligemma_variant="gemma_2b_lora"` + `action_expert_variant="gemma_300m_lora"`） | 只训 LoRA 适配器；**rank 内置于这两个变体名，不是配置项** |
| `action_horizon` | **50**（@30fps ≈ 1.67s 动作块） | 推理 chunk 长度，别改（与数据/部署对齐） |
| `discrete_state_input` | **False** | state 走连续后缀输入（非语言 token），别改 |
| `batch_size` | **32** | 24GB 卡默认；VRAM 紧降到 8；RAM 足可加大 |
| `lr_schedule` | **cosine**：warmup 1000 步 → peak **1e-4** → 30000 步衰减到 **1e-5** | 可 `--lr_schedule.peak_lr=...` 覆盖 |
| `optimizer` | **AdamW**, `clip_gradient_norm=1.0` | |
| `ema_decay` | **None**（LoRA 关闭 EMA） | |
| freeze | **非 LoRA 参数全冻结** | `freeze_filter` 由变体自动生成，勿手动改 |
| `weight_loader` | pi05_base（`gs://openpi-assets/...`，已缓存则秒级） | |
| `num_train_steps` | **30000** | 训练总步数；对应你的合并集 100 段/21321 帧、batch32 ≈ 45 epochs |
| 数据 prompt | `prompt_from_task=True`（每段 task 文本→prompt）+ `default_prompt="perform the manipulation task"` 兜底 | |

**可调项**（CLI `--xxx` 覆盖）：`batch_size`、`lr_schedule.*`、`num_train_steps`、`optimizer.*`。
**不建议动**：`action_horizon`、`discrete_state_input`、模型变体、freeze。


## 4. 本机已踩的坑（目标机别重蹈）

1. **`OPENPI_DATA_HOME` 别乱设**：本机曾指向 `/home/robot/openpi/ckpt` → 每次运行都重新下载 11.6GB
   权重（网络慢卡 1h+）。**默认（不设）就是 `~/.cache/openpi`**；权重放对路径即可，`maybe_download`
   秒级命中。
2. **RAM < 16GB 一定 OOM**：内核直接 SIGKILL（无 Python 报错，查 `journalctl -k | grep oom`）。
   `XLA_PYTHON_CLIENT_PREALLOCATE=false`、降 batch、`MEM_FRACTION` 都救不了——权重必须整体进 RAM。
3. **`--wandb_enabled=false` 无效**：bool 旗标要用 `--no-wandb-enabled`。
4. **checkpoint 目录已存在**：加 `--overwrite`（或 `--resume` 续训）。
5. **首次运行慢正常**：XLA 编译 pi05-lora 需 2-5 分钟，之后每步快。

## 5. 目标机验收清单（全绿才算就绪）

```bash
uv run python -c "import jax; print(jax.local_devices())"      # → [CudaDevice(id=0)]
uv run python -c "from openpi.shared import download; print(download.maybe_download('gs://openpi-assets/checkpoints/pi05_base'))"  # 秒级、无下载
uv run scripts/compute_norm_stats.py --config-name pi05_astral_lora   # 1 分钟
uv run scripts/train.py pi05_astral_lora --exp-name=smoke --num_train_steps=30 --batch_size=8 --no-wandb-enabled --overwrite
#   预期：数据加载器打印 state(8,32)/images(8,224,224,3)/actions(8,50,32) → 权重 restore → 30 步 loss 下降 → checkpoint
```
