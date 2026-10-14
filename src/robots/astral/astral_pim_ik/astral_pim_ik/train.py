"""Training script for the Astral PiM-IK arm-angle network (stage 1).

DDP port of the PiM-IK reference trainer (``training/trainer.py``), adapted to
the Astral ``psi`` dataset and ``PhysicsInformedLoss``. Run with torchrun:

    torchrun --nproc_per_node=2 astral_pim_ik/train.py \
        --data_path astral_arm_angle_dataset.npz --epochs 50
"""

from __future__ import annotations

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
import torch.distributed as dist
import torch.nn as nn
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.nn.utils import clip_grad_norm_
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingWarmRestarts, LinearLR, SequentialLR
from torch.utils.data import DataLoader
from torch.utils.data.distributed import DistributedSampler

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from astral_pim_ik.dataset import SwivelSequenceDataset  # noqa: E402
from astral_pim_ik.kinematics import PhysicsInformedLoss  # noqa: E402
from astral_pim_ik.network import PiM_IK_Net  # noqa: E402


def collate_fn(batch):
    return {k: torch.stack([b[k] for b in batch], dim=0) for k in batch[0]}


def parse_args():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--data_path", type=str, default="astral_arm_angle_dataset.npz")
    p.add_argument("--window_size", type=int, default=30, choices=[1, 15, 30])
    p.add_argument("--train_stride", type=int, default=5)
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--batch_size", type=int, default=2048)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--weight_decay", type=float, default=1e-4)
    p.add_argument("--warmup_epochs", type=int, default=2)
    p.add_argument("--grad_clip", type=float, default=1.0)
    p.add_argument("--t0", type=int, default=10)
    p.add_argument("--t_mult", type=int, default=2)
    p.add_argument("--d_model", type=int, default=256)
    p.add_argument("--num_layers", type=int, default=4)
    p.add_argument("--backbone", type=str, default="transformer",
                   choices=["mamba", "lstm", "transformer"])
    p.add_argument("--dropout", type=float, default=0.1)
    p.add_argument("--save_dir", type=str, default="./checkpoints")
    p.add_argument("--w_psi", type=float, default=1.0)
    p.add_argument("--w_elbow", type=float, default=1.0)
    p.add_argument("--w_smooth", type=float, default=0.01)
    p.add_argument("--elbow_loss_type", type=str, default="rmse",
                   choices=["mse", "rmse", "l1"])
    return p.parse_args()


def train_one_epoch(model, loader, optimizer, scheduler, loss_fn, device, local_rank, epoch):
    model.train()
    tot = {"loss": 0.0, "psi": 0.0, "elbow": 0.0, "smooth": 0.0}
    n = 0
    for batch in loader:
        T_ee = batch["T_ee"].to(device)
        gt_psi = batch["gt_psi"].to(device)
        p_s = batch["p_s"].to(device)
        p_e_gt = batch["p_e_gt"].to(device)
        p_w = batch["p_w"].to(device)
        L_upper = batch["L_upper"].to(device)
        L_lower = batch["L_lower"].to(device)
        is_valid = batch["is_valid"].to(device)

        pred_psi = model(T_ee)
        loss, ld = loss_fn(pred_psi, gt_psi, p_s, p_e_gt, p_w, L_upper, L_lower, is_valid)

        optimizer.zero_grad()
        loss.backward()
        clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        scheduler.step()

        tot["loss"] += ld["total_loss"]
        tot["psi"] += ld["L_psi"]
        tot["elbow"] += ld["L_elbow"]
        tot["smooth"] += ld["L_smooth"]
        n += 1
    return {k: v / n for k, v in tot.items()}


@torch.no_grad()
def validate(model, loader, loss_fn, device):
    model.eval()
    tot = {"loss": 0.0, "psi": 0.0, "elbow": 0.0, "smooth": 0.0}
    n = 0
    for batch in loader:
        T_ee = batch["T_ee"].to(device)
        gt_psi = batch["gt_psi"].to(device)
        p_s = batch["p_s"].to(device)
        p_e_gt = batch["p_e_gt"].to(device)
        p_w = batch["p_w"].to(device)
        L_upper = batch["L_upper"].to(device)
        L_lower = batch["L_lower"].to(device)
        is_valid = batch["is_valid"].to(device)
        pred_psi = model(T_ee)
        _, ld = loss_fn(pred_psi, gt_psi, p_s, p_e_gt, p_w, L_upper, L_lower, is_valid)
        tot["loss"] += ld["total_loss"]
        tot["psi"] += ld["L_psi"]
        tot["elbow"] += ld["L_elbow"]
        tot["smooth"] += ld["L_smooth"]
        n += 1
    out = {k: torch.tensor(v / n, device=device) for k, v in tot.items()}
    for v in out.values():
        dist.all_reduce(v, op=dist.ReduceOp.SUM)
    return {k: (v / dist.get_world_size()).item() for k, v in out.items()}


def main():
    args = parse_args()
    local_rank = int(os.environ.get("LOCAL_RANK", 0))
    world_size = int(os.environ.get("WORLD_SIZE", 1))
    dist.init_process_group(backend="nccl", init_method="env://")
    torch.cuda.set_device(local_rank)
    device = torch.device(f"cuda:{local_rank}")

    train_ds = SwivelSequenceDataset(args.data_path, args.window_size, True, args.train_stride)
    val_ds = SwivelSequenceDataset(args.data_path, args.window_size, False, 1)
    train_sampler = DistributedSampler(train_ds, num_replicas=world_size, rank=local_rank, shuffle=True)
    val_sampler = DistributedSampler(val_ds, num_replicas=world_size, rank=local_rank, shuffle=False)
    train_loader = DataLoader(train_ds, args.batch_size, sampler=train_sampler,
                              num_workers=4, pin_memory=True, collate_fn=collate_fn, drop_last=True)
    val_loader = DataLoader(val_ds, args.batch_size, sampler=val_sampler,
                            num_workers=4, pin_memory=True, collate_fn=collate_fn)

    model = PiM_IK_Net(d_model=args.d_model, num_layers=args.num_layers,
                       backbone_type=args.backbone, dropout=args.dropout).to(device)
    model = DDP(model, device_ids=[local_rank], output_device=local_rank,
                find_unused_parameters=True)

    optimizer = AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    warmup_steps = len(train_loader) * args.warmup_epochs
    t0_steps = len(train_loader) * args.t0
    warmup = LinearLR(optimizer, start_factor=0.01, total_iters=warmup_steps)
    cosine = CosineAnnealingWarmRestarts(optimizer, T_0=t0_steps, T_mult=args.t_mult, eta_min=1e-6)
    scheduler = SequentialLR(optimizer, schedulers=[warmup, cosine], milestones=[warmup_steps])

    loss_fn = PhysicsInformedLoss(w_psi=args.w_psi, w_elbow=args.w_elbow,
                                  w_smooth=args.w_smooth, elbow_loss_type=args.elbow_loss_type).to(device)

    best = float("inf")
    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    for epoch in range(args.epochs):
        train_sampler.set_epoch(epoch)
        tm = train_one_epoch(model, train_loader, optimizer, scheduler, loss_fn, device, local_rank, epoch)
        vm = validate(model, val_loader, loss_fn, device)
        if local_rank == 0:
            print(f"Epoch {epoch+1}/{args.epochs} train={tm['loss']:.4f} val={vm['loss']:.4f}")
            if vm["loss"] < best:
                best = vm["loss"]
                torch.save({
                    "epoch": epoch + 1,
                    "model_state_dict": model.module.state_dict(),
                    "val_loss": vm["loss"],
                    "args": vars(args),
                }, save_dir / f"best_{args.backbone}_L{args.num_layers}_w{args.window_size}.pth")
    dist.destroy_process_group()


if __name__ == "__main__":
    main()
