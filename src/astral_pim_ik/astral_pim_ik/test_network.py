#!/usr/bin/env python3
"""Torch unit tests for the network and physics-informed loss.

Skipped gracefully when torch is unavailable (the deterministic stage-2 and
dataset/pipeline tests run without it). Covers the 9D pose representation, the
``PiM_IK_Net`` forward/L2-normalization/gradient, and — critically — that the
torch ``circle_basis`` reproduces the numpy geometric solver's elbow circle, so
the loss supervises the *same* ``psi`` convention stage 2 consumes.

Usage:
  PYTHONPATH=src/astral_pim_ik:src/astral_arm_teleop python3 \
    src/astral_pim_ik/astral_pim_ik/test_network.py
"""

from __future__ import annotations

import math
import sys

import numpy as np

try:
    import torch
    import torch.nn.functional as F
except ImportError:  # pragma: no cover
    print("torch unavailable — skipping network tests")
    sys.exit(0)

from astral_pim_ik.kinematics import DifferentiableKinematicsLayer, PhysicsInformedLoss, circle_basis
from astral_pim_ik.network import PiM_IK_Net, rotation_6d_to_matrix, transform_to_9d


def test_transform_9d() -> bool:
    print("\n  --- transform_to_9d + rotation_6d_to_matrix ---")
    ok = True
    T = torch.eye(4).repeat(3, 5, 1, 1)  # (B,W,4,4)
    x9 = transform_to_9d(T)
    if x9.shape != (3, 5, 9):
        print(f"  FAIL shape {x9.shape}")
        ok = False
    # first 3 = translation, next 6 = first two rotation columns
    if not torch.allclose(x9[:, 0, :3], torch.zeros(3), atol=1e-6):
        print("  FAIL translation")
        ok = False
    R = rotation_6d_to_matrix(x9[:, 0, 3:])  # (B,3,3)
    I = torch.eye(3).unsqueeze(0).repeat(3, 1, 1)
    if not torch.allclose(R.transpose(1, 2) @ R, I, atol=1e-5):
        print("  FAIL rotation not orthonormal")
        ok = False
    # Non-identity round-trip: an actual SO(3) must survive encode -> decode.
    a = math.radians(30.0)
    Rref = torch.tensor([
        [math.cos(a), -math.sin(a), 0.0],
        [math.sin(a),  math.cos(a), 0.0],
        [0.0, 0.0, 1.0],
    ])
    T2 = torch.eye(4).unsqueeze(0).repeat(2, 4, 1, 1).clone()
    T2[:, :, :3, :3] = Rref
    T2[:, :, :3, 3] = torch.tensor([0.1, -0.2, 0.3])
    x9b = transform_to_9d(T2)
    Rb = rotation_6d_to_matrix(x9b[:, 0, 3:])
    if not torch.allclose(Rb[0], Rref, atol=1e-5):
        print(f"  FAIL rotation round-trip\n  {Rb[0]}\n  vs\n  {Rref}")
        ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_net_forward() -> bool:
    print("\n  --- PiM_IK_Net forward / L2 / gradient ---")
    ok = True
    for backbone in ("transformer", "lstm"):
        m = PiM_IK_Net(d_model=64, num_layers=2, backbone_type=backbone, dropout=0.0)
        x = torch.randn(2, 6, 4, 4)
        y = m(x)
        if y.shape != (2, 6, 2):
            print(f"  FAIL {backbone} shape {y.shape}")
            ok = False
        if not torch.allclose(torch.norm(y, dim=-1), torch.ones(2, 6), atol=1e-4):
            print(f"  FAIL {backbone} L2 != 1")
            ok = False
        loss = y.sum()
        loss.backward()
        if any(p.grad is None or torch.isnan(p.grad).any() for p in m.parameters() if p.requires_grad):
            print(f"  FAIL {backbone} bad gradients")
            ok = False
    # W=1 baseline (no temporal components)
    m = PiM_IK_Net(d_model=64, num_layers=2, backbone_type="transformer", dropout=0.0)
    y = m(torch.randn(2, 1, 4, 4))
    if y.shape != (2, 1, 2):
        print("  FAIL W=1 shape")
        ok = False
    print(f"  Result: {'PASS' if ok else 'FAIL'}")
    return ok


def test_loss_and_convention() -> bool:
    print("\n  --- PhysicsInformedLoss + psi convention vs numpy geometric ---")
    ok = True
    B, W = 3, 8
    gt_psi = torch.randn(B, W, 2)
    gt_psi = F.normalize(gt_psi, dim=-1)
    pred_psi_raw = torch.randn(B, W, 2, requires_grad=True)
    pred_psi = F.normalize(pred_psi_raw, dim=-1)
    S = torch.randn(B, W, 3)
    Wp = torch.randn(B, W, 3) + 0.4
    E_gt = torch.randn(B, W, 3)
    l_se = torch.full((B, W), 0.24)
    l_ew = torch.full((B, W), 0.22)
    is_valid = torch.ones(B, W)

    loss_fn = PhysicsInformedLoss()
    total, d = loss_fn(pred_psi, gt_psi, S, E_gt, Wp, l_se, l_ew, is_valid)
    total.backward()
    if not torch.isfinite(total) or pred_psi_raw.grad is None or torch.isnan(pred_psi_raw.grad).any():
        print("  FAIL loss/gradient NaN")
        ok = False
    for k in ("L_psi", "L_elbow", "L_smooth"):
        if k not in d:
            print(f"  FAIL missing {k}")
            ok = False

    # Convention check: torch circle_basis must equal numpy geometric elbow.
    from astral_arm_teleop.ik.geometric import _circle_basis_sw, _elbow_point
    from astral_pim_ik.geometry import GeometricArmAngleSolver

    g = GeometricArmAngleSolver("left").geom
    q = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0])
    T = np.eye(4)
    T[:3, :3] = GeometricArmAngleSolver("left").fk(q)[:3, :3]
    T[:3, 3] = GeometricArmAngleSolver("left").fk(q)[:3, 3]
    S_np = g.S.astype(np.float64)
    from astral_arm_teleop.ik.geometric import _compute_sw
    _, W_np, _ = _compute_sw(T, g)
    psi = 0.7
    E_np = _elbow_point(psi, S_np, W_np, g)

    St = torch.tensor(S_np).float()
    Wt = torch.tensor(W_np).float()
    C, u, e1, e2, r = circle_basis(St.unsqueeze(0), Wt.unsqueeze(0),
                                   torch.tensor([[g.l_se]]), torch.tensor([[g.l_ew]]))
    Et = C[0] + r[0] * (np.cos(psi) * e1[0] + np.sin(psi) * e2[0])
    err = float(np.linalg.norm(Et.detach().numpy() - E_np))
    if err > 1e-5:
        print(f"  FAIL torch/numpy elbow mismatch {err:.2e} m — psi convention drift")
        ok = False

    print(f"  torch/numpy elbow err {err:.2e} m, loss={d['total_loss']:.4f} -> {'PASS' if ok else 'FAIL'}")
    return ok


def test_elbow_loss_measures_elbow() -> bool:
    print("\n  --- L_elbow measures real elbow error (shadowing regression) ---")
    ok = True
    # A perfect prediction on solver-consistent geometry must drive L_elbow to ~0.
    # Regression for a variable-shadowing bug: PhysicsInformedLoss.forward
    # unpacked ``B, W, _ = pred_psi.shape``, clobbering the wrist tensor ``W``
    # with the window length, so L_elbow diffed against the integer window size
    # and blew up to ~1e1 m for any input.
    from astral_arm_teleop.ik.geometric import _compute_sw
    from astral_pim_ik.geometry import GeometricArmAngleSolver, elbow_position

    s = GeometricArmAngleSolver("left")
    q = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.0, 0.0])
    q = np.clip(q, s.geom.lower + 0.01, s.geom.upper - 0.01)
    T = s.fk(q)
    S, Wn, _ = _compute_sw(T, s.geom)
    E = elbow_position(q, s.geom)
    psi = s.psi_from_config(q)
    psi_vec = np.array([math.cos(psi), math.sin(psi)], dtype=np.float32)

    Ww = 4
    pred = torch.tensor([psi_vec] * Ww).unsqueeze(0)  # (1, Ww, 2) — perfect
    St = torch.tensor(S, dtype=torch.float32).repeat(1, Ww, 1)
    Wt = torch.tensor(Wn, dtype=torch.float32).repeat(1, Ww, 1)
    Et = torch.tensor(E, dtype=torch.float32).repeat(1, Ww, 1)
    lse = torch.full((1, Ww), s.geom.l_se)
    lew = torch.full((1, Ww), s.geom.l_ew)

    loss_fn = PhysicsInformedLoss()
    _, d = loss_fn(pred, pred, St, Et, Wt, lse, lew, torch.ones(1, Ww))
    if d["L_elbow"] > 1e-3:
        print(f"  FAIL L_elbow={d['L_elbow']:.4f} m for a perfect prediction (expected ~0)")
        ok = False
    print(f"  L_elbow={d['L_elbow']:.2e} m (perfect pred) -> {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> None:
    print("Astral PiM-IK — Network test suite")
    results = {
        "transform_9d": test_transform_9d(),
        "net_forward": test_net_forward(),
        "loss_convention": test_loss_and_convention(),
        "elbow_loss_measures_elbow": test_elbow_loss_measures_elbow(),
    }
    print("\n  SUMMARY:")
    for k, v in results.items():
        print(f"    {k}: {'PASS' if v else 'FAIL'}")
    if not all(results.values()):
        sys.exit(1)
    print("FINAL: ALL PASSED")


if __name__ == "__main__":
    main()
