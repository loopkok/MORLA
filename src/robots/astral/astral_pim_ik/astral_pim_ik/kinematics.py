"""Differentiable kinematics layer + physics-informed loss (Astral arm angle).

Ported from ``core/pim_ik_kinematics.py`` of the PiM-IK reference project and
adapted to the Astral ``psi`` arm-angle convention. The critical change: the
orbit-circle basis reproduces ``astral_arm_teleop.ik.geometric._circle_basis_sw``
exactly (reference vector ``psi_ref = S``, the shoulder center), so the elbow
the loss supervises is the *same* elbow the geometric solver produces for that
``psi``. If this convention ever drifts from ``geometry.py`` the network learns
an arm angle the solver consumes wrongly — keep them in lockstep.
"""

from __future__ import annotations

from typing import Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F

EPS = 1e-6


# ============================================================================
# Vectorized port of geometric._circle_basis_sw + _elbow_point
# ============================================================================

def circle_basis(
    S: torch.Tensor, W: torch.Tensor, l_se: torch.Tensor, l_ew: torch.Tensor
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Elbow-circle frame, batched: returns (C, u, e1, e2, r).

    ``S``, ``W``: (..., 3). ``l_se``/``l_ew``: (...) link lengths.
    Faithful to ``geometric._circle_basis_sw`` including its fallback when the
    reference vector (here ``psi_ref = S``) is ~parallel to the S-W axis.
    """
    sw = W - S                                   # (..., 3)
    l_sw = torch.norm(sw, dim=-1)                # (...)
    u = sw / (l_sw.unsqueeze(-1) + EPS)

    x = (l_se ** 2 - l_ew ** 2 + l_sw ** 2) / (2.0 * l_sw + EPS)
    r = torch.sqrt(torch.clamp(l_se ** 2 - x ** 2, min=EPS))
    C = S + x.unsqueeze(-1) * u

    # Reference vector psi_ref = S (same as extract_arm_geometry).
    rx, ry, rz = S[..., 0], S[..., 1], S[..., 2]
    ux, uy, uz = u[..., 0], u[..., 1], u[..., 2]
    # t0 = psi_ref x u
    t0x, t0y, t0z = ry * uz - rz * uy, rz * ux - rx * uz, rx * uy - ry * ux
    n0 = torch.sqrt(t0x ** 2 + t0y ** 2 + t0z ** 2)
    # fallback 1 (u ~parallel x): t1 = [0, -uz, uy]
    t1x, t1y, t1z = torch.zeros_like(ux), -uz, uy
    n1 = torch.sqrt(t1x ** 2 + t1y ** 2 + t1z ** 2)
    # fallback 2 (u ~parallel y): t2 = [uz, 0, -ux]
    t2x, t2y, t2z = uz, torch.zeros_like(uy), -ux
    n2 = torch.sqrt(t2x ** 2 + t2y ** 2 + t2z ** 2)

    use0 = n0 > 1e-10
    use1 = (~use0) & (n1 > 1e-10)
    tx = torch.where(use0, t0x, torch.where(use1, t1x, t2x))
    ty = torch.where(use0, t0y, torch.where(use1, t1y, t2y))
    tz = torch.where(use0, t0z, torch.where(use1, t1z, t2z))
    n = torch.sqrt(tx ** 2 + ty ** 2 + tz ** 2).clamp_min(EPS)
    e1 = torch.stack([tx / n, ty / n, tz / n], dim=-1)
    e2 = torch.linalg.cross(u, e1, dim=-1)
    return C, u, e1, e2, r


def elbow_from_psi(
    psi: torch.Tensor, C: torch.Tensor, e1: torch.Tensor, e2: torch.Tensor, r: torch.Tensor
) -> torch.Tensor:
    """E = C + r (cos psi e1 + sin psi e2). ``psi``: (..., 2) unit vector."""
    cos_psi = psi[..., 0:1]
    sin_psi = psi[..., 1:2]
    return C + r.unsqueeze(-1) * (cos_psi * e1 + sin_psi * e2)


# ============================================================================
# Differentiable kinematics layer
# ============================================================================

class DifferentiableKinematicsLayer(nn.Module):
    """Parameter-free layer: predicted arm angle -> elbow 3D position."""

    def forward(
        self,
        pred_psi: torch.Tensor,
        S: torch.Tensor,
        W: torch.Tensor,
        l_se: torch.Tensor,
        l_ew: torch.Tensor,
    ) -> torch.Tensor:
        pred_psi_norm = F.normalize(pred_psi, p=2, dim=-1)
        C, _u, e1, e2, r = circle_basis(S, W, l_se, l_ew)
        return elbow_from_psi(pred_psi_norm, C, e1, e2, r)


# ============================================================================
# Physics-informed loss
# ============================================================================

class PhysicsInformedLoss(nn.Module):
    """Joint loss: L_psi (L1 on the arm-angle vector) + L_elbow (RMSE on the
    elbow recovered through the differentiable layer) + L_smooth (Jerk on psi).

    Every term is masked by ``is_valid`` (False = singular / unobservable psi).
    """

    def __init__(
        self,
        w_psi: float = 1.0,
        w_elbow: float = 1.0,
        w_smooth: float = 0.01,
        elbow_loss_type: str = "rmse",
    ):
        super().__init__()
        self.w_psi = w_psi
        self.w_elbow = w_elbow
        self.w_smooth = w_smooth
        self.elbow_loss_type = elbow_loss_type.lower()
        assert self.elbow_loss_type in ("mse", "rmse", "l1")
        self.kinematics_layer = DifferentiableKinematicsLayer()

    def forward(
        self,
        pred_psi: torch.Tensor,
        gt_psi: torch.Tensor,
        S: torch.Tensor,
        E_gt: torch.Tensor,
        W: torch.Tensor,
        l_se: torch.Tensor,
        l_ew: torch.Tensor,
        is_valid: torch.Tensor,
    ) -> Tuple[torch.Tensor, dict]:
        # NOTE: the wrist-position tensor arrives as ``W``; do NOT unpack the
        # window length into a variable named ``W`` here (it would shadow the
        # wrist and L_elbow would diff against the integer window size).
        B, T, _ = pred_psi.shape
        valid_mask = is_valid.float()
        num_valid = valid_mask.sum() + 1e-6

        # L_psi: L1 on the arm-angle unit vector.
        psi_l1 = torch.abs(pred_psi - gt_psi).sum(dim=-1)
        L_psi = (psi_l1 * valid_mask).sum() / num_valid

        # L_elbow: elbow recovered via the differentiable layer vs ground truth.
        E_pred = self.kinematics_layer(pred_psi, S, W, l_se, l_ew)
        if self.elbow_loss_type == "mse":
            err = ((E_pred - E_gt) ** 2).sum(dim=-1)
            L_elbow = (err * valid_mask).sum() / num_valid
        elif self.elbow_loss_type == "rmse":
            err = torch.sqrt(((E_pred - E_gt) ** 2).sum(dim=-1) + 1e-8)
            L_elbow = (err * valid_mask).sum() / num_valid
        else:  # l1
            err = torch.abs(E_pred - E_gt).sum(dim=-1)
            L_elbow = (err * valid_mask).sum() / num_valid

        # L_smooth: second-order difference (Jerk) on the psi vector.
        if T >= 3:
            jerk = pred_psi[:, 2:] - 2.0 * pred_psi[:, 1:-1] + pred_psi[:, :-2]
            jerk_sq = (jerk ** 2).sum(dim=-1)
            mask_smooth = valid_mask[:, 2:] * valid_mask[:, 1:-1] * valid_mask[:, :-2]
            L_smooth = (jerk_sq * mask_smooth).sum() / (mask_smooth.sum() + 1e-6)
        else:
            L_smooth = pred_psi.new_zeros(())

        total = self.w_psi * L_psi + self.w_elbow * L_elbow + self.w_smooth * L_smooth
        loss_dict = {
            "L_psi": L_psi.item(),
            "L_elbow": L_elbow.item(),
            "L_smooth": L_smooth.item(),
            "total_loss": total.item(),
        }
        return total, loss_dict
