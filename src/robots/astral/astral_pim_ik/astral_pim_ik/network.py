"""PiM-IK arm-angle network, ported for the Astral arm.

Stage 1 of the two-stage pipeline: map a window of end-effector poses
``T_ee`` to the arm angle ``psi`` as a unit vector ``[cos psi, sin psi]``.
Direct port of ``core/pim_ik_net.py`` from the PiM-IK reference project, with
the output renamed from the G1 "swivel angle" to the Astral "arm angle psi".
The 9D continuous pose representation (3D translation + 6D rotation, after
Zhou et al. CVPR 2019) and the Stem/Backbone/Head layout are unchanged.

Default backbone is ``transformer`` (PiM-IK's ablation winner) so the network
needs only ``torch``; ``mamba`` is available when ``mamba-ssm`` is installed.
"""

from __future__ import annotations

from typing import Optional

import torch
import torch.nn as nn
import torch.nn.functional as F

try:
    from mamba_ssm import Mamba  # type: ignore

    MAMBA_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on optional dep
    MAMBA_AVAILABLE = False


# ============================================================================
# 9D continuous pose representation
# ============================================================================

def transform_to_9d(T_ee: torch.Tensor) -> torch.Tensor:
    """4x4 homogeneous transform -> 9D continuous pose (3D translation + 6D rot).

    The 6D rotation is the first two columns of the 3x3 rotation matrix,
    flattened (Zhou et al., CVPR 2019). Avoids quaternion/Euler singularities.

    Args:
        T_ee: (B, 4, 4) or (B, W, 4, 4).

    Returns:
        (B, 9) or (B, W, 9): [x, y, z, r00, r10, r20, r01, r11, r21].
    """
    if T_ee.dim() == 3:
        translation = T_ee[:, :3, 3]
        # First two columns, flattened column-major: [col0; col1] =
        # [r00, r10, r20, r01, r11, r21]. Transpose so `view` walks columns
        # before rows (the raw `[:3, :2]` block is row-major, which would
        # interleave r00,r01,r10,... and break rotation_6d_to_matrix).
        rotation_6d = T_ee[:, :3, :2].transpose(0, 1).contiguous().view(-1, 6)
    elif T_ee.dim() == 4:
        translation = T_ee[:, :, :3, 3]
        rotation_6d = T_ee[:, :, :3, :2].transpose(-1, -2).contiguous()
        rotation_6d = rotation_6d.view(*rotation_6d.shape[:2], 6)
    else:
        raise ValueError(f"T_ee must be (B,4,4) or (B,W,4,4), got shape {tuple(T_ee.shape)}")
    return torch.cat([translation, rotation_6d], dim=-1)


def rotation_6d_to_matrix(rotation_6d: torch.Tensor) -> torch.Tensor:
    """6D rotation -> 3x3 rotation matrix via Gram-Schmidt (for verification)."""
    shape = rotation_6d.shape[:-1]
    a1 = rotation_6d[..., :3]
    a2 = rotation_6d[..., 3:]
    b1 = F.normalize(a1, dim=-1)
    b2 = a2 - (b1 * a2).sum(dim=-1, keepdim=True) * b1
    b2 = F.normalize(b2, dim=-1)
    b3 = torch.cross(b1, b2, dim=-1)
    return torch.stack([b1, b2, b3], dim=-1)


# ============================================================================
# Backbone blocks
# ============================================================================

class MambaBlock(nn.Module):
    """Pre-LN Mamba block with residual connection (requires mamba-ssm)."""

    def __init__(self, d_model: int, d_state: int = 16, d_conv: int = 4, expand: int = 2):
        super().__init__()
        if not MAMBA_AVAILABLE:
            raise ImportError("backbone_type='mamba' requires mamba-ssm")
        self.norm = nn.LayerNorm(d_model)
        self.mamba = Mamba(d_model=d_model, d_state=d_state, d_conv=d_conv, expand=expand)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x + self.mamba(self.norm(x))


# ============================================================================
# PiM_IK_Net
# ============================================================================

class PiM_IK_Net(nn.Module):
    """Arm-angle network: (B, W, 4, 4) -> (B, W, 2) unit vector [cos psi, sin psi].

    Structure (identical to the PiM-IK reference):
      1. Stem: Linear(9 -> d_model) + Conv1d(k=3, replicate) + GELU + Dropout1d
      2. Backbone: stacked Mamba / unidirectional LSTM / causal Transformer
      3. Head: MLP(256 -> 128 -> 2) then L2 normalize to the unit circle

    ``W == 1`` skips the temporal components (pure per-frame MLP baseline).
    """

    def __init__(
        self,
        d_model: int = 256,
        num_layers: int = 4,
        d_state: int = 16,
        d_conv: int = 4,
        expand: int = 2,
        backbone_type: str = "transformer",
        dropout: float = 0.1,
    ):
        super().__init__()
        self.d_model = d_model
        self.num_layers = num_layers
        self.backbone_type = backbone_type
        self.dropout = dropout

        self.dropout_layer = nn.Dropout(dropout)
        self.dropout1d = nn.Dropout1d(dropout)

        # 1. Stem
        self.stem_linear = nn.Linear(9, d_model)
        self.stem_conv = nn.Conv1d(d_model, d_model, kernel_size=3, padding=1, padding_mode="replicate")
        self.stem_act = nn.GELU()

        # 2. Backbone
        if backbone_type == "mamba":
            if not MAMBA_AVAILABLE:
                raise ImportError("backbone_type='mamba' requires mamba-ssm")
            self.mamba_blocks = nn.ModuleList(
                [MambaBlock(d_model, d_state, d_conv, expand) for _ in range(num_layers)]
            )
        elif backbone_type == "lstm":
            self.lstm = nn.LSTM(
                input_size=d_model, hidden_size=d_model, num_layers=num_layers, batch_first=True
            )
        elif backbone_type == "transformer":
            self.pos_embedding = nn.Parameter(torch.zeros(1, 500, d_model))
            nn.init.trunc_normal_(self.pos_embedding, std=0.02)
            encoder_layer = nn.TransformerEncoderLayer(
                d_model=d_model, nhead=8, dim_feedforward=d_model * 4,
                batch_first=True, norm_first=True,
            )
            self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        else:
            raise ValueError(f"unknown backbone_type={backbone_type!r} (mamba/lstm/transformer)")

        # 3. Head
        self.head = nn.Sequential(
            nn.Linear(d_model, 128),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(128, 2),
        )

    def forward(self, T_ee: torch.Tensor) -> torch.Tensor:
        B, W, _, _ = T_ee.shape
        x = transform_to_9d(T_ee)          # (B, W, 9)
        x = self.stem_linear(x)            # (B, W, d_model)

        if W == 1:
            x = self.stem_act(x)           # (B, 1, d_model) — MLP baseline
        else:
            x = x.permute(0, 2, 1)         # (B, d_model, W)
            x = self.stem_conv(x)
            x = x.permute(0, 2, 1)         # (B, W, d_model)
            x = self.stem_act(x)

            x = x.permute(0, 2, 1)
            x = self.dropout1d(x)
            x = x.permute(0, 2, 1)

            if self.backbone_type == "mamba":
                for blk in self.mamba_blocks:
                    x = blk(x)
                    x = self.dropout_layer(x)
            elif self.backbone_type == "lstm":
                x, _ = self.lstm(x)
            elif self.backbone_type == "transformer":
                x = x + self.pos_embedding[:, :W, :]
                causal_mask = nn.Transformer.generate_square_subsequent_mask(W, device=x.device)
                x = self.transformer(x, mask=causal_mask)

        out = self.head(x)                 # (B, W, 2)
        pred_psi = F.normalize(out, p=2, dim=-1)  # unit vector on the psi circle
        return pred_psi
