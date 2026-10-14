"""astral_pim_ik — PiM-IK-style two-stage IK for the Astral arm.

Two stages, following the PiM-IK idea (neural arm angle -> closed-form IK):

1. a neural network predicts the arm angle ``psi`` (the elbow's angle on the
   orbit circle around the shoulder->wrist axis) from the end-effector pose;
2. the DH-free geometric closed-form IK from ``astral_arm_teleop.ik.geometric``
   resolves the 7 joints from ``(T_ee, psi)``.

The geometric stage is deterministic: it reuses the POE + Paden-Kahan arm-angle
solution and strips away every VR-teleop constraint (escape hysteresis, soft
psi_ref prior, singular-hold, 1D QP, continuity scoring, reach clamp). Only the
hard joint-limit filter and a minimal warm-start branch selection remain.
"""

__version__ = "0.1.0"

__all__ = [
    "GeometricArmAngleSolver",
    "PiMIKPipeline",
    "SyntheticPsiPredictor",
]


def __getattr__(name):
    # Lazy imports so the package imports cleanly even without torch/pinocchio.
    if name in ("GeometricArmAngleSolver",):
        from astral_pim_ik.geometry import GeometricArmAngleSolver

        return GeometricArmAngleSolver
    if name in ("PiMIKPipeline", "SyntheticPsiPredictor"):
        from astral_pim_ik.pipeline import PiMIKPipeline, SyntheticPsiPredictor

        return {"PiMIKPipeline": PiMIKPipeline, "SyntheticPsiPredictor": SyntheticPsiPredictor}[name]
    raise AttributeError(name)
