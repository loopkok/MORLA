"""Astral dual-arm IK layer.

URDF numerical IK poses are in ``left_base_link`` / ``right_base_link``.
Analytic DH uses the same arm-base frames (clean MDH after ``R_base``).

Layout::

  ik/
    base.py          -- IKSolverBase protocol
    robot_params.py  -- Modified DH parameters
    analytic.py      -- Nero-port IKSolver / AstralParams (closed-form DH)
    geometric.py     -- GeometricIKSolver (DH-free S/E/W arm-angle, POE+PK)
    urdf_solver.py   -- URDFNumericalIKSolver (Pinocchio LM)
    factory.py       -- AstralIKBridge + make_ik_solver

Use ``make_ik_solver(solver_type)`` from teleop; do not import solvers ad-hoc.
"""

from astral_arm_teleop.ik.factory import (
    AstralIKBridge,
    default_astral_urdf_path,
    default_urdf_path,
    make_ik_solver,
    make_single_arm_ik,
)

__all__ = [
    "AstralIKBridge",
    "default_astral_urdf_path",
    "default_urdf_path",
    "make_ik_solver",
    "make_single_arm_ik",
]
