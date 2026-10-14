"""Compatibility helpers for Astral DH.

Canonical params live in :mod:`astral_arm_teleop.ik.analytic` (``AstralParams``).
IK frame is ``left_base_link`` / ``right_base_link`` (see astral_robot_description).
"""

from __future__ import annotations


def __getattr__(name: str):
    if name in ("AstralParams", "RobotMainArmParams"):
        from astral_arm_teleop.ik.analytic import AstralParams

        return AstralParams
    raise AttributeError(name)
