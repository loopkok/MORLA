"""ROS-free coordinate and rigid-transform helpers used across MORLA.

Transforms use column vectors and ``T_target_from_source`` naming. RPY follows
the fixed-axis XYZ convention used by URDF (``Rz(yaw) Ry(pitch) Rx(roll)``).
"""
from __future__ import annotations

import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path


class CoordinateError(ValueError):
    """Raised when a coordinate mapping is malformed or frame-incompatible."""


UNITY_TO_QUEST3_CANONICAL = np.array(
    [[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]], dtype=float
)


def validate_rotation(rotation: np.ndarray, *, atol: float = 1e-7) -> np.ndarray:
    """Return a validated proper SO(3) rotation (reflections are rejected)."""
    value = np.asarray(rotation, dtype=float)
    if value.shape != (3, 3) or not np.isfinite(value).all():
        raise CoordinateError("rotation must be a finite 3x3 matrix")
    if not np.allclose(value.T @ value, np.eye(3), atol=atol, rtol=0.0):
        raise CoordinateError("rotation must be orthonormal")
    determinant = float(np.linalg.det(value))
    if not np.isclose(determinant, 1.0, atol=atol, rtol=0.0):
        raise CoordinateError(f"rotation determinant must be +1, got {determinant:g}")
    return value.copy()


def validate_basis_change(basis: np.ndarray, *, atol: float = 1e-7) -> np.ndarray:
    """Validate an orthogonal basis change, including an explicit reflection."""
    value = np.asarray(basis, dtype=float)
    if value.shape != (3, 3) or not np.isfinite(value).all():
        raise CoordinateError("basis change must be a finite 3x3 matrix")
    if not np.allclose(value.T @ value, np.eye(3), atol=atol, rtol=0.0):
        raise CoordinateError("basis change must be orthogonal")
    if not np.isclose(abs(float(np.linalg.det(value))), 1.0, atol=atol, rtol=0.0):
        raise CoordinateError("basis change determinant must be +1 or -1")
    return value.copy()


def validate_transform(transform: np.ndarray, *, atol: float = 1e-7) -> np.ndarray:
    """Return a validated SE(3) transform; basis reflections are not transforms."""
    value = np.asarray(transform, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise CoordinateError("transform must be a finite 4x4 matrix")
    validate_rotation(value[:3, :3], atol=atol)
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=atol, rtol=0.0):
        raise CoordinateError("transform last row must be [0, 0, 0, 1]")
    return value.copy()


def rpy_to_matrix(rpy: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    """Convert URDF fixed-axis XYZ roll/pitch/yaw values to a proper rotation."""
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    if not np.isfinite([roll, pitch, yaw]).all():
        raise CoordinateError("rpy must contain three finite angles")
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
         [-sp, cp * sr, cp * cr]],
        dtype=float,
    )


def transform_from_xyz_rpy(
    xyz: np.ndarray | tuple[float, float, float],
    rpy: np.ndarray | tuple[float, float, float],
) -> np.ndarray:
    """Build an SE(3) transform from a URDF origin."""
    position = np.asarray(xyz, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise CoordinateError("xyz must contain three finite values")
    result = np.eye(4)
    result[:3, :3] = rpy_to_matrix(rpy)
    result[:3, 3] = position
    return result


def fixed_transform_between_urdf(
    urdf_path: str | Path, source_frame: str, target_frame: str
) -> np.ndarray:
    """Read a fixed-joint path and return ``T_source_from_target``.

    The path must exist and every traversed joint must be fixed; this prevents
    silently treating a movable joint as a static installation transform.
    """
    root = ET.parse(str(urdf_path)).getroot()
    edges: dict[str, list[tuple[str, np.ndarray]]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None or not parent.get("link") or not child.get("link"):
            continue
        if joint.get("type") != "fixed":
            continue
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0] if origin is None else [float(x) for x in origin.get("xyz", "0 0 0").split()]
        rpy = [0.0, 0.0, 0.0] if origin is None else [float(x) for x in origin.get("rpy", "0 0 0").split()]
        T_parent_child = transform_from_xyz_rpy(xyz, rpy)
        parent_name, child_name = parent.get("link"), child.get("link")
        edges.setdefault(parent_name, []).append((child_name, T_parent_child))
        edges.setdefault(child_name, []).append((parent_name, inverse(T_parent_child)))

    # Breadth-first traversal carrying T_source_from_current.
    queue: list[tuple[str, np.ndarray]] = [(source_frame, np.eye(4))]
    visited = {source_frame}
    while queue:
        current, T_source_current = queue.pop(0)
        if current == target_frame:
            return validate_transform(T_source_current)
        for neighbor, T_current_neighbor in edges.get(current, []):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            queue.append((neighbor, compose(T_source_current, T_current_neighbor)))
    raise CoordinateError(
        f"no fixed URDF path from {source_frame!r} to {target_frame!r} in {urdf_path}"
    )


def inverse(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid transform without general-purpose matrix inversion."""
    value = validate_transform(transform)
    result = np.eye(4)
    result[:3, :3] = value[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ value[:3, 3]
    return result


def compose(*transforms: np.ndarray) -> np.ndarray:
    """Compose transforms in application order: ``T_a_b @ T_b_c``."""
    result = np.eye(4)
    for transform in transforms:
        result = result @ validate_transform(transform)
    return validate_transform(result)


def change_basis_vector(basis: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Express a vector in a new basis (basis may explicitly be a reflection)."""
    value = np.asarray(vector, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise CoordinateError("vector must contain three finite values")
    return validate_basis_change(basis) @ value


def change_basis_rotation(basis: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """Change orientation coordinates by conjugation, including handedness changes."""
    change = validate_basis_change(basis)
    orientation = validate_rotation(rotation)
    result = change @ orientation @ change.T
    return validate_rotation(result)


def transform_pose(
    parent_from_source: np.ndarray, source_from_object: np.ndarray
) -> np.ndarray:
    """Express a pose in another parent frame: ``T_parent_source @ T_source_object``."""
    return compose(parent_from_source, source_from_object)


def solver_tip_to_tcp(
    T_solver_tip: np.ndarray, T_solver_tip_tcp: np.ndarray
) -> np.ndarray:
    """Convert solver FK to the configured TCP pose."""
    return compose(T_solver_tip, T_solver_tip_tcp)


def tcp_to_solver_tip(
    T_parent_tcp: np.ndarray, T_solver_tip_tcp: np.ndarray
) -> np.ndarray:
    """Convert a TCP target to the solver's tip target."""
    return compose(T_parent_tcp, inverse(T_solver_tip_tcp))


def incremental_pose(
    source_rotation_to_target: np.ndarray,
    position_delta_source: np.ndarray,
    rotation_delta_source: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Map a translational and rotational increment between proper frames."""
    align = validate_rotation(source_rotation_to_target)
    dp = np.asarray(position_delta_source, dtype=float)
    if dp.shape != (3,) or not np.isfinite(dp).all():
        raise CoordinateError("position increment must contain three finite values")
    dr = validate_rotation(rotation_delta_source)
    return align @ dp, validate_rotation(align @ dr @ align.T)
