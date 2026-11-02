"""Behaviour-preservation checks for the rebase back to raw Unity input.

The receiver now publishes what Quest sends (Unity: X right, Y up, Z forward).
Every robot declares its own Unity -> solver-base matrix; for Nero that matrix
is the pre-refactor ``vr_to_arm_rot`` literal again, so the joint commands must
come out identical to the canonical-era behaviour.
"""
import numpy as np
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation

from morla_core.coordinate_contract import build_coordinate_contract
from morla_core.coordinates import (
    UNITY_LEFT_HANDED_BASIS_ID,
    UNITY_TO_QUEST3_CANONICAL,
    rebased_to_unity,
)

M = UNITY_TO_QUEST3_CANONICAL
# Effective pre-refactor mapping: Unity deltas straight into each solver base.
# These literals were ``vr_to_arm_rot`` in the canonical-era pipeline, where the
# receiver had already applied ``c = M u``; their determinant is -1.
OLD = {
    "left": np.array([[0, 1, 0], [0, 0, 1], [-1, 0, 0]], dtype=float),
    "right": np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]], dtype=float),
}
# The same mappings declared in the canonical basis (proper rotations); M is
# its own inverse only up to M @ M = diag(1, -1, -1), so undo it with M.T.
OLD_CANONICAL = {side: matrix @ M.T for side, matrix in OLD.items()}


class StubSolver:
    """Tip translation follows the commanded target; joints echo the seed."""

    def __init__(self):
        self.seed = None

    def sync_state(self, seed):
        self.seed = np.asarray(seed, dtype=float)

    def fk(self, seed):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(seed, dtype=float)[:3]
        return pose

    def solve(self, target):
        result = self.seed.copy()
        result[:3] = target[:3, 3]
        return result


def _pose(position, rotation):
    return {"position": np.asarray(position, dtype=float).tolist(),
            "quaternion": Rotation.from_matrix(rotation).as_quat().tolist()}


def _tick(matrix, seed, anchor, pose):
    """Anchor on ``anchor`` then command ``pose`` in one controller instance."""
    from morla_teleop.kinematics import CartesianController

    controller = CartesianController(StubSolver())
    parameters = {"input_rotation": np.asarray(matrix, dtype=float).tolist()}
    controller(anchor, seed, 0.02, parameters)
    return np.asarray(controller(pose, seed, 0.02, parameters), dtype=float)


def test_rebasing_a_declared_matrix_is_a_right_multiplication_by_M():
    # canonical = M @ unity, so a matrix declared in the canonical basis is
    # rebased by right-multiplication: A_new = A_old @ M.
    rng = np.random.default_rng(7)
    directions = [sign * np.eye(3)[axis] for axis in range(3) for sign in (-1, 1)]
    rotations = [Rotation.from_rotvec(np.eye(3)[axis] * angle).as_matrix()
                 for axis in range(3) for angle in (-1.1, -0.2, 0.3, 1.4)]
    rotations.extend(Rotation.random(200, random_state=rng).as_matrix())
    for old_declaration in (np.eye(3), *OLD_CANONICAL.values()):
        new_declaration = rebased_to_unity(old_declaration)
        # Rebasing by M flips handedness: a proper canonical rotation becomes
        # the det -1 basis change the controller consumes.
        assert np.linalg.det(new_declaration) == -np.linalg.det(old_declaration)
        for delta in directions:
            # Same Unity delta, same base delta: new @ unity == old @ canonical.
            np.testing.assert_allclose(
                new_declaration @ delta, old_declaration @ (M @ delta), atol=1e-14
            )
        for rotation in rotations:
            # Unity rotation and its canonical image are conjugated by M.
            old_result = old_declaration @ (M @ rotation @ M.T) @ old_declaration.T
            new_result = new_declaration @ rotation @ new_declaration.T
            np.testing.assert_allclose(new_result, old_result, atol=1e-14)


def test_effective_matrix_restores_the_legacy_unity_mapping():
    root = Path(__file__).resolve().parents[4]
    profile_path = root / "src/robots/nero/morla_nero_robot/config/profiles/nero_dual_xhand.yaml"
    profile = yaml.safe_load(profile_path.read_text())
    contract = build_coordinate_contract(profile)
    assert contract.input_basis_id == UNITY_LEFT_HANDED_BASIS_ID
    declared = np.asarray(contract.input_rotation_to_robot_body, dtype=float).reshape(3, 3)
    # Raw Unity input makes every declared alignment a basis change.
    assert np.isclose(np.linalg.det(declared), -1.0, atol=1e-7)
    urdf_path = root / "src/robot_description/urdf/nero/xhand_nero_description.urdf"
    assert set(contract.arms) == {"left", "right"}
    for side, legacy_matrix in OLD.items():
        effective = contract.resolve_arm_input_rotation(side, str(urdf_path))
        np.testing.assert_allclose(effective, legacy_matrix, atol=4e-6)


def test_unity_input_with_the_new_matrix_commands_the_same_joints():
    """One controller tick: Unity + rebased matrix == canonical + old matrix."""
    rng = np.random.default_rng(902)
    seed = np.array([0.2, 0.35, -0.15, 0.1, -0.2, 0.05])
    for side in ("left", "right"):
        # Unity input uses the effective det -1 matrix; the canonical-era
        # controller received the same mapping declared in the canonical basis.
        old_declaration = OLD_CANONICAL[side]
        new_declaration = OLD[side]
        assert np.allclose(rebased_to_unity(old_declaration), new_declaration)
        anchor_position = rng.normal(0, 0.2, 3)
        anchor_rotation = Rotation.random(random_state=rng).as_matrix()
        for _ in range(10):
            position = anchor_position + rng.normal(0, 0.05, 3)
            rotation = Rotation.random(random_state=rng).as_matrix()
            unity_joints = _tick(
                new_declaration, seed,
                _pose(anchor_position, anchor_rotation),
                _pose(position, rotation),
            )
            canonical_joints = _tick(
                old_declaration, seed,
                _pose(M @ anchor_position, M @ anchor_rotation @ M.T),
                _pose(M @ position, M @ rotation @ M.T),
            )
            np.testing.assert_allclose(unity_joints, canonical_joints, atol=1e-12)
