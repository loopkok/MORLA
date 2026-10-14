import numpy as np
import yaml
from pathlib import Path
from scipy.spatial.transform import Rotation

from morla_core.coordinate_contract import build_coordinate_contract
from morla_core.coordinates import UNITY_TO_QUEST3_CANONICAL

M = np.array([[-1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
OLD = {
    "left": np.array([[0, 1, 0], [0, 0, 1], [-1, 0, 0]], dtype=float),
    "right": np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]], dtype=float),
}


def _tcp_transform(seed):
    rng = np.random.default_rng(seed)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.random(random_state=rng).as_matrix()
    transform[:3, 3] = rng.uniform(-0.08, 0.08, 3)
    return transform


def test_six_translation_directions_match_the_legacy_unity_mapping():
    np.testing.assert_array_equal(UNITY_TO_QUEST3_CANONICAL, M)
    directions = [sign * np.eye(3)[axis] for axis in range(3) for sign in (-1, 1)]
    for side, old_matrix in OLD.items():
        new_matrix = old_matrix @ M.T
        for delta in directions:
            np.testing.assert_allclose(
                new_matrix @ (M @ delta), old_matrix @ delta, atol=1e-14
            )


def test_three_axis_and_random_quaternion_rotation_deltas_match():
    rng = np.random.default_rng(521)
    rotations = [
        Rotation.from_rotvec(np.eye(3)[axis] * angle).as_matrix()
        for axis in range(3)
        for angle in (-1.1, -0.2, 0.3, 1.4)
    ]
    rotations.extend(Rotation.random(500, random_state=rng).as_matrix())
    for side, old_matrix in OLD.items():
        new_matrix = old_matrix @ M.T
        for rotation in rotations:
            old_result = old_matrix @ rotation @ old_matrix.T
            robot_rotation = M @ rotation @ M.T
            new_result = new_matrix @ robot_rotation @ new_matrix.T
            np.testing.assert_allclose(new_result, old_result, atol=1e-14)


def test_nonzero_tcp_offset_and_initial_tool_pose_are_unchanged():
    rng = np.random.default_rng(902)
    tcp_to_flange = np.linalg.inv(_tcp_transform(11))
    initial = _tcp_transform(12)
    for side, old_matrix in OLD.items():
        new_matrix = old_matrix @ M.T
        for _ in range(40):
            delta_u = rng.normal(0, 0.1, 3)
            delta_rot_u = Rotation.random(random_state=rng).as_matrix()
            old_p = initial[:3, 3] + old_matrix @ delta_u
            new_p = initial[:3, 3] + new_matrix @ (M @ delta_u)
            old_r = (old_matrix @ delta_rot_u @ old_matrix.T) @ initial[:3, :3]
            new_delta_r = M @ delta_rot_u @ M.T
            new_r = (new_matrix @ new_delta_r @ new_matrix.T) @ initial[:3, :3]
            old_target = np.eye(4)
            new_target = np.eye(4)
            old_target[:3, :3], old_target[:3, 3] = old_r, old_p
            new_target[:3, :3], new_target[:3, 3] = new_r, new_p
            np.testing.assert_allclose(
                old_target @ tcp_to_flange,
                new_target @ tcp_to_flange,
                atol=1e-13,
            )


def test_profile_alignment_is_derived_from_common_body_frame_and_urdf_mounts():
    root = Path(__file__).resolve().parents[4]
    profile_path = root / "src/robots/nero/morla_nero_robot/config/profiles/nero_dual_xhand.yaml"
    profile = yaml.safe_load(profile_path.read_text())
    contract = build_coordinate_contract(profile)
    urdf_path = root / "src/robot_description/urdf/nero/xhand_nero_description.urdf"
    assert set(contract.arms) == {"left", "right"}
    for side, legacy_matrix in OLD.items():
        effective = contract.resolve_arm_input_rotation(side, str(urdf_path))
        np.testing.assert_allclose(effective, legacy_matrix @ M.T, atol=4e-6)
