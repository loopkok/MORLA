"""The receiver's default output is raw Unity; conversion is a debug escape hatch."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip("rclpy")

from morla_input.quest3_udp_mocap import (  # noqa: E402
    BODY_FRAME_ID_CANONICAL,
    BODY_FRAME_ID_UNITY,
    WORLD_FRAME_ID_CANONICAL,
    WORLD_FRAME_ID_UNITY,
    Quest3UDPMocap as Receiver,
    unity_landmarks_to_robot,
    unity_pose_to_robot,
)


def receiver(convert_to_robot=False):
    published = {"poses": [], "landmarks": []}
    node = SimpleNamespace(
        convert_to_robot=convert_to_robot,
        both=True,
        arm_side="both",
        get_logger=lambda: Mock(),
        process_landmarks=lambda points, label: Receiver.process_landmarks(
            node, points, label
        ),
        _publish_side_pose=lambda side, pos, quat, when, src: published["poses"].append(
            (side, np.array(pos), np.array(quat), src)
        ),
        publish_mocap_data=lambda points, side, when: published["landmarks"].append(
            (side, np.array(points))
        ),
    )
    node._unity_to_out = lambda pos, quat: Receiver._unity_to_out(node, pos, quat)
    node._parse_pose7 = Receiver._parse_pose7
    return node, published


def landmarks_line(side, points):
    values = ", ".join(str(value) for value in np.asarray(points).reshape(-1))
    return f"{side.capitalize()} landmarks:, {values}"


def hand_points():
    rng = np.random.default_rng(41)
    points = rng.normal(size=(21, 3)) * 0.02
    # Asymmetric wrist offset so any axis flip or permutation is visible.
    points[0] = [0.11, -0.07, 0.23]
    points[8] = [0.13, -0.03, 0.27]
    return points


def test_default_mode_republishes_the_unity_pose_unchanged():
    node, published = receiver()
    position = np.array([0.3, 1.2, -0.4])
    quaternion = np.array([0.1, -0.2, 0.3, 0.9273618495495703])
    line = "Right wrist:, " + ", ".join(
        str(value) for value in np.concatenate((position, quaternion))
    )
    Receiver.process_line(node, line, 1.0)
    side, out_position, out_quaternion, source = published["poses"][0]
    assert (side, source) == ("right", "hand")
    np.testing.assert_array_equal(out_position, position)
    np.testing.assert_array_equal(out_quaternion, quaternion)


def test_default_mode_republishes_unity_landmarks_without_a_flip():
    node, published = receiver()
    points = hand_points()
    Receiver.process_line(node, landmarks_line("right", points), 1.0)
    side, out = published["landmarks"][0]
    assert side == "right"
    # Only the documented wrist-centering happens; no X flip, axis swap or
    # handedness change sits between the wire and the retargeter.
    np.testing.assert_array_equal(out, points - points[:1])
    assert out[8, 0] > 0.0 and out[8, 2] > 0.0


def test_conversion_escape_hatch_restores_the_canonical_output():
    node, published = receiver(convert_to_robot=True)
    points = hand_points()
    Receiver.process_line(node, landmarks_line("left", points), 1.0)
    expected = unity_landmarks_to_robot(points)
    np.testing.assert_allclose(published["landmarks"][0][1], expected - expected[:1])

    position = np.array([0.3, 1.2, -0.4])
    quaternion = np.array([0.0, 0.0, 0.0, 1.0])
    line = "Left wrist:, " + ", ".join(
        str(value) for value in np.concatenate((position, quaternion))
    )
    Receiver.process_line(node, line, 1.0)
    expected_position, expected_quaternion = unity_pose_to_robot(position, quaternion)
    np.testing.assert_allclose(published["poses"][0][1], expected_position)
    np.testing.assert_allclose(published["poses"][0][2], expected_quaternion)


def test_frame_ids_name_the_basis_they_contain():
    assert WORLD_FRAME_ID_UNITY == "quest3_tracking_world_unity"
    assert BODY_FRAME_ID_UNITY == "quest3_operator_body_unity"
    assert WORLD_FRAME_ID_CANONICAL == "quest3_tracking_world_canonical"
    assert BODY_FRAME_ID_CANONICAL == "quest3_operator_body_canonical"
    node = SimpleNamespace(
        _iobt_active=lambda: False, _world_frame_id=WORLD_FRAME_ID_UNITY
    )
    assert Receiver._pose_frame_id(node) == WORLD_FRAME_ID_UNITY
    node = SimpleNamespace(
        _iobt_active=lambda: True, _body_frame_id=BODY_FRAME_ID_UNITY
    )
    assert Receiver._pose_frame_id(node) == BODY_FRAME_ID_UNITY
