import numpy as np
import pytest

from xhand_retargeting.xhand_dex_retargeting_node import (
    HAND_JOINT_NAMES,
    urdf_values_to_hardware,
)


class RetargeterNames:
    joint_names = [
        "left_hand_" + name for name in reversed(HAND_JOINT_NAMES)
    ]


def test_urdf_mapping_reorders_and_applies_right_index_sign_once():
    names = ["right_hand_" + name for name in reversed(HAND_JOINT_NAMES)]
    values = np.arange(12, dtype=float)
    retargeter = RetargeterNames()
    retargeter.joint_names = names
    output = urdf_values_to_hardware(values, retargeter, "right", -1.0)
    expected = np.asarray([values[names.index("right_hand_" + name)] for name in HAND_JOINT_NAMES])
    expected[3] *= -1.0
    assert np.array_equal(output, expected)
    assert np.array_equal(output[[0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11]], expected[[0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11]])


def test_urdf_mapping_rejects_missing_joint_or_bad_output():
    class Bad:
        joint_names = ["left_hand_" + name for name in HAND_JOINT_NAMES[:-1]]

    with pytest.raises(ValueError, match="missing configured"):
        urdf_values_to_hardware(np.zeros(11), Bad(), "left")

    with pytest.raises(ValueError, match="non-finite"):
        urdf_values_to_hardware(np.full(12, np.nan), RetargeterNames(), "left")
