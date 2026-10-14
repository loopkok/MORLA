import pytest

from xhand_adapter.core import (
    HAND_JOINT_NAMES,
    XHandContractError,
    native_state_positions,
    reorder_named_positions,
    validate_joint_limits,
)


def test_standard_joint_order_is_recovered_by_name():
    names = list(reversed(HAND_JOINT_NAMES))
    values = list(range(12))
    output = reorder_named_positions(names, values)
    assert output == tuple(values[names.index(name)] for name in HAND_JOINT_NAMES)


@pytest.mark.parametrize(
    "names,values",
    [
        (HAND_JOINT_NAMES[:-1], range(11)),
        (HAND_JOINT_NAMES, [0.0] * 11),
        (HAND_JOINT_NAMES[:-1] + (HAND_JOINT_NAMES[0],), range(12)),
        (HAND_JOINT_NAMES[:-1] + ("unknown",), range(12)),
        (HAND_JOINT_NAMES, [0.0] * 11 + [float("nan")]),
    ],
)
def test_invalid_standard_joint_contract_is_rejected(names, values):
    with pytest.raises(XHandContractError):
        reorder_named_positions(names, values)


def test_native_feedback_selects_by_hand_id_and_accepts_sdk_prefix():
    class State:
        def __init__(self, prefix, offset):
            self.name = [prefix + name for name in HAND_JOINT_NAMES]
            self.position = [offset + i for i in range(12)]

    class Message:
        hand_id = [5, 9]
        hand_states = [State("left_hand_", 0), State("right_hand_", 100)]

    assert native_state_positions(Message(), 9, "right") == tuple(float(100 + i) for i in range(12))


def test_native_feedback_rejects_missing_id_and_misaligned_arrays():
    class State:
        name = list(HAND_JOINT_NAMES)
        position = [0.0] * 12

    class Message:
        hand_id = [7]
        hand_states = [State()]

    with pytest.raises(XHandContractError, match="hand_id"):
        native_state_positions(Message(), 8, "left")
    Message.hand_states = []
    with pytest.raises(XHandContractError, match="not aligned"):
        native_state_positions(Message(), 7, "left")


def test_profile_joint_limits_reject_out_of_range_and_nonfinite_values():
    lower = [-0.1] * 12
    upper = [1.0] * 12
    assert validate_joint_limits([0.5] * 12, lower, upper) == (0.5,) * 12
    with pytest.raises(XHandContractError, match="exceeds"):
        validate_joint_limits([0.5] * 11 + [1.01], lower, upper)
    with pytest.raises(XHandContractError, match="finite"):
        validate_joint_limits([0.5] * 11 + [float("inf")], lower, upper)
