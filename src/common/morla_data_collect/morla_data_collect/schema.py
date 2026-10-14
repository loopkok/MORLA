"""Compatibility imports for schemas now defined by :mod:`morla_core.schema`."""
from morla_core.schema import (
    ACTION_COMMAND, ACTION_NEXT_STATE, ACTION_SOURCES, ARM_DIM, BODY_HEAD_SLICE,
    BODY_STATE_DIM, BODY_WAIST_SLICE, EE_GRIPPER, EE_NONE, EE_TYPES, EE_WUJI,
    GRIPPER_DIM, HAND_DIM, HEAD_DIM, SIDES, STREAM_TOPICS, WAIST_DIM,
    ActionBlockSpec, CollectSchema, FrozenSchema, StateBlock, StateBlockSpec,
    StreamSpec, upgrade_v1_schema,
)

__all__ = [name for name in globals() if not name.startswith("_")]
