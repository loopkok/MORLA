"""Strict ROS-independent joint-name and numeric mapping helpers."""
from __future__ import annotations

import math
from collections.abc import Sequence


class JointMappingError(ValueError):
    """A command/state vector does not match its declared joint contract."""


def validate_joint_positions(
    names: Sequence[str],
    positions: Sequence[float],
    expected_names: Sequence[str],
    *,
    allow_positional_legacy: bool = False,
    limits: Sequence[tuple[float, float]] | None = None,
) -> tuple[float, ...]:
    """Return positions in ``expected_names`` order, rejecting ambiguous input.

    Positional legacy input is accepted only when explicitly enabled and its
    vector has the exact expected dimension. Named input must have the exact
    same name set; duplicates, unknown joints, missing joints, NaN/Inf and
    out-of-range values are errors.
    """
    expected = tuple(expected_names)
    if not expected or any(not isinstance(name, str) or not name for name in expected):
        raise JointMappingError("expected_names must be nonempty strings")
    if len(set(expected)) != len(expected):
        raise JointMappingError("expected_names contains duplicates")
    values = _finite_values(positions)
    supplied = tuple(names or ())
    if not supplied:
        if not allow_positional_legacy:
            raise JointMappingError("joint names are required")
        if len(values) != len(expected):
            raise JointMappingError(f"positional legacy dimension {len(values)} != {len(expected)}")
        ordered = values
    else:
        if len(supplied) != len(values):
            raise JointMappingError(f"name count {len(supplied)} != position count {len(values)}")
        if len(set(supplied)) != len(supplied):
            raise JointMappingError("joint names contain duplicates")
        unknown = sorted(set(supplied) - set(expected))
        missing = sorted(set(expected) - set(supplied))
        if unknown or missing:
            raise JointMappingError(f"joint name set mismatch; missing={missing}, unknown={unknown}")
        by_name = dict(zip(supplied, values))
        ordered = tuple(by_name[name] for name in expected)
    if limits is not None:
        bounds = tuple(limits)
        if len(bounds) != len(ordered):
            raise JointMappingError("limit count does not match joint count")
        for name, value, bound in zip(expected, ordered, bounds):
            if len(bound) != 2 or not all(math.isfinite(float(x)) for x in bound):
                raise JointMappingError(f"invalid limits for {name}")
            if value < bound[0] or value > bound[1]:
                raise JointMappingError(f"{name}={value} outside [{bound[0]}, {bound[1]}]")
    return ordered


def map_joint_values(
    values: Sequence[float],
    source_names: Sequence[str],
    target_names: Sequence[str],
    *,
    signs: dict[str, float] | None = None,
    offsets: dict[str, float] | None = None,
) -> tuple[float, ...]:
    """Reorder named values and apply one explicit sign/offset conversion."""
    source = tuple(source_names)
    target = tuple(target_names)
    ordered = validate_joint_positions(source, values, source)
    by_name = dict(zip(source, ordered))
    signs = signs or {}
    offsets = offsets or {}
    unknown = (set(signs) | set(offsets)) - set(target)
    if unknown:
        raise JointMappingError(f"mapping has unknown target names: {sorted(unknown)}")
    if set(source) != set(target):
        missing = sorted(set(target) - set(source))
        extra = sorted(set(source) - set(target))
        raise JointMappingError(f"source/target joint sets differ; missing={missing}, extra={extra}")
    output=[]
    for name in target:
        sign=float(signs.get(name, 1.0)); offset=float(offsets.get(name, 0.0))
        if not math.isfinite(sign) or not math.isfinite(offset):
            raise JointMappingError(f"non-finite mapping for {name}")
        output.append(sign*by_name[name]+offset)
    return tuple(output)


def _finite_values(values: Sequence[float]) -> tuple[float, ...]:
    try:
        result=tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise JointMappingError("positions must be a numeric sequence") from exc
    if not all(math.isfinite(value) for value in result):
        raise JointMappingError("positions contain NaN or infinity")
    return result
