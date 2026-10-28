"""Schema-driven observation/action layout for policy inference & replay.

The single source of truth is ``morla_core.schema.CollectSchema``:
the same layout that teleop data collection froze into ``meta.json``. Feeding
that schema here yields:

* the ROS topics to observe (``obs_sources``) in state-vector order,
* ``assemble_state``: per-tick obs vector (``None`` when any live block stale),
* ``split_action``: an absolute action vector into per-stream command targets
  consumed by :mod:`executor`.

So switching robot config (right arm / gripper→wuji / waist/head / camera
labels) requires editing only the same yaml used at collection time — nothing
else in this package changes.
"""

from __future__ import annotations

import dataclasses

import cv2
import numpy as np

from morla_core.schema import (
    EE_GRIPPER,
    EE_WUJI,
    ARM_DIM,
    HAND_DIM,
    HEAD_DIM,
    WAIST_DIM,
    CollectSchema,
    FrozenSchema,
    STREAM_TOPICS,
    upgrade_v1_schema,
)
from morla_core.profile import ProfileRegistry
from morla_core.contracts.features import SchemaV3

# stream keys whose values we can obtain at inference time as *state feedback*.
# blocks that do NOT have a standalone command topic we can write (waist is
# only reachable via the 18-dim /astral/joint_commands full-body stream, which
# v1 does not write to avoid clobbering arm topics).
_WRITABLE_BLOCK_PREFIXES = ("arm",)  # handled via side-specific arm topics


class UnsupportedActionBlockError(RuntimeError):
    """A block has no v1 command channel (e.g. waist); refusing to guess."""


def reorder_joint_positions(
    names: list[str] | tuple[str, ...], positions, expected: tuple[str, ...], *, strict: bool,
) -> np.ndarray | None:
    """Validate/reorder one named JointState vector into profile joint order."""
    values = np.asarray(positions, dtype=np.float64).reshape(-1)
    names = tuple(str(name) for name in names)
    if strict:
        if len(names) != len(values) or len(expected) != len(values):
            return None
        if len(set(names)) != len(names) or set(names) != set(expected):
            return None
        by_name = dict(zip(names, values))
        values = np.asarray([by_name[name] for name in expected], dtype=np.float64)
    elif len(values) < len(expected):
        return None
    else:
        values = values[:len(expected)]
    if not np.isfinite(values).all():
        return None
    return values


def topic_for(stream_key: str) -> str:
    topic, _ = STREAM_TOPICS[stream_key]
    return topic


def dim_for(stream_key: str) -> int:
    _, dim = STREAM_TOPICS[stream_key]
    return dim


@dataclasses.dataclass(frozen=True)
class ObsSource:
    """One live ROS subscription feeding a slice of the obs vector."""

    key: str            # stream key (matches schema.stream names / HDF5 group)
    topic: str          # ROS topic to subscribe
    dim: int            # vector length published on that topic
    kind: str           # 'joints' | 'ratio' | 'body' | 'hand'
    joint_names: tuple[str, ...] = ()
    strict_names: bool = False

    def __post_init__(self):
        if not self.key or not self.topic.startswith("/") or self.dim < 1:
            raise ValueError("observation sources need a key, absolute topic, and positive dimension")


@dataclasses.dataclass(frozen=True)
class CmdTarget:
    """One command a robot-side writer must emit for a state/action block."""

    stream: str   # 'left_arm_cmd' etc.
    topic: str
    kind: str     # 'joints' -> JointState position; 'ratio' -> Float64 ratio
    values: np.ndarray  # joints: (7,) rad ; ratio: (1,) in [0,1]
    joint_names: tuple[str, ...] = ()
    limits: tuple[tuple[float, float], ...] = ()
    writer_id: str | None = None
    semantics: str = "absolute_position"

    @property
    def name(self) -> str:
        return self.stream

    def __str__(self) -> str:
        vals = np.round(self.values, 4)
        return f"{self.stream}@{self.topic} {vals.tolist()}"


class ObsLayout:
    """Runtime I/O rules derived from a profile's frozen schema.

    ``CollectSchema`` remains accepted for old launch files and is upgraded at
    this boundary. New robot deployments pass a validated ``FrozenSchema``.
    """

    def __init__(self, schema: CollectSchema | FrozenSchema, *, strict_names=False):
        self.schema = schema if isinstance(schema, (FrozenSchema, SchemaV3)) else upgrade_v1_schema(schema.to_dict())
        self.blocks = tuple(self.schema.state_blocks)
        self.action_blocks = tuple(self.schema.action_blocks)
        self.state_dim = self.schema.state_dim
        self.action_dim = self.schema.action_dim
        self.strict_names = bool(strict_names)
        self.streams = {stream.stream_key: stream for stream in self.schema.streams}
        self.sources: dict[str, ObsSource] = {}
        for block in self.blocks:
            stream = self.streams[block.stream_key]
            if stream.source_kind not in {"state", "command_echo"}:
                raise ValueError(f"state block {block.block_id} uses non-state stream {stream.stream_key}")
            if stream.message_type == "std_msgs/msg/Float64":
                kind = "ratio"
            elif stream.message_type == "sensor_msgs/msg/JointState":
                kind = "joints"
            else:
                raise ValueError(f"unsupported policy observation type {stream.message_type!r}")
            # A stream can back several blocks (e.g. a full-body feedback vector).
            self.sources.setdefault(stream.stream_key, ObsSource(
                key=stream.stream_key, topic=stream.topic, dim=int(stream.shape[-1]),
                kind=kind, joint_names=stream.joint_names,
                strict_names=self.strict_names and bool(stream.joint_names),
            ))
        if self.schema.metadata.get("upgraded_from") and any(b.block_id == "head" for b in self.blocks):
            fallback = self.streams.get("head_state")
            if fallback is not None:
                self.sources.setdefault("head_state", ObsSource(
                    key="head_state", topic=fallback.topic, dim=int(fallback.shape[-1]),
                    kind="joints", joint_names=fallback.joint_names, strict_names=False,
                ))
        self._cmd_streams = self.streams

    # -- obs assembly ------------------------------------------------------

    def assemble_state(
        self, vectors: dict[str, np.ndarray | None]
    ) -> tuple[np.ndarray | None, list[str]]:
        """Return ``(state_vector, missing_keys)``.

        Missing = stale/never-received source required by the schema. Waist/
        head prefer the body_state slice and fall back to head_state, mirroring
        align_data's availability logic. Returns (None, missing) if any
        required block cannot be filled (caller decides whether that is fatal).
        """
        parts: list[np.ndarray] = []
        missing: list[str] = []
        state = np.empty(self.state_dim, dtype=np.float32)
        for block in self.blocks:
            source = block.stream_key
            value = vectors.get(source)
            # Compatibility for legacy Astral sim, whose head feedback is a
            # separate stream while its frozen v1 schema records body_state.
            if value is None and block.block_id == "head" and self.schema.metadata.get("upgraded_from"):
                source = "head_state"
                value = vectors.get(source)
            if value is None:
                missing.append(source)
                continue
            raw = np.asarray(value, dtype=np.float32).reshape(-1)
            indices = block.stream_indices
            if source == "head_state" and source != block.stream_key:
                indices = tuple(range(block.dim))
            if len(raw) <= max(indices, default=-1):
                missing.append(source)
                continue
            selected = raw[np.asarray(indices, dtype=int)]
            state[np.asarray(block.state_indices, dtype=int)] = selected
        if missing:
            return None, missing
        if state.shape[0] != self.state_dim:
            raise RuntimeError(
                f"obs dim mismatch: assembled {state.shape[0]} != schema "
                f"{self.state_dim}"
            )
        return state, []

    # -- action split --------------------------------------------------------

    def split_action(self, action: np.ndarray) -> list[CmdTarget]:
        """Split one absolute action row into per-stream command targets.

        Every writable block resolves its own command stream, names, limits,
        semantics, and writer registration from the frozen profile.
        """
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape[0] != self.action_dim:
            raise ValueError(
                f"action dim {action.shape[0]} != schema action_dim "
                f"{self.action_dim}"
            )
        targets: list[CmdTarget] = []
        for block in self.action_blocks:
            if not block.writable or not block.command_stream:
                raise UnsupportedActionBlockError(f"action block {block.block_id} has no writable command stream")
            if block.semantics == "delta":
                raise UnsupportedActionBlockError(f"delta action semantics are not yet enabled for {block.block_id}")
            stream = self._cmd_streams.get(block.command_stream)
            if stream is None or stream.source_kind not in {"command", "command_echo"}:
                raise UnsupportedActionBlockError(f"command stream {block.command_stream!r} is not registered")
            if stream.message_type == "std_msgs/msg/Float64":
                kind = "ratio"
            elif stream.message_type == "sensor_msgs/msg/JointState":
                kind = "joints"
            else:
                raise UnsupportedActionBlockError(f"unsupported command type {stream.message_type!r}")
            seg = action[np.asarray(block.action_indices, dtype=int)].copy()
            target_stream = (
                f"{block.block_id}_cmd"
                if self.schema.metadata.get("upgraded_from") else stream.stream_key
            )
            targets.append(CmdTarget(
                stream=target_stream, topic=stream.topic, kind=kind, values=seg,
                joint_names=stream.joint_names, limits=block.limits,
                writer_id=block.writer_id, semantics=block.semantics,
            ))
        return targets


# ----------------------------------------------------------------- image helpers


def decode_jpeg_rgb(jpeg: bytes) -> np.ndarray | None:
    """CompressedImage JPEG -> HxWx3 uint8 RGB (None on decode failure)."""
    buf = np.frombuffer(jpeg, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        return None
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def letterbox(img: np.ndarray, size: int) -> np.ndarray:
    """Aspect-preserving resize + zero pad to ``size``x``size``.

    Mirrors the geometry used at conversion time (convert_to_lerobot._letterbox)
    and openpi's resize_with_pad: ratio=max(w,h)/size, symmetric borders with
    remainder pushed to bottom/right.
    """
    h, w = img.shape[:2]
    if (h, w) == (size, size):
        return img
    ratio = max(w / size, h / size)
    new_w, new_h = int(w / ratio), int(h / ratio)
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    pad_h0 = (size - new_h) // 2
    pad_h1 = size - new_h - pad_h0
    pad_w0 = (size - new_w) // 2
    pad_w1 = size - new_w - pad_w0
    return cv2.copyMakeBorder(
        resized, pad_h0, pad_h1, pad_w0, pad_w1, cv2.BORDER_CONSTANT, value=0
    )


def split_camera_map(
    camera_map: dict[str, str], image_size: int
) -> list[tuple[str, str, int]]:
    """(slot, collect_label, image_size) rows for each mapped camera."""
    out = []
    for slot, label in camera_map.items():
        out.append((slot, label, int(image_size)))
    return out
