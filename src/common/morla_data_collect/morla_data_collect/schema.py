"""Collector schema adapters.

The Astral v1 schema remains supported for existing sessions and launch files.
New MORLA recordings use the profile's immutable ``FrozenSchema``.  The small
view below gives the established offline pipeline a common block-oriented API
while preserving the complete frozen schema in metadata.
"""
from dataclasses import dataclass, replace
from typing import Any

from morla_core.schema import (
    ACTION_COMMAND, ACTION_NEXT_STATE, ACTION_SOURCES, ARM_DIM, BODY_HEAD_SLICE,
    BODY_STATE_DIM, BODY_WAIST_SLICE, EE_GRIPPER, EE_NONE, EE_TYPES, EE_WUJI,
    GRIPPER_DIM, HAND_DIM, HEAD_DIM, SIDES, STREAM_TOPICS, WAIST_DIM,
    ActionBlockSpec, CollectSchema, FrozenSchema, StateBlock, StateBlockSpec,
    StreamSpec, upgrade_v1_schema,
)

@dataclass(frozen=True)
class PipelineBlock:
    name: str
    dim: int
    state_stream: str
    cmd_stream: str | None
    body_slice: tuple[int, int] | None = None
    stream_indices: tuple[int, ...] = ()


class FrozenPipelineSchema:
    """Block-oriented compatibility view over an immutable profile schema."""

    def __init__(self, frozen: FrozenSchema):
        self.frozen = frozen
        self.profile_id = frozen.profile_id
        self.robot_id = frozen.robot_id
        self.cameras = list(frozen.cameras)
        self.dataset_fps = frozen.dataset_fps
        self.action_source = frozen.action_source
        self.hold_frames = frozen.hold_frames
        self.max_gap_ms = frozen.max_gap_ms
        self.sensor_specs = tuple(dict(item) for item in frozen.sensor_specs)
        self.stream_specs = {item.stream_key: item for item in frozen.streams}
        self._action_by_id = {item.block_id: item for item in frozen.action_blocks}

    @property
    def state_dim(self) -> int:
        return self.frozen.state_dim

    @property
    def action_dim(self) -> int:
        return self.frozen.action_dim

    @property
    def schema_hash(self) -> str:
        return self.frozen.schema_hash

    @property
    def layout_hash(self) -> str:
        return self.frozen.layout_hash

    def state_blocks(self) -> list[PipelineBlock]:
        return [
            PipelineBlock(
                name=block.block_id,
                dim=block.dim,
                state_stream=block.stream_key,
                cmd_stream=self._action_by_id.get(block.block_id).command_stream
                if self._action_by_id.get(block.block_id) is not None
                else None,
                stream_indices=block.stream_indices,
            )
            for block in self.frozen.state_blocks
        ]

    def state_names(self) -> list[str]:
        names = [""] * self.state_dim
        for block in self.frozen.state_blocks:
            for index, name in zip(block.state_indices, block.feature_names):
                names[index] = name
        return names

    def required_streams(self) -> dict[str, int]:
        """Numeric streams required to construct this layout's state/actions.

        Commands are required only for command-supervised layouts.  Profiles may
        advertise writable command endpoints while using ``next_state`` actions;
        old demonstrations without command logs can still be imported without
        fabricating a command stream.
        """
        return {
            key: int(spec.shape[-1])
            for key, spec in self.stream_specs.items()
            if (spec.source_kind == "state"
                or (self.action_source == ACTION_COMMAND and spec.source_kind == "command"))
            and spec.required
            and len(spec.shape) == 1 and spec.shape[-1] is not None
        }

    def numeric_streams(self) -> dict[str, int]:
        """All profile numeric streams to subscribe and archive, required or optional."""
        return {
            key: int(spec.shape[-1])
            for key, spec in self.stream_specs.items()
            if spec.source_kind in {"state", "command"}
            and len(spec.shape) == 1 and spec.shape[-1] is not None
        }

    def block_state_source(self, block: PipelineBlock):
        return block.state_stream, None

    def action_block_stream(self, block: PipelineBlock) -> str:
        action = self._action_by_id.get(block.name)
        return (action.command_stream if action is not None and action.command_stream
                else block.state_stream)

    def map_cameras(self, mapping: dict[str, str]) -> None:
        """Return camera relabeling as a new hashed frozen contract."""
        labels = tuple(mapping.get(label, label) for label in self.frozen.cameras)
        if len(set(labels)) != len(labels):
            raise ValueError("camera mapping must preserve unique labels")
        streams = []
        for stream in self.frozen.streams:
            if stream.source_kind != "camera":
                streams.append(stream)
                continue
            old = stream.stream_key.removeprefix("camera:")
            new = mapping.get(old, old)
            parent = stream.topic.rsplit("/", 1)[0]
            streams.append(replace(stream, stream_key=f"camera:{new}", topic=f"{parent}/{new}"))
        self.frozen = replace(self.frozen, streams=tuple(streams), cameras=labels)
        self.cameras = list(labels)
        self.stream_specs = {item.stream_key: item for item in self.frozen.streams}

    def to_dict(self) -> dict[str, Any]:
        return self.frozen.to_dict()

    def to_json(self) -> str:
        return self.frozen.to_json()


def pipeline_schema_from_dict(value: dict[str, Any]):
    """Load a current frozen schema or an explicitly supported Astral v1 schema."""
    if isinstance(value, dict) and value.get("schema_version") == 2 and "streams" in value:
        return FrozenPipelineSchema(FrozenSchema.from_dict(value))
    return CollectSchema.from_dict(value)


def pipeline_schema_from_json(value: str):
    import json
    return pipeline_schema_from_dict(json.loads(value))


__all__ = [name for name in globals() if not name.startswith("_")]
