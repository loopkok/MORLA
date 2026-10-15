"""Load recorded episodes for real-robot / sim playback.

Supported sources (absolute actions, same vector layout as the driver):

* LeRobot v2.1 dataset directory produced by ``convert_to_lerobot`` —
  ``meta/info.json`` (fps / features.action.shape) + per-episode parquet
  ``data/chunk-*/episode_*.parquet`` (``action`` column),
* an ``aligned_data.h5`` file (attrs carry fps / state_names / schema),
* a closed raw episode (``meta.json`` + ``robot_data.h5``), a session
  directory plus episode index, or the episode's ``robot_data.h5``.

Raw playback resamples measured state streams onto the frozen profile timeline;
it does not read cameras, write aligned files, clean pauses, or add training
hold rows. All playback sources use the same profile and safety execution path.
"""

from __future__ import annotations

import dataclasses
import json
import os

import numpy as np

from morla_core.schema import CollectSchema, FrozenSchema

INFO_JSON = "info.json"
CHUNK_SIZE = 1000  # must match morla_data_collect.pipeline.convert_to_lerobot.CHUNK_SIZE


@dataclasses.dataclass(frozen=True)
class ReplayEpisode:
    actions: np.ndarray          # (N, action_dim) float64 absolute
    fps: int
    action_dim: int
    source: str
    schema: CollectSchema | FrozenSchema | None = None
    notes: tuple[str, ...] = ()

    @property
    def num_frames(self) -> int:
        return int(self.actions.shape[0])

    def duration_s(self) -> float:
        return (self.num_frames - 1) / self.fps


def _read_action_parquet(path: str) -> np.ndarray:
    import pyarrow.parquet as pq

    table = pq.read_table(path, columns=["action"])
    cols = table.column("action").to_pylist()
    return np.asarray(cols, dtype=np.float64)


def _json(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _parse_schema_from_info(info: dict) -> CollectSchema | None:
    features = info.get("features", {})
    state = features.get("observation.state", {})
    names = state.get("names")
    if not names:
        return None
    # Rebuild a schema that reproduces the same state vector: block layout is
    # inferred from feature names (e.g. left_arm_0..6, left_ee_0, waist_0..1).
    arms: list[str] = []
    ee_left = ee_right = "none"
    include_waist = include_head = False
    for side in ("left", "right"):
        pre = f"{side}_arm_"
        if any(n.startswith(pre) for n in names):
            arms.append(side)
    for side in arms:
        if any(n.startswith(f"{side}_ee_") for n in names):
            # only gripper end-effectors exist in v2.1 exports today
            ee = "gripper"
        else:
            ee = "none"
        if side == "left":
            ee_left = ee
        else:
            ee_right = ee
    include_waist = any(n.startswith("waist_") for n in names)
    include_head = any(n.startswith("head_") for n in names)
    cameras = [
        k.split("observation.images.")[1]
        for k in features
        if k.startswith("observation.images.")
    ]
    return CollectSchema(
        arms=arms,
        end_effector_left=ee_left,
        end_effector_right=ee_right,
        include_waist=include_waist,
        include_head=include_head,
        cameras=cameras,
        dataset_fps=int(info.get("fps", 30)),
    )


def _load_frozen_schema(path: str) -> FrozenSchema | None:
    if not os.path.isfile(path):
        return None
    with open(path, encoding="utf-8") as stream:
        value = json.load(stream)
    # LeRobot export keeps the frozen contract beside dataset metadata:
    # {schema: <FrozenSchema>, profile_id, schema_hash, layout_hash, ...}.
    # Older aligned/replay files store FrozenSchema directly.
    envelope = value if isinstance(value, dict) else {}
    if "schema" in envelope:
        value = envelope["schema"]
        if not isinstance(value, dict):
            raise ValueError("LeRobot morla_schema.json has no schema object")
    if "schema_version" not in value:
        return None
    schema = FrozenSchema.from_dict(value)
    for key in ("schema_hash", "layout_hash", "kinematics_hash"):
        declared = envelope.get(key)
        actual = getattr(schema, key)
        if declared and declared != actual:
            raise ValueError(
                f"LeRobot morla_schema.json {key} mismatch: {declared} != {actual}"
            )
    return schema


def load_lerobot_actions(dataset_dir: str, episode_index: int) -> ReplayEpisode:
    dataset_dir = os.path.abspath(os.path.expanduser(dataset_dir))
    info = _json(os.path.join(dataset_dir, "meta", INFO_JSON))
    fps = int(info["fps"])
    chunk = int(episode_index) // CHUNK_SIZE
    rel = info.get(
        "data_path", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
    )
    path = os.path.join(
        dataset_dir,
        rel.format(episode_chunk=chunk, episode_index=int(episode_index)),
    )
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"episode {episode_index} not found in dataset {dataset_dir} "
            f"(looked for {path})"
        )
    actions = _read_action_parquet(path)
    action_dim = int(actions.shape[1])
    schema = _load_frozen_schema(os.path.join(dataset_dir, "meta", "morla_schema.json"))
    if schema is None:
        schema = _parse_schema_from_info(info)
    return ReplayEpisode(
        actions=actions,
        fps=fps,
        action_dim=action_dim,
        source=dataset_dir,
        schema=schema,
    )


def load_aligned_actions(h5_path: str) -> ReplayEpisode:
    import h5py

    h5_path = os.path.abspath(os.path.expanduser(h5_path))
    with h5py.File(h5_path, "r") as fa:
        if "action" not in fa:
            raise ValueError(f"{h5_path}: no /action dataset (not an aligned file?)")
        actions = np.asarray(fa["action"], dtype=np.float64)
        fps = int(fa.attrs.get("fps", 30))
        schema_json = fa.attrs.get("schema")
        schema = (
            (FrozenSchema.from_dict(json.loads(schema_json))
             if schema_json and 'schema_version' in json.loads(schema_json)
             else CollectSchema.from_json(schema_json)) if schema_json else None
        )
    return ReplayEpisode(
        actions=actions,
        fps=fps,
        action_dim=int(actions.shape[1]),
        source=h5_path,
        schema=schema,
    )


def load_raw_actions(episode_dir: str) -> ReplayEpisode:
    """Read a saved raw episode and map measured state into profile actions."""
    import h5py

    from morla_data_collect.schema import pipeline_schema_from_dict

    episode_dir = os.path.abspath(os.path.expanduser(episode_dir))
    meta_path = os.path.join(episode_dir, "meta.json")
    if not os.path.isfile(meta_path):
        raise ValueError("raw playback needs saved meta.json; stop/save recording first")
    meta = _json(meta_path)
    schema = pipeline_schema_from_dict(meta["schema"])
    frozen = getattr(schema, "frozen", None)

    # Frozen MORLA contracts can have different state and action vector layouts.
    # Pair blocks by stable id and write each measured state block into its
    # action indices. Legacy Astral v1 schemas have a shared sequential layout.
    mappings = []
    if frozen is not None:
        state_by_id = {block.block_id: block for block in frozen.state_blocks}
        for action in frozen.action_blocks:
            state = state_by_id.get(action.block_id)
            if state is None or state.dim != action.dim:
                raise ValueError(
                    f"raw playback cannot map action block {action.block_id!r} "
                    "to a measured state block"
                )
            if action.semantics == "delta":
                raise ValueError(
                    f"raw playback does not support delta action block {action.block_id!r}"
                )
            mappings.append((
                action.block_id, action.action_indices, state.stream_key,
                state.stream_indices,
            ))
        action_dim = frozen.action_dim
        result_schema = frozen
    else:
        cursor = 0
        for block in schema.state_blocks():
            name, body_slice = schema.block_state_source(block)
            stream_indices = (tuple(range(*body_slice)) if body_slice is not None
                              else tuple(range(block.dim)))
            indices = tuple(range(cursor, cursor + block.dim))
            mappings.append((block.name, indices, name, stream_indices))
            cursor += block.dim
        action_dim = schema.state_dim
        result_schema = schema

    start = float(meta["start_time_wall"])
    end = float(meta["end_time_wall"])
    if not np.isfinite([start, end]).all() or end <= start:
        raise ValueError("raw recording has invalid episode boundaries")
    episode_start, episode_end = start, end

    source_specs = getattr(schema, "stream_specs", {})
    streams = []
    with h5py.File(os.path.join(episode_dir, "robot_data.h5"), "r") as f:
        stream_group = f.get("streams")
        if stream_group is None:
            raise ValueError("raw recording has no /streams group")
        for block_id, action_indices, stream_name, stream_indices in mappings:
            # Legacy Astral head blocks can fall back from /astral/joint_states
            # to the independently recorded /head/joint_states stream.
            if block_id == "head" and (
                stream_name not in stream_group
                or len(stream_group[stream_name]["timestamps"]) == 0
            ):
                if "head_state" in stream_group:
                    stream_name, stream_indices = "head_state", tuple(range(len(stream_indices)))
            if stream_name not in stream_group:
                raise ValueError(f"raw playback missing measured stream: {stream_name}")
            group = stream_group[stream_name]
            if "timestamps" not in group or "values" not in group:
                raise ValueError(f"{stream_name}: missing timestamps or values")
            ts = np.asarray(group["timestamps"], dtype=np.float64)
            values = np.asarray(group["values"], dtype=np.float64)
            if ts.ndim != 1 or len(ts) < 2 or not np.isfinite(ts).all():
                raise ValueError(f"{stream_name}: need at least two finite timestamps")
            if np.any(np.diff(ts) <= 0):
                raise ValueError(f"{stream_name}: timestamps must be strictly increasing")
            expected = meta.get("streams", {}).get(stream_name, {}).get("count")
            if expected is not None and int(expected) != len(ts):
                raise ValueError(f"{stream_name}: saved metadata/sample count mismatch")
            if values.ndim != 2 or len(values) != len(ts):
                raise ValueError(f"{stream_name}: value/timestamp shape mismatch")
            indices = np.asarray(stream_indices, dtype=int)
            if len(indices) != len(action_indices) or np.any(indices >= values.shape[1]):
                raise ValueError(f"{stream_name}: invalid dimensions for block {block_id!r}")
            selected = values[:, indices]
            if not np.isfinite(selected).all():
                raise ValueError(f"{stream_name}: nonfinite joint values")
            spec = source_specs.get(stream_name)
            is_ratio = (stream_name.endswith("_gripper_ratio")
                        or getattr(spec, "unit", None) == "ratio")
            streams.append((block_id, action_indices, stream_name, ts, selected, is_ratio))

    start = max(start, max(item[3][0] for item in streams))
    end = min(end, min(item[3][-1] for item in streams))
    fps = int(schema.dataset_fps)
    if end - start < 1.0 / fps:
        raise ValueError("raw streams have insufficient common recording time")

    paused_at = None
    for event in sorted(meta.get("events", []), key=lambda item: float(item.get("t", 0))):
        if event.get("name") != "pause":
            continue
        t = float(event["t"])
        if event.get("data"):
            paused_at = t
        elif paused_at is not None:
            if min(t, end) > max(paused_at, start):
                raise ValueError("raw episode contains a pause; split the recording first")
            paused_at = None
    if paused_at is not None and end > max(paused_at, start):
        raise ValueError("raw episode contains a pause; split the recording first")

    grid = start + np.arange(int(np.floor((end - start) * fps)) + 1) / fps
    actions = np.empty((len(grid), action_dim), dtype=np.float64)
    for block_id, action_indices, stream_name, ts, values, is_ratio in streams:
        overlaps = (ts[:-1] < end) & (ts[1:] > start)
        if np.any(np.diff(ts)[overlaps] > schema.max_gap_ms / 1000.0):
            raise ValueError(
                f"{stream_name}: sampling gap exceeds max_gap_ms={schema.max_gap_ms}"
            )
        if is_ratio:
            sample_indices = np.searchsorted(ts, grid, side="right") - 1
            sampled = values[sample_indices]
        else:
            sampled = np.column_stack([
                np.interp(grid - start, ts - start, values[:, col])
                for col in range(values.shape[1])
            ])
        actions[:, np.asarray(action_indices, dtype=int)] = sampled

    return ReplayEpisode(
        actions=actions, fps=fps, action_dim=action_dim, source=episode_dir,
        schema=result_schema,
        notes=(
            "raw measured trajectory; using common stream interval, "
            f"excluding {start - episode_start:.3f}s at start and "
            f"{episode_end - grid[-1]:.3f}s at end; no camera alignment/cleaning",
        ),
    )


def load_replay(source: str, episode_index: int = 0) -> ReplayEpisode:
    """Detect raw episode/session/HDF5, aligned HDF5, or LeRobot v2.1."""
    src = os.path.abspath(os.path.expanduser(source))
    if os.path.isdir(src):
        if os.path.isfile(os.path.join(src, "robot_data.h5")):
            return load_raw_actions(src)
        if not os.path.isfile(os.path.join(src, "meta", INFO_JSON)):
            episode_dir = os.path.join(src, f"episode{int(episode_index):06d}")
            if os.path.isdir(episode_dir):
                return load_raw_actions(episode_dir)
        return load_lerobot_actions(src, episode_index)
    if os.path.isfile(src):
        if os.path.basename(src) == "robot_data.h5":
            return load_raw_actions(os.path.dirname(src))
        return load_aligned_actions(src)
    raise FileNotFoundError(f"replay source not found: {source}")


class PlaybackSession:
    """Steps through a ReplayEpisode, one absolute action per frame.

    Human takeover / pause semantics: the session keeps the current index but
    lets the caller re-anchor an additive offset to the robot's *actual* pose,
    so the remainder of the episode continues relative to wherever the robot
    ended up after human intervention (no jump on resume).
    """

    def __init__(self, episode: ReplayEpisode):
        self.episode = episode
        self.idx = 0
        self._offset = np.zeros(episode.action_dim, dtype=np.float64)
        self.done = False

    @property
    def remaining(self) -> int:
        return max(0, self.episode.num_frames - self.idx)

    def target(self) -> np.ndarray | None:
        """Current offset-adjusted absolute action, or None at end."""
        if self.done or self.idx >= self.episode.num_frames:
            return None
        return self.episode.actions[self.idx] + self._offset

    def advance(self) -> None:
        if self.idx < self.episode.num_frames:
            self.idx += 1
        if self.idx >= self.episode.num_frames:
            self.done = True

    def reanchor(self, robot_state: np.ndarray) -> None:
        """Re-anchor offset so the next target continues from ``robot_state``."""
        state = np.asarray(robot_state, dtype=np.float64).reshape(-1)
        if state.shape[0] != self.episode.action_dim:
            raise ValueError(
                f"reanchor state dim {state.shape[0]} != replay action_dim "
                f"{self.episode.action_dim}"
            )
        base = self.episode.actions[min(self.idx, self.episode.num_frames - 1)]
        self._offset = state - base

    def reset_offset(self) -> None:
        self._offset = np.zeros(self.episode.action_dim, dtype=np.float64)
