"""Load recorded episodes for real-robot / sim playback.

Supported sources (absolute actions, same vector layout as the driver):

* LeRobot v2.1 dataset directory produced by ``convert_to_lerobot`` —
  ``meta/info.json`` (fps / features.action.shape) + per-episode parquet
  ``data/chunk-*/episode_*.parquet`` (``action`` column),
* an ``aligned_data.h5`` file (attrs carry fps / state_names / schema).

Both store **absolute** next-state joint targets (7 arm rad + gripper ratio),
which the arm driver / MuJoCo node consume directly — no further conversion.
"""

from __future__ import annotations

import dataclasses
import json
import os

import numpy as np

from morla_data_collect.schema import CollectSchema

INFO_JSON = "info.json"
CHUNK_SIZE = 1000  # must match morla_data_collect.pipeline.convert_to_lerobot.CHUNK_SIZE


@dataclasses.dataclass(frozen=True)
class ReplayEpisode:
    actions: np.ndarray          # (N, action_dim) float64 absolute
    fps: int
    action_dim: int
    source: str
    schema: CollectSchema | None = None

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
    side_names = [n.rsplit("_", 1)[0] for n in names]
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


def load_lerobot_actions(dataset_dir: str, episode_index: int) -> ReplayEpisode:
    dataset_dir = os.path.abspath(os.path.expanduser(dataset_dir))
    info = _json(os.path.join(dataset_dir, "meta", INFO_JSON))
    fps = int(info["fps"])
    chunk = int(episode_index) // CHUNK_SIZE
    rel = info.get("data_path", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet")
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
    state_feat = info.get("features", {}).get("observation.state", {})
    action_dim = int(state_feat.get("shape", [actions.shape[1]])[0])
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
            CollectSchema.from_json(schema_json) if schema_json else None
        )
    return ReplayEpisode(
        actions=actions,
        fps=fps,
        action_dim=int(actions.shape[1]),
        source=h5_path,
        schema=schema,
    )


def load_replay(source: str, episode_index: int = 0) -> ReplayEpisode:
    """Auto-detect: directory = LeRobot v2.1, file = aligned_data.h5."""
    src = os.path.abspath(os.path.expanduser(source))
    if os.path.isdir(src):
        return load_lerobot_actions(src, episode_index)
    if os.path.isfile(src):
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

