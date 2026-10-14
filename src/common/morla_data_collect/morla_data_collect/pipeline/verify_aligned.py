"""Verify repaired frames against an explicitly supplied source session (read-only)."""
import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from .artifacts import require_source_signature
from morla_data_collect.schema import pipeline_schema_from_json


def verify_session(source: str, session: str) -> dict:
    episodes = sorted(p for p in Path(session).glob('episode*') if p.is_dir())
    if not episodes:
        raise ValueError('no episodes to verify')
    frames = 0
    for ep in episodes:
        source_ep = Path(source) / ep.name
        with h5py.File(source_ep / 'aligned_data.h5') as src, h5py.File(ep / 'aligned_data.h5') as dst:
            require_source_signature(src, str(source_ep))
            require_source_signature(dst, str(ep))
            if 'source_frame_index' not in dst:
                raise ValueError(f'{ep.name}: missing source_frame_index; rebuild with current repair')
            schema = pipeline_schema_from_json(dst.attrs['schema'])
            original_ids = src['source_frame_index'][:] if 'source_frame_index' in src else np.arange(len(src['timestamps']))
            lookup = {int(value): index for index, value in enumerate(original_ids)}
            indices = np.array([lookup[int(value)] for value in dst['source_frame_index'][:]])
            state, action = dst['observation/state'][:], dst['action'][:]
            np.testing.assert_array_equal(state, src['observation/state'][:][indices])
            for key in ('control_owner', 'intervention', 'intervention_valid'):
                if key in src:
                    if key not in dst:
                        raise ValueError(f'{ep.name}: missing {key}')
                    np.testing.assert_array_equal(dst[key][:], src[key][:][indices])
            ts = dst['timestamps'][:]
            if len(ts) < 2 or not np.all(np.isfinite(ts)):
                raise ValueError(f'{ep.name}: invalid timestamps')
            np.testing.assert_allclose(np.diff(ts), 1 / schema.dataset_fps, atol=1e-5, rtol=0)
            if schema.action_source == 'next_state':
                np.testing.assert_array_equal(action[:-1], state[1:])
                np.testing.assert_array_equal(action[-1], state[-1])
            else:
                col = 0
                original_action = src['action'][:][indices]
                for block in schema.state_blocks():
                    columns = slice(col, col + block.dim)
                    if block.cmd_stream is not None:
                        np.testing.assert_array_equal(action[:, columns], original_action[:, columns])
                    else:
                        np.testing.assert_array_equal(action[:-1, columns], state[1:, columns])
                        np.testing.assert_array_equal(action[-1, columns], state[-1, columns])
                    col += block.dim
            for camera in schema.cameras:
                np.testing.assert_array_equal(dst[f'{camera}/src_timestamps'][:], src[f'{camera}/src_timestamps'][:][indices])
                if len(dst[f'{camera}/images']) != len(indices):
                    raise ValueError(f'{ep.name}/{camera}: frame count mismatch')
                for i, original in enumerate(indices):
                    if bytes(dst[f'{camera}/images'][i]) != bytes(src[f'{camera}/images'][original]):
                        raise ValueError(f'{ep.name}/{camera}/{i}: image does not match source frame')
            source_sensors = set(src["sensors"].keys()) if "sensors" in src else set()
            target_sensors = set(dst["sensors"].keys()) if "sensors" in dst else set()
            if source_sensors != target_sensors:
                raise ValueError(f'{ep.name}: sensor sidecars differ: {source_sensors} != {target_sensors}')
            for key in sorted(source_sensors):
                source_group, target_group = src[f"sensors/{key}"], dst[f"sensors/{key}"]
                if set(source_group.keys()) != set(target_group.keys()):
                    raise ValueError(f'{ep.name}/sensors/{key}: fields differ')
                for field in source_group:
                    if not isinstance(source_group[field], h5py.Dataset):
                        continue
                    expected = source_group[field][:][indices]
                    actual = target_group[field][:]
                    if expected.shape != actual.shape:
                        raise ValueError(f'{ep.name}/sensors/{key}/{field}: shape differs')
                    np.testing.assert_array_equal(actual, expected)
            frames += len(state)
    return {'episodes': len(episodes), 'frames': frames, 'source': str(Path(source).resolve()),
            'session': str(Path(session).resolve()), 'status': 'pass'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, help='source session before repair')
    parser.add_argument('--session', required=True, help='repaired session')
    args = parser.parse_args()
    print(json.dumps(verify_session(args.source, args.session), ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
