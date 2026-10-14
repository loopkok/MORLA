"""Regression tests for the October data-pipeline audit."""
import json
import sys
from pathlib import Path

import h5py
import numpy as np
import pytest

from morla_data_collect.pipeline.align_data import align_episode
from morla_data_collect.schema import CollectSchema
from morla_data_collect.pipeline.validate_data import validate_episode
from conftest import write_raw_episode

from morla_data_collect.pipeline.repair_aligned import repair_episode


def test_aligned_action_nan_rejected(tmp_path):
    ep = write_raw_episode(str(tmp_path / 'episode000000'))
    align_episode(ep)
    with h5py.File(Path(ep) / 'aligned_data.h5', 'r+') as f:
        f['action'][10, 0] = np.nan
    assert validate_episode(ep).status == 'fail'


def test_raw_timestamp_nan_rejected(tmp_path):
    ep = write_raw_episode(str(tmp_path / 'episode000000'))
    with h5py.File(Path(ep) / 'robot_data.h5', 'r+') as f:
        f['streams/left_arm_state/timestamps'][10] = np.nan
    assert validate_episode(ep).status == 'fail'
    with pytest.raises(ValueError, match='timestamp'):
        align_episode(ep)


def test_training_requires_aligned(tmp_path):
    ep = write_raw_episode(str(tmp_path / 'episode000000'))
    assert validate_episode(ep, require_aligned=True).status == 'fail'


def test_unsigned_aligned_is_not_training_ready(tmp_path):
    ep = write_raw_episode(str(tmp_path / 'episode000000'))
    align_episode(ep)
    with h5py.File(Path(ep) / 'aligned_data.h5', 'r+') as f:
        del f.attrs['raw_signature']
    report = validate_episode(ep, require_aligned=True)
    assert report.status == 'fail'
    assert any('raw_signature' in issue['msg'] for issue in report.issues)
    from morla_data_collect.pipeline.validate_data import ensure_training_ready
    with pytest.raises(RuntimeError, match='raw_signature'):
        ensure_training_ready(str(Path(ep).parent))


def test_next_state_terminal_action_hold_is_validated(tmp_path):
    ep = write_raw_episode(str(tmp_path / 'episode000000'))
    align_episode(ep)
    with h5py.File(Path(ep) / 'aligned_data.h5', 'r+') as f:
        f['action'][-1, 0] += 0.5
    report = validate_episode(ep, require_aligned=True)
    assert report.status == 'fail'
    assert any('terminal hold' in issue['msg'] for issue in report.issues)
    from morla_data_collect.pipeline.validate_data import ensure_training_ready
    with pytest.raises(RuntimeError, match='terminal hold'):
        ensure_training_ready(str(Path(ep).parent))


def test_prepare_session_rejects_unsigned_aligned_cache(tmp_path):
    from morla_data_collect.pipeline.prepare_session import prepare_session
    source = tmp_path / 'raw'
    ep = write_raw_episode(str(source / 'episode000000'))
    align_episode(ep)
    with h5py.File(Path(ep) / 'aligned_data.h5', 'r+') as f:
        del f.attrs['raw_signature']
    with pytest.raises(ValueError, match='no raw_signature'):
        prepare_session(str(source), str(tmp_path / 'prepared'), include_aligned=True)
    assert not (tmp_path / 'prepared').exists()


def test_repair_and_preview_reject_unsigned_aligned_cache(tmp_path):
    from morla_data_collect.pipeline.preview_session import preview_session
    ep = write_raw_episode(str(tmp_path / 'raw' / 'episode000000'))
    align_episode(ep)
    with h5py.File(Path(ep) / 'aligned_data.h5', 'r+') as f:
        del f.attrs['raw_signature']
    with pytest.raises(ValueError, match='no raw_signature'):
        repair_episode(ep, str(tmp_path / 'repaired' / 'episode000000'), None, True)
    with pytest.raises(ValueError, match='no raw_signature'):
        preview_session(str(Path(ep).parent), str(tmp_path / 'preview'))
    assert not (tmp_path / 'preview').exists()


def test_aligned_quantification_requires_current_raw_signature(tmp_path, monkeypatch):
    from morla_data_collect.pipeline.quantify_cmd_state import main
    ep = write_raw_episode(str(tmp_path / 'episode000000'))
    align_episode(ep)
    aligned = str(Path(ep) / 'aligned_data.h5')
    monkeypatch.setattr(sys, 'argv', ['quantify_cmd_state', '--aligned', aligned])
    assert main() == 0
    with h5py.File(aligned, 'r+') as f:
        del f.attrs['raw_signature']
    with pytest.raises(ValueError, match='no raw_signature'):
        main()


def test_head_fallback_is_valid(tmp_path):
    schema = CollectSchema(arms=['left'], include_head=True, cameras=['base'])
    ep = write_raw_episode(str(tmp_path / 'episode000000'), schema=schema)
    with h5py.File(Path(ep) / 'robot_data.h5', 'r+') as f:
        del f['streams/body_state']
    align_episode(ep)
    assert validate_episode(ep).status == 'pass'


def test_repair_preserves_command_and_provenance(tmp_path):
    schema = CollectSchema(arms=['left'], cameras=['base'], action_source='command')
    ep = write_raw_episode(str(tmp_path / 'src' / 'episode000000'), schema=schema)
    align_episode(ep)
    out = tmp_path / 'out' / 'episode000000'
    repair_episode(ep, str(out), 30, False, keep_intent=True)
    with h5py.File(Path(ep) / 'aligned_data.h5') as src, h5py.File(out / 'aligned_data.h5') as dst:
        idx = dst['source_frame_index'][:]
        np.testing.assert_array_equal(dst['action'][:], src['action'][:][idx])
        np.testing.assert_array_equal(dst['base/src_timestamps'][:], src['base/src_timestamps'][:][idx])
        np.testing.assert_allclose(dst['base/src_offsets'][:],
                                   dst['base/src_timestamps'][:] - dst['timestamps'][:])
        assert dst.attrs['fps'] == 30
        assert json.loads(dst.attrs['schema'])['action_source'] == 'command'
    assert validate_episode(str(out)).status == 'pass'


def test_pause_does_not_create_training_frames(tmp_path):
    ep = Path(write_raw_episode(str(tmp_path / 'episode000000')))
    for filename in ['robot_data.h5', 'camera_data.h5']:
        with h5py.File(ep / filename, 'r+') as f:
            groups = f['streams'].values() if filename.startswith('robot') else f.values()
            for group in groups:
                ts = group['timestamps'][:]
                ts[ts >= ts[0] + 1.5] += 5
                group['timestamps'][:] = ts
    meta = json.loads((ep / 'meta.json').read_text())
    t0 = meta['start_time_wall']
    meta['events'] += [{'t': t0 + 1.5, 'name': 'pause', 'data': True},
                       {'t': t0 + 6.5, 'name': 'pause', 'data': False}]
    (ep / 'meta.json').write_text(json.dumps(meta))
    with pytest.raises(ValueError, match='pause'):
        align_episode(str(ep))
    assert not (ep / 'aligned_data.h5').exists()


def test_export_failure_preserves_previous_dataset(tmp_path):
    from morla_data_collect.pipeline.convert_to_lerobot import convert_session
    ep = write_raw_episode(str(tmp_path / 'raw' / 'episode000000'))
    align_episode(ep)
    output = tmp_path / 'dataset'
    convert_session(str(tmp_path / 'raw'), str(output), codec='h264', image_size=64)
    previous = (output / 'meta' / 'info.json').read_bytes()
    with h5py.File(Path(ep) / 'aligned_data.h5', 'r+') as f:
        f['wrist_left/images'][2] = np.array([1, 2, 3], dtype=np.uint8)
    with pytest.raises(ValueError, match='JPEG undecodable'):
        convert_session(str(tmp_path / 'raw'), str(output), codec='h264', image_size=64)
    assert (output / 'meta' / 'info.json').read_bytes() == previous
    assert not list(tmp_path.glob('.dataset-*'))


def test_export_rejects_mixed_fps_and_missing_episode(tmp_path):
    from morla_data_collect.pipeline.convert_to_lerobot import convert_session
    root = tmp_path / 'raw'
    for i, fps in enumerate([30, 15]):
        ep = write_raw_episode(str(root / f'episode{i:06d}'),
                               schema=CollectSchema(cameras=['base'], dataset_fps=fps), fps=fps)
        align_episode(ep)
    with pytest.raises(RuntimeError, match='schema'):
        convert_session(str(root), str(tmp_path / 'out'))
    (root / 'episode000001' / 'aligned_data.h5').unlink()
    with pytest.raises(RuntimeError, match='missing aligned'):
        convert_session(str(root), str(tmp_path / 'out'))


def test_command_waist_head_fallback_uses_next_state(tmp_path):
    schema = CollectSchema(arms=['left'], include_head=True, include_waist=True,
                           action_source='command', cameras=['base'])
    ep = write_raw_episode(str(tmp_path / 'episode000000'), schema=schema)
    align_episode(ep)
    with h5py.File(Path(ep) / 'aligned_data.h5') as f:
        np.testing.assert_array_equal(f['action'][:-1, -4:], f['observation/state'][1:, -4:])


def test_repair_defaults_protect_intent_and_uses_schema_fps(tmp_path):
    schema = CollectSchema(arms=['left'], cameras=['base'], dataset_fps=15)
    ep = write_raw_episode(str(tmp_path / 'src' / 'episode000000'), schema=schema, fps=15)
    align_episode(ep)
    with pytest.raises(ValueError, match='fps'):
        repair_episode(ep, str(tmp_path / 'wrong' / 'episode000000'), 30, False)
    repair_episode(ep, str(tmp_path / 'out' / 'episode000000'), None, False)
    with h5py.File(tmp_path / 'out' / 'episode000000' / 'aligned_data.h5') as f:
        assert f.attrs['fps'] == 15
    assert validate_episode(str(tmp_path / 'out' / 'episode000000')).status == 'pass'
