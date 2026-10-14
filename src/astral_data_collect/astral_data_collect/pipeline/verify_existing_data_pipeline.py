#!/usr/bin/env python3
"""Read-only real-data QA plus isolated alignment/repair/export verification."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

import h5py
import numpy as np
from astral_data_collect.pipeline.align_data import align_episode
from astral_data_collect.pipeline.artifacts import reject_overlap, source_signature
from astral_data_collect.pipeline.convert_to_act import run_pipeline
from astral_data_collect.pipeline.convert_to_lerobot import convert_session
from astral_data_collect.pipeline.validate_data import validate_episode
from astral_data_collect.pipeline.repair_aligned import repair_episode


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session', required=True)
    ap.add_argument('--output', required=True, help='new verification artifact directory')
    ap.add_argument('--check-python')
    ap.add_argument('--sample-count', type=int, default=5)
    ap.add_argument('--camera-map', default='', help='explicit historical rename: old=new,old=new')
    args = ap.parse_args()
    session, output = Path(args.session).resolve(), Path(args.output).resolve()
    reject_overlap(str(session), str(output))
    output.mkdir(parents=True, exist_ok=False)
    episodes = sorted(p for p in session.glob('episode*') if p.is_dir())
    if not episodes or args.sample_count < 1:
        raise ValueError('need episodes and a positive sample count')
    camera_map = dict(pair.split('=', 1) for pair in args.camera_map.split(',') if pair)
    started = time.monotonic()
    signatures = {ep.name: source_signature(str(ep)) for ep in episodes}
    reports, repairs = [], []
    for ep in episodes:
        report = validate_episode(str(ep), require_aligned=True).to_dict()
        reports.append(report)
        provenance_failure = any(
            'raw_signature' in issue['msg'] or 'raw source changed' in issue['msg']
            for issue in report['issues'] if issue['level'] == 'fail'
        )
        if provenance_failure:
            repairs.append({'episode': ep.name, 'status': 'skipped_untrusted_alignment',
                            'frames': 0, 'kept': 0, 'pct': 0.0})
        else:
            repairs.append(repair_episode(str(ep), str(output / 'dry-run'), None, True,
                                          camera_map=camera_map))
    (output / 'source_validation.json').write_text(json.dumps(reports, ensure_ascii=False, indent=2))
    (output / 'repair_preview.json').write_text(json.dumps(repairs, ensure_ascii=False, indent=2))
    selected = sorted(set(np.linspace(0, len(episodes)-1, min(args.sample_count, len(episodes)), dtype=int)))
    rows = []
    for i in selected:
        source = episodes[i]
        copied = output / 'sample_raw' / source.name
        copied.mkdir(parents=True)
        # Only raw files are hard-linked. Pipeline reads them; derived files are independent.
        for name in ['robot_data.h5', 'camera_data.h5']:
            os.link(source / name, copied / name)
        shutil.copy2(source / 'meta.json', copied / 'meta.json')
        if camera_map:
            meta = json.loads((copied / 'meta.json').read_text())
            meta['schema']['cameras'] = [camera_map.get(c, c) for c in meta['schema']['cameras']]
            (copied / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2))
        align_episode(str(copied), force=True, log=lambda _: None)
        repaired = output / 'sample_repaired' / source.name
        row = repair_episode(str(copied), str(repaired), None, False)
        validation = validate_episode(str(repaired), require_aligned=True)
        if validation.status == 'fail':
            raise RuntimeError(str(validation.issues))
        with h5py.File(copied / 'aligned_data.h5') as a, h5py.File(repaired / 'aligned_data.h5') as b:
            idx = b['source_frame_index'][:]
            np.testing.assert_array_equal(b['observation/state'][:], a['observation/state'][:][idx])
            np.testing.assert_array_equal(b['action'][:-1], b['observation/state'][1:])
            for cam in json.loads(b.attrs['schema'])['cameras']:
                np.testing.assert_array_equal(b[f'{cam}/src_timestamps'][:], a[f'{cam}/src_timestamps'][:][idx])
                np.testing.assert_allclose(b[f'{cam}/src_offsets'][:], b[f'{cam}/src_timestamps'][:] - b['timestamps'][:])
                for dest_index, source_index in enumerate(idx):
                    assert hashlib.sha256(b[f'{cam}/images'][dest_index]).digest() == hashlib.sha256(a[f'{cam}/images'][source_index]).digest()
        row['validation'] = validation.status
        rows.append(row)
    v21 = output / 'lerobot_v21'
    summary = convert_session(str(output / 'sample_repaired'), str(v21), codec='h264', image_size=224, log=lambda _: None)
    v3 = run_pipeline(str(output / 'lerobot_v3'), v21_root=str(v21), check_python=args.check_python, log=lambda _: None)
    assert signatures == {ep.name: source_signature(str(ep)) for ep in episodes}
    result = {'source': str(session), 'source_episodes': len(episodes),
              'source_status_counts': {s: sum(r['status'] == s for r in reports) for s in ['pass', 'warn', 'fail']},
              'preview_source_frames': sum(r.get('frames', 0) for r in repairs),
              'preview_output_frames': sum(r.get('kept', 0) for r in repairs),
              'sample_episodes': rows, 'v21': summary, 'v3': v3,
              'source_unchanged': True, 'camera_map': camera_map, 'elapsed_seconds': round(time.monotonic()-started, 2)}
    (output / 'verification_report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
