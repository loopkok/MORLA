"""Copy selected raw episodes into an isolated working session, with explicit camera mapping."""
import argparse
import json
from pathlib import Path
import shutil

import h5py

from .artifacts import atomic_directory, reject_overlap, require_source_signature, source_signature
from morla_data_collect.schema import CollectSchema


def prepare_session(source, output, *, camera_map=None, episodes=None, include_aligned=False):
    source, output = Path(source).resolve(), Path(output).resolve()
    reject_overlap(str(source), str(output))
    if output.exists():
        raise ValueError('output must be a new directory')
    selected = sorted(p for p in source.glob('episode*') if p.is_dir())
    if episodes:
        requested = {f'episode{int(i):06d}' for i in episodes}
        available = {p.name for p in selected}
        if requested - available:
            raise ValueError(f'unknown episodes: {sorted(requested - available)}')
        selected = [p for p in selected if p.name in requested]
    if not selected:
        raise ValueError('no raw episodes found')
    mapping = camera_map or {}
    signatures = {p.name: source_signature(str(p)) for p in selected}
    with atomic_directory(str(output)) as staged:
        for index, ep in enumerate(selected):
            if not ep.resolve().is_relative_to(source):
                raise ValueError('episode symlink escapes source session')
            dest = Path(staged) / ep.name
            dest.mkdir()
            files = ['robot_data.h5', 'camera_data.h5', 'meta.json']
            if include_aligned and (ep / 'aligned_data.h5').exists():
                with h5py.File(ep / 'aligned_data.h5') as aligned:
                    try:
                        require_source_signature(aligned, str(ep))
                    except ValueError as exc:
                        raise ValueError(
                            f'{ep.name}: {exc}; run align --force on a working copy before including it'
                        ) from exc
                files.append('aligned_data.h5')
            for name in files:
                if not (ep / name).resolve().is_relative_to(source):
                    raise ValueError(f'{ep.name}/{name}: symlink escapes source session')
                shutil.copy2(ep / name, dest / name)
            meta = json.loads((dest / 'meta.json').read_text())
            schema = CollectSchema.from_dict(meta['schema'])
            if mapping:
                schema.cameras = [mapping.get(c, c) for c in schema.cameras]
                schema.__post_init__()
                for name in ['camera_data.h5', 'aligned_data.h5']:
                    if not (dest / name).exists():
                        continue
                    with h5py.File(dest / name, 'r+') as f:
                        for old, new in mapping.items():
                            if old == new or old not in f:
                                continue
                            if new in f:
                                raise ValueError(f'{ep.name}: camera name collision {old}={new}')
                            f.move(old, new)
                        if name == 'aligned_data.h5' and 'schema' in f.attrs:
                            old_schema = CollectSchema.from_json(f.attrs['schema'])
                            old_schema.cameras = [mapping.get(c, c) for c in old_schema.cameras]
                            old_schema.__post_init__()
                            f.attrs['schema'] = old_schema.to_json()
                meta['schema'] = schema.to_dict()
                if isinstance(meta.get('cameras'), dict):
                    meta['cameras'] = {mapping.get(c, c): value for c, value in meta['cameras'].items()}
                (dest / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2))
                if (dest / 'aligned_data.h5').exists():
                    with h5py.File(dest / 'aligned_data.h5', 'r+') as f:
                        f.attrs['raw_signature'] = source_signature(str(dest))
            if source_signature(str(ep)) != signatures[ep.name]:
                raise ValueError(f'{ep.name}: source changed while copying; stop recording before processing')
            print(f'copy {index+1}/{len(selected)} {ep.name}', flush=True)
        manifest = {'source': str(source), 'output': str(output), 'episodes': list(signatures),
                    'camera_map': mapping, 'source_signatures': signatures}
        (Path(staged) / 'preparation_report.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--camera-map', default='')
    ap.add_argument('--episodes', default='', help='comma-separated episode indices; empty=all')
    ap.add_argument('--include-aligned', action='store_true')
    args = ap.parse_args()
    mapping = dict(pair.split('=', 1) for pair in args.camera_map.split(',') if pair)
    if any(not a or not b for a, b in mapping.items()):
        ap.error('camera-map must be old=new pairs')
    prepare_session(args.source, args.output, camera_map=mapping,
                    episodes=[int(i) for i in args.episodes.split(',') if i.strip()],
                    include_aligned=args.include_aligned)


if __name__ == '__main__':
    main()
