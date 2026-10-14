"""Generate a browser-readable contact sheet and state traces from aligned data."""
import argparse
import html
import json
from pathlib import Path

import cv2
import h5py
import numpy as np

from .artifacts import atomic_directory, reject_overlap, require_source_signature


def preview_session(session, output):
    session, output = Path(session).resolve(), Path(output).resolve()
    reject_overlap(str(session), str(output))
    rows = []
    with atomic_directory(str(output)) as staged:
        staged = Path(staged)
        for ep in sorted(p for p in session.glob('episode*') if p.is_dir()):
            with h5py.File(ep / 'aligned_data.h5') as f:
                require_source_signature(f, str(ep))
                state, timestamps = f['observation/state'][:], f['timestamps'][:]
                if len(state) < 2:
                    raise ValueError(f'{ep.name}: too few frames')
                if state.ndim != 2 or not np.all(np.isfinite(state)) or not np.all(np.isfinite(timestamps)):
                    raise ValueError(f'{ep.name}: invalid state/timestamps; validate before preview')
                samples = sorted(set([0, len(state)//2, len(state)-1]))
                cams = [key for key in f if isinstance(f[key], h5py.Group) and 'images' in f[key]]
                cards = []
                for camera in cams:
                    images = []
                    for i in samples:
                        image = cv2.imdecode(f[f'{camera}/images'][i], cv2.IMREAD_COLOR)
                        if image is None:
                            raise ValueError(f'{ep.name}/{camera}/{i}: corrupt JPEG')
                        scale = min(1.0, 360 / max(image.shape[:2]))
                        image = cv2.resize(image, (max(1, round(image.shape[1]*scale)), max(1, round(image.shape[0]*scale))))
                        filename = f'{ep.name}_{camera}_{i}.jpg'
                        if not cv2.imwrite(str(staged / filename), image):
                            raise ValueError(f'cannot save {filename}')
                        images.append(f'<figure><img src="{html.escape(filename, quote=True)}"><figcaption>帧 {i} · {timestamps[i]-timestamps[0]:.2f}s</figcaption></figure>')
                    cards.append(f'<h3>{html.escape(camera)}</h3><div class="strip">'+''.join(images)+'</div>')
                names = json.loads(f.attrs.get('state_names', '[]'))
                traces = []
                for col in range(min(state.shape[1], 8)):
                    indices = np.linspace(0, len(state)-1, min(600, len(state)), dtype=int)
                    values = state[indices, col]
                    low, high = float(values.min()), float(values.max())
                    points = ' '.join(f'{j*600/max(1,len(values)-1):.1f},{65-(v-low)*55/max(high-low,1e-6):.1f}' for j,v in enumerate(values))
                    name = names[col] if col < len(names) else str(col)
                    traces.append(f'<div>{html.escape(name)} · {low:.3f}–{high:.3f}<svg viewBox="0 0 600 75"><polyline points="{points}" fill="none" stroke="#38bdf8"/></svg></div>')
                task = str(f.attrs.get('task', ''))
                rows.append(f'<details><summary>{html.escape(ep.name)} · {len(state)} 帧 · {html.escape(task)}</summary>'+''.join(cards)+'<h3>状态曲线（前 8 维）</h3>'+''.join(traces)+'</details>')
        if not rows:
            raise ValueError('no aligned episodes')
        page = '''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>数据预览</title><style>body{background:#0b0f17;color:#e5e7eb;font:14px system-ui;margin:24px}details{background:#1f2937;padding:16px;margin:12px 0;border-radius:8px}summary{cursor:pointer}figure{margin:0}img{max-width:100%;border-radius:6px}.strip{display:flex;flex-wrap:wrap;gap:12px}svg{display:block;width:100%;max-width:700px;height:75px}figcaption{color:#9ca3af}</style><h1>对齐数据预览</h1><p>每段显示首、中、末帧与状态曲线，用于抽查图像和轨迹；不能替代完整任务回放。</p>'''+''.join(rows)+'</html>'
        (staged/'index.html').write_text(page)
    print(f'preview episodes={len(rows)} → {output / "index.html"}', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    preview_session(args.session, args.output)


if __name__ == '__main__':
    main()
