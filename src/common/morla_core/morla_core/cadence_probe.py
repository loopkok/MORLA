"""Opt-in per-process telemetry for cadence investigations.

Set ``MORLA_CADENCE_PROBE=1`` to enable. It is off by default so a normal run
writes nothing: this is shared framework code, and a probe that writes into the
user's home directory on every process is a diagnostic, not a default.

When enabled, each process appends one JSON line per second to
``~/.local/share/morla/cadence-probe/<wall_ns>-<pid>.jsonl`` with per-prefix
counts, latency percentiles (p50/p95/max) and recent events. Files are never
rotated, so remove them once an investigation is done.
"""
import json
import os
from pathlib import Path
import threading
import time

_ENABLED = os.environ.get('MORLA_CADENCE_PROBE', '').strip().lower() in ('1', 'true', 'yes', 'on')
_lock = threading.Lock()
_counts = {}
_times = {}
_edges = []
_started = False


def _flush():
    try:
        directory = Path.home() / '.local/share/morla/cadence-probe'
        directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        with (directory / f'{time.time_ns()}-{os.getpid()}.jsonl').open('x') as stream:
            while True:
                time.sleep(1)
                global _counts, _times, _edges
                with _lock:
                    counts, _counts = _counts, {}
                    times, _times = _times, {}
                    edges, _edges = _edges, []
                if not (counts or times or edges):
                    continue
                latency = {}
                for name, values in times.items():
                    values.sort()
                    latency[name] = {
                        'n': len(values),
                        'p50_ms': values[(len(values) - 1) // 2] / 1e6,
                        'p95_ms': values[int(.95 * (len(values) - 1))] / 1e6,
                        'max_ms': values[-1] / 1e6,
                    }
                stream.write(json.dumps({'wall_ns': time.time_ns(), 'mono_ns': time.monotonic_ns(),
                                         'count': counts, 'latency': latency, 'events': edges}) + '\n')
                stream.flush()
    except OSError as exc:
        print(f'MORLA cadence probe: {exc}', file=__import__('sys').stderr)


def record(prefix, *, counts=(), timings=(), event=None):
    if not _ENABLED:
        return
    global _started
    with _lock:
        if not _started:
            _started = True
            threading.Thread(target=_flush, name='morla-cadence-probe', daemon=True).start()
        for key in counts:
            name = prefix + '.' + key
            _counts[name] = _counts.get(name, 0) + 1
        for key, duration in timings:
            values = _times.setdefault(prefix + '.' + key, [])
            if len(values) < 4096:
                values.append(duration)
        if event is not None and len(_edges) < 128:
            _edges.append({'name': prefix, 'wall_ns': time.time_ns(), 'data': event})
