"""Project controller events onto acquisition time; never invent legacy labels."""
import numpy as np

OWNER_CODES = {'unknown': 0, 'policy': 1, 'human': 2, 'playback': 3, 'pending': 4, 'idle': 5}


def labels_at(meta, timestamps, timeout=2.0):
    times = np.asarray(timestamps)
    owner = np.zeros(len(times), dtype=np.int64)
    valid = np.zeros(len(times), dtype=bool)
    events = sorted((e for e in meta.get('events', []) if e.get('name') == 'control_owner'),
                    key=lambda e: e['t'])
    for index, event in enumerate(events):
        value = event.get('data') or {}
        if not isinstance(value, dict): continue
        start = float(event['t'])
        end = min(start + timeout, float(value.get('sent_at', start)) + timeout,
                  float(events[index+1]['t']) if index+1 < len(events) else float('inf'))
        mask = (times >= start) & (times < end)
        activity = value.get('activity')
        pending = value.get('takeover_pending') or value.get('policy_start_pending')
        code = OWNER_CODES['pending'] if pending else OWNER_CODES.get(activity or 'idle', 0)
        if value.get('unknown'): code = 0
        owner[mask] = code
        valid[mask] = code in (1, 2, 3, 5)
    return {'control_owner': owner, 'intervention': owner == 2, 'intervention_valid': valid}
