#!/usr/bin/env python3
"""Read-only piano shift recorder. Never sends commands or browser heartbeats."""
import argparse
from collections import deque
import datetime
import json
import math
from pathlib import Path
import time
import urllib.request


def degrees(values):
    return [math.degrees(v) for v in values]


def prediction(config, side, shift):
    hand = config['hands'][side]
    explicit = hand.get('arm_positions_rad', {}).get(str(shift))
    calculated = [q + shift*d for q, d in zip(hand['initial_arm_rad'], hand['arm_step_rad_per_white'])]
    target = list(explicit) if explicit is not None else calculated
    low, high = hand['shift_range_white']
    violations = [i+1 for i, (q, bounds) in enumerate(zip(target, hand['arm_limits_rad']))
                  if not bounds[0] <= q <= bounds[1]]
    return dict(requested_shift=shift, allowed_range=[low, high], range_allowed=low <= shift <= high,
                source='saved_measured_pose' if explicit is not None else 'zero_plus_steps',
                zero_deg=degrees(hand['initial_arm_rad']), step_deg=degrees(hand['arm_step_rad_per_white']),
                linear_target_deg=degrees(calculated), target_deg=degrees(target),
                limit_violations=violations, executable=low <= shift <= high and not violations)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--url', default='http://127.0.0.1:8080/api/v1/piano/state')
    parser.add_argument('--shift', type=int, default=2)
    parser.add_argument('--seconds', type=float, default=900)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = dict(expected_shift=args.shift, status='waiting_for_manual_test', sessions={}, attempts=[])
    session = None
    config = None
    predictions = None
    before = deque(maxlen=20)
    attempt = None
    settle_until = None
    deadline = time.monotonic()+args.seconds

    def save():
        path = args.output/'summary.json'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
        temporary.replace(path)

    with (args.output/'samples.jsonl').open('a', buffering=1) as log:
        save()
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(args.url, timeout=2) as response:
                    state = json.load(response)['data']
                stamp = datetime.datetime.now(datetime.timezone.utc).isoformat()
                sampled = {k: state.get(k) for k in ('active', 'state', 'error', 'session_id', 'config_revision',
                    'job', 'busy', 'result', 'positions', 'command_targets', 'feedback_age_s', 'wrist_shifts')}
                log.write(json.dumps(dict(utc=stamp, state=sampled), ensure_ascii=False, allow_nan=False)+'\n')
                current = state.get('session_id')
                if current != session:
                    if attempt is not None:
                        attempt['interrupted_by_session_change'] = True
                    session, attempt, settle_until = current, None, None
                    before.clear()
                    config = None
                    path = args.data/'sessions'/str(session)/'config.json'
                    if current and path.is_file():
                        config = json.loads(path.read_text())
                        predictions = {s: prediction(config, s, args.shift) for s in ('left', 'right')}
                        summary['sessions'][current] = dict(predictions=predictions, config_revision=state.get('config_revision'))
                        save()
                job = state.get('job') or {}
                if job.get('operation') == 'shift' and attempt is None and config is not None:
                    baseline = before[-1] if before else state
                    attempt = dict(job_id=job['id'], session_id=session, started_utc=stamp,
                                   predictions=predictions, baseline=baseline.get('positions', {}),
                                   samples=0, max_change_deg={}, max_command_change_deg={})
                    summary['attempts'].append(attempt)
                    summary['status'] = 'recording_manual_shift'
                    save()
                if attempt is not None:
                    attempt['samples'] += 1
                    for side in ('left', 'right'):
                        key = side+'_arm'
                        q = state.get('positions', {}).get(key)
                        initial = attempt['baseline'].get(key)
                        target = state.get('command_targets', {}).get(key)
                        if q and initial:
                            change = max(abs(a-b) for a,b in zip(q, initial))
                            attempt['max_change_deg'][side] = max(attempt['max_change_deg'].get(side, 0), math.degrees(change))
                            attempt.setdefault('final_deg', {})[side] = degrees(q)
                            errors = [a-b for a,b in zip(degrees(q), predictions[side]['target_deg'])]
                            attempt.setdefault('final_error_to_expected_deg', {})[side] = errors
                        if target and initial:
                            attempt['max_command_change_deg'][side] = max(attempt['max_command_change_deg'].get(side, 0),
                                math.degrees(max(abs(a-b) for a,b in zip(target, initial))))
                            attempt.setdefault('last_command_deg', {})[side] = degrees(target)
                    result = state.get('result') or {}
                    if result.get('job_id') == attempt['job_id'] and not state.get('busy'):
                        attempt['result'] = result
                        attempt['reported_shifts'] = state.get('wrist_shifts')
                        settle_until = settle_until or time.monotonic()+.8
                    if not state.get('active'):
                        attempt['session_released'] = True
                        settle_until = settle_until or time.monotonic()+.2
                    if settle_until and time.monotonic() >= settle_until:
                        summary['status'] = 'manual_shift_recorded'
                        save()
                        return
                elif not state.get('busy'):
                    before.append(state)
            except (OSError, ValueError, KeyError) as exc:
                summary['last_read_error'] = str(exc)
                save()
            time.sleep(.1)
        summary['status'] = 'recording_window_ended'
        save()


if __name__ == '__main__':
    main()
