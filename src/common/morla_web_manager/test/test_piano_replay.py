"""Exercise real manager/worker replay IPC with a virtual-clock software plant."""
import json
import shutil

import pytest

from morla_web_manager.piano_store import PianoConflict
from test_piano_lane import manager, store, open_software, wait_job, wait_until


def test_finished_shifted_performance_can_explicitly_prepare_and_replay(manager, tmp_path):
    project = tmp_path/'project'
    for name in ('morla_piano', 'config'):
        shutil.copytree(manager.store.project/name, project/name, ignore=shutil.ignore_patterns('__pycache__'))
    engine = project/'morla_piano/engine.py'
    engine.write_text(engine.read_text()+'\nClock = VirtualClock\n')
    manager.store.project = project
    manager.store._model = None
    score = {'tempo_bpm': 60, 'events': [
        {'start': 0, 'duration': 1, 'hand': 'right', 'note': 'C4'},
        {'start': 3, 'duration': 1, 'hand': 'left', 'note': 'B2', 'finger': 'pinky', 'shift_white': -1},
    ]}
    imported = manager.store.import_score('Replay test', json.dumps(score))
    token = open_software(manager)
    prepared = manager.command(token, 'prepare')
    assert wait_job(manager, prepared, token)['ok']
    played = manager.command(token, 'play', score_id=imported['id'], tempo_bpm=60)
    assert wait_job(manager, played, token)['ok']
    first = manager.snapshot()
    assert first['state'] == 'FINISHED' and first['wrist_shifts']['left'] == -1
    assert first['playback']['percent'] == 100 and first['playback']['completed_notes'] == 2
    with pytest.raises(PianoConflict, match='准备'):
        manager.command(token, 'play', score_id=imported['id'], tempo_bpm=60)
    replayed = manager.command(token, 'play', score_id=imported['id'], tempo_bpm=60, prepare_first=True)
    assert wait_job(manager, replayed, token)['ok']
    second = manager.snapshot()
    assert second['result']['ok'] and second['state'] == 'FINISHED'
    assert second['playback']['percent'] == 100 and second['playback']['completed_notes'] == 2
    events = [json.loads(line) for line in (manager._directory/'events.jsonl').read_text().splitlines()]
    assert sum(e['event'] == 'prepare_approach_complete' for e in events) == 1
    assert sum(e['event'] == 'replay_return_to_zero' for e in events) == 1
    assert sum(e['event'] == 'work_pose_stage' for e in events) == 5
    assert sum(e['event'] == 'play_begin' for e in events) == 2
    returned = [e for e in events if e['event'] == 'ready']
    assert len(returned) == 2
    homed = manager.command(token, 'home', calibration_confirmed=True)
    assert wait_job(manager, homed, token)['ok']
    wait_until(lambda: manager.snapshot()['state'] == 'STOPPED')
    assert not manager.snapshot()['replay_direct_allowed']
    with pytest.raises(PianoConflict, match='不能直接'):
        manager.command(token, 'play', score_id=imported['id'], tempo_bpm=60, prepare_first=True)
    with manager.lock:
        manager._state['state'] = 'FAULT'
    with pytest.raises(PianoConflict, match='结束或已停止'):
        manager.command(token, 'play', score_id=imported['id'], tempo_bpm=60, prepare_first=True)
