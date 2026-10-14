"""Offline job lifecycle and original-data isolation, without a ROS graph."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import h5py
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
WS = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(WS/'src/astral_data_collect'))
from astral_web_monitor.data_process_manager import DataProcessManager, PipelineConflict
from astral_data_collect.schema import CollectSchema

spec = importlib.util.spec_from_file_location('raw_test_helpers', WS/'src/astral_data_collect/test/conftest.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
PYTHON = WS.parent/'VLA/openpi/.venv/bin/python'


def make_manager(tmp_path, python=PYTHON):
    return DataProcessManager(tmp_path, python, WS/'src/astral_data_collect')


def wait(manager, job_id, timeout=40):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        job = manager.job(job_id)
        if job['status'] in {'succeeded', 'failed', 'cancelled'}:
            return job
        time.sleep(.05)
    manager.cancel()
    raise AssertionError('job did not finish')


def test_full_job_copies_maps_cleans_and_exports(tmp_path):
    if not PYTHON.exists():
        pytest.skip('offline environment unavailable')
    source = tmp_path/'raw/session with spaces'
    schema = CollectSchema(arms=['left'], cameras=['video8'])
    ep = Path(helpers.write_raw_episode(str(source/'episode000000'), schema=schema))
    from astral_data_collect.pipeline.align_data import align_episode
    align_episode(str(ep))
    with h5py.File(ep/'aligned_data.h5', 'r+') as aligned:
        aligned['observation/state'][0, 0] += 0.123
        del aligned.attrs['raw_signature']  # Legacy cache cannot be trusted.
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in ep.iterdir()}
    manager = make_manager(tmp_path)
    job = manager.start({'source': str(source), 'action': 'full', 'target': 'both',
                         'repair': True, 'camera_map': 'video8=base', 'image_size': 32})
    with pytest.raises(PipelineConflict):
        manager.start({'source': str(source), 'action': 'align'})
    job = wait(manager, job['id'])
    assert job['status'] == 'succeeded', job
    assert all(s['status'] == 'succeeded' for s in job['steps'])
    root = Path(job['directory'])
    assert (root/'openpi/meta/info.json').exists()
    assert (root/'act/meta/info.json').exists()
    assert (root/'preview_after/index.html').exists()
    assert json.loads((root/'working/episode000000/meta.json').read_text())['schema']['cameras'] == ['base']
    with h5py.File(root/'working/episode000000/aligned_data.h5') as aligned, \
            h5py.File(ep/'aligned_data.h5') as old_aligned:
        assert 'raw_signature' in aligned.attrs
        assert aligned['observation/state'][0, 0] != pytest.approx(
            old_aligned['observation/state'][0, 0]
        )
    assert json.loads((root/'repair_summary.json').read_text())['output_frames'] > 0
    assert {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in ep.iterdir()} == before
    restored = make_manager(tmp_path)
    assert restored.job(job['id'])['status'] == 'succeeded'
    assert restored.artifact(job['id'], 'preview_after/index.html').exists()
    with pytest.raises(ValueError):
        restored.artifact(job['id'], '../../../etc/passwd')
    with pytest.raises(ValueError):
        restored.artifact(job['id'], 'working/episode000000/robot_data.h5')


def test_failed_validation_stops_later_steps(tmp_path):
    if not PYTHON.exists():
        pytest.skip('offline environment unavailable')
    source = tmp_path/'raw/bad'
    helpers.write_raw_episode(str(source/'episode000000'), nan_in='left_arm_state')
    manager = make_manager(tmp_path)
    job = wait(manager, manager.start({'source': str(source), 'action': 'full'})['id'])
    assert job['status'] == 'failed'
    assert job['steps'][1]['status'] == 'failed'
    assert all(s['status'] == 'skipped' for s in job['steps'][2:])
    assert not (Path(job['directory'])/'openpi').exists()
    assert not (source/'quarantine').exists()
    assert any(v['fail'] for v in job['validation'])


def test_cancel_child_and_keep_history(tmp_path):
    source = tmp_path/'raw/session'
    helpers.write_raw_episode(str(source/'episode000000'))
    fake = tmp_path/'fake-python'
    fake.write_text('#!/usr/bin/python3\nimport time\nprint("started", flush=True)\ntime.sleep(30)\n')
    fake.chmod(0o755)
    manager = make_manager(tmp_path, fake)
    job = manager.start({'source': str(source), 'action': 'align'})
    deadline = time.monotonic()+5
    while not manager.snapshot()['current'].get('log_tail') and time.monotonic() < deadline:
        time.sleep(.02)
    manager.cancel()
    finished = wait(manager, job['id'], timeout=10)
    assert finished['status'] == 'cancelled'
    assert not manager.snapshot()['busy']
    assert Path(finished['directory'], 'job.json').exists()


def test_input_boundary_and_option_validation(tmp_path):
    source = tmp_path/'raw/session'
    helpers.write_raw_episode(str(source/'episode000000'))
    manager = make_manager(tmp_path)
    with pytest.raises(ValueError, match='根目录'):
        manager.resolve_source('/etc')
    outside = tmp_path.parent/'outside-session'
    outside.mkdir(exist_ok=True)
    (tmp_path/'raw/escape').symlink_to(outside)
    with pytest.raises(ValueError, match='根目录'):
        manager.resolve_source('raw/escape')
    for override in [{'camera_map': 'a=b;touch /tmp/x'}, {'episodes': '0;1'}, {'image_size': -1}]:
        with pytest.raises(ValueError):
            manager.start({'source': str(source), 'action': 'full', **override})
    assert not manager.snapshot()['busy']


def test_restarted_service_marks_unfinished_job(tmp_path):
    root = tmp_path/'web_jobs/job-test'
    root.mkdir(parents=True)
    record = {'id': 'job-test', 'directory': str(root), 'source': str(tmp_path/'raw/session'),
              'status': 'running', 'action': 'align', 'started_at': 1, 'steps': [
                  {'name': 'copy', 'status': 'running'}, {'name': 'align', 'status': 'pending'}]}
    (root/'job.json').write_text(json.dumps(record))
    manager = make_manager(tmp_path)
    job = manager.job('job-test')
    assert job['status'] == 'interrupted'
    assert [s['status'] for s in job['steps']] == ['interrupted', 'skipped']
    assert not manager.snapshot()['busy']


def test_web_rejects_session_currently_recording(tmp_path, monkeypatch):
    import asyncio
    from fastapi import HTTPException
    from astral_web_monitor import web_server
    from astral_web_monitor.schemas import DataProcessRequest
    source = tmp_path/'raw/session'
    helpers.write_raw_episode(str(source/'episode000000'))
    manager = make_manager(tmp_path)
    class Node:
        def snapshot(self):
            return {'data_collect': {'stale': False, 'state': 'RECORDING', 'folder': str(source/'episode000000')}}
    monkeypatch.setattr(web_server, '_data_mgr', manager)
    monkeypatch.setattr(web_server, 'get_node', lambda: Node())
    with pytest.raises(HTTPException) as exc:
        asyncio.run(web_server.data_job_start(DataProcessRequest(source=str(source))))
    assert exc.value.status_code == 409
    assert not manager.snapshot()['busy']
