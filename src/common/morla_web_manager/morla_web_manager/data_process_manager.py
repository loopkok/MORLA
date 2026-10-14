"""Background offline jobs. Data algorithms stay in morla_data_collect.pipeline."""
from __future__ import annotations

from collections import deque
import copy
import json
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import threading
import time
import uuid


ACTIONS = {'full', 'raw-check', 'quantify', 'align', 'validate', 'repair-preview', 'repair', 'verify', 'export', 'preview', 'upgrade-v3'}
TERMINAL = {'succeeded', 'failed', 'cancelled', 'interrupted'}


class PipelineConflict(ValueError):
    pass


class DataProcessManager:
    def __init__(self, data_root, python, package_root, check_python=''):
        self.data_root = Path(data_root).resolve()
        self.jobs_root = self.data_root / 'web_jobs'
        self.python = str(python)
        self.package_root = str(package_root)
        self.check_python = str(check_python)
        self._lock = threading.RLock()
        self._jobs = {}
        self._active = None
        self._proc = None
        self._worker = None
        self._cancel = threading.Event()
        self._logs = deque(maxlen=2000)
        self._load_history()

    def _load_history(self):
        if not self.jobs_root.is_dir() or not self.jobs_root.resolve().is_relative_to(self.data_root):
            return
        for path in sorted(self.jobs_root.glob('*/job.json'))[-20:]:
            if not path.resolve().is_relative_to(self.jobs_root.resolve()):
                continue
            try:
                job = json.loads(path.read_text())
                if job['id'] != path.parent.name:
                    continue
                if Path(job['directory']).resolve() != path.parent.resolve():
                    continue
                if job['status'] not in TERMINAL:
                    job['status'] = 'interrupted'
                    job['error'] = 'Web 服务已重启；检查已有产物后重新提交任务'
                    job['finished_at'] = time.time()
                    for step in job.get('steps', []):
                        if step['status'] == 'running':
                            step['status'] = 'interrupted'
                        elif step['status'] == 'pending':
                            step['status'] = 'skipped'
                    self._save(job)
                self._jobs[job['id']] = job
            except (ValueError, KeyError, OSError):
                continue

    def resolve_source(self, value):
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = self.data_root / path
        path = path.resolve()
        if not path.is_relative_to(self.data_root) or path == self.data_root:
            raise ValueError('输入必须是数据根目录内的 session 或数据集')
        if not path.is_dir():
            raise ValueError(f'输入目录不存在：{path}')
        return path

    def sessions(self):
        result = []
        # Raw sessions plus outputs from this UI. Avoid walking millions of videos.
        candidates = list((self.data_root / 'raw').glob('*'))
        for folder in ['work', 'clean', 'qa', 'pi', 'act']:
            candidates.extend((self.data_root / folder).glob('*'))
        with self._lock:
            jobs = list(self._jobs.values())
        for job in jobs:
            candidates.extend(Path(job['directory']) / name for name in ['working', 'cleaned', 'openpi', 'act'])
        for path in sorted(set(candidates)):
            if not path.is_dir() or not path.resolve().is_relative_to(self.data_root):
                continue
            episodes = [p for p in path.glob('episode*') if p.is_dir()]
            if episodes:
                first = episodes[0]
                try:
                    meta = json.loads((first/'meta.json').read_text())
                except (OSError, ValueError):
                    meta = {}
                if not isinstance(meta, dict):
                    meta = {}
                schema = meta.get('schema') or {}
                if not isinstance(schema, dict):
                    schema = {}
                cameras = schema.get('cameras', [])
                if not isinstance(cameras, list):
                    cameras = []
                result.append({'path': str(path), 'name': str(path.relative_to(self.data_root)),
                               'episodes': len(episodes), 'kind': 'session',
                               'cameras': [c for c in cameras if isinstance(c, str)],
                               'task': meta.get('task', '')})
            elif (path/'meta/info.json').is_file():
                result.append({'path': str(path), 'name': str(path.relative_to(self.data_root)),
                               'episodes': 0, 'kind': 'dataset', 'cameras': [], 'task': ''})
        return result

    def capabilities(self):
        return {'data_root': str(self.data_root), 'python_available': Path(self.python).is_file(),
                'deep_check_available': bool(self.check_python and Path(self.check_python).is_file()),
                'actions': sorted(ACTIONS)}

    def _plan(self, req, directory):
        work, clean = directory/'working', directory/'cleaned'
        action, source = req['action'], req['source']
        steps = []
        def add(name, command, *args):
            steps.append({'name': name, 'command': command, 'args': list(map(str, args)), 'status': 'pending'})
        if action == 'upgrade-v3':
            add('升级 LeRobot v3', 'upgrade-v3', '--v21-root', source, '--output', directory/'act')
            return steps
        prepare = ['--source', source, '--output', work]
        if req.get('camera_map'):
            prepare += ['--camera-map', req['camera_map']]
        if req.get('episodes'):
            prepare += ['--episodes', req['episodes']]
        if action not in {'full', 'raw-check', 'quantify', 'align'}:
            prepare += ['--include-aligned']
        add('建立工作副本', 'prepare', *prepare)
        if action == 'full':
            add('原始数据校验', 'validate', '--session', work)
            if req.get('quantify', True):
                add('指令与状态诊断', 'quantify', '--h5', work, '--out', directory/'quantify.json')
            add('时间对齐', 'align', '--session', work, '--force')
            add('对齐数据校验', 'validate', '--session', work)
            add('生成清洗前预览', 'preview', '--session', work, '--output', directory/'preview_before')
        elif action in {'raw-check', 'validate'}:
            add('数据校验', 'validate', '--session', work)
        elif action == 'quantify':
            add('指令与状态诊断', 'quantify', '--h5', work, '--out', directory/'quantify.json')
        elif action == 'align':
            add('时间对齐', 'align', '--session', work, '--force')
            add('对齐数据校验', 'validate', '--session', work)
            add('生成预览', 'preview', '--session', work, '--output', directory/'preview')
        elif action == 'repair-preview':
            add('清洗预览（不删除帧）', 'repair', '--session', work, '--dry-run')
        elif action == 'verify':
            add('验证帧来源', 'verify-aligned', '--source', req['reference'], '--session', work)
        elif action == 'preview':
            add('生成预览', 'preview', '--session', work, '--output', directory/'preview')
        current = work
        if action == 'repair' or (action == 'full' and req.get('repair', False)):
            add('清洗方案预览', 'repair', '--session', work, '--dry-run')
            add('保守清洗', 'repair', '--session', work, '--out-session', clean)
            add('清洗结果校验', 'validate', '--session', clean)
            add('验证帧来源', 'verify-aligned', '--source', work, '--session', clean)
            add('生成清洗后预览', 'preview', '--session', clean, '--output', directory/'preview_after')
            current = clean
        if action in {'full', 'export'}:
            target = req.get('target', 'openpi')
            if target in {'openpi', 'both'}:
                add('导出 OpenPI v2.1', 'export-openpi', '--session', current, '--output', directory/'openpi',
                    '--image-size', req.get('image_size', 224))
            if target in {'act', 'both'}:
                args = ['--v21-root', directory/'openpi'] if target == 'both' else ['--session', current]
                args += ['--output', directory/'act', '--image-size', req.get('image_size', 224)]
                if self.check_python and Path(self.check_python).is_file():
                    args += ['--check-python', self.check_python]
                add('导出 ACT v3 并自检', 'export-act', *args)
        return steps

    def start(self, request):
        req = dict(request)
        if req.get('action') not in ACTIONS:
            raise ValueError('未知处理步骤')
        req['source'] = str(self.resolve_source(req.get('source', '')))
        source = Path(req['source'])
        if req['action'] == 'upgrade-v3':
            if not (source/'meta/info.json').is_file():
                raise ValueError('升级输入必须是 LeRobot v2.1 数据集')
        elif not any(source.glob('episode*/robot_data.h5')):
            raise ValueError('输入没有 raw episode')
        if req['action'] == 'verify':
            req['reference'] = str(self.resolve_source(req.get('reference', '')))
        if req.get('target', 'openpi') not in {'openpi', 'act', 'both'}:
            raise ValueError('未知导出目标')
        size = int(req.get('image_size', 224))
        if size < 0 or size > 2048:
            raise ValueError('图像尺寸必须为 0–2048')
        for pair in req.get('camera_map', '').split(','):
            if pair and not re.fullmatch(r'[A-Za-z0-9_-]+=[A-Za-z0-9_-]+', pair):
                raise ValueError('相机映射格式：video8=base,video0=left_wrist')
        if req.get('episodes') and not re.fullmatch(r'\d+(,\d+)*', req['episodes']):
            raise ValueError('段号格式：0,24,49；留空处理全部')
        if not Path(self.python).is_file():
            raise ValueError('数据处理 Python 不存在，请设置 MORLA_DATA_PYTHON')
        with self._lock:
            if self._active:
                raise PipelineConflict('已有数据任务运行中')
            self.jobs_root.mkdir(parents=True, exist_ok=True)
            if not self.jobs_root.resolve().is_relative_to(self.data_root):
                raise ValueError('任务目录不能通过符号链接指向数据根目录外')
            job_id = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:8]
            directory = self.jobs_root/job_id
            directory.mkdir()
            job = {'id': job_id, 'status': 'running', 'action': req['action'], 'source': req['source'],
                   'directory': str(directory), 'started_at': time.time(), 'finished_at': None,
                   'steps': self._plan(req, directory), 'artifacts': [], 'error': '', 'request': req,
                   'deep_check': self.capabilities()['deep_check_available']}
            self._save(job)
            self._jobs[job_id] = job
            self._active = job_id
            self._logs.clear()
            self._cancel.clear()
            self._worker = threading.Thread(target=self._run, args=(job,), daemon=True)
            self._worker.start()
            return self.job(job_id)

    def _save(self, job):
        path = Path(job['directory'])/'job.json'
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(job, ensure_ascii=False, indent=2))
        temp.replace(path)

    def _discover(self, job):
        root = Path(job['directory'])
        known = {'preparation_report.json', 'validation_report.json', 'quantify.json', 'repair_report.json',
                 'processing_manifest.json', 'info.json', 'index.html', 'job.log',
                 'raw_validation.json', 'repair_summary.json'}
        repairs = []
        for report in sorted((root/'cleaned').glob('episode*/repair_report.json')):
            repairs.append({'episode': report.parent.name, **json.loads(report.read_text())})
        if repairs:
            (root/'repair_summary.json').write_text(json.dumps({
                'source_frames': sum(r['source_frames'] for r in repairs),
                'output_frames': sum(r['output_frames'] for r in repairs), 'episodes': repairs},
                ensure_ascii=False, indent=2))
        job['artifacts'] = [{'path': str(p.relative_to(root)), 'name': str(p.relative_to(root)),
                             'kind': 'preview' if p.name == 'index.html' else 'report'}
                            for p in sorted(root.rglob('*')) if p.is_file() and p.name in known
                            and p.name != 'repair_report.json'
                            and p.resolve().is_relative_to(root.resolve())][:300]
        summary = []
        validation_files = list(root.glob('*/validation_report.json'))
        if (root/'raw_validation.json').exists():
            validation_files.insert(0, root/'raw_validation.json')
        for p in validation_files:
            try:
                report = json.loads(p.read_text())
                summary.append({'stage': 'raw' if p.name == 'raw_validation.json' else p.parent.name,
                                **{k: report.get(k, 0) for k in ['total', 'pass', 'warn', 'fail']}})
            except (OSError, ValueError):
                pass
        job['validation'] = summary

    def _run(self, job):
        env = {**os.environ, 'PYTHONUNBUFFERED': '1', 'PYTHONPATH': self.package_root + os.pathsep + os.environ.get('PYTHONPATH', '')}
        try:
            with (Path(job['directory'])/'job.log').open('w') as log:
                for step in job['steps']:
                    with self._lock:
                        if self._cancel.is_set():
                            break
                        step['status'] = 'running'
                        step['started_at'] = time.time()
                        command = [self.python, '-m', 'morla_data_collect.pipeline', step['command'], *step['args']]
                        log.write('\n=== '+step['name']+' ===\n'); log.flush()
                        proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                                text=True, errors='replace', env=env, start_new_session=True)
                        self._proc = proc
                        self._save(job)
                    for line in proc.stdout:
                        log.write(line); log.flush()
                        with self._lock:
                            self._logs.append(line.rstrip()[:1500])
                    code = proc.wait()
                    with self._lock:
                        self._proc = None
                        if step['name'] == '原始数据校验' and (Path(job['directory'])/'working/validation_report.json').exists():
                            shutil.copy2(Path(job['directory'])/'working/validation_report.json',
                                         Path(job['directory'])/'raw_validation.json')
                        step['returncode'] = code
                        step['finished_at'] = time.time()
                        step['status'] = 'cancelled' if self._cancel.is_set() else ('succeeded' if code == 0 else 'failed')
                        self._discover(job)
                        self._save(job)
                    if code != 0 or self._cancel.is_set():
                        if not self._cancel.is_set():
                            job['error'] = f"{step['name']}未通过（退出码 {code}），查看日志与报告后修复"
                        break
            with self._lock:
                job['status'] = 'cancelled' if self._cancel.is_set() else ('failed' if job['error'] else 'succeeded')
        except Exception as exc:
            with self._lock:
                job['status'] = 'cancelled' if self._cancel.is_set() else 'failed'
                for step in job['steps']:
                    if step['status'] == 'running':
                        step['status'] = job['status']
                job['error'] = str(exc)
                self._logs.append(str(exc))
        finally:
            with self._lock:
                for step in job['steps']:
                    if step['status'] == 'pending':
                        step['status'] = 'skipped'
                job['finished_at'] = time.time()
                job['log_tail'] = list(self._logs)[-60:]
                try:
                    self._discover(job)
                    self._save(job)
                finally:
                    self._active = None
                    self._proc = None

    def cancel(self):
        with self._lock:
            if not self._active:
                raise PipelineConflict('没有正在执行的数据任务')
            if self._cancel.is_set():
                return
            self._cancel.set()
            self._jobs[self._active]['status'] = 'stopping'
            proc = self._proc
            if proc and proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                threading.Thread(target=self._kill_after, args=(proc,), daemon=True).start()
            try:
                self._save(self._jobs[self._active])
            except OSError as exc:
                self._logs.append(f'停止已请求，但状态保存失败：{exc}')

    def _kill_after(self, proc):
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def close(self):
        with self._lock:
            if self._active:
                self.cancel()
            worker = self._worker
        if worker and worker is not threading.current_thread():
            worker.join(timeout=12)

    def job(self, job_id):
        with self._lock:
            if job_id not in self._jobs:
                raise KeyError(job_id)
            job = copy.deepcopy(self._jobs[job_id])
            if job_id == self._active:
                job['log_tail'] = list(self._logs)[-60:]
            return job

    def snapshot(self):
        with self._lock:
            latest = self._active or next(reversed(self._jobs), None)
            history = [{'id': j['id'], 'status': j['status'], 'action': j['action'], 'source': j['source'],
                        'started_at': j['started_at']} for j in list(self._jobs.values())[-20:]][::-1]
            current = self.job(latest) if latest else None
            if current:
                current.pop('request', None)
                current['steps'] = [{k: v for k, v in step.items() if k != 'args'} for step in current['steps']]
                current['log_tail'] = [line[:500] for line in current.get('log_tail', [])[-15:]]
            return {'busy': bool(self._active), 'current': current, 'history': history}

    def artifact(self, job_id, relative):
        job = self.job(job_id)
        root = Path(job['directory']).resolve()
        path = (root/relative).resolve()
        if not path.is_relative_to(root) or path.suffix not in {'.json', '.html', '.jpg', '.png', '.mp4', '.txt', '.log'}:
            raise ValueError('不允许读取该路径')
        if not path.is_file():
            raise FileNotFoundError(relative)
        return path
