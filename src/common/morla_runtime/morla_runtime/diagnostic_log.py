"""Robot-neutral run diagnostics. Bounded enqueue on control paths; disk I/O on one worker.

Not a dataset recorder and never an execution authority. Records have explicit
clock domains and provenance; binary images are described, never duplicated.
"""
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import threading
import time


_SECRET = ('token', 'password', 'secret', 'credential', 'authorization')


def snapshot(value, *, budget=4096, depth=12, max_bytes=65536, accounting=None):
    """Bounded immutable JSON projection, also used to redact config snapshots."""
    remaining = [budget]
    remaining_bytes = [max_bytes]
    def visit(item, level):
        remaining[0] -= 1
        remaining_bytes[0] -= 32
        if remaining[0] < 0 or remaining_bytes[0] < 0 or level < 0:
            return {'omitted': 'snapshot_limit'}
        if item is None or isinstance(item, bool): return item
        if isinstance(item,int):
            return item if item.bit_length()<=256 else {'omitted':'integer_too_large'}
        if isinstance(item, float):
            return item if math.isfinite(item) else {'invalid_float': str(item)}
        if isinstance(item, str):
            limit=max(0,min(4096,remaining_bytes[0]//6))
            text=item[:limit]+('…' if len(item)>limit else '')
            remaining_bytes[0]-=6*len(text)
            return text
        if isinstance(item, (bytes, bytearray, memoryview)):
            return {'omitted': 'binary', 'bytes': len(item)}
        if isinstance(item, dict):
            result = {}
            for index, (key, child) in enumerate(item.items()):
                if index >= 256 or remaining[0] < 0 or remaining_bytes[0] < 0:
                    result['_omitted'] = 'snapshot_limit'; break
                key = str(key)[:256]
                remaining_bytes[0]-=6*len(key)
                result[key] = '[redacted]' if any(word in key.lower() for word in _SECRET) else visit(child, level-1)
            return result
        if isinstance(item, (list, tuple)):
            result = []
            for child in item[:512]:
                if remaining[0] < 0 or remaining_bytes[0] < 0:
                    result.append({'omitted': 'snapshot_limit'}); break
                result.append(visit(child, level-1))
            if len(item) > 512: result.append({'omitted_items': len(item)-512})
            return result
        if hasattr(item, 'tolist') and getattr(item, 'size', 0) <= 512:
            return visit(item.tolist(), level-1)
        return {'omitted_type': type(item).__name__}
    result=visit(value, depth)
    if accounting is not None: accounting.append(max_bytes-max(0,remaining_bytes[0])+256)
    return result


def options(resolved):
    raw = resolved.get('source_assembly', {}).get('metadata', {}).get('diagnostic_logging', {})
    if not isinstance(raw, dict) or set(raw)-{'enabled','feedback_hz','queue_size','segment_mb','max_run_mb'}:
        raise ValueError('Invalid diagnostic_logging options')
    result = {'enabled': True, 'feedback_hz': 50, 'queue_size': 4096, 'segment_mb': 16, 'max_run_mb': 1024, **raw}
    if type(result['enabled']) is not bool: raise ValueError('diagnostic_logging.enabled must be boolean')
    for key, lower, upper in [('feedback_hz',1,200),('queue_size',16,65536),('segment_mb',1,256),('max_run_mb',1,8192)]:
        value = result[key]
        if type(value) not in (int,float) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError('Invalid diagnostic_logging.'+key)
    if type(result['queue_size']) is not int: raise ValueError('diagnostic_logging.queue_size must be integer')
    if result['segment_mb'] > result['max_run_mb']: raise ValueError('Diagnostic segment exceeds run budget')
    return result


class DiagnosticLog:
    def __init__(self, session_id, resolved, *, root=None, clock=time.monotonic_ns):
        self.session_id, self.resolved, self.clock = session_id, resolved, clock
        self.options = options(resolved)
        default = Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state')))/'morla/teleop'
        self.root = Path(root or os.environ.get('MORLA_DIAGNOSTIC_LOG_ROOT', str(default))).expanduser()
        self.queue = queue.Queue(maxsize=self.options['queue_size'])
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.closing = threading.Event()
        self.started = False
        self.closed = False
        self.directory = None
        self.error = None
        self.dropped = Counter()
        self.written = Counter()
        self.sequence = 0
        self.bytes_written = 0
        self.queued_bytes = 0
        self.queue_byte_limit = 16*1024*1024
        self.segments = []
        self.feedback_times = {}
        self.outcome = None

    def start(self):
        if self.started or not self.options['enabled']: return
        self.started = True
        try:
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            # Assembly name is not trusted as a filesystem path.
            assembly = ''.join(c for c in str(self.resolved.get('assembly_id','system')) if c.isalnum() or c in '_-')[:64]
            self.root.mkdir(parents=True, exist_ok=True)
            self.directory = self.root/(stamp+'_'+assembly+'_'+self.session_id)
            self.directory.mkdir(mode=0o700)
            manifest = {'schema_version':1, 'session_id':self.session_id,
                        'started_at_utc':datetime.now(timezone.utc).isoformat(),
                        'options':self.options, 'clocks':{'wall_time_ns':'unix_epoch_utc','monotonic_ns':'runtime_clock'},
                        'feedback_policy':'cached_health_sample_on_runtime_poll_or_explicit_trace_feedback',
                        'resolved_system':snapshot(self.resolved,budget=200000,depth=24,max_bytes=4*1024*1024)}
            (self.directory/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,allow_nan=False,indent=2))
            self.thread = threading.Thread(target=self._write_loop, name='morla-diagnostics', daemon=True)
            self.thread.start()
            self.emit('run_started', {'assembly_id':self.resolved.get('assembly_id')})
        except Exception as exc:
            self.error = 'LOG_START_FAILED: '+str(exc)
            self.closed = True

    def emit(self, kind, payload, *, component_id=None, binding_id=None, monotonic_ns=None):
        if not self.options['enabled'] or not self.started: return False
        if self.closed or self.closing.is_set(): return False
        try:
            # Refuse before copying a payload when already saturated.
            if self.queue.full():
                with self.lock:
                    self.sequence += 1
                    self.dropped[kind] += 1
                return False
            cost=[]
            value = snapshot(payload,accounting=cost)
            record = {'schema_version':1,'kind':kind,'session_id':self.session_id,
                      'monotonic_ns':self.clock() if monotonic_ns is None else monotonic_ns,
                      'wall_time_ns':time.time_ns(),'data':value}
            if component_id is not None: record['component_id']=component_id
            if binding_id is not None: record['binding_id']=binding_id
            with self.lock:
                self.sequence += 1
                record['record_sequence'] = self.sequence
                if self.queued_bytes+cost[0]>self.queue_byte_limit:
                    self.dropped[kind]+=1
                    return False
                record['_queue_cost']=cost[0]
                try:
                    self.queue.put_nowait(record)
                    self.queued_bytes+=cost[0]
                except queue.Full:
                    self.dropped[kind] += 1
                    return False
            self.wake.set()
            return True
        except Exception as exc:
            with self.lock:
                self.dropped[kind] += 1
                self.error = 'LOG_ENQUEUE_FAILED: '+str(exc)
            return False

    def feedback(self, component_id, sample, *, now_ns):
        if not self.options['enabled'] or not self.started or self.closed: return
        stamp = sample.get('sample_ns')
        marker = (stamp, sample.get('ready'), sample.get('fault'), sample.get('reason'))
        old = self.feedback_times.get(component_id)
        period = int(1e9/self.options['feedback_hz'])
        if old and marker == old[1]: return
        if old and now_ns-old[0] < period and marker[1:] == old[1][1:]: return
        if self.emit('feedback', sample, component_id=component_id, monotonic_ns=now_ns):
            self.feedback_times[component_id] = (now_ns,marker)

    def status(self):
        with self.lock:
            return {'enabled':self.options['enabled'], 'state':'disabled' if not self.options['enabled'] else
                    'degraded' if self.error or self.dropped else 'closed' if self.closed else 'recording' if self.started else 'not_started',
                    'directory':str(self.directory) if self.directory else None,
                    'files':list(self.segments), 'written':dict(self.written),'dropped':dict(self.dropped),
                    'queued':self.queue.qsize(),'queued_bytes_estimate':self.queued_bytes,'queue_byte_limit':self.queue_byte_limit,'bytes_written':self.bytes_written,'error':self.error,
                    'feedback_hz':self.options['feedback_hz']}

    def _open_segment(self, index):
        name=f'trace-{index:05d}.jsonl'
        stream=(self.directory/name).open('x',encoding='utf-8')
        with self.lock: self.segments.append(name)
        return stream

    def _write_loop(self):
        stream = None
        segment_bytes = 0
        index = 0
        last_flush = time.monotonic()
        try:
            stream = self._open_segment(index)
            while not self.closing.is_set() or not self.queue.empty():
                try: record = self.queue.get(timeout=.1)
                except queue.Empty:
                    if time.monotonic()-last_flush >= .5:
                        stream.flush(); last_flush=time.monotonic()
                    continue
                with self.lock: self.queued_bytes=max(0,self.queued_bytes-record.pop('_queue_cost',0))
                try:
                    line=json.dumps(record,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n'
                    size=len(line.encode('utf-8'))
                    if size > 262144:
                        with self.lock: self.dropped[record['kind']] += 1
                        continue
                    if self.bytes_written+size > self.options['max_run_mb']*1024*1024:
                        with self.lock:
                            self.error='LOG_RUN_BUDGET_EXHAUSTED'
                            self.dropped[record['kind']] += 1
                        continue
                    if segment_bytes and segment_bytes+size > self.options['segment_mb']*1024*1024:
                        stream.close();index+=1;stream=self._open_segment(index);segment_bytes=0
                    stream.write(line)
                    segment_bytes+=size
                    with self.lock:
                        self.bytes_written+=size
                        self.written[record['kind']]+=1
                    if time.monotonic()-last_flush >= .5:
                        stream.flush();last_flush=time.monotonic()
                finally: self.queue.task_done()
        except Exception as exc:
            with self.lock: self.error='LOG_WRITE_FAILED: '+str(exc)
        finally:
            if stream is not None:
                try: stream.close()
                except Exception as exc:
                    with self.lock: self.error='LOG_CLOSE_FAILED: '+str(exc)
            with self.lock: self.closed=True
            try:
                summary={**self.status(),'outcome':snapshot(self.outcome),
                         'ended_at_utc':datetime.now(timezone.utc).isoformat(),
                         'clean_shutdown':self.closing.is_set() and self.queue.empty() and not self.error and not self.dropped}
                temporary=self.directory/'summary.tmp'
                temporary.write_text(json.dumps(summary,ensure_ascii=False,allow_nan=False,indent=2))
                temporary.replace(self.directory/'summary.json')
            except Exception as exc:
                with self.lock: self.error='LOG_SUMMARY_FAILED: '+str(exc)

    def close(self, outcome=None, timeout=2.):
        if not self.started or self.closed: return self.status()
        self.outcome=outcome
        self.emit('run_ended', outcome or {})
        self.closing.set();self.wake.set()
        self.thread.join(timeout=timeout)
        if self.thread.is_alive():
            with self.lock: self.error='LOG_DRAIN_TIMEOUT'
        return self.status()
