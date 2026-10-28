"""ROS-independent recording boundary for compiled assemblies and Web sessions.

Only measured snapshots and explicitly dispatched commands may be appended. This
class does not synthesize feedback or action labels from requested commands.
"""
from copy import deepcopy
import json
from pathlib import Path
from threading import RLock
import time
import numpy as np
from morla_data_collect.schema import pipeline_schema_from_dict
from morla_data_collect.data_writer import StreamDataWriter, CameraDataWriter
from morla_data_collect.codecs import DEFAULT_CODECS


class ComposableRecorder:
    def __init__(self, capture_schema: dict, session_dir: str):
        if capture_schema.get("schema_version") != 3:
            raise ValueError("ComposableRecorder requires an explicitly migrated schema v3")
        self._frozen = deepcopy(capture_schema)
        self.schema = pipeline_schema_from_dict(self._frozen)
        self.session_dir = Path(session_dir).expanduser().resolve()
        self._lock = RLock()
        self._active = False
        self._paused = False
        self._last = {}

    def _write_meta(self):
        target = self.episode_dir / 'meta.json'
        staged = target.with_suffix('.json.tmp')
        staged.write_text(json.dumps(self._meta, ensure_ascii=False, indent=2, allow_nan=False))
        staged.replace(target)

    def start(self, task=''):
        with self._lock:
            if self._active:
                raise ValueError('recording already active')
            self.session_dir.mkdir(parents=True, exist_ok=True)
            index = 0
            while True:
                episode = self.session_dir / f'episode{index:06d}'
                try:
                    episode.mkdir()
                    break
                except FileExistsError:
                    index += 1
            self.episode_dir = episode
            self._meta = {'format': 'raw_hdf5_v1', 'schema': deepcopy(self._frozen),
                          'schema_hash': self.schema.schema_hash, 'layout_hash': self.schema.layout_hash,
                          'episode_index': index, 'task': str(task), 'events': [],
                          'start_time_wall': time.time(), 'recording_status': 'recording',
                          'provenance': {'recorder': 'composable_v1', 'codecs': DEFAULT_CODECS.manifest(),
                                         'assembly': deepcopy(self._frozen.get('metadata', {}))}}
            self._write_meta()
            self._robot = self._camera = None
            try:
                self._robot = StreamDataWriter(str(episode / 'robot_data.h5'), self.schema.numeric_streams(),
                                               self.schema.sensor_specs).open()
                self._camera = CameraDataWriter(str(episode / 'camera_data.h5'), self.schema.cameras).open()
            except Exception as exc:
                for writer in (self._camera, self._robot):
                    if writer is not None:
                        try: writer.close()
                        except Exception: pass
                self._meta['recording_status'] = 'failed'
                self._meta['failure'] = f'{type(exc).__name__}: {exc}'
                try: self._write_meta()
                except OSError: pass  # Full storage may also prevent failure metadata.
                raise
            self._active = True
            self._paused = False
            self._last = {}
            return str(episode)

    def _timestamp(self, key, timestamp):
        value = time.time() if timestamp is None else float(timestamp)
        if not np.isfinite(value) or value <= self._last.get(key, -np.inf):
            raise ValueError(f'{key}: timestamp must be finite and strictly increasing')
        return value

    def append_stream(self, stream_key, values, timestamp=None, provenance=None):
        with self._lock:
            if not self._active:
                raise ValueError('recorder is not active')
            if self._paused:
                return False
            dims = self.schema.numeric_streams()
            if stream_key not in dims:
                raise ValueError(f'undeclared numeric stream {stream_key!r}')
            array = np.asarray(values, np.float32)
            if array.shape != (dims[stream_key],) or not np.isfinite(array).all():
                raise ValueError(f'{stream_key}: shape/finite-value mismatch')
            provenance = dict(provenance or {})
            if set(provenance) - {'source', 'valid', 'measured', 'synthetic', 'command_echo', 'source_time_ns', 'received_time_ns'}:
                raise ValueError('unknown sample provenance fields')
            for field in ('valid', 'measured', 'synthetic', 'command_echo'):
                if field in provenance and type(provenance[field]) not in {bool, np.bool_}:
                    raise ValueError('provenance masks must be bool')
            for field in ('source_time_ns', 'received_time_ns'):
                if field in provenance and (type(provenance[field]) is not int or not 0 <= provenance[field] <= np.iinfo(np.int64).max):
                    raise ValueError('source/received ns must be nonnegative int64')
            if provenance.get('measured') and (provenance.get('synthetic') or provenance.get('command_echo')):
                raise ValueError('sample cannot be both measured and synthetic/command_echo')
            if 'source' in provenance and not isinstance(provenance['source'], str):
                raise ValueError('provenance source must be string')
            stamp = self._timestamp(stream_key, timestamp)
            try:
                self._robot.write(stream_key, array, stamp, provenance=provenance)
            except Exception as exc:
                self._meta['recording_status'] = 'failed'
                self._meta['failure'] = str(exc)
                self._write_meta()
                raise
            self._last[stream_key] = stamp
            return True

    def append_snapshot(self, samples, timestamp=None, provenance=None):
        stamp = time.time() if timestamp is None else timestamp
        with self._lock:
            if not self._active: raise ValueError('recorder is not active')
            if self._paused: return False
            dims = self.schema.numeric_streams()
            for key, values in samples.items():
                array = np.asarray(values, np.float32)
                if key not in dims or array.shape != (dims[key],) or not np.isfinite(array).all():
                    raise ValueError(f'{key}: undeclared stream or shape/finite mismatch')
                self._timestamp(key, stamp)
            for key, values in samples.items():
                self.append_stream(key, values, stamp, provenance=(provenance or {}).get(key))
            return True

    def append_camera(self, label, jpeg, timestamp=None):
        with self._lock:
            if not self._active: raise ValueError('recorder is not active')
            if self._paused: return False
            if label not in self.schema.cameras: raise ValueError('undeclared camera label')
            stamp = self._timestamp('camera:' + label, timestamp)
            if not isinstance(jpeg, (bytes, bytearray)) or not (jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9')):
                raise ValueError('image must contain JPEG bytes')
            self._camera.write_image(label, bytes(jpeg), stamp, time.time())
            self._last['camera:' + label] = stamp
            return True

    def append_sensor(self, sensor_key, sample, timestamp=None):
        with self._lock:
            if not self._active: raise ValueError('recorder is not active')
            if self._paused: return False
            declared = {str(s['sensor_key']) for s in self.schema.sensor_specs}
            if sensor_key not in declared: raise ValueError('undeclared sensor')
            stamp = self._timestamp('sensor:' + sensor_key, timestamp)
            self._robot.write_sensor(sensor_key, sample, stamp, time.time())
            self._last['sensor:' + sensor_key] = stamp
            return True

    def event(self, name, data, timestamp=None):
        with self._lock:
            if not self._active: raise ValueError('recorder is not active')
            json.dumps(data, allow_nan=False)
            stamp = time.time() if timestamp is None else float(timestamp)
            if not np.isfinite(stamp): raise ValueError('event timestamp must be finite')
            self._meta['events'].append({'name': str(name), 'data': deepcopy(data), 't': stamp})

    def pause(self):
        with self._lock:
            self.event('pause', True)
            self._paused = True

    def resume(self):
        with self._lock:
            self.event('pause', False)
            self._paused = False

    def stop(self, *, cancelled=False):
        with self._lock:
            if not self._active: raise ValueError('recorder is not active')
            self._meta['end_time_wall'] = time.time()
            self._meta['duration_s'] = self._meta['end_time_wall'] - self._meta['start_time_wall']
            try:
                counts = self._robot.close()
                cameras = self._camera.close()
                self._meta['counts'] = counts
                self._meta['camera_counts'] = cameras
                missing = [key for key in self.schema.required_streams() if counts.get(key, 0) == 0]
                missing += ['camera:' + key for key in self.schema.cameras if cameras.get(key, 0) == 0]
                missing += ['sensor:' + str(spec['sensor_key']) for spec in self.schema.sensor_specs
                            if spec.get('required', False) and counts.get('sensor:' + str(spec['sensor_key']), 0) == 0]
                if cancelled:
                    self._meta['recording_status'] = 'cancelled'
                elif missing:
                    self._meta['recording_status'] = 'failed'
                    self._meta['failure'] = 'required streams are empty: ' + ', '.join(missing)
                elif self._meta['recording_status'] != 'failed':
                    self._meta['recording_status'] = 'complete'
            except Exception as exc:
                self._meta['recording_status'] = 'failed'
                self._meta['failure'] = str(exc)
                raise
            finally:
                self._active = False
                self._write_meta()
            return {'episode_dir': str(self.episode_dir), 'counts': counts,
                    'camera_counts': cameras, 'status': self._meta['recording_status']}


class AsyncComposableRecorder:
    """Bounded recorder queue: gateway callbacks never perform HDF5 or disk IO.

    Start and stop are lifecycle operations. All sample/event calls enqueue
    immutable snapshots using put_nowait. Overflow permanently fails this episode
    rather than silently producing a successful, incomplete training artifact.
    """
    def __init__(self, capture_schema, session_dir, queue_size=1024):
        from queue import Queue
        if type(queue_size) is not int or queue_size < 1: raise ValueError('invalid recorder queue size')
        self._writer = ComposableRecorder(capture_schema, session_dir)
        self.schema = self._writer.schema
        self._queue = Queue(maxsize=queue_size)
        self._thread = None
        self._failure = None
        self._drops = 0
        self._active = False
        self._lock = RLock()

    @property
    def episode_dir(self): return self._writer.episode_dir

    @property
    def status(self):
        return {'active': self._active, 'failure': self._failure, 'dropped_samples': self._drops,
                'queued_samples': self._queue.qsize()}

    def start(self, task=''):
        from threading import Thread
        with self._lock:
            if self._active: raise ValueError('recording already active')
            directory = self._writer.start(task)
            self._failure, self._drops = None, 0
            self._active = True
            self._thread = Thread(target=self._run, name='morla-record-writer', daemon=True)
            self._thread.start()
            return directory

    def _enqueue(self, method, *args, **kwargs):
        from queue import Full
        # Freeze capture time before any queue lag; source ns remains separately
        # supplied by the adapter so wall and monotonic domains are never mixed.
        positions = {'append_stream': 2, 'append_camera': 2, 'append_sensor': 2,
                     'event': 2, 'append_snapshot': 1}
        if method in positions:
            position = positions[method]
            args = list(args)
            if len(args) > position:
                if args[position] is None: args[position] = time.time()
            elif kwargs.get('timestamp') is None:
                kwargs['timestamp'] = time.time()
        with self._lock:
            if not self._active: raise ValueError('recorder is not active')
            try:
                self._queue.put_nowait((method, deepcopy(args), deepcopy(kwargs)))
            except Full:
                self._drops += 1
                self._failure = self._failure or 'RECORDER_QUEUE_OVERFLOW'
                return False
            return True

    def _run(self):
        while True:
            item = self._queue.get()
            try:
                if item is None: return
                method, args, kwargs = item
                try:
                    getattr(self._writer, method)(*args, **kwargs)
                except Exception as exc:
                    with self._lock:
                        self._failure = self._failure or f'{method}: {type(exc).__name__}: {exc}'
            finally:
                self._queue.task_done()

    def append_stream(self, *args, **kwargs): return self._enqueue('append_stream', *args, **kwargs)
    def append_snapshot(self, *args, **kwargs): return self._enqueue('append_snapshot', *args, **kwargs)
    def append_camera(self, *args, **kwargs): return self._enqueue('append_camera', *args, **kwargs)
    def append_sensor(self, *args, **kwargs): return self._enqueue('append_sensor', *args, **kwargs)
    def event(self, *args, **kwargs): return self._enqueue('event', *args, **kwargs)
    def pause(self): return self._enqueue('pause')
    def resume(self): return self._enqueue('resume')

    def stop(self, *, cancelled=False, timeout=10.):
        from queue import Full
        with self._lock:
            if not self._active: raise ValueError('recorder is not active')
            self._active = False
        # FIFO sentinel follows all accepted samples. Blocking is restricted to
        # explicit lifecycle shutdown, outside the gateway's command callbacks.
        try:
            self._queue.put(None, timeout=timeout)
        except Full as exc:
            self._failure = self._failure or 'RECORDER_DRAIN_TIMEOUT'
            raise TimeoutError(self._failure) from exc
        self._thread.join(timeout=timeout)
        if self._thread.is_alive():
            self._failure = self._failure or 'RECORDER_DRAIN_TIMEOUT'
            raise TimeoutError(self._failure)
        if self._failure:
            self._writer._meta['recording_status'] = 'failed'
            self._writer._meta['failure'] = self._failure
        self._writer._meta['dropped_samples'] = self._drops
        result = self._writer.stop(cancelled=cancelled)
        result.update(dropped_samples=self._drops, failure=self._failure)
        return result
