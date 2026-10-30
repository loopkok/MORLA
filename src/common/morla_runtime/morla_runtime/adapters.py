"""Reference fake adapters. They never claim physical measurements."""
import copy


class FakeAdapter:
    backend = 'fake'

    def __init__(self, instance):
        self.instance = copy.deepcopy(instance)
        self.calls = []
        self.values = [0.0] * len(instance.get('joints', []))
        self.active = False
        self.configured = False

    def configure(self, context):
        self.calls.append(('configure', context['session_id']))
        self.configured = True

    def read(self, now_ns):
        ports = {}
        for name, definition in self.instance.get('ports', {}).items():
            if definition.get('direction') != 'output':
                continue
            sample = {'sample_ns': now_ns, 'source': 'fake', 'measured': False, 'synthetic': True, 'valid': self.configured}
            if definition.get('semantic') == 'image':
                sample.update(data=_FAKE_JPEG, encoding='jpeg', width=8, height=8)
            else:
                dim = int(definition.get('dim', len(self.values)))
                sample['values'] = list(self.values) if name == 'position' and len(self.values) == dim else [0.0] * dim
                sample['unit'] = definition.get('unit', '')
            ports[name] = sample
        return {'ports': ports, 'sample_ns': now_ns, 'source': 'fake', 'synthetic': True, 'measured': False}

    def health(self, now_ns):
        ports = {key: value for key, value in self.read(now_ns)['ports'].items() if 'data' not in value}
        return {'ready': self.configured, 'sample_ns': now_ns, 'source': 'fake',
                'measured': False, 'synthetic': True, 'values': list(self.values), 'ports': ports}

    def prepare(self):
        self.calls.append(('prepare',))
        return True

    def activate(self):
        self.calls.append(('activate',))
        self.active = True
        return True

    def submit(self, command):
        if not self.active:
            raise RuntimeError('ADAPTER_DISARMED')
        self.calls.append(('submit', copy.deepcopy(command)))
        self.values = list(command['values'])
        return {'accepted': True}

    def stop(self):
        self.calls.append(('stop',))
        self.active = False
        return True

    def operate(self, operation):
        self.calls.append(('operation', operation))
        if operation == 'disable':
            self.active = False
        return True

    def close(self):
        self.calls.append(('close',))
        self.configured = False


class Ros2ControlFakeAdapter(FakeAdapter):
    """Contract prototype: joint-state/trajectory mapping, no physical simulator."""
    def trajectory(self, command):
        return {'joint_names': list(command['names']), 'points': [
            {'positions': list(command['values']), 'time_from_start_ns': 0}]}


class LatestMailbox:
    """A bounded latest-value mailbox; never replay a backlog of motion."""
    def __init__(self):
        import threading
        self._lock = threading.Lock()
        self.value = None
        self.dropped = 0

    def put(self, value):
        with self._lock:
            if self.value is not None:
                self.dropped += 1
            self.value = value

    def take(self):
        with self._lock:
            value, self.value = self.value, None
            return value


# Small deterministic protocol fixture, explicitly synthetic; no camera dependency.
import base64
_FAKE_JPEG = base64.b64decode('/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/2wBDAQgJCQwLDBgNDRgyIRwhMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjL/wAARCAAIAAgDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDzmiiivVPMP//Z')
