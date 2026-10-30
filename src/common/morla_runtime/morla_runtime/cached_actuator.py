"""Cached, single-thread device I/O shared by ROS and composable MORLA."""
import copy
import math
import threading


class CachedActuatorAdapter:
    backend = 'real'
    feedback_source = 'hardware'
    feedback_measured = True
    supports_watchdog = True

    def __init__(self, instance, context, transport_factory=None):
        self.instance = copy.deepcopy(instance)
        self.clock = context['clock']
        self.parameters = instance['parameters']
        self.factory = transport_factory
        self.transport = None
        self.io = threading.Lock()
        self.lock = threading.Lock()
        self.shutdown = threading.Event()
        self.thread = None
        self.active = False
        self.pending = None
        self.sample = {'ready':False,'sample_ns':0,'source':'unavailable',
                       'measured':False,'synthetic':False,'values':[]}

    def validate_configuration(self):
        """Device subclasses validate deployment parameters without connecting."""

    def make_transport(self, parameters):
        raise NotImplementedError('Device transport factory required')

    def configure(self, context):
        if self.thread is not None:
            raise RuntimeError('Already configured')
        self.validate_configuration()
        try:
            self.transport = (self.factory or self.make_transport)(self.parameters)
            self.thread = threading.Thread(target=self._run,daemon=True)
            self.thread.start()
        except Exception:
            if self.transport: self.transport.close()
            raise

    def _run(self):
        while not self.shutdown.is_set():
            try:
                with self.io:
                    with self.lock:
                        pending,self.pending = self.pending,None
                    if pending is not None:
                        self.transport.write(pending)
                    feedback = self.transport.read()
                    values = feedback['values'] if isinstance(feedback,dict) else feedback
                    if len(values)!=len(self.instance['joints']) or not all(math.isfinite(v) for v in values):
                        raise ValueError('Invalid device feedback')
                    stamp=feedback.get('sample_ns',self.clock()) if isinstance(feedback,dict) else self.clock()
                    with self.lock:
                        self.sample={'ready':True,'sample_ns':stamp,
                                     'source':self.feedback_source,
                                     'measured':self.feedback_measured,'synthetic':False,'values':list(values),
                                     'ports':{'position':{'values':list(values),'valid':True,'received_ns':stamp}}}
                        if isinstance(feedback,dict):
                            self.sample['device_status']={k:v for k,v in feedback.items() if k not in ('values','sample_ns')}
            except Exception as exc:
                with self.lock:
                    was_active=self.active
                    self.active=False; self.pending=None
                    self.sample.update(ready=False,fault=True,reason=str(exc))
                if was_active:
                    try:
                        with self.io: self.transport.stop()
                    except Exception as stop_error:
                        with self.lock: self.sample['stop_error']=str(stop_error)
                return
            self.shutdown.wait(1/self.parameters['rate_hz'])

    def health(self, now_ns):
        with self.lock:
            result=copy.deepcopy(self.sample)
        result['ready'] = result['ready'] and 0<=now_ns-result['sample_ns']<=500_000_000
        return result

    def read(self, now_ns):
        return self.health(now_ns)

    def prepare(self):
        if not self.health(self.clock())['ready']: raise RuntimeError('Fresh feedback required')
        return True

    def activate(self):
        with self.io:
            self.prepare()
            self.transport.activate()
            with self.lock:
                # Wait for an explicitly authorized first target; activation is not a command stream.
                self.active=True; self.pending=None
        return True

    def submit(self, target):
        with self.lock:
            if not self.active:
                raise RuntimeError('ADAPTER_DISARMED')
            self.pending=list(target['values'])
        return {'accepted':True,'completed':False,'phase':'queued'}

    def _stop_locked(self):
        with self.lock:
            self.active=False; self.pending=None
        self.transport.stop()

    def stop(self):
        if self.transport is not None:
            with self.io: self._stop_locked()
        return True

    def close(self):
        self.shutdown.set()
        if self.thread is not None:
            self.thread.join(timeout=2)
            if self.thread.is_alive(): raise TimeoutError('Device worker did not exit')
        if self.transport is not None:
            try:
                # No control opcode on closing a never-activated read-only connection.
                if self.active: self.stop()
            finally: self.transport.close()
