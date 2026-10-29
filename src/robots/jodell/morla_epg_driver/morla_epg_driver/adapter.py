"""EPG cached worker with explicit moving activation and confirmed software stop."""
import copy
import threading
import time
from .protocol import MODEL,STROKE_M,number


class EPGAdapter:
    backend='real'

    def __init__(self,instance,context,transport_factory=None):
        self.instance=copy.deepcopy(instance);self.parameters=instance['parameters']
        self.clock=context['clock'];self.factory=transport_factory;self.transport=None
        self.lock=threading.RLock();self.io=threading.Lock()
        self.shutdown=threading.Event();self.cancel=threading.Event()
        self.thread=None;self.active=False;self.owned=False;self.activating=False
        self.pending=None;self.last_command=None;self.fault=None
        self.sample=None;self.stop_error=None
        self.deployment_verified=bool(instance.get('deployment_verified'))
        # The manual reports a communication fault after 1 s, but does not
        # prove physical stop on link loss. Require independent commissioning.
        self.watchdog_verified=(instance.get('watchdog_verified') is True and
                               self.parameters.get('device_watchdog_verified') is True)

    def configure(self,context):
        if self.thread is not None:raise RuntimeError('EPG already configured')
        p=self.parameters
        if p['model']!=MODEL:raise ValueError('Unsupported EPG model')
        if not isinstance(p['port'],str) or not p['port'].startswith('/dev/serial/by-id/'):
            raise ValueError('EPG requires stable /dev/serial/by-id/ port')
        if type(p['slave_id']) is not int or not 1<=p['slave_id']<=247:raise ValueError('Invalid slave_id')
        if p['baudrate'] not in (4800,9600,19200,38400,57600,115200):raise ValueError('Invalid baudrate')
        number(p['poll_hz'],1,50,'poll_hz');number(p['timeout_s'],.01,.5,'timeout_s')
        number(p['command_timeout_s'],.05,1,'command_timeout_s')
        number(p['feedback_timeout_s'],.05,1,'feedback_timeout_s')
        number(p['enable_timeout_s'],.1,60,'enable_timeout_s');number(p['stop_timeout_s'],.1,3,'stop_timeout_s')
        number(p['max_step_m'],.0001,STROKE_M,'max_step_m')
        from .protocol import byte
        byte(p['speed_raw'],'speed_raw');byte(p['force_raw'],'force_raw')
        if self.factory is None:
            from .transport import EPGTransport
            self.factory=EPGTransport
        try:
            self.transport=self.factory(p)
            # Opening the device is read-only: no enable/reset/stop/mode changes.
            with self.io:self._read()
            self.thread=threading.Thread(target=self._run,name='epg-io',daemon=True);self.thread.start()
        except Exception:
            if self.transport is not None:self.transport.close()
            raise

    def _read(self):
        sample=self.transport.read()
        if sample['fault_code']&~1:raise RuntimeError('EPG fault 0x%02x'%sample['fault_code'])
        if not sample['parameter_mode']:raise RuntimeError('EPG must be in parameter mode')
        with self.lock:
            if self.active and (sample['fault_code'] or not sample['enabled'] or sample['activation_state']!=3):
                raise RuntimeError('EPG lost activation')
            self.sample=sample
        return sample

    def _run(self):
        period=1/self.parameters['poll_hz'];next_tick=time.monotonic()
        while not self.shutdown.is_set():
            try:
                with self.io:
                    with self.lock:
                        if (self.active and self.last_command is not None and
                            self.clock()-self.last_command>self.parameters['command_timeout_s']*1e9):
                            raise TimeoutError('EPG command watchdog expired')
                        target=self.pending;self.pending=None
                        if self.cancel.is_set() or not self.active:target=None
                    if target is not None:self.transport.write(target)
                    self._read()
            except Exception as exc:
                with self.lock:self.fault=str(exc);self.active=False;self.pending=None
                try:self.stop()
                except Exception as stop_exc:self.stop_error=str(stop_exc)
                return  # No auto-reconnect or auto-resume after failure.
            next_tick+=period
            if next_tick<time.monotonic():next_tick=time.monotonic()+period
            self.shutdown.wait(max(0,next_tick-time.monotonic()))

    def health(self,now_ns):
        with self.lock:
            sample=copy.deepcopy(self.sample);fault=self.fault
            active=self.active;activating=self.activating
        if sample is None:return {'ready':False,'sample_ns':0,'source':'unavailable','measured':False,'values':[]}
        stamp=sample['sample_ns'];fresh=0<=now_ns-stamp<=self.parameters['feedback_timeout_s']*1e9
        measured=self.parameters['position_feedback_verified'] is True
        source='hardware_verified' if measured else 'device_reported_unverified_position'
        def port(values,unit,verified=True):
            return {'values':values,'unit':unit,'sample_ns':stamp,'received_ns':stamp,'valid':fresh and not fault,
                    'source':source if not verified else 'hardware','measured':measured if not verified else True,'synthetic':False}
        ports={'position':port([sample['width_m']],'m',False),
               'closure_ratio':port([sample['closure_ratio']],'ratio',False),
               'speed_setting':port([sample['speed_setting_ratio']],'ratio',False),
               'motor_current':port([sample['motor_current_ratio']],'ratio'),
               'voltage':port([sample['bus_voltage_v']],'V'),
               'temperature':port([sample['temperature_c']],'degC'),
               'status':port(sample['raw_registers'],'count')}
        ports['speed_setting']['measured']=False
        ports['speed_setting']['source']='device_setting_echo'
        return {'ready':fresh and not fault,'fault':bool(fault),'reason':fault,'sample_ns':stamp,
                'source':source,'measured':measured,'synthetic':False,'values':[sample['width_m']],
                'ports':ports,'device_status':sample,'active':active,'activating':activating,'stop_error':self.stop_error}

    def read(self,now_ns):return self.health(now_ns)

    def prepare(self):
        if not self.health(self.clock())['ready']:raise RuntimeError('EPG fresh healthy status required')
        return True  # Readiness only; does not run the initialization stroke.

    def activate(self):
        if not (self.deployment_verified and self.watchdog_verified and
                self.parameters['activation_motion_verified'] is True and
                self.parameters['position_feedback_verified'] is True):
            raise RuntimeError('EPG deployment, activation stroke, position and watchdog unverified')
        self.prepare()
        with self.io:
            with self.lock:
                if self.active or self.activating:raise RuntimeError('EPG already active/activating')
                self.cancel.clear();self.owned=True;self.activating=True
            # Initialization may move! Mark owned before sending, even on lost ACK.
            try:self.transport.enable()
            except Exception:
                with self.lock:self.activating=False
                raise
        deadline=time.monotonic()+self.parameters['enable_timeout_s']
        try:
            while time.monotonic()<deadline:
                if self.cancel.is_set() or self.shutdown.is_set():raise RuntimeError('EPG activation cancelled')
                with self.io:
                    if self.cancel.is_set():raise RuntimeError('EPG activation cancelled')
                    state=self._read()
                if state['enabled'] and state['activation_state']==3 and not state['go_to'] and not state['fault_code']:
                    with self.lock:
                        if self.cancel.is_set():raise RuntimeError('EPG activation cancelled')
                        self.active=True;self.pending=None;self.last_command=None
                    return True
                self.cancel.wait(.02)
            raise TimeoutError('EPG activation feedback unconfirmed')
        finally:
            with self.lock:self.activating=False

    def submit(self,target):
        from morla_runtime.session import RuntimeSession
        RuntimeSession._validate_target(target,self.instance)
        with self.lock:
            if not self.active or self.cancel.is_set() or not self.health(self.clock())['ready']:
                raise RuntimeError('EPG disarmed/stale/faulted')
            value=target['values'][0]
            if abs(value-self.sample['width_m'])>self.parameters['max_step_m']:
                raise ValueError('EPG target exceeds measured width step')
            self.pending=value;self.last_command=self.clock()
        return {'accepted':True,'completed':False,'phase':'queued'}

    def stop(self):
        self.cancel.set()
        with self.lock:self.active=False;self.pending=None;self.last_command=None
        with self.io:
            if not self.owned or self.transport is None:return True
            self.transport.stop()
            deadline=time.monotonic()+self.parameters['stop_timeout_s']
            # An ACK alone is not a stop confirmation. Never infer stopped from
            # speed_raw=0 (it means minimum speed). Object status alone also fails.
            while time.monotonic()<deadline:
                state=self._read()
                if state['enabled'] and state['activation_state']==3 and not state['go_to'] and not state['fault_code']:
                    with self.lock:self.owned=False;self.activating=False
                    return True
                time.sleep(.02)
            raise TimeoutError('EPG stop feedback unconfirmed')

    def close(self):
        self.shutdown.set();self.cancel.set()
        try:
            if self.transport is not None:self.stop()
        finally:
            if self.thread is not None:
                self.thread.join(timeout=2)
                if self.thread.is_alive():raise TimeoutError('EPG worker did not close')
            if self.transport is not None:self.transport.close()


def create_adapter(instance,context):
    if instance['backend']=='fake':
        from .fake import EPGFakeAdapter
        return EPGFakeAdapter(instance)
    if instance['backend']!='real':raise ValueError('EPG backend unsupported')
    return EPGAdapter(instance,context)
