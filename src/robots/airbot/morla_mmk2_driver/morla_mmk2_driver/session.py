"""One connection per robot, latest commands, cached measured state, bounded stop."""
import copy
import threading
import time
from .contract import CONNECTION_FIELDS, ROLES, role_of, validate_configuration, validate_target, limits
from .feedback import decode


class DeviceSession:
    def __init__(self, rows, clock=time.monotonic_ns, transport_factory=None, diagnostics=None):
        self.rows=copy.deepcopy(rows);self.clock=clock;self.diagnostics=diagnostics
        self.lock=threading.RLock();self.io=threading.RLock();self.lifecycle=threading.RLock()
        self.shutdown=threading.Event();self.samples={};self.device_stamps={};self.last_sent={};self.pending={};self.desired={};self.active=set();self.command_times={}
        self.fault='';self.stop_error='';self.started=False;self.closed=False;self.threads=[];self.transport=None;self.captures=[]
        roles=[];signature=None;backend=None
        for row in self.rows.values():
            validate_configuration(row)
            sig=tuple(row['parameters'][k] for k in CONNECTION_FIELDS)
            if signature is not None and (signature!=sig or backend!=row['backend']):
                raise ValueError('MMK2_SHARED_CONNECTION_CONFIGURATION_MISMATCH')
            signature=sig;backend=row['backend']
            role=role_of(row)
            if role!='camera':
                if role in roles:raise ValueError('MMK2_DUPLICATE_COMPONENT')
                roles.append(role)
        self.p=next(iter(self.rows.values()))['parameters'];self.backend=backend
        if transport_factory is None:
            from .fake import FakeTransport
            from .transport import Transport
            transport_factory=Transport if backend=='real' else FakeTransport
        self.transport_factory=transport_factory

    def start(self):
        with self.lifecycle:
            if self.started:return
            if self.closed:raise RuntimeError('MMK2_SESSION_CLOSED')
            self.transport=self.transport_factory(self.p)
            try:
                self.transport.open()
                for fn in (self._receive,self._worker):
                    thread=threading.Thread(target=fn,daemon=True,name='mmk2-'+fn.__name__)
                    self.threads.append(thread);thread.start()
                actuators=[k for k,r in self.rows.items() if role_of(r)!='camera']
                end=time.monotonic()+self.p['startup_timeout_s']
                while True:
                    with self.lock:
                        ready=all(k in self.samples for k in actuators)
                        fault=self.fault
                    if fault:raise RuntimeError(fault)
                    if ready:break
                    if time.monotonic()>=end:raise TimeoutError('MMK2_FIRST_VALID_FEEDBACK_TIMEOUT')
                    self.shutdown.wait(.01)
                self.started=True
            except Exception:
                self.close();raise

    def _receive(self):
        try:
            for state in self.transport.states():
                if self.shutdown.is_set():return
                now=self.clock();decoded={}
                for key,row in self.rows.items():
                    if role_of(row)=='camera':continue
                    stamp,values,ports=decode(state,row)
                    with self.lock:
                        previous=self.device_stamps.get(key,-1)
                    if stamp<previous:raise ValueError('MMK2_DEVICE_CLOCK_REGRESSED')
                    if stamp==previous:continue # duplicates never refresh host freshness
                    if stamp<=0:raise ValueError('MMK2_DEVICE_TIMESTAMP_MISSING')
                    for name,port in ports.items():
                        port.update(valid=True,sample_ns=now,source_ns=stamp,unit=row['ports'][name]['unit'])
                    decoded[key]=(stamp,dict(values=values,ports=ports,sample_ns=now,source_ns=stamp,ready=True))
                with self.lock:
                    for key,(stamp,sample) in decoded.items():
                        self.device_stamps[key]=stamp;self.samples[key]=sample
            if not self.shutdown.is_set():raise RuntimeError('MMK2_STATE_STREAM_ENDED')
        except Exception as exc:
            if not self.shutdown.is_set():self._fault(exc)

    def _fault(self, exc):
        # Serialize against an in-flight command: fallback must be the last write.
        with self.io:
            with self.lock:
                if self.fault:return
                self.fault=str(exc);had_motion=bool(self.active or self.last_sent)
                self.active.clear();self.pending.clear();self.desired.clear()
            if had_motion:
                try:self._hold()
                except Exception as stop_exc:self.stop_error=str(stop_exc)
            if self.diagnostics:
                try:self.diagnostics('device_fault',{'driver':'mmk2_sdk','reason':self.fault,'stop_error':self.stop_error})
                except Exception:pass

    def snapshot(self,key):
        with self.lock:
            sample=copy.deepcopy(self.samples.get(key,{'sample_ns':0,'ports':{},'values':[]}))
            ready=bool(sample['sample_ns']) and not self.fault and 0<=self.clock()-sample['sample_ns']<=self.p['feedback_timeout_s']*1e9
            sample.update(ready=ready,fault=bool(self.fault),reason=self.fault,stop_error=self.stop_error,source='hardware' if self.backend=='real' else 'fake',measured=self.backend=='real' and bool(sample['sample_ns']),synthetic=self.backend=='fake',software_dispatch_authorized=key in self.active,motor_enabled_verified=False,physical_stop_verified=False)
            sample['device_status']={'registration':copy.deepcopy(getattr(self.transport,'registration',{})),'motor_status':'unsupported_by_sdk'}
            for port in sample['ports'].values():
                port.update(valid=ready,source=sample['source'],measured=sample['measured'],synthetic=sample['synthetic'])
            return sample

    def activate(self,key):
        with self.io:
            if self.backend=='real' and not all(self.rows[key]['parameters'][k] for k in ('allow_motion','mapping_verified','hold_stop_accepted')):
                raise RuntimeError('MMK2_READ_ONLY: allow_motion, mapping_verified, hold_stop_accepted must be explicitly verified')
            sample=self.snapshot(key)
            if not sample['ready']:raise RuntimeError('MMK2_FEEDBACK_NOT_READY')
            row=self.rows[key];role=role_of(row)
            if role!='base':
                validate_target(row,{'names':ROLES[role][1],'values':sample['values'],'mode':'position','units':ROLES[role][2]})
            with self.lock:
                self.active.add(key);self.pending.pop(key,None);self.desired.pop(key,None);self.last_sent.pop(key,None)

    def submit(self,key,command):
        q=validate_target(self.rows[key],command)
        with self.lock:
            if self.fault:raise RuntimeError('MMK2_DEVICE_FAULT: '+self.fault)
            if key not in self.active:raise RuntimeError('ADAPTER_DISARMED')
            sample=self.snapshot(key)
            if not sample['ready']:raise RuntimeError('MMK2_FEEDBACK_NOT_READY')
            row=self.rows[key]
            if role_of(row)!='base' and max(abs(v-m) for v,m in zip(q,sample['values']))>row['parameters']['max_command_lead']:
                raise ValueError('MMK2_COMMAND_LEAD_EXCEEDED')
            self.pending[key]=q;self.command_times[key]=self.clock()
        return {'accepted':True,'completed':False,'phase':'queued'}

    def _worker(self):
        period=1/self.p['rate_hz']
        while not self.shutdown.wait(period):
            try:
                with self.io:
                    targets={}
                    with self.lock:
                        if self.fault:continue
                        for key in list(self.active):
                            if not self.snapshot(key)['ready']:raise TimeoutError('MMK2_FEEDBACK_STALE')
                            if key in self.pending:self.desired[key]=self.pending.pop(key)
                            if key not in self.desired:continue
                            row=self.rows[key];p=row['parameters'];goal=self.desired[key]
                            if role_of(row)=='base':
                                if self.clock()-self.command_times.get(key,0)>p['command_timeout_s']*1e9:
                                    goal=[0.,0.];self.desired[key]=goal
                                q=goal
                            else:
                                measured=self.samples[key]['values'];start=self.last_sent.get(key,measured);step=p['max_velocity']*period
                                q=[a+max(-step,min(step,b-a)) for a,b in zip(start,goal)]
                                if max(abs(v-m) for v,m in zip(q,measured))>p['max_command_lead']:raise RuntimeError('MMK2_COMMAND_LEAD_EXCEEDED')
                            # Validate mapped hardware limits before sending too.
                            validate_target(row,{'names':[j['name'] for j in row['joints']],'mode':'velocity' if role_of(row)=='base' else 'position','units':ROLES[role_of(row)][2],'values':q})
                            targets[key]=q
                    if targets:
                        # A timeout may still have reached the controller.
                        # Record attempted writes so fault stop covers them.
                        with self.lock:self.last_sent.update(targets)
                        self.transport.write(targets,self.rows)
            except Exception as exc:self._fault(exc)

    def _hold(self):
        # Software hold request only. No motor-disable or physical-stop claim.
        with self.lock:
            targets={k:([0.,0.] if role_of(self.rows[k])=='base' else list(q)) for k,q in self.last_sent.items()}
        self.transport.write(targets,self.rows)

    def stop(self):
        with self.io:
            with self.lock:
                self.active.clear();self.pending.clear();self.desired.clear()
            try:self._hold()
            except Exception as exc:
                self.stop_error=str(exc)
                self._fault(exc)
                raise
        return True

    def close(self):
        with self.lifecycle:
            if self.closed:return
            self.shutdown.set()
            error=None
            try:
                if self.transport is not None:self.stop()
            except Exception as exc:error=exc
            finally:
                for capture in self.captures:
                    try:capture._close_capture()
                    except Exception as exc:error=error or exc
                if self.transport is not None:self.transport.close()
            for thread in self.threads:
                if thread is not threading.current_thread():thread.join(timeout=2*self.p['rpc_timeout_s']+.5)
            if any(t.is_alive() for t in self.threads):raise TimeoutError('MMK2_WORKER_DID_NOT_EXIT')
            self.closed=True
            if error:raise error
