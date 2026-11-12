"""One connection per robot, latest commands, cached measured state, bounded stop."""
import copy
import threading
import time
from morla_teleop.kinematics import limit_joint_step
from .contract import CONNECTION_FIELDS, ROLES, COMMAND_RPC_TIMEOUT_DEFAULT_S, goal_velocity, role_of, tracks_goal_velocity, validate_configuration, validate_target, limits
from .feedback import decode
try:from morla_core.cadence_probe import record as _cadence
except ImportError:
    def _cadence(*args,**kwargs):pass


class DeviceSession:
    def __init__(self, rows, clock=time.monotonic_ns, transport_factory=None, diagnostics=None):
        self.rows=copy.deepcopy(rows);self.clock=clock;self.diagnostics=diagnostics
        self.lock=threading.RLock();self.io=threading.RLock();self.lifecycle=threading.RLock()
        self.shutdown=threading.Event();self.samples={};self.device_stamps={};self.last_sent={};self.pending={};self.desired={};self.active=set();self.command_times={}
        self.fault='';self.stop_error='';self.started=False;self.closed=False;self.threads=[];self.transport=None;self.captures=[]
        # Dispatch telemetry: how often goals actually reach the controller is the
        # real teleop rate limit, so expose it instead of inferring it.
        self.writes=0;self.write_hz=None;self.last_write=None
        # Recent goal-target speed per component, for goal_velocity_tracking.
        self.rates={}
        # Measured joints found (tolerably) outside the command envelope when a
        # component was armed; reported by snapshot() instead of blocking.
        self.start_outside={}
        roles=[];signature=None;backend=None
        for row in self.rows.values():
            validate_configuration(row)
            sig=tuple(row['parameters'].get(k) for k in CONNECTION_FIELDS)
            if signature is not None and (signature!=sig or backend!=row['backend']):
                raise ValueError('MMK2_SHARED_CONNECTION_CONFIGURATION_MISMATCH')
            signature=sig;backend=row['backend']
            role=role_of(row)
            if role!='camera':
                if role in roles:raise ValueError('MMK2_DUPLICATE_COMPONENT')
                roles.append(role)
        self.p=next(iter(self.rows.values()))['parameters'];self.backend=backend
        self.command_rpc_timeout_s=self.p.get('command_rpc_timeout_s',COMMAND_RPC_TIMEOUT_DEFAULT_S)
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
            sample['dispatch']={'writes':self.writes,
                                'rate_hz':None if self.write_hz is None else round(self.write_hz,2),
                                'goal_rtt_s':None if self.transport is None else getattr(self.transport,'last_goal_rtt_s',None)}
            sample['start_outside_limits']=copy.deepcopy(self.start_outside.get(key))
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
                self._check_start_pose(key,row,sample['values'])
            with self.lock:
                self.active.add(key);self.pending.pop(key,None);self.desired.pop(key,None);self.last_sent.pop(key,None)

    def _check_start_pose(self,key,row,values):
        """Reject only a start pose that is *grossly* outside the command envelope.

        The measured pose is not a command. A joint that settled or was nudged a
        few degrees past a soft range still accepts commands -- both controllers
        clip their output to these same limits -- and refusing to activate there
        would block every path that could bring it back, teleop included. So a
        small excursion is recorded in snapshot()['start_outside_limits'] and
        activation proceeds. Anything wider than 10% of the joint range is the
        signature of a wrong joint order/sign/offset, not of a resting arm, and
        still fails.
        """
        role=role_of(row)
        try:
            validate_target(row,{'names':ROLES[role][1],'values':values,'mode':'position','units':ROLES[role][2]})
        except ValueError as exc:
            if str(exc)!='MMK2_SDK_TARGET_OUTSIDE_LIMITS':raise
            low,high=limits(row)
            joints=[(n,v,lo,hi) for n,v,lo,hi in zip(ROLES[role][1],values,low,high) if not lo<=v<=hi]
            gross=[(n,round(v,4),lo,hi) for n,v,lo,hi in joints if v<lo-.1*(hi-lo) or v>hi+.1*(hi-lo)]
            if gross:raise RuntimeError(f'MMK2_START_POSE_OUTSIDE_LIMITS: {gross}') from None
            with self.lock:self.start_outside[key]=[(n,round(v,4),lo,hi) for n,v,lo,hi in joints]
        else:
            with self.lock:self.start_outside.pop(key,None)

    def submit(self,key,command):
        row=self.rows[key]
        measured=None
        if role_of(row)!='base':
            with self.lock:measured=list(self.samples.get(key,{}).get('values') or [])
        q=validate_target(row,command,measured or None)
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

    def release_target(self, key):
        with self.lock:
            self.pending.pop(key, None)
            self.desired.pop(key, None)
            self.command_times.pop(key, None)
            if key in self.active and role_of(self.rows[key]) == 'base':
                self.pending[key] = [0., 0.]

    def _worker(self):
        period=1/self.p['rate_hz']
        next_tick = time.monotonic() + period
        # The rate limit is per unit of wall-clock time, not per loop iteration:
        # set_goal itself takes ~1 s on the real controller, so the loop runs far
        # below rate_hz and a nominal-period step would throttle commands ~20x.
        last_send = time.monotonic()
        while not self.shutdown.wait(max(0., next_tick - time.monotonic())):
            # Count RPC time toward the period, without sending catch-up bursts.
            next_tick = time.monotonic() + period
            try:
                with self.io:
                    targets={};probe={};cruises={}
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
                                # A resting base must not ride along on every
                                # request: the controller zeroes a base that
                                # hears nothing for 250 ms anyway, so only the
                                # transition to zero needs sending. Keeping it
                                # cost ~100 ms per request on the real
                                # controller and cut the arm rate from ~18 Hz
                                # to ~6 Hz (2026-11-13, zero-displacement probe).
                                if not any(goal) and not any(self.last_sent.get(key,(0.,0.))):
                                    continue
                                q=goal
                            else:
                                measured=self.samples[key]['values'];start=self.last_sent.get(key,measured)
                                elapsed=max(period,time.monotonic()-last_send);step=p['max_velocity']*elapsed
                                limited,_=limit_joint_step(start,goal,step)
                                q=limited.tolist()
                                lag=max(abs(v-m) for v,m in zip(q,measured))
                                if lag>p['max_command_lead']:raise RuntimeError('MMK2_COMMAND_LEAD_EXCEEDED')
                                advance=max(abs(v-s) for v,s in zip(q,start))/elapsed
                                if tracks_goal_velocity(row):
                                    rate=max(advance,self.rates.get(key,0.))
                                    self.rates[key]=rate if advance>rate else .7*rate+.3*advance
                                    # Each forward_position goal is "move here at this
                                    # speed, then stop", so a cruise above the target's
                                    # own speed parks the arm until the next write. At
                                    # 33 Hz writes and 0.007 rad steps that stop is ~1/3
                                    # of every period: a buzz at the write rate, in
                                    # teleop and in the pure-interpolation work position
                                    # alike. 20% headroom makes each goal last about one
                                    # write interval; a lagging arm still gets a fast
                                    # goal (lag/0.25 s) so this cannot trip the lead gate.
                                    cap=goal_velocity(row)
                                    # Floor: the first write has no rate history yet,
                                    # and a zero cruise is not a meaningful goal.
                                    cruises[key]=min(cap,max(cap*.1,1.2*rate,lag/.25))
                                # Per-write record: the delivered rate and step are the
                                # real teleop smoothness limit, and nothing else observes
                                # them (the ik.* probe only samples feedback at 10 Hz).
                                probe[key]={'q':[round(v,5) for v in q],
                                            'm':[round(v,5) for v in measured],
                                            'step':round(max(abs(v-s) for v,s in zip(q,start)),5),
                                            'clip':round(max(abs(v-g) for v,g in zip(q,goal)),5),
                                            'v':round(cruises.get(key,goal_velocity(row)),4)}
                            # Validate mapped hardware limits before sending too.
                            validate_target(row,{'names':[j['name'] for j in row['joints']],'mode':'velocity' if role_of(row)=='base' else 'position','units':ROLES[role_of(row)][2],'values':q},
                                            None if role_of(row)=='base' else measured)
                            targets[key]=q
                    if targets:
                        # A timeout may still have reached the controller.
                        # Record attempted writes so fault stop covers them.
                        with self.lock:
                            self.last_sent.update(targets)
                            now=time.monotonic()
                            if self.last_write is not None and now>self.last_write:
                                rate=1/(now-self.last_write)
                                self.write_hz=rate if self.write_hz is None else .8*self.write_hz+.2*rate
                            self.last_write=now;self.writes+=1
                        last_send=time.monotonic()
                        started=time.monotonic()
                        self.transport.write(targets,self.rows,cruises)
                        _cadence('dispatch',counts=('write',),
                                 timings=(('goal_rtt',int((time.monotonic()-started)*1e9)),),
                                 event={'components':sorted(targets),'arms':probe} if probe else None)
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
