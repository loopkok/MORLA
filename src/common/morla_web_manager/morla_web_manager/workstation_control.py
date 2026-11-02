"""Shared workstation operator commands, ownership and direct joint work positions."""
import copy
import math
from collections import OrderedDict



class WorkstationControl:
    def __init__(self, service):
        self.service = service
        self._runtime = None
        self.activity = 'paused'
        self.paused = True
        self.gate = None
        self.interrupted = None
        self.return_to = None
        self.playback = None
        self.work = None
        self.holds = {}
        self.hold_sequences = OrderedDict()
        self.requests = OrderedDict()
        self.error = None
        from .handoff import Handoff
        self.handoff = Handoff(self)

    @property
    def runtime(self): return self._runtime if self._runtime is not None else self.service.session

    def state(self):
        return {'activity': self.activity, 'paused':self.paused, 'return_to': self.return_to, 'interrupted':copy.deepcopy(self.interrupted),
                'transition':self.handoff.state(), 'error': self.error, 'hold_active':self.held(),
                'work_position': None if self.work is None else {k:v for k,v in self.work.items() if k not in ('targets','start')},
                'playback': None if self.playback is None else {'index':self.playback['index'],
                    'count':len(self.playback['actions']), 'paused':self.activity!='playback'}}

    def hold(self, client, held, *, stamp=None, ttl_ns=350_000_000):
        if type(held) is not bool: raise ValueError('held must be boolean')
        previous=client in self.holds
        if held:
            if len(self.holds)>=64 and client not in self.holds: raise ValueError('too many hold sources')
            self.holds[client] = (self.service.clock() if stamp is None else stamp,ttl_ns)
        else: self.holds.pop(client, None)
        if held!=previous:
            self.runtime.trace('operator_hold',{'source':client,'held':held,'source_ns':stamp,'ttl_ns':ttl_ns})
        return {'held':held}

    def held(self):
        now = self.service.clock()
        expired=[k for k,v in self.holds.items() if not 0<=now-v[0]<=v[1]]
        self.holds = {k:v for k,v in self.holds.items() if 0 <= now-v[0] <= v[1]}
        for client in expired:self.runtime.trace('operator_hold',{'source':client,'held':False,'reason':'expired'})
        return bool(self.holds)

    def keyboard_hold(self, client, held, sequence):
        if type(sequence) is not int or not 0<=sequence<=2**53-1:
            raise ValueError('invalid keyboard hold sequence')
        if sequence<=self.hold_sequences.get(client,-1):
            return {'accepted':False,'reason':'stale_hold'}
        result=self.hold(client,held)
        self.hold_sequences[client]=sequence
        self.hold_sequences.move_to_end(client)
        if len(self.hold_sequences)>128:self.hold_sequences.popitem(last=False)
        return {'accepted':True,**result}

    def dispatch(self, action, request_id):
        if not isinstance(request_id,str) or not 1 <= len(request_id) <= 200:
            raise ValueError('request_id required')
        if request_id in self.requests:
            old, result = self.requests[request_id]
            if old != action: raise ValueError('request_id reused for different action')
            return copy.deepcopy(result)
        self.runtime.trace('operator_request', {'action':action,'request_id':request_id})
        try:
            result = self._dispatch(action, request_id)
            if result.get('status')!='FAILED':self.error = None
        except Exception as exc:
            self.error = str(exc)
            self.runtime.trace('operator_result', {'action':action,'request_id':request_id,'error':str(exc)})
            raise
        self.runtime.trace('operator_result', {'action':action,'request_id':request_id,'result':result})
        self.requests[request_id] = (action, copy.deepcopy(result))
        if len(self.requests)>512: self.requests.popitem(last=False)
        return result

    def _dispatch(self, action, request_id):
        svc, rt = self.service, self.runtime
        if rt.status in ('STOPPED','STOPPING','FAULTED','ESTOP_LATCHED'):
            raise ValueError('workstation not ready: '+rt.status)
        if self.handoff.pending and action!='teleop_pause':
            raise ValueError('控制权切换正在进行，请等待或暂停/停止')
        if action == 'teleop_start':
            if self.work and self.work['status']=='RUNNING': raise ValueError('正在移动到工作位，请等待到位或暂停遥操')
            return self.handoff.begin('human',request_id)
        if action == 'teleop_pause':
            return svc.operation(rt.session_id, {'operation':'disarm','request_id':request_id})
        if action == 'resume':
            if self.activity not in ('policy','playback') or not self.paused:
                raise ValueError('没有可继续的已暂停策略/回放')
            return self.handoff.begin(self.activity,request_id,kind='resume')
        if action in ('record_or_release','record_start'):
            if self.return_to:
                return self.handoff.begin(self.return_to,request_id,desired_paused=self.interrupted['paused'],kind='release')
            if self.activity in ('policy','playback'):
                raise ValueError('请先人工接管，再开始人工录制')
            if svc.recorder: return {'status':'already_recording'}
            return svc.start_recording(rt.session_id,'operator demonstration')
        if action == 'record_stop': return svc.stop_recording()
        if action == 'record_discard': return svc.discard_recording()
        if action == 'workpos': return self.start_work_position(request_id)
        raise ValueError('unsupported operator action: '+action)

    def guard(self, command):
        """Runs inside Runtime.submit, never acquires the service lock."""
        rt=self.runtime
        if self.gate is None or self.handoff.pending or self.paused or rt.status!='ACTIVE':
            raise RuntimeError('WORKSTATION_OUTPUT_FROZEN')
        lease=rt.lease
        if (command.get('producer')!=self.gate or not lease or lease['producer']!=self.gate or
                command.get('epoch')!=lease['epoch'] or command.get('token')!=lease['token'] or
                self.service.clock()>lease['expires_ns'] or
                not {item.get('component_id') for item in command.get('commands',[])}<=set(lease['resources'])):
            raise RuntimeError('WORKSTATION_OWNERSHIP_MISMATCH')

    def attach(self):
        self._runtime=self.service.session
        self.runtime.command_guard=self.guard

    def device_activated(self):
        if self.handoff.pending or self.activity in ('policy','playback','workpos') or self.return_to:
            raise ValueError('请通过开始遥操操作切换当前控制权')
        self.activity='human';self.paused=False;self.gate='human_mobile'

    def pause(self):
        if self.handoff.pending:self.handoff.abort('操作员暂停',restore=False)
        self.gate=None;self.paused=True;self.holds.clear()
        if self.service.policy and not self.service.policy.closed:self.service.policy.suspend(revoke=False)
        if self.work and self.work['status']=='RUNNING':
            self.work['status']='CANCELLED';self.activity='paused'

    def stop(self):
        self.pause();self.playback=None;self.return_to=None;self.interrupted=None;self.activity='paused'

    def reanchor_playback(self, feedback, bounds):
        p=self.playback
        if p is None or p['index']>=len(p['actions']):raise ValueError('回放已经结束')
        base=p['actions'][p['index']]
        offset=[0.]*len(base)
        schema=self.runtime.resolved['capture_schema']
        for block in schema['action_blocks']:
            key=block['writer_id'];indices=block.get('stream_indices') or list(range(block['dim']))
            for action_index,joint_index in zip(block['action_indices'],indices):
                offset[action_index]=feedback[key][joint_index]-base[action_index]
        # Never clip an offset trajectory silently: reject an out-of-limits
        # remainder so a failed release leaves HUMAN in charge.
        for block in schema['action_blocks']:
            joints=self.runtime.instances[block['writer_id']]['joints']
            indices=block.get('stream_indices') or list(range(block['dim']))
            for action_index,joint_index in zip(block['action_indices'],indices):
                joint=joints[joint_index]
                if (bounds['minimum'][action_index]+offset[action_index]<joint.get('lower',-math.inf) or
                        bounds['maximum'][action_index]+offset[action_index]>joint.get('upper',math.inf)):
                    raise ValueError('重锚后的剩余回放超出关节限位，请保持人工控制')
        return offset

    def start_playback(self, raw):
        from morla_policy_inference.composable import RuntimeReplay, action_commands
        rt = self.runtime
        if self.handoff.pending:raise ValueError('控制权切换正在进行')
        if self.service.policy and not self.service.policy.closed: raise ValueError('policy already active')
        if self.playback or self.return_to: raise ValueError('automated controller already active')
        if self.work and self.work['status']=='RUNNING': raise ValueError('正在移动到工作位，请等待到位或暂停遥操')
        replay = RuntimeReplay(rt, raw['frozen_resolved'])
        actions = copy.deepcopy(raw['actions'])
        hz = raw.get('hz',30)
        if (not isinstance(actions,list) or not 1 <= len(actions) <= 100000 or
                type(hz) not in (int,float) or not math.isfinite(hz) or not 1 <= hz <= 200):
            raise ValueError('invalid playback actions/rate')
        for action in actions: action_commands(rt.resolved,action)
        self.gate=None
        replay.permit = rt.switch('playback')
        self.playback = {'replay':replay,'actions':actions,'index':0,'period_ns':int(1e9/hz),'next_ns':self.service.clock(),'offset':[0.]*len(actions[0])}
        self.activity='playback'; self.paused=False;self.gate='playback'; self.service._reset_bindings()
        return self.state()

    def start_work_position(self, request_id):
        rt, svc = self.runtime, self.service
        if svc.recorder: raise ValueError('请先结束或丢弃本段录制，再回工作位')
        if self.work and self.work['status']=='RUNNING': raise ValueError('正在移动到工作位，请等待到位或暂停遥操')
        config = rt.resolved.get('source_assembly',{}).get('metadata',{}).get('operator_control',{})
        if not isinstance(config,dict): raise ValueError('invalid operator_control configuration')
        targets = copy.deepcopy(config.get('work_positions',{}))
        speed = config.get('work_speed',.15)
        if type(speed) not in (int,float) or not math.isfinite(speed) or not 0 < speed <= 1:
            raise ValueError('work_speed must be in (0,1] joint units/s')
        if not isinstance(targets,dict) or not targets: raise ValueError('请先在 Web 遥操作设置中配置并保存工作位')
        start = {}
        duration = 0.
        # Freeze and validate every target before activating any device.
        for key, values in targets.items():
            instance = rt.instances.get(key,{})
            joints = instance.get('joints',[])
            if instance.get('kind')!='actuator' or not joints or not isinstance(values,list) or len(values)!=len(joints):
                raise ValueError('invalid work position component/dimensions: '+key)
            for value,joint in zip(values,joints):
                if (type(value) not in (int,float) or not math.isfinite(value) or
                        not joint.get('lower',-math.inf) <= value <= joint.get('upper',math.inf)):
                    raise ValueError('work position outside joint limits: '+key)
                velocity=joint.get('max_velocity',speed)
                if type(velocity) not in (int,float) or not math.isfinite(velocity) or velocity<=0:
                    raise ValueError('invalid joint velocity limit: '+key)
            health = rt.adapters[key].health(svc.clock())
            if health.get('ready') is not True or health.get('fault') or not 0 <= svc.clock()-health.get('sample_ns',-10**20) <= rt.max_age_ns:
                raise ValueError('work position requires fresh ready feedback: '+key)
            measured = health.get('values')
            if not isinstance(measured,list) or len(measured)!=len(joints) or not all(math.isfinite(v) for v in measured):
                raise ValueError('work position feedback dimensions: '+key)
            start[key] = measured
            duration=max(duration,*(abs(b-a)/min(speed,joint.get('max_velocity',speed))
                         for a,b,joint in zip(measured,values,joints)))
        self.gate=None
        svc.stop_policy(rt.session_id); self.playback=None; self.return_to=None;self.interrupted=None
        svc._reset_bindings(); rt.revoke(reason='work_position')
        operation = rt.activate(request_id+'-activate',list(targets))
        now = svc.clock()
        self.work={'status':'RUNNING','targets':targets,'start':start,'duration_ns':max(1,int(duration*1e9)),
                   'started_ns':None,'elapsed_ns':0,'last_ns':now,'requested_ns':now,
                   'activation_id':request_id+'-activate','deadline_ns':now+int((duration+30)*1e9)}
        self.activity='workpos';self.paused=False; self.holds.clear()
        rt.trace('work_position',{'phase':'requested','targets':targets,'speed':speed,'activation':operation})
        return {'status':'RUNNING'}

    def tick(self):
        rt, svc = self.runtime, self.service
        now = svc.clock()
        self.handoff.poll()
        if self.handoff.pending:return
        if self.activity=='playback' and self.playback and not self.paused:
            p=self.playback
            if now>=p['next_ns']:
                receipt=p['replay'].send([float(v)+d for v,d in zip(p['actions'][p['index']],p['offset'])],sample_ns=now)
                if not receipt.get('accepted'): raise RuntimeError('playback command rejected: '+str(receipt))
                p['index']+=1; p['next_ns']=now+p['period_ns']
                if p['index']==len(p['actions']):
                    rt.revoke(reason='playback_completed'); self.playback=None; self.activity='paused';self.paused=True;self.gate=None
        if not self.work or self.work['status']!='RUNNING': return
        w=self.work
        if now>w['deadline_ns']: raise RuntimeError('work position timed out')
        operation=rt.operation(w['activation_id'])
        if operation['status'] in ('RUNNING','DISPATCHED'): return
        if operation['status']!='SUCCEEDED' or rt.status!='ACTIVE': raise RuntimeError('work position activation failed/cancelled')
        if w['started_ns'] is None:w['started_ns']=now;w['last_ns']=now
        # Do not jump across a scheduler stall by replaying elapsed trajectory time.
        w['elapsed_ns']+=min(100_000_000,max(0,now-w['last_ns']));w['last_ns']=now
        ratio=min(1.,w['elapsed_ns']/w['duration_ns'])
        commands=[];arrived=True
        for key,target in w['targets'].items():
            health=rt.adapters[key].health(now)
            if health.get('ready') is not True or health.get('fault') or not 0<=now-health.get('sample_ns',-10**20)<=rt.max_age_ns:
                raise RuntimeError('work position feedback lost: '+key)
            measured=health.get('values',[])
            if len(measured)!=len(target) or not all(math.isfinite(v) for v in measured): raise RuntimeError('invalid work position feedback')
            arrived=arrived and all(abs(a-b)<=.02 for a,b in zip(measured,target))
            joints=rt.instances[key]['joints']
            commands.append({'component_id':key,'names':[j['name'] for j in joints],
                'units':[j['unit'] for j in joints],'mode':'position',
                'values':[a+ratio*(b-a) for a,b in zip(w['start'][key],target)]})
        if self.gate!='workpos':
            w['permit']=rt.acquire('workpos');self.gate='workpos'
        w['permit']=rt.renew('workpos',w['permit']['token'])
        svc.sequence+=1
        receipt=rt.submit({'session_id':rt.session_id,'producer':'workpos','sequence':svc.sequence,
                           'issued_ns':now,'commands':commands,'epoch':w['permit']['epoch'],'token':w['permit']['token']})
        if not receipt.get('accepted'): raise RuntimeError('work position command rejected')
        if ratio==1 and arrived:
            w['status']='SUCCEEDED'; self.activity='paused';self.paused=True;self.gate=None
            rt.revoke(reason='work_position_arrived')
            rt.trace('work_position',{'phase':'arrived'})

    def fail(self, exc):
        if self.handoff.pending:self.handoff.abort(str(exc),restore=False)
        self.error=str(exc)
        self.gate=None;self.paused=True
        if self.work and self.work['status']=='RUNNING':self.work['status']='FAILED'
        self.activity='paused';self.return_to=None;self.playback=None;self.holds.clear()
        self.runtime.revoke(reason='operator_motion_failed')
        self.runtime.trace('operator_error',{'reason':str(exc)})
