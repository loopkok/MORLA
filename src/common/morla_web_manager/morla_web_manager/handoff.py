"""Event-driven transactional handoffs. Staged controllers never command devices."""
import copy
import math
import threading
from concurrent.futures import Future


class Handoff:
    def __init__(self, operator):
        self.op=operator
        self.pending=None
        self.last=None
        self.slots=threading.BoundedSemaphore(4)

    @property
    def svc(self):return self.op.service
    @property
    def rt(self):return self.op.runtime

    def state(self):
        p=self.pending or self.last
        if p is None:return None
        return {k:copy.deepcopy(p[k]) for k in ('request_id','kind','phase','status','deadline_ns','acks','error') if k in p}

    def feedback(self):
        now=self.svc.clock();result={}
        for key in self.rt._actuators():
            health=self.rt.adapters[key].health(now)
            values=health.get('values')
            if (health.get('ready') is not True or health.get('fault') or
                    type(health.get('sample_ns')) is not int or not 0<=now-health['sample_ns']<=self.rt.max_age_ns or
                    not isinstance(values,(list,tuple)) or len(values)!=len(self.rt.instances[key]['joints']) or
                    any(type(v) not in (int,float) or not math.isfinite(v) for v in values)):
                raise ValueError('接管需要新鲜且有效的设备反馈：'+key)
            result[key]=list(values)
        return result

    def _snapshot(self):
        return {'activity':self.op.activity,'paused':self.op.paused,'return_to':self.op.return_to,
                'interrupted':copy.deepcopy(self.op.interrupted),'runtime_status':self.rt.status,
                'bindings':self.svc.bindings}

    def begin(self, destination, request_id, *, desired_paused=False, kind='takeover'):
        if self.pending:raise ValueError('控制权切换正在进行，请等待完成或暂停/停止')
        if self.rt._pending_operation is not None:raise ValueError('device operation pending')
        seeds=self.feedback()
        jobs=[]
        if destination=='human':
            for binding in self.rt.resolved.get('bindings',[]):
                source=self.svc.inputs.get(binding['source'].split('.',1)[0])
                frame=source.read() if source is not None else None
                if frame is None:raise ValueError('重锚缺少新鲜输入：'+binding['id'])
                jobs.append({'binding':copy.deepcopy(binding),'frame':copy.deepcopy(frame),'future':None})
        elif destination=='policy' and (self.svc.policy is None or self.svc.policy.closed):
            raise ValueError('先前接管的策略已不可用')
        elif destination=='playback' and self.op.playback is None:
            raise ValueError('先前接管的回放已不可用')
        snapshot=self._snapshot()
        p={'request_id':request_id,'kind':kind,'destination':destination,'desired_paused':desired_paused,
           'phase':'reanchoring' if destination=='human' else 'preparing','status':'RUNNING',
           # A real device enable is not instant: the CR5 needs ~5 s to release
           # its brakes before it acknowledges. Bound the wait, do not race it.
           'deadline_ns':self.svc.clock()+15_000_000_000,'acks':{},'jobs':jobs,'seeds':seeds,
           'snapshot':snapshot,'staged':{},'prepared':None,'prediction':None,'activation':None,'replay_bounds':None}
        p['stage_prepared']=False
        self.pending=p
        self.rt.trace('handoff',{'phase':'requested','request_id':request_id,'source':snapshot['activity'],
                                  'destination':destination,'source_paused':snapshot['paused']})
        try:
            self.op.gate=None
            if self.svc.policy and not self.svc.policy.closed:self.svc.policy.suspend(revoke=False)
            # Retire the old epoch immediately; enable/hold state is retained.
            self.rt.acquire('_handoff')
        except Exception:
            self.abort('控制权切换准备失败',restore=True)
            raise
        self.poll()
        return {'status':p['status'],'transition':self.state()}

    def _launch(self, job, seeds, *, building=False):
        from morla_teleop.engine import BindingEngine
        if not self.slots.acquire(blocking=False):return
        future=Future();job['build_future' if building else 'future']=future
        def run():
            controller=None
            try:
                binding=job['binding'];frame=job['frame']
                engine=BindingEngine(self.rt,bindings={})
                if building:
                    controller=engine._binding_controller(binding)
                    future.set_result({'controller':controller})
                    return
                controller=job['build_future'].result()['controller']
                if not engine._input_basis_matches(binding,frame):raise ValueError('reanchor input basis mismatch')
                ack=controller.reanchor(frame,seeds[binding['target'].split('.',1)[0]])
                future.set_result({'controller':controller,'ack':ack,'frame':frame})
            except BaseException as exc:
                if controller is not None:
                    try:controller.close()
                    except Exception:pass
                future.set_exception(exc)
            finally:self.slots.release()
        threading.Thread(target=run,daemon=True,name='morla-reanchor').start()

    def _validate(self, p):
        feedback=self.feedback()
        for key,seed in p['seeds'].items():
            if any(abs(a-b)>.02 for a,b in zip(seed,feedback[key])):
                raise ValueError('重锚期间设备姿态已改变，请重新开始：'+key)
        for key,seed in p.get('freeze_seeds',{}).items():
            if any(abs(a-b)>.02 for a,b in zip(seed,feedback[key])):
                raise ValueError('接管期间设备姿态已改变，请重新开始：'+key)
        for job in p['jobs']:
            binding=job['binding'];controller=p['staged'][binding['id']]
            if not controller.input_fresh(job['frame']):raise ValueError('重锚输入在确认前过期：'+binding['id'])
            source=self.svc.inputs.get(job['frame'].source_id)
            latest=source.read() if source is not None else None
            if latest is None or not controller.input_fresh(latest) or latest.source_epoch!=job['frame'].source_epoch:
                raise ValueError('重锚期间输入已断开或切换：'+binding['id'])
            if latest.input_basis_id!=job['frame'].input_basis_id:
                raise ValueError('重锚期间输入坐标系声明已改变')
            port=binding['source'].split('.',1)[1]
            old=(job['frame'].port_metadata or {}).get(port,{}).get('frame_id',job['frame'].frame_id)
            new=(latest.port_metadata or {}).get(port,{}).get('frame_id',latest.frame_id)
            if old!=new:raise ValueError('重锚期间输入坐标系已改变')
        return feedback

    def _replay_bounds(self, p):
        if not self.slots.acquire(blocking=False):return
        future=Future();p['replay_bounds']=future
        replay=self.op.playback;index=replay['index']
        def run():
            try:
                minimum=list(replay['actions'][index]);maximum=list(minimum)
                for row_index in range(index,len(replay['actions'])):
                    if row_index%1000==0 and p['status']!='RUNNING':raise RuntimeError('retired replay validation')
                    for i,value in enumerate(replay['actions'][row_index]):
                        minimum[i]=min(minimum[i],value);maximum[i]=max(maximum[i],value)
                future.set_result({'minimum':minimum,'maximum':maximum})
            except BaseException as exc:future.set_exception(exc)
            finally:self.slots.release()
        threading.Thread(target=run,daemon=True,name='morla-replay-reanchor').start()

    def poll(self):
        p=self.pending
        if p is None:return
        try:
            if self.svc.clock()>=p['deadline_ns']:raise TimeoutError('控制权切换确认超时')
            if self.rt.status in ('FAULTED','ESTOP_LATCHED','STOPPED','STOPPING'):raise RuntimeError('设备状态不允许提交控制权')
            # Device enable can be asynchronous. Confirm it before sampling the
            # actual robot/input anchor or starting a fresh resume prediction.
            if not p['desired_paused'] and (self.rt.status!='ACTIVE' or p['activation'] is not None):
                if p['activation'] is None:
                    p['phase']='activating'
                    p['activation']=self.rt.activate(p['request_id']+'-handoff-enable')['request_id']
                operation=self.rt.operation(p['activation'])
                if operation['status']=='RUNNING':return
                if operation['status']!='SUCCEEDED':raise RuntimeError('设备激活未确认：'+operation['status'])
            if not p['stage_prepared']:
                p['seeds']=self.feedback()
                p['freeze_seeds']=copy.deepcopy(p['seeds'])
                for job in p['jobs']:
                    source=self.svc.inputs.get(job['frame'].source_id)
                    frame=source.read() if source is not None else None
                    if frame is None or frame.source_epoch!=job['frame'].source_epoch:
                        raise ValueError('设备准备期间输入已断开或切换')
                    job['frame']=copy.deepcopy(frame)
                if p['destination']=='policy' and not p['desired_paused']:
                    p['prediction']=self.svc.policy.prepare_resume()
                p['stage_prepared']=True
                p['phase']='reanchoring' if p['destination']=='human' else 'preparing'
            # Construct every plugin before starting the age-limited anchor
            # phase. A fast mapping must not expire behind a cold IK model.
            for job in p['jobs']:
                if job.get('build_future') is None:self._launch(job,p['seeds'],building=True)
            if any(job.get('build_future') is None or not job['build_future'].done() for job in p['jobs']):return
            for job in p['jobs']:job['build_future'].result()
            if p['jobs'] and not p.get('anchor_sampled'):
                feedback=self.feedback()
                if any(abs(a-b)>.02 for key,seed in p['seeds'].items() for a,b in zip(seed,feedback[key])):
                    raise ValueError('控制器准备期间设备姿态已改变，请重新开始')
                frames={}
                for job in p['jobs']:
                    old=job['frame'];iid=old.source_id
                    if iid not in frames:
                        source=self.svc.inputs.get(iid)
                        frames[iid]=copy.deepcopy(source.read()) if source is not None else None
                    frame=frames[iid]
                    if frame is None or frame.source_epoch!=old.source_epoch or frame.input_basis_id!=old.input_basis_id:
                        raise ValueError('控制器准备期间输入已断开或切换')
                    port=job['binding']['source'].split('.',1)[1]
                    if ((old.port_metadata or {}).get(port,{}).get('frame_id',old.frame_id)!=
                            (frame.port_metadata or {}).get(port,{}).get('frame_id',frame.frame_id)):
                        raise ValueError('控制器准备期间输入坐标系已改变')
                    job['frame']=frame
                p['seeds']=feedback;p['anchor_sampled']=True
            for job in p['jobs']:
                if job['future'] is None:self._launch(job,p['seeds'])
                future=job['future']
                if future is not None and future.done() and job['binding']['id'] not in p['staged']:
                    try:result=future.result()
                    except Exception as exc:raise RuntimeError('重锚失败 '+job['binding']['id']+': '+str(exc)) from exc
                    # Keep the job frame equal to the one the controller anchored
                    # on; _validate re-checks freshness against exactly that frame.
                    job['frame']=result.get('frame',job['frame'])
                    p['staged'][job['binding']['id']]=result['controller']
                    p['acks'][job['binding']['id']]=result['ack']
                    self.rt.trace('handoff',{'phase':'binding_ack','request_id':p['request_id'],
                        'ack':result['ack'],'input_epoch':job['frame'].source_epoch,
                        'input_sequence':job['frame'].sequence,'feedback_seed':p['seeds'][job['binding']['target'].split('.',1)[0]]},
                        binding_id=job['binding']['id'])
            if len(p['staged'])!=len(p['jobs']):return
            if p['destination']=='playback':
                if p['replay_bounds'] is None:self._replay_bounds(p)
                if p['replay_bounds'] is None or not p['replay_bounds'].done():return
                p['replay_bounds'].result()
            if p['prediction'] is not None:
                if not p['prediction'].done():return
                p['prepared']=p['prediction'].result()
                if not 0<=self.svc.clock()-p['prepared']['sample_ns']<self.svc.policy.max_age_ns:
                    raise ValueError('交还策略的首次预测已过期')
            self._validate(p)
            self._commit(p)
        except Exception as exc:self.abort(str(exc),restore=True)

    def _commit(self, p):
        snapshot=p['snapshot'];destination=p['destination']
        feedback=self._validate(p)
        offset=self.op.reanchor_playback(feedback,p['replay_bounds'].result()) if destination=='playback' else None
        if p['desired_paused']:
            self.rt.revoke(reason='handoff_restored_pause');permit=None
            if self.rt.status!='READY_DISARMED':raise RuntimeError('无法确认恢复暂停状态')
        else:permit=self.rt.acquire('human_mobile' if destination=='human' else destination)
        if destination=='policy' and not p['desired_paused']:
            self.svc.policy.resume(permit,p['prepared'])
            self.svc.last_policy_ns=0
        if destination=='human':
            self.svc.bindings=p['staged']
            if snapshot['activity'] in ('policy','playback'):
                self.op.return_to=snapshot['activity']
                self.op.interrupted={'activity':snapshot['activity'],'paused':snapshot['paused']}
        if destination=='playback':
            self.op.playback['offset']=offset
            self.op.playback['replay'].permit=permit
            self.op.playback['next_ns']=self.svc.clock()
        if destination!='human':
            self.op.return_to=None;self.op.interrupted=None
            self.svc._reset_bindings()
        self.op.activity=destination;self.op.paused=p['desired_paused']
        self.op.gate=None if self.op.paused else ('human_mobile' if destination=='human' else destination)
        self.op.error=None
        p.update(status='SUCCEEDED',phase='committed')
        self.last=p;self.pending=None
        if destination=='human':self._cleanup_bindings(snapshot['bindings'])
        self.rt.trace('handoff',{'phase':'committed','request_id':p['request_id'],'destination':destination,
                                 'paused':self.op.paused,'acks':p['acks']})

    @staticmethod
    def _cleanup_bindings(bindings):
        def close():
            for controller in bindings.values():
                try:controller.close()
                except Exception:pass
        threading.Thread(target=close,daemon=True,name='morla-retired-controllers').start()

    def _retire(self, p):
        if p['prediction'] is not None:p['prediction'].cancel()
        for job in p['jobs']:
            future=job['future'] or job.get('build_future')
            if future is None:continue
            def cleanup(future):
                try:self._cleanup_bindings({'staged':future.result()['controller']})
                except BaseException:pass
            future.add_done_callback(cleanup)

    def abort(self, reason, *, restore):
        p=self.pending
        if p is None:return None
        self.pending=None;self.op.gate=None
        p.update(status='FAILED' if restore else 'CANCELLED',phase='rolled_back' if restore else 'cancelled',error=reason)
        self.last=p;self._retire(p)
        snapshot=p['snapshot'];self.op.error=reason
        self.op.activity=snapshot['activity'];self.op.paused=snapshot['paused']
        self.op.return_to=snapshot['return_to'];self.op.interrupted=snapshot['interrupted']
        self.svc.bindings=snapshot['bindings']
        try:
            if restore and snapshot['runtime_status']=='ACTIVE' and self.rt.status=='ACTIVE':
                self.feedback()  # Never resume an old owner against lost/faulted devices.
                permit=self.rt.acquire('human_mobile' if snapshot['activity']=='human' else snapshot['activity'])
                if snapshot['activity']=='policy' and not snapshot['paused']:
                    self.svc.policy.resume(permit);self.svc.last_policy_ns=0
                if snapshot['activity']=='playback' and self.op.playback:
                    self.op.playback['replay'].permit=permit;self.op.playback['next_ns']=self.svc.clock()
                self.op.gate=None if snapshot['paused'] else ('human_mobile' if snapshot['activity']=='human' else snapshot['activity'])
            else:
                if self.rt.status not in ('FAULTED','ESTOP_LATCHED','STOPPED','STOPPING'):
                    self.rt.revoke(reason='handoff_cancelled')
                self.op.paused=True
        except Exception as exc:
            p['phase']='rollback_failed'
            self.op.gate=None;self.op.paused=True;self.op.error=reason+'; rollback failed: '+str(exc)
            self.rt.revoke(reason='handoff_rollback_failed')
        self.rt.trace('handoff',{'phase':p['phase'],'request_id':p['request_id'],'reason':self.op.error})
        return snapshot
