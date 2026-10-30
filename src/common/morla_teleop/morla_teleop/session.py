"""One control host for headless ROS and embedded applications."""
from .engine import BindingEngine


class ControlHost:
    def __init__(self, runtime, *, start_recording=None, stop_recording=None):
        self.runtime=runtime
        self.engine=BindingEngine(runtime)
        self.sequence=0
        self.inputs=None
        self.errors=[]
        self.start_recording,self.stop_recording=start_recording,stop_recording

    def ingest(self, frame):
        adapter=self.runtime.adapters.get(frame.source_id)
        if hasattr(adapter,'feed'): adapter.feed(frame)
        if self.runtime.status!='ACTIVE':
            self.engine.reset()
            return {'accepted':False,'reason':'explicit_activation_required'}
        targets=self.engine.targets(frame)
        if not targets:return {'accepted':False,'reason':'no_targets'}
        self.sequence+=1
        return self.runtime.submit({'session_id':self.runtime.session_id,'producer':'human',
            'sequence':self.sequence,'issued_ns':self.runtime.clock(),'commands':targets})

    def event(self,input_id,frame,actions):
        from .actions import dispatch_input_action
        for action in actions:
            request='input-'+input_id+'-'+frame.source_epoch+'-'+str(frame.sequence)+'-'+action
            dispatch_input_action(action,request,operation=self.runtime.invoke,
                start_recording=self.start_recording,stop_recording=self.stop_recording)

    def connect(self, manager_factory=None):
        if self.inputs is not None:return
        if manager_factory is None:
            from morla_input.ros_sources import RosInputManager
            manager_factory=RosInputManager
        manager=manager_factory([i for i in self.runtime.instances.values() if i['kind']=='input'],
            self.ingest,lambda source: self.engine.reset(),clock=self.runtime.clock,on_event=self.event)
        try:manager.start()
        except Exception:
            manager.close();raise
        self.inputs=manager

    def close(self):
        if self.inputs is not None:
            try:self.inputs.close()
            except Exception as exc:self.errors.append(str(exc))
            finally:self.inputs=None
        self.errors.extend(self.engine.close())
