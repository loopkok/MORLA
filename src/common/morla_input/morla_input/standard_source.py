"""Common ROS intent payload for any upstream device; no downstream identity."""
import json
from .adapters.normalized import InputStream


class StandardRosSource:
    def __init__(self, instance, *, clock, on_transition):
        self.source=InputStream(instance['instance_id'],clock=clock)
        self.topic=instance.get('parameters',{}).get('input_topic',instance['namespace']+'/intent')
        self.on_transition=on_transition
        self.epoch=None
        self._actions=[]

    def consume(self, text):
        row=json.loads(text)
        self._actions=list(row.get('actions',[]))
        if row.get('connected',True) is False:
            self.source.disconnect(); self.on_transition(); return None
        if not self.source.connected or self.epoch!=row['epoch']:
            self.on_transition(); self.source.reconnect(); self.epoch=row['epoch']
        return self.source.accept(row['ports'],row['sequence'],epoch=self.source.epoch,
            frame_id=row.get('frame_id','input'),source_time_ns=row.get('source_time_ns'))

    def control_event(self, frame): return frame is not None
    @property
    def actions(self): return self._actions


def create(instance, *, clock, on_transition):
    return StandardRosSource(instance,clock=clock,on_transition=on_transition)
