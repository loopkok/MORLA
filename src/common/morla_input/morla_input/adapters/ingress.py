"""A sampled intent provider, fed by an authenticated network or ROS adapter."""
from copy import deepcopy
import threading


class InputIngressAdapter:
    def __init__(self,instance,clock):
        self.instance,self.clock=instance,clock
        self.lock=threading.Lock()
        self.configured=False
        self.frame=None

    def configure(self,context): self.configured=True
    def close(self): self.configured=False; self.frame=None
    def stop(self): self.frame=None; return True
    def feed(self,frame):
        if frame.source_id!=self.instance['instance_id']: raise ValueError('wrong input source')
        with self.lock: self.frame=deepcopy(frame)
    def health(self,now_ns):
        # Listener readiness does not grant movement. Each binding independently
        # checks input freshness and requires a release edge after reconnect.
        return {'ready':self.configured,'sample_ns':now_ns,'source':'input_listener',
                'measured':False,'synthetic':False,'values':[]}
    def read(self,now_ns):
        with self.lock:
            if self.frame is None: return {**self.health(now_ns),'ports':{}}
            frame=self.frame
            ports={}
            for key,value in frame.ports.items():
                if isinstance(value,dict) and {'position','quaternion'}<=set(value):
                    value=[*value['position'],*value['quaternion']]
                elif isinstance(value,dict) and key=='axes':
                    order=('x','y','z','roll','pitch','yaw') if 'x' in value else ('left_x','left_y','right_x','right_y','left_trigger','right_trigger')
                    if not all(name in value for name in order): continue
                    value=[value[name] for name in order]
                elif isinstance(value,list) and value and isinstance(value[0],list):
                    value=[v for row in value for v in row]
                if isinstance(value,(list,tuple)):
                    ports[key]={'values':list(value),'valid':frame.valid,**(frame.port_metadata or {}).get(key,{})}
                elif isinstance(value,(float,int)):
                    ports[key]={'values':[float(value)],'valid':frame.valid}
            return {'ready':self.configured,'sample_ns':frame.received_ns,'source':'input_device',
                    'measured':False,'synthetic':False,'source_epoch':frame.source_epoch,
                    'sequence':frame.sequence,'ports':ports}
