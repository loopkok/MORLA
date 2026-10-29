"""Fresh, source-aware events reuse GamepadAdapter and its InputStream fencing."""
from dataclasses import replace
import json
import math
import time
from morla_input.adapters.normalized import GamepadAdapter,InputError
from .mapping import BUTTONS,AXES,configuration


def vector(value,size,lo,hi):
    if not isinstance(value,list) or len(value)!=size or any(type(v) not in (int,float) or
        not math.isfinite(v) or not lo<=v<=hi for v in value):raise InputError('Invalid Vader vector')
    return value


class Vader5Source(GamepadAdapter):
    def __init__(self,source_id,*,parameters=None,backend='real',clock=time.monotonic_ns,on_transition=None):
        self.parameters=parameters or {};self.deadman,self.rotation,self.action_map=configuration(self.parameters)
        super().__init__(source_id,deadzone=self.parameters.get('deadzone',.08),clock=clock)
        self.max_age_ns=int(self.parameters.get('max_age_ms',250)*1e6)
        self.backend=backend;self.on_transition=on_transition or (lambda:None)
        self.upstream=None;self.upstream_sequence=-1;self.retired=set()
        self.previous=set();self.released=set();self.actions=[];self.control_tick=False

    def disconnect(self):
        super().disconnect();self.previous=set();self.released=set();self.actions=[]

    def read(self):
        frame=super().read()
        if frame is None:return None
        stamps=[m['received_ns'] for m in (frame.port_metadata or {}).values()]
        return frame if stamps and all(0<=self.clock()-stamp<=self.max_age_ns for stamp in stamps) else None

    def event(self,row):
        fields={'version','epoch','sequence','sample_ns','connected','backend','control_tick',
                'sticks','triggers','buttons','gyro','accel','raw_hex','unknown_bits'}
        if not isinstance(row,dict) or set(row)!=fields or type(row['version']) is not int or row['version']!=1:
            raise InputError('Invalid Vader event schema')
        if row['backend']!=self.backend:raise InputError('Vader backend identity mismatch')
        if type(row['sample_ns']) is not int or not 0<=self.clock()-row['sample_ns']<=self.max_age_ns:
            raise InputError('Stale/foreign-clock Vader event')
        if type(row['connected']) is not bool or type(row['control_tick']) is not bool:
            raise InputError('Invalid Vader flags')
        epoch=row['epoch'];seq=row['sequence']
        if not isinstance(epoch,str) or not 1<=len(epoch)<=128 or epoch in self.retired:
            raise InputError('Retired/invalid Vader epoch')
        if type(seq) is not int or seq<0 or epoch==self.upstream and seq<=self.upstream_sequence:
            raise InputError('Replayed Vader sequence')
        sticks=vector(row['sticks'],4,-1,1);triggers=vector(row['triggers'],2,0,1)
        gyro=vector(row['gyro'],3,-100,100);accel=vector(row['accel'],3,-200,200)
        if (not isinstance(row['buttons'],list) or any(type(b) is not str for b in row['buttons']) or
            len(set(row['buttons']))!=len(row['buttons']) or set(row['buttons'])-set(BUTTONS)):
            raise InputError('Invalid Vader buttons')
        if not isinstance(row['raw_hex'],str) or len(row['raw_hex'])>128:
            raise InputError('Invalid Vader raw report')
        try:raw=bytes.fromhex(row['raw_hex'])
        except ValueError as exc:raise InputError('Invalid raw report hex') from exc
        vector(row['unknown_bits'],4,0,255)
        if epoch!=self.upstream:
            if self.upstream is not None:self.retired.add(self.upstream)
            if len(self.retired)>64:raise InputError('Vader epoch history exhausted; restart input host')
            self.upstream=epoch;self.upstream_sequence=-1
        self.upstream_sequence=seq
        if not row['connected']:
            self.on_transition();self.disconnect();return None
        if not self.connected or epoch!=getattr(self,'connected_epoch',None):
            self.on_transition();self.reconnect();self.previous=set();self.released=set();self.connected_epoch=epoch
        pressed=set(row['buttons'])
        # Reuse SDL axis validation/deadzone and InputStream sequence identity.
        sdl={'a':'A','b':'B','x':'X','y':'Y','back':'SELECT','guide':'HOME','start':'START',
             'left_stick':'L3','right_stick':'R3','left_shoulder':'LB','right_shoulder':'RB',
             'dpad_up':'UP','dpad_down':'DOWN','dpad_left':'LEFT','dpad_right':'RIGHT'}
        base=super().joy([*sticks,*triggers],[sdl[b] in pressed for b in self.BUTTONS[:15]])
        controls=base.ports['axes'];x=controls['left_x'];y=controls['left_y']
        rx=controls['right_x'];ry=controls['right_y']
        speed=1-.8*triggers[0] # LT precision modifier, 1.0 down to 0.2.
        axes=dict.fromkeys(AXES,0.)
        if self.rotation in pressed:axes.update(roll=x*speed,pitch=y*speed,yaw=rx*speed)
        else:axes.update(x=y*speed,y=x*speed,z=ry*speed)
        axes={key:value*sign for (key,value),sign in zip(axes.items(),self.parameters.get('axis_signs',[1]*6))}
        emergency={'START','SELECT'}<=pressed
        self.actions=[]
        if emergency and not {'START','SELECT'}<=self.previous:self.actions=['estop']
        elif not emergency:
            self.actions=[action for button,action in self.action_map.items()
                if button in pressed and button not in self.previous and button in self.released]
        self.released|=set(BUTTONS)-pressed;self.previous=pressed
        ports={'axes':axes,'scalar':triggers[1],'hold_to_control':self.deadman in pressed and not emergency,
               'raw_axes':[*sticks,*triggers],'buttons':[int(b in pressed) for b in BUTTONS],
               'gyro':gyro,'accel':accel,'unknown_bits':row['unknown_bits'],
               'raw_report':list(raw.ljust(64,b'\0')),'raw_length':len(raw)}
        stamp=row['sample_ns'];meta={k:{'received_ns':stamp,'frame_id':'vader5_device'} for k in ports}
        frame=replace(base,ports=ports,frame_id='vader5_device',port_metadata=meta,synthetic=self.backend=='fake')
        self.latest=frame;self.control_tick=row['control_tick'] or not ports['hold_to_control']
        return frame


class RosBridge:
    def __init__(self,instance,*,clock,on_transition):
        self.source=Vader5Source(instance['instance_id'],parameters=instance['parameters'],
                                backend=instance['backend'],clock=clock,on_transition=on_transition)
        self.topic=instance['parameters']['input_topic']
    def consume(self,text):
        if len(text)>8192:raise InputError('Vader event oversized')
        return self.source.event(json.loads(text))
    def control_event(self,frame):return self.source.control_tick
    @property
    def actions(self):return list(self.source.actions)


def create_ros_source(instance,*,clock,on_transition):
    return RosBridge(instance,clock=clock,on_transition=on_transition)
