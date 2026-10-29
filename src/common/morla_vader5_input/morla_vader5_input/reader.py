"""SDK batch -> bounded-rate ROS-independent envelopes. Keeps every button edge."""
import secrets
import time
from .mapping import BUTTONS


class ReportStream:
    def __init__(self,*,publish_hz=50,backend='real',clock=time.monotonic_ns):
        if type(publish_hz) not in (int,float) or not 1<=publish_hz<=100:raise ValueError('Invalid publish_hz')
        self.clock=clock;self.period=int(1e9/publish_hz);self.backend=backend
        self.epoch=secrets.token_hex(16);self.sequence=0;self.last_tick=None;self.previous=None

    def packet(self,state=None,*,connected=True,allow_tick=True):
        self.sequence+=1
        stamp=int(state.received_at*1e9) if state is not None else self.clock()
        pressed=set(state.buttons) if state is not None else set()
        tick=allow_tick and (self.last_tick is None or stamp-self.last_tick>=self.period)
        edge=self.previous is None or pressed!=self.previous;self.previous=pressed
        if connected and not tick and not edge:return None
        if tick:self.last_tick=stamp
        unknown=dict(state.raw.unknown_bits) if state is not None else {}
        return {'version':1,'epoch':self.epoch,'sequence':self.sequence,'sample_ns':stamp,
            'connected':connected,'backend':self.backend,'control_tick':tick,
            'sticks':[*state.left_stick,*state.right_stick] if state is not None else [0.]*4,
            'triggers':list(state.triggers) if state is not None else [0.]*2,
            'buttons':[b for b in BUTTONS if b in pressed],
            'gyro':list(state.gyro_rad_s) if state is not None else [0.]*3,
            'accel':list(state.accel_m_s2) if state is not None else [0.]*3,
            'raw_hex':state.raw.raw.hex() if state is not None else '',
            'unknown_bits':[unknown.get(i,0) for i in (11,12,13,14)]}
