"""Device-independent gripper signal processing, extracted from the legacy path."""
import math


class PinchFilter:
    def __init__(self, *, close=.015, opened=.08, auto_range=True,
                 forget_s=8., min_span=.01, alpha=.4):
        self.close,self.opened,self.auto_range=close,opened,auto_range
        self.forget_s,self.min_span,self.alpha=forget_s,min_span,alpha
        self.reset()

    def reset(self):
        self.low=self.high=None
        self.last_time=0.
        self.ratio=None

    def envelope(self,distance,now):
        if not self.auto_range:self.low,self.high=self.close,self.opened;return
        if self.low is None or self.last_time<=0:
            self.low=self.high=distance;self.last_time=now;return
        dt=max(1e-3,now-self.last_time);self.last_time=now
        k=1-math.exp(-dt/max(1e-3,self.forget_s))
        self.low=distance if distance<self.low else self.low+k*(distance-self.low)
        self.high=distance if distance>self.high else self.high+k*(distance-self.high)
        if self.high-self.low<self.min_span:
            middle=.5*(self.high+self.low)
            self.low,self.high=middle-.5*self.min_span,middle+.5*self.min_span

    def step(self,distance,now):
        self.envelope(distance,now)
        span=max(self.min_span,self.high-self.low)
        raw=1-max(0.,min(1.,(distance-self.low)/span))
        a=max(0.,min(1.,self.alpha))
        self.ratio=raw if self.ratio is None else a*raw+(1-a)*self.ratio
        return self.ratio


class TriggerFilter:
    def __init__(self, *, invert=False,deadzone=.05,gamma=1.4,alpha=.4):
        self.invert,self.deadzone,self.gamma,self.alpha=invert,deadzone,max(.05,gamma),alpha
        self.reset()
    def reset(self):self.ratio=None
    def step(self,value):
        value=1-value if self.invert else value
        value=max(0.,min(1.,(value-self.deadzone)/max(1e-3,1-self.deadzone)))**self.gamma
        a=max(0.,min(1.,self.alpha))
        self.ratio=value if self.ratio is None else a*value+(1-a)*self.ratio
        return self.ratio


def limit_ratio_step(target, previous, dt, rate):
    """Legacy output slew shaping; first sample snaps to the requested ratio."""
    if rate>0 and dt>0:
        maximum=rate*min(dt,.1)
        return previous+max(-maximum,min(maximum,target-previous))
    return target


class FilteredGripperController:
    def __init__(self,parameters,*,landmarks=False,clock):
        self.landmarks,self.clock=landmarks,clock
        self.parameters=parameters
        self.last_output=None
        self.last_time=None
        if landmarks:
            self.filter=PinchFilter(close=parameters.get('close_dist_m',.015),
                opened=parameters.get('open_dist_m',.08),auto_range=parameters.get('auto_range',True),
                forget_s=parameters.get('auto_range_forget_s',8.),
                min_span=parameters.get('auto_range_min_span_m',.01),alpha=parameters.get('ema_alpha',.4))
        else:
            self.filter=TriggerFilter(invert=parameters.get('trigger_invert',False),
                deadzone=parameters.get('trigger_deadzone',.05),gamma=parameters.get('trigger_gamma',1.4),
                alpha=parameters.get('trigger_ema_alpha',.4))
    def reset(self):
        self.filter.reset();self.last_output=None;self.last_time=None
    def __call__(self,value,measured,dt,parameters):
        if len(measured)!=1:raise ValueError('gripper controller requires one target')
        if self.landmarks:
            import numpy as np
            points=np.asarray(value,dtype=float).reshape(21,3)
            if not np.isfinite(points).all():raise ValueError('invalid landmarks')
            ratio=self.filter.step(float(np.linalg.norm(points[4]-points[8])),self.clock()/1e9)
        else:
            if not math.isfinite(value):raise ValueError('invalid trigger')
            ratio=self.filter.step(float(value))
        now=self.clock()/1e9
        dt=now-self.last_time if self.last_time is not None else 0.
        ratio=limit_ratio_step(ratio,self.last_output if self.last_output is not None else ratio,
                              dt,parameters.get('max_ratio_rate',2.5))
        self.last_output,self.last_time=ratio,now
        lo,hi=parameters.get('output_range',[0.,1.])
        return [lo+ratio*(hi-lo)]
