"""Quest stick to native SDK [linear_x m/s, angular_z rad/s]."""
import math


class BaseTwist:
    synchronous_reanchor=True
    def __call__(self,value,measured,dt,parameters):
        if not isinstance(value,dict) or not {'x','y'}<=set(value):raise ValueError('MMK2_BASE_NAMED_AXES_REQUIRED')
        x,y=value['x'],value['y']
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>1 for v in (x,y)):raise ValueError('MMK2_BASE_INVALID_AXES')
        v=parameters.get('linear_speed',.05);w=parameters.get('angular_speed',.15)
        if any(isinstance(a,bool) or not isinstance(a,(int,float)) or not math.isfinite(a) or not 0<a<=limit for a,limit in ((v,.15),(w,.3))):raise ValueError('MMK2_BASE_INVALID_SPEED')
        return [v*y,-w*x]


def create_controller(context):
    if context['instance']['plugin_id']!='morla.airbot.mmk2_sdk.base':raise ValueError('MMK2_BASE_WRONG_TARGET')
    return BaseTwist()
