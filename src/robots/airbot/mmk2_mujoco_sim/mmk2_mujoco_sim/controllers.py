"""MMK2 planar base reference mapping; input axes are named and bounded."""
import math


class DifferentialDrive:
    synchronous_reanchor = True

    def __call__(self,value,measured,dt,parameters):
        if not isinstance(value,dict) or not {'x','y'}<=set(value):
            raise ValueError('MMK2_BASE_REQUIRES_NAMED_X_Y_AXES')
        x,y=value['x'],value['y']
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>1 for v in (x,y)):
            raise ValueError('MMK2_BASE_INVALID_AXES')
        linear=parameters.get('linear_speed',.08);angular=parameters.get('angular_speed',.3)
        if not (0<float(linear)<=.3 and 0<float(angular)<=1):
            raise ValueError('MMK2_BASE_INVALID_SPEED')
        v=float(linear)*y;w=-float(angular)*x
        radius,track=.08401,.3265
        wheels=[(v-w*track/2)/radius,(v+w*track/2)/radius]
        if max(map(abs,wheels))>8:raise ValueError('MMK2_BASE_WHEEL_SPEED_LIMIT')
        return wheels


def create_controller(context):
    if context['instance']['plugin_id']!='morla.airbot.mmk2.base':
        raise ValueError('MMK2_BASE_CONTROLLER_TARGET')
    return DifferentialDrive()
