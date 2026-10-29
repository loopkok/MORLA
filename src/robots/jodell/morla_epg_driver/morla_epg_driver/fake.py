"""Protocol fixture, not a physical gripper or force simulation."""
from morla_runtime.adapters import FakeAdapter
from .protocol import STROKE_M,width_to_raw


class EPGFakeAdapter(FakeAdapter):
    def __init__(self,instance):
        super().__init__(instance);self.values=[STROKE_M]

    def read(self,now_ns):
        result=super().read(now_ns)
        values={'closure_ratio':[1-self.values[0]/STROKE_M],'speed_setting':[0.],
                'motor_current':[0.],'voltage':[24.],'temperature':[25.],
                'status':[0x31 if self.active else 0,width_to_raw(self.values[0])<<8,0,25<<8|24]}
        for name,value in values.items():result['ports'][name]['values']=value
        return result
