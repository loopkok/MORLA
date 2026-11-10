"""Synthetic SDK protocol fixture; never represents a measured robot."""
import threading
import time
from .contract import ROLES, role_of, mapping
from .vendor import common_pb2 as c, mmk2_pb2 as m


class FakeTransport:
    def __init__(self, parameters):
        self.p = parameters
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.q = {name:0. for role,(_,names,_) in ROLES.items() if role!='base' for name in names}
        self.base = [0.,0.]
        self.calls = [];self.registration={'version':'synthetic_protocol_fixture'}

    def open(self):
        pass

    def states(self):
        while not self.closed.is_set():
            stamp = time.time_ns(); t = c.Time(sec=stamp//10**9,nanosec=stamp%10**9)
            with self.lock:
                state=m.RobotState(stamp=t,joint_state=c.JointState(header=c.Header(stamp=t),name=list(self.q),position=list(self.q.values()),velocity=[0.]*len(self.q)),base_state=m.BaseState(stamp=t,velocity=c.Twist3D(x=self.base[0],omega=self.base[1]),pose=c.Pose3D()))
            yield state
            self.closed.wait(1/self.p['rate_hz'])

    def write(self, targets, rows):
        with self.lock:
            self.calls.append({key:list(q) for key,q in targets.items()})
            for key,q in targets.items():
                row=rows[key];role=role_of(row);signs,offsets=mapping(row)
                if role=='base':self.base=[v*s for v,s in zip(q,signs)]
                else:self.q.update(zip(ROLES[role][1],[(v-o)/s for v,s,o in zip(q,signs,offsets)]))

    def images(self, camera):
        import numpy as np
        stamp=time.time_ns()
        image=c.Image(stamp=c.Time(sec=stamp//10**9,nanosec=stamp%10**9))
        for name,array in [('color',np.full((16,16,3),80,dtype=np.uint8)),('depth',np.full((16,16),100,dtype=np.uint16))]:
            image.data[name].CopyFrom(c.NumpyArray(data=array.tobytes(),shape=array.shape,dtype=str(array.dtype)))
        return m.GetImageResponse(images={camera:image})

    def close(self):
        self.closed.set()
