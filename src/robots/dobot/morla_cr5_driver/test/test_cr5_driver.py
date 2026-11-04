import json
import math
from pathlib import Path
import time
import pytest
from morla_core.composition import compile_system, verify_resolved, load_document
from morla_core.plugins.registry import Catalog
from morla_runtime import RuntimeSession
from morla_runtime.adapters import FakeAdapter
ROOT=Path(__file__).parents[1]

def system():
    catalog=Catalog([json.loads((ROOT/'descriptor.json').read_text())])
    resolved=compile_system(load_document(ROOT/'config/assembly.fake.yaml'),registry=catalog)
    return catalog,resolved

from morla_cr5_driver.adapter import CR5Adapter as DeviceAdapter
from morla_cr5_driver.transport import CR5Transport, DobotChannel, servo_parameters

class FragmentedSocket:
    def __init__(self,parts): self.parts=iter(parts);self.sent=[]
    def sendall(self,value):self.sent.append(value)
    def recv(self,n):return next(self.parts,b'')
    def settimeout(self,value):pass

def test_cr5_fragmented_ack_and_rejection():
    channel=object.__new__(DobotChannel)
    channel.timeout=.2
    channel.socket=FragmentedSocket([b'0,{1,2',b',3,4,5,6},GetAngle();'])
    assert channel.request('GetAngle()')=='1,2,3,4,5,6'
    for response in [b'-1,{},EnableRobot();',b'0,{},DisableRobot();',b'garbage;']:
        channel.socket=FragmentedSocket([response])
        with pytest.raises(RuntimeError):channel.request('EnableRobot()')


def test_servoj_keeps_the_dobot_api_values_unless_a_deployment_overrides_them():
    class Motion:
        def __init__(self):self.sent=[]
        def request(self,command):self.sent.append(command);return ''
    transport=object.__new__(CR5Transport)
    transport.servo=servo_parameters({});transport.motion=Motion()
    transport.write([math.pi/2,0,0,0,0,0])
    assert transport.motion.sent[-1]==(
        'ServoJ(90.000000,0.000000,0.000000,0.000000,0.000000,0.000000,'
        't=0.100000,lookahead_time=50.000000,gain=500.000000)')
    transport=object.__new__(CR5Transport)
    transport.servo=servo_parameters({'servo_t':.03,'servo_lookahead_time':20,'servo_gain':300})
    transport.motion=Motion()
    transport.write([0.]*6)
    assert transport.motion.sent[-1]==(
        'ServoJ(0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,'
        't=0.030000,lookahead_time=20.000000,gain=300.000000)')


@pytest.mark.parametrize('parameters',[
    {'servo_t':0}, {'servo_t':1e4}, {'servo_t':'0.1'},
    {'servo_lookahead_time':10}, {'servo_lookahead_time':200},
    {'servo_gain':100}, {'servo_gain':2000}, {'servo_gain':True},
    {'servo_t':float('nan')},
])
def test_servoj_parameters_outside_the_dobot_ranges_are_rejected(parameters):
    with pytest.raises(ValueError):servo_parameters(parameters)


def test_dashboard_timeout_marks_channel_dirty_and_late_reply_is_discarded():
    """A late ack must never be matched against the next command."""
    import socket
    left,right=socket.socketpair()
    channel=object.__new__(DobotChannel)
    channel.timeout=.05; channel.socket=left; channel.dirty=False
    try:
        with pytest.raises(TimeoutError):channel.request('EnableRobot()')
        assert channel.dirty is True
        # The controller answers only after the deadline; that reply belongs to
        # EnableRobot and would otherwise be read as the DisableRobot reply.
        right.sendall(b'0,{},EnableRobot();')
        channel._resync()
        assert channel.dirty is False
        left.settimeout(.05)
        with pytest.raises(TimeoutError):left.recv(64)
    finally:left.close();right.close()


def test_enable_ack_timeout_is_confirmed_by_feedback(monkeypatch):
    """Measured: the ack takes ~5.2 s but enable_status=1 arrives at ~4.2 s."""
    import threading
    from morla_cr5_driver import transport as mod
    def transport():
        t=object.__new__(mod.CR5Transport)
        t.updated=threading.Condition(); t.error=None; t.enable_timeout=.05
        return t
    class Late:
        timeout=.2
        def request(self,command):raise TimeoutError('timed out')
    class Rejected:
        timeout=.2
        def request(self,command):raise RuntimeError('CR5 rejected or mismatched response: -1,{},EnableRobot();')
    t=transport();t.dashboard=Late()
    t.sample={'sample_ns':10**18,'enable_status':1,'error_status':0,'values':[]}
    t.activate()  # A missed ack is not a failure; feedback is the authority.
    assert t.dashboard.timeout == .2  # FK/IK share the channel: keep them short.
    t.sample={'sample_ns':10**18,'enable_status':0,'error_status':0,'values':[]}
    t.stop()
    t=transport();t.dashboard=Late()
    t.sample={'sample_ns':10**18,'enable_status':0,'error_status':0,'values':[]}
    with pytest.raises(RuntimeError,match='unconfirmed'):t.activate()
    t=transport();t.dashboard=Rejected()
    t.sample={'sample_ns':10**18,'enable_status':1,'error_status':0,'values':[]}
    with pytest.raises(RuntimeError,match='rejected'):t.activate()

class Transport:
    def __init__(self,p):self.q=[0.0]*6;self.writes=[];self.stops=0;self.closed=False
    def read(self):return self.q[:]
    def write(self,q):self.writes.append(q);self.q=q[:]
    def activate(self):pass
    def stop(self):self.stops+=1
    def close(self):self.closed=True

def test_worker_feedback_and_close():
    _,resolved=system();instance=next(i for i in resolved['instances'] if i['instance_id']=='arm')
    instance.update(backend='real')
    instance['parameters'].update(joint_lower=[-1.]*6,joint_upper=[1.]*6,rate_hz=100)
    adapter=DeviceAdapter(instance,{'clock':time.monotonic_ns},Transport)
    adapter.configure({})
    try:
        deadline=time.monotonic()+1
        while not adapter.health(time.monotonic_ns())['ready'] and time.monotonic()<deadline:time.sleep(.005)
        assert adapter.health(time.monotonic_ns())['measured']
        adapter.activate()
        time.sleep(.08)
        assert adapter.active and adapter.health(time.monotonic_ns())['ready']
        assert not adapter.health(time.monotonic_ns()).get('fault')
        target={'component_id':'arm','names':[j['name'] for j in instance['joints']],
                'values':[.1]*6,'mode':'position','units':['rad']*6}
        assert adapter.submit(target)['accepted']
        target['values']=[.01]*6;assert adapter.submit(target)['accepted']
        deadline=time.monotonic()+1
        while not any(w==pytest.approx([.01]*6) for w in adapter.transport.writes) and time.monotonic()<deadline:time.sleep(.005)
        assert adapter.transport.writes
    finally:adapter.close()
    assert adapter.transport.closed

def test_cr5_binary_measured_feedback_units_and_faults():
    import numpy as np
    from morla_cr5_driver.vendor.dobot_feedback import MyType
    from morla_cr5_driver.transport import parse_cr5_feedback
    frame=np.zeros(1,dtype=MyType.newbyteorder('<'))
    frame['len']=MyType.itemsize;frame['test_value']=0x123456789abcdef
    frame['q_actual']=[0,90,-90,180,-180,45];frame['enable_status']=1
    result=parse_cr5_feedback(frame.tobytes())
    assert result['values']==pytest.approx([0,math.pi/2,-math.pi/2,math.pi,-math.pi,math.pi/4])
    assert result['enable_status']==1
    frame['test_value']=0
    with pytest.raises(ValueError,match='magic'):parse_cr5_feedback(frame.tobytes())
    with pytest.raises(ValueError,match='length'):parse_cr5_feedback(b'short')

def test_single_device_runtime_ignores_stale_authority(tmp_path):
    catalog,resolved=system();assert verify_resolved(resolved,registry=catalog)
    assert resolved['feature_contract']['state_dim']==6
    assert resolved['feature_contract']['action_dim']==6
    session=RuntimeSession(resolved,registry=catalog,resource_root=tmp_path,
                           adapters={i['instance_id']:FakeAdapter(i) for i in resolved['instances']})
    try:
        assert session.start()['status']=='READY_DISARMED'
        session.prepare('p');session.activate('a');permit=session.acquire('test')
        targets=[{'component_id':i['instance_id'],'names':[j['name'] for j in i['joints']],
                  'values':[0.0]*len(i['joints']),'units':['rad']*len(i['joints']),'mode':'position'} for i in resolved['instances']]
        cmd={**permit,'sequence':1,'issued_ns':session.clock(),'ttl_ns':100000000,'commands':targets}
        assert session.submit(cmd)['accepted']
        assert session.submit(cmd)['accepted'] # Duplicate sequence is not gated.
        cmd['issued_ns']=0
        assert session.submit(cmd)['accepted'] # Authority token/TTL is not enforced.
        session.revoke('test')
        cmd['sequence']=2
        assert not session.submit(cmd)['accepted'] # Revoke disarms the adapters.
    finally: session.stop()
    assert not session.resources.handles


def test_unreachable_controller_names_host_and_port(monkeypatch):
    """Bare "timed out" hides the only actionable fact; the operator needs host:port."""
    from morla_cr5_driver import transport as mod
    class Ok:
        def settimeout(self,*args):pass
        def close(self):pass
    def unreachable(*args,**kwargs): raise TimeoutError('timed out')
    monkeypatch.setattr(mod.socket,'create_connection',unreachable)
    with pytest.raises(ConnectionError,match=r'192\.168\.5\.1:29999'):
        mod.DobotChannel('192.168.5.1',29999,.2)
    def reachable(port):
        def connect(address,**kwargs):
            if address[1]==port:return Ok()
            raise TimeoutError('timed out')
        return connect
    monkeypatch.setattr(mod.socket,'create_connection',reachable(29999))
    with pytest.raises(ConnectionError,match=r'192\.168\.5\.1:30003'):
        mod.CR5Transport({'host':'192.168.5.1','timeout_s':.2})
    def both(port):
        def connect(address,**kwargs):
            if address[1] in (29999,30003):return Ok()
            raise TimeoutError('timed out')
        return connect
    monkeypatch.setattr(mod.socket,'create_connection',both(30004))
    with pytest.raises(ConnectionError,match=r'192\.168\.5\.1:30004'):
        mod.CR5Transport({'host':'192.168.5.1','timeout_s':.2})
