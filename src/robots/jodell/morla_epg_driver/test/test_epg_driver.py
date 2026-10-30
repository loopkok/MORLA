import copy
import importlib.util
import json
from pathlib import Path
import threading
import time
import numpy as np
import pytest
from morla_core.composition import compile_system,load_document,verify_resolved
from morla_core.plugins.registry import Catalog
from morla_runtime import RuntimeSession
from morla_epg_driver.protocol import decode,width_to_raw,MODEL
from morla_epg_driver.transport import EPGTransport
from morla_epg_driver.adapter import EPGAdapter
from morla_epg_driver.fake import EPGFakeAdapter

ROOT=Path(__file__).parents[1]


def system(real=False):
    d=json.loads((ROOT/'descriptor.json').read_text());catalog=Catalog([d])
    a=load_document(ROOT/'config/assembly.fake.yaml')
    if real:a['instances']['gripper']['backend']='real';a['instances']['gripper']['parameters']={'port':'/dev/serial/by-id/test'}
    return catalog,compile_system(a,{'instances':{'gripper':{'device_id':'test-epg','resource_key':'epg:test'}}} if real else None,registry=catalog)


def instance():
    _,r=system(True);i=r['instances'][0]
    i.update(deployment_verified=True,watchdog_verified=True)
    i['parameters'].update(position_feedback_verified=True,activation_motion_verified=True,
       device_watchdog_verified=True,poll_hz=50,command_timeout_s=.05,stop_timeout_s=.1,enable_timeout_s=.2)
    return i


def status(flags=0x31,pos=0,fault=0):return decode([flags,pos<<8|fault,0x8040,0xfe18])


def test_model_conversion_status_and_feedback_truth():
    assert width_to_raw(.05)==0 and width_to_raw(0)==255 and width_to_raw(.04)==51
    assert status()['temperature_c']==-2 and status()['bus_voltage_v']==24
    assert status(0xb1)['object_detected'] is False # gOBJ invalid when gGTO=0.
    assert status(0xb9)['object_detected'] is True
    assert status(0x39)['speed_setting_ratio']>0 # Never treat raw0 as stopped.
    for value in (-.001,.051,float('nan'),True):
        with pytest.raises(ValueError):width_to_raw(value)
    for regs in ('error',(0,1),[0,0,0,65536],[True,0,0,0]):
        with pytest.raises(ValueError):decode(regs)


class SDK:
    def __init__(self):self.calls=[];self.response=(0x31,0,0,0x1918);self.ack=1;self.mode=(0,)
    def readRegisterData(self,*args):self.calls.append(('read',args));return self.mode if args[2]==0x03FD else self.response
    def enableClamp(self,*args):self.calls.append(('enable',args));return self.ack
    def runWithParam(self,*args):self.calls.append(('run',args));return self.ack
    def writeDataToRegister(self,*args):self.calls.append(('write',args));return self.ack
    def stopClaw(self,*args):raise AssertionError('Undocumented stop address must not be used')


def test_sdk_commands_byte_order_ack_and_protocol_conflict():
    i=instance();sdk=SDK();t=EPGTransport(i['parameters'],sdk=sdk)
    sample=t.read();assert sample['width_m']==.05
    t.enable();t.write(.04);t.stop()
    assert sdk.calls==[('read',(9,4,0x07D0,4)),('read',(9,3,0x03FD,1)),('enable',(9,True)),('run',(9,51,32,32)),('write',(9,0x03E8,[5]))]
    for bad in ('timeout',True,0,None):
        sdk.ack=bad
        with pytest.raises(RuntimeError):t.write(.04)
    sdk.response='disconnected'
    with pytest.raises(ValueError):t.read()


def test_actual_vendor_sdk_with_recording_modbus_master():
    # Vendor SDK installed externally; no vendor code is distributed in MORLA.
    module=pytest.importorskip('jodellSdk.jodellSdkDemo')
    class Master:
        def __init__(self):self.calls=[]
        def execute(self,*args,**kwargs):
            self.calls.append((args,kwargs))
            return (0x31,0,0,0x1918) if args[1]==4 else ((0,) if args[1]==3 else (0,))
    sdk=module.EpgClawControl();master=Master();sdk.setMaster(master)
    t=EPGTransport(instance()['parameters'],sdk=sdk)
    t.read();t.enable();t.write(.04);t.stop()
    assert master.calls[0][0]==(9,4,0x07D0,4)
    assert master.calls[1]==((9,3,0x03FD,1),{})
    assert master.calls[2]==((9,16,0x03E8),{'output_value':[1]})
    assert master.calls[3]==((9,16,0x03E8),{'output_value':[9,51<<8,32|32<<8]})
    assert master.calls[4]==((9,16,0x03E8),{'output_value':[5]})


class Transport:
    def __init__(self,p):self.calls=[];self.flags=0x30;self.pos=0;self.closed=False;self.stuck=False;self.fail=False
    def read(self):
        if self.fail:raise IOError('lost serial')
        row=status(self.flags,self.pos);row['sample_ns']=time.monotonic_ns();return row
    def enable(self):self.calls.append('enable');self.flags=0x31
    def write(self,q):self.calls.append(('write',q));self.pos=width_to_raw(q);self.flags=0x39
    def stop(self):
        self.calls.append('stop')
        if not self.stuck:self.flags=0x31
    def close(self):self.closed=True


def make_adapter(i=None,factory=Transport):
    a=EPGAdapter(i or instance(),{'clock':time.monotonic_ns},factory);a.configure({});return a


def wait_for(predicate):
    end=time.monotonic()+1
    while not predicate() and time.monotonic()<end:time.sleep(.003)
    assert predicate()


def command(value):return {'component_id':'gripper','names':['opening_width'],'values':[value],'units':['m'],'mode':'position'}


def test_readonly_configuration_close_and_activation():
    i=instance();i['parameters']['position_feedback_verified']=False
    a=make_adapter(i)
    try:
        assert a.prepare()
        h=a.health(time.monotonic_ns());assert h['ready'] and not h['measured']
        assert not h['ports']['position']['measured'] and h['ports']['voltage']['measured']
        assert not a.transport.calls
        assert a.activate() and a.active and a.owned
        assert a.transport.calls==['enable']
    finally:a.close()
    assert a.transport.closed and a.transport.calls[-1]=='stop'
    assert not a.thread.is_alive()


def test_disabled_default_position_is_not_published_as_width():
    # Disabled devices report a default position (0 was observed while physically
    # closed). Publishing it as a valid width displayed 0.05 m on a closed gripper.
    a=make_adapter()
    try:
        h=a.health(time.monotonic_ns())
        assert h['ready'] and not h['measured'] and h['values']==[]
        assert h['ports']['position']['valid'] is False
        assert h['ports']['position']['source']=='device_position_unavailable'
        assert h['ports']['closure_ratio']['valid'] is False
        # Voltage/current/status stay usable while merely disabled.
        assert h['ports']['voltage']['valid'] is True
        a.transport.flags=0x31
        wait_for(lambda:a.health(time.monotonic_ns())['values'])
        h=a.health(time.monotonic_ns())
        assert h['measured'] and h['ports']['position']['valid']
        assert h['values']==[.05] and h['ports']['position']['source']=='hardware_verified'
    finally:a.close()


def test_feedback_width_stop_and_cleanup():
    a=make_adapter()
    try:
        a.activate();assert a.active
        time.sleep(.07);assert a.active # No host command watchdog.
        assert a.submit(command(.048))['accepted']
        wait_for(lambda:a.transport.pos==width_to_raw(.048))
        h=a.health(time.monotonic_ns());assert h['values']==pytest.approx([.048],abs=1e-3)
        a.stop();assert not a.active and not a.owned
        assert 'stop' in a.transport.calls
    finally:a.close()
    assert a.transport.closed and not a.thread.is_alive()


def test_stop_ack_does_not_mean_stopped_and_never_clear_brake():
    a=make_adapter()
    try:
        a.activate();a.transport.flags=0x39;a.transport.stuck=True
        with pytest.raises(TimeoutError,match='stop feedback'):a.stop()
        assert a.owned and not a.active
        a.transport.stuck=False;a.stop();assert not a.owned
    finally:a.close()


def test_activation_cancel_fences_late_enable():
    entered=threading.Event();finish=threading.Event()
    class Slow(Transport):
        def enable(self):entered.set();finish.wait(.3);super().enable()
    a=make_adapter(factory=Slow);errors=[]
    def enable():
        try:a.activate()
        except Exception as e:errors.append(str(e))
    t=threading.Thread(target=enable);t.start();assert entered.wait(.2)
    s=threading.Thread(target=a.stop);s.start();wait_for(a.cancel.is_set);finish.set()
    t.join(1);s.join(1)
    try:
        assert not a.active and not a.owned and errors
        assert a.transport.calls==['enable','stop']
    finally:a.close()


def test_runtime_fake_gateway_auxiliary_and_capture(tmp_path):
    catalog,resolved=system();assert verify_resolved(resolved,registry=catalog)
    assert resolved['feature_contract']['state_dim']==resolved['feature_contract']['action_dim']==1
    a=EPGFakeAdapter(resolved['instances'][0])
    s=RuntimeSession(resolved,registry=catalog,resource_root=tmp_path,adapters={'gripper':a})
    try:
        assert s.start()['status']=='READY_DISARMED';s.prepare('p');s.activate('a');permit=s.acquire('test')
        row={'session_id':s.session_id,'producer':'test','token':permit['token'],'epoch':permit['epoch'],
            'sequence':1,'issued_ns':s.clock(),'ttl_ns':100_000_000,'commands':[command(.04)]}
        assert s.submit(row)['accepted']
        h=a.health(s.clock());assert h['values']==[.04]
        assert h['ports']['closure_ratio']['values']==pytest.approx([.2])
        assert not h['measured'] and h['synthetic']
        # Authority sequence/TTL fields are recorded but no longer enforced.
        row['sequence']=2;row['commands']=[command(.03)]
        assert s.submit(row)['accepted'] and a.health(s.clock())['values']==[.03]
        row['commands']=[command(.02)];row['issued_ns']=0
        assert s.submit(row)['accepted'] and a.health(s.clock())['values']==[.02]
    finally:s.stop()
    assert not s.resources.handles


@pytest.mark.parametrize('key,value',[('slave_id',0),('poll_hz',True),('speed_raw',256),('port','/dev/ttyUSB0'),('model','EPG50-100')])
def test_bad_real_parameters_fail_before_sdk_open(key,value):
    i=instance();i['parameters'][key]=value
    a=EPGAdapter(i,{'clock':time.monotonic_ns},lambda p:(_ for _ in ()).throw(AssertionError('opened')))
    with pytest.raises(ValueError):a.configure({})


def test_actual_sdk_transport_open_is_readonly_bounded_and_exclusive(monkeypatch):
    pytest.importorskip('jodellSdk.jodellSdkDemo')
    import serial
    from modbus_tk import modbus_rtu
    calls=[]
    class Port:
        def __init__(self,**kwargs):calls.append(('open',kwargs));self.closed=False
        def close(self):self.closed=True
    class Master:
        def __init__(self,port):self.port=port
        def set_timeout(self,t):calls.append(('timeout',t))
        def set_verbose(self,v):assert v is False
        def execute(self,*args,**kwargs):calls.append(('execute',args,kwargs));return (0x31,0,0,0x1918)
        def close(self):calls.append(('close',))
    monkeypatch.setattr(serial,'Serial',Port);monkeypatch.setattr(modbus_rtu,'RtuMaster',Master)
    t=EPGTransport(instance()['parameters'])
    try:
        assert len(calls)==2 and calls[0][1]['exclusive'] is True
        assert calls[0][1]['write_timeout']==calls[0][1]['timeout']==.1
        assert t.read()['width_m']==.05
        assert [x[1][1] for x in calls if x[0]=='execute']==[4]
    finally:t.close()
    assert t.port.closed


def test_serial_failure_is_sticky_and_does_not_resend_old_target():
    a=make_adapter()
    try:
        a.activate();a.transport.fail=True
        wait_for(lambda:a.fault is not None)
        assert not a.active and a.pending is None
        with pytest.raises(RuntimeError):a.submit(command(.048))
        a.transport.fail=False
        assert a.health(time.monotonic_ns())['fault']
    finally:
        a.transport.fail=False;a.close()


def test_configuration_only_cr5_composition():
    root=ROOT.parents[3]
    arm=json.loads((root/'src/robots/dobot/morla_cr5_driver/descriptor.json').read_text())
    hand=json.loads((ROOT/'descriptor.json').read_text())
    r=compile_system(load_document(root/'examples/cr5_epg/assembly.fake.yaml'),registry=Catalog([arm,hand]))
    assert r['feature_contract']['state_dim']==r['feature_contract']['action_dim']==7
    assert not r['mounts']
    assert r['instances'][0]['plugin_id']=='morla.dobot.cr5'
    assert r['instances'][1]['plugin_id']=='morla.jodell.epg50_060'


def test_io_mode_is_not_silently_switched_or_enabled():
    sdk=SDK();sdk.mode=(0x55,);t=EPGTransport(instance()['parameters'],sdk=sdk)
    with pytest.raises(RuntimeError,match='IO mode'):t.enable()
    assert sdk.calls==[('read',(9,3,0x03FD,1))]
