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

from morla_loophand_driver.adapter import LoopHandAdapter as DeviceAdapter
from morla_loophand_driver.transport import LoopHandTransport
from morla_loophand_driver.vendor.aero_hand_constants import AeroHandConstants
from morla_loophand_driver.vendor.joints_to_actuations import JointsToActuationsModel
from morla_loophand_driver.vendor.actuations_to_joints import ActuationsToJointsModelCompact

class SerialStub:
    def __init__(self,raw):self.raw=raw;self.written=[]
    def reset_input_buffer(self):pass
    def read(self,n):return self.raw
    def write(self,packet):self.written.append(packet);return len(packet)

def hand_transport(raw):
    hand=object.__new__(LoopHandTransport);hand.constants=AeroHandConstants()
    hand.forward=JointsToActuationsModel();hand.inverse=ActuationsToJointsModelCompact()
    hand.serial=SerialStub(raw);return hand

def test_hand_compact_roundtrip_and_invalid_feedback():
    hand=hand_transport(b'');q=[.2,.2,.3,.3,.3,.3,.3];hand.write(q)
    packet=hand.serial.written[-1]
    assert packet[:2]==b'\x11\x00'
    hand.serial.raw=bytes([0x22,0])+packet[2:]
    assert hand.read()==pytest.approx(q,abs=.0002)
    hand.serial.raw=b'short'
    with pytest.raises(IOError):hand.read()
    with pytest.raises(RuntimeError,match='STOP_UNCONFIRMED'):hand.stop()

def test_hand_never_claims_verified_watchdog():
    _,resolved=system();instance=next(i for i in resolved['instances'] if i['instance_id']=='hand')
    instance.update(backend='real',deployment_verified=True,watchdog_verified=True)
    adapter=DeviceAdapter(instance,{'clock':time.monotonic_ns})
    assert not adapter.watchdog_verified
    with pytest.raises(RuntimeError,match='read-only'):adapter.activate()

def test_single_device_runtime_and_stale_authority(tmp_path):
    catalog,resolved=system();assert verify_resolved(resolved,registry=catalog)
    assert resolved['feature_contract']['state_dim']==7
    assert resolved['feature_contract']['action_dim']==7
    session=RuntimeSession(resolved,registry=catalog,resource_root=tmp_path,
                           adapters={i['instance_id']:FakeAdapter(i) for i in resolved['instances']})
    try:
        assert session.start()['status']=='READY_DISARMED'
        session.prepare('p');session.activate('a');permit=session.acquire('test')
        targets=[{'component_id':i['instance_id'],'names':[j['name'] for j in i['joints']],
                  'values':[0.0]*len(i['joints']),'units':['rad']*len(i['joints']),'mode':'position'} for i in resolved['instances']]
        cmd={**permit,'sequence':1,'issued_ns':session.clock(),'ttl_ns':100000000,'commands':targets}
        assert session.submit(cmd)['accepted']
        assert not session.submit(cmd)['accepted']
        session.revoke('test')
        cmd['sequence']=2
        assert not session.submit(cmd)['accepted']
    finally: session.stop()
    assert not session.resources.handles
