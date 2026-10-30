import json
import math
from pathlib import Path
import time
import pytest
from morla_core.composition import compile_system, verify_resolved, load_document
from morla_core.plugins.registry import Catalog
from morla_runtime import RuntimeSession
from morla_runtime.adapters import FakeAdapter
ROOT=Path(__file__).parents[3]

def system():
    paths=['src/robots/dobot/morla_cr5_driver/descriptor.json',
           'src/robots/loophand/morla_loophand_driver/descriptor.json']
    catalog=Catalog([json.loads((ROOT/p).read_text()) for p in paths])
    resolved=compile_system(load_document(ROOT/'examples/cr5_loophand/assembly.fake.yaml'),registry=catalog)
    return catalog,resolved

def test_dual_runtime_and_disarmed_submit(tmp_path):
    catalog,resolved=system();assert verify_resolved(resolved,registry=catalog)
    assert resolved['feature_contract']['state_dim']==13
    assert resolved['feature_contract']['action_dim']==13
    session=RuntimeSession(resolved,registry=catalog,resource_root=tmp_path,
                           adapters={i['instance_id']:FakeAdapter(i) for i in resolved['instances']})
    try:
        assert session.start()['status']=='READY_DISARMED'
        session.prepare('p');session.activate('a');permit=session.acquire('test')
        targets=[{'component_id':i['instance_id'],'names':[j['name'] for j in i['joints']],
                  'values':[0.0]*len(i['joints']),'units':['rad']*len(i['joints']),'mode':'position'} for i in resolved['instances']]
        cmd={**permit,'sequence':1,'issued_ns':session.clock(),'ttl_ns':100000000,'commands':targets}
        assert session.submit(cmd)['accepted']
        # The rollback no longer enforces authority sequence; a repeated
        # sequence is still accepted while the adapter remains active.
        assert session.submit(cmd)['accepted']
        session.revoke('test')
        cmd['sequence']=2
        assert not session.submit(cmd)['accepted']  # revoke stopped the adapter
    finally: session.stop()
    assert not session.resources.handles
