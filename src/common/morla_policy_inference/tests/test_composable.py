import time
from threading import Event
import numpy as np
import pytest
from morla_core.composition import compile_system
from morla_runtime import RuntimeSession
from morla_learning.contracts import ModelManifest,digest
from morla_policy_inference.composable import RuntimePolicy,RuntimeReplay,action_commands,observe,ComposablePolicyError


def resolved():
    return compile_system({'assembly_version':1,'assembly_id':'policy_cell','revision':1,
        'instances':{'tool':{'plugin':'morla.astral.gripper','version':'1.0.0'}},
        'capture':{'state':['tool.position','tool.velocity'],'action':['tool.closure'],'images':{}}})


def manifest(value):
    features=value['feature_contract']
    return ModelManifest(1,'act','revision',digest(features),value['geometry_hash'],'norm','checkpoint',
                         'dataset',features,{},32,'inference_verified')


def test_policy_distinct_state_action_dims_and_gateway(tmp_path):
    value=resolved(); runtime=RuntimeSession(value,resource_root=tmp_path)
    runtime.start();runtime.activate('activate')
    runner=RuntimePolicy(runtime,lambda obs:[.4],manifest(value),normalization_hash='norm')
    try:
        runner.acquire(); obs=runner.request()
        assert obs['state'].shape==(2,)
        assert obs['provenance']['tool']['synthetic'] is True
        result=None
        for _ in range(100):
            result=runner.poll()
            if result is not None: break
            time.sleep(.001)
        assert result['accepted'],result
        assert runtime.adapters['tool'].values==[.4]
    finally: runner.close();runtime.stop()


def test_late_prediction_after_takeover_cannot_dispatch(tmp_path):
    value=resolved(); clock=[1000000000]
    runtime=RuntimeSession(value,resource_root=tmp_path,clock=lambda:clock[0]);runtime.start();runtime.activate('activate')
    release=Event()
    runner=RuntimePolicy(runtime,lambda obs:(release.wait(2),[.8])[1],manifest(value),normalization_hash='norm')
    try:
        runner.acquire();runner.request()
        runtime.revoke(reason='human_takeover')
        clock[0]+=300_000_000
        release.set()
        result=runner.poll()
        assert not result['accepted']
        assert runtime.adapters['tool'].values==[0.0]
    finally: release.set();runner.close();runtime.stop()


def test_replay_frozen_contract_and_action_finiteness(tmp_path):
    value=resolved();runtime=RuntimeSession(value,resource_root=tmp_path);runtime.start();runtime.activate('activate')
    try:
        replay=RuntimeReplay(runtime,value);replay.acquire()
        assert replay.send([.2],sample_ns=runtime.clock())['accepted']
        with pytest.raises(ComposablePolicyError): action_commands(value,[float('nan')])
        with pytest.raises(ComposablePolicyError): RuntimeReplay(runtime,{**value,'geometry_hash':'changed'})
        assert not replay.send([.3],sample_ns=runtime.clock()-300_000_000)['accepted']
    finally:runtime.stop()
