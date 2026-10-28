"""Composed-system observation, prediction and replay through the runtime gateway."""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
import copy
import math
import time

import numpy as np
from morla_core.contracts.features import SchemaV3
from morla_learning import validate_model


class ComposablePolicyError(ValueError):
    pass


def observe(runtime, *, max_age_ns=250_000_000):
    """Assemble state/images from current named providers, preserving source quality."""
    now=runtime.clock()
    schema=SchemaV3.from_dict(runtime.resolved['capture_schema'])
    sources=schema.metadata.get('stream_sources',{})
    state=np.empty(schema.state_dim,dtype=np.float32)
    samples={}; provenance={}
    def sample(instance_id):
        if instance_id not in samples:
            adapter=runtime.adapters[instance_id]
            value=adapter.read(now) if hasattr(adapter,'read') else adapter.health(now)
            stamp=value.get('sample_ns')
            if type(stamp) is not int or not 0<=now-stamp<=max_age_ns:
                raise ComposablePolicyError('stale provider '+instance_id)
            samples[instance_id]=value
            provenance[instance_id]={key:value.get(key) for key in ('sample_ns','source','measured','synthetic')}
        return samples[instance_id]
    for block in schema.state_blocks:
        ref=sources.get(block.stream_key)
        if ref is None: raise ComposablePolicyError('state stream has no explicit provider')
        source=sample(ref['instance_id'])
        port=source.get('ports',{}).get(ref['port_id'],{})
        values=port.get('values')
        if values is None or port.get('valid',True) is not True:
            raise ComposablePolicyError('missing/invalid observation port')
        values=np.asarray(values,dtype=np.float32)
        if values.ndim!=1 or not np.isfinite(values).all():
            raise ComposablePolicyError('nonfinite observation')
        state[list(block.state_indices)]=values[list(block.stream_indices)]
    images={}
    for label,ref in runtime.resolved['feature_contract'].get('cameras',{}).items():
        source=sample(ref['instance_id'])
        port=source.get('ports',{}).get(ref['port_id'],{})
        if not port.get('data') or port.get('valid',True) is not True:
            raise ComposablePolicyError('missing camera '+label)
        from .robot_io import decode_jpeg_rgb
        image=decode_jpeg_rgb(port['data'])
        if image is None: raise ComposablePolicyError('invalid JPEG '+label)
        images[label]=image
    return {'state':state,'images':images,'sample_ns':now,'provenance':provenance}


def action_commands(resolved, action):
    schema=SchemaV3.from_dict(resolved['capture_schema'])
    values=np.asarray(action,dtype=np.float64)
    if values.shape!=(schema.action_dim,) or not np.isfinite(values).all():
        raise ComposablePolicyError('action shape/nonfinite mismatch')
    targets={}
    instances={x['instance_id']:x for x in resolved['instances']}
    for block in schema.action_blocks:
        if not block.writable or block.semantics not in {'absolute_position','closure_ratio'}:
            raise ComposablePolicyError('selected runtime writer does not support '+block.semantics)
        instance=instances[block.writer_id]
        joints=instance['joints']
        names=[joint['name'] for joint in joints]
        indices=block.stream_indices or tuple(range(block.dim))
        if block.dim!=len(names) or sorted(indices)!=list(range(len(names))):
            raise ComposablePolicyError('partial device commands require an explicit merger controller')
        target=[None]*len(names)
        for source_index,dest_index in zip(block.action_indices,indices): target[dest_index]=float(values[source_index])
        if block.writer_id in targets:
            raise ComposablePolicyError('multiple actions claim the same command resource')
        targets[block.writer_id]={'component_id':block.writer_id,'mode':'position','names':names,
                                 'values':target,'units':[joint['unit'] for joint in joints]}
    return list(targets.values())


class RuntimePolicy:
    """One bounded prediction in flight; late work can never regain motion authority."""
    def __init__(self,runtime,predictor,manifest,*,normalization_hash,backend_revision=None,
                 max_age_ns=250_000_000,producer='policy',camera_map=None):
        validate_model(runtime.resolved,manifest,normalization_hash=normalization_hash,
                       backend_revision=backend_revision,camera_map=camera_map)
        self.runtime,self.predictor,self.manifest=runtime,predictor,manifest
        self.max_age_ns,self.producer=max_age_ns,producer
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='morla-predictor')
        self.pending=None; self.permit=None; self.sequence=0
        self.prediction_stamp=None; self.prediction_epoch=None; self.closed=False
        self.actions=[]
        self.chunk_stamp=None
        self.chunk_epoch=None

    def acquire(self):
        if self.closed: raise ComposablePolicyError('policy closed')
        self.permit=self.runtime.acquire(self.producer,ttl_ns=self.max_age_ns)
        return self.permit.copy()

    def request(self):
        if self.closed or self.pending is not None or self.actions or self.permit is None:
            raise ComposablePolicyError('policy unowned, closed or prediction already pending')
        observation=observe(self.runtime,max_age_ns=self.max_age_ns)
        self.prediction_stamp=observation['sample_ns']
        self.prediction_epoch=self.permit['epoch']
        self.pending=self.pool.submit(self.predictor,observation)
        return observation

    def poll(self):
        if self.pending is None and not self.actions: return None
        now=self.runtime.clock()
        stamp=self.chunk_stamp if self.actions else self.prediction_stamp
        if now-stamp>=self.max_age_ns:
            self.close(reason='prediction_expired')
            return {'accepted':False,'reason':'prediction_expired'}
        if self.pending is not None:
            if not self.pending.done(): return None
            pending,self.pending=self.pending,None
            try:
                values=np.asarray(pending.result(),dtype=np.float64)
                if values.ndim==1: values=values[None,:]
                if values.ndim!=2 or not len(values) or values.shape[1]!=len(self.manifest.features['action_features']) or not np.isfinite(values).all():
                    raise ComposablePolicyError('invalid policy chunk')
                self.actions=list(values)
                self.chunk_stamp=self.prediction_stamp
                self.chunk_epoch=self.prediction_epoch
            except Exception:
                self.close(reason='prediction_failed');raise
        lease=self.runtime.lease
        if not lease or lease['producer']!=self.producer or lease['epoch']!=self.chunk_epoch:
            self.close(reason='authority_changed')
            return {'accepted':False,'reason':'authority_changed'}
        try:
            action=self.actions.pop(0)
            commands=action_commands(self.runtime.resolved,action)
            self.permit=self.runtime.renew(self.producer,lease['token'],self.max_age_ns)
            self.sequence+=1
            return self.runtime.submit({'session_id':self.runtime.session_id,'producer':self.producer,
                'token':self.permit['token'],'epoch':self.permit['epoch'],'sequence':self.sequence,
                'issued_ns':self.chunk_stamp,'ttl_ns':self.max_age_ns,'commands':commands})
        except Exception:
            self.close(reason='prediction_failed')
            raise

    def close(self,reason='policy_stopped'):
        if self.closed: return
        self.closed=True
        if self.pending is not None: self.pending.cancel()
        self.pending=None
        self.actions=[]
        if self.runtime.lease and self.runtime.lease['producer']==self.producer:
            self.runtime.revoke(self.producer,reason=reason)
        self.pool.shutdown(wait=False,cancel_futures=True)
        if hasattr(self.predictor,'close'):self.predictor.close()


class RuntimeReplay:
    def __init__(self,runtime,frozen_resolved):
        for key in ('feature_contract_hash','geometry_hash'):
            if runtime.resolved.get(key)!=frozen_resolved.get(key):
                raise ComposablePolicyError('replay frozen '+key+' mismatch')
        self.runtime=runtime
        self.frozen=copy.deepcopy(frozen_resolved)
        self.permit=None; self.sequence=0

    def acquire(self):
        self.permit=self.runtime.acquire('playback')
        return self.permit

    def send(self,action,*,sample_ns):
        if self.permit is None: raise ComposablePolicyError('playback authority required')
        self.sequence+=1
        return self.runtime.submit({'session_id':self.runtime.session_id,'producer':'playback',
            'token':self.permit['token'],'epoch':self.permit['epoch'],'sequence':self.sequence,
            'issued_ns':sample_ns,'ttl_ns':200_000_000,'commands':action_commands(self.frozen,action)})
