"""Numeric canaries validate independent layouts all the way to exported values."""
import json
from pathlib import Path
from types import SimpleNamespace
import h5py
import numpy as np
import pytest
from morla_core.schema import StreamSpec, StateBlockSpec
from morla_core.contracts import SchemaV3, ActionBlockV3
from morla_data_collect.schema import FrozenPipelineSchema
from morla_data_collect.composable_recorder import ComposableRecorder
from morla_data_collect.pipeline.align_data import align_episode
from morla_data_collect.pipeline.validate_data import validate_episode
from morla_data_collect.pipeline.convert_to_lerobot import convert_session
from morla_data_collect.codecs import DEFAULT_CODECS, Codec, decode_numeric


def schema(source='next_state', action_dim=8):
    streams = tuple(StreamSpec(k, '/'+k, 'std_msgs/msg/Float64MultiArray', (dim,), 'float32', 'rad',
                               'sensor_data', 'arrival', True, kind)
                    for k, dim, kind in [('feedback',11,'state'),('commands',8,'command')])
    # Nontrivial state order and action order are intentional canaries.
    block = StateBlockSpec('sensor', 'feedback', 11, tuple('s'+str(i) for i in range(11)),
                           'rad', tuple(range(11)), stream_indices=(10,9,8,7,6,5,4,3,2,1,0))
    actions = (() if not action_dim else (ActionBlockV3('arm',8,tuple('a'+str(i) for i in range(8)),
                    'rad', (7,6,5,4,3,2,1,0), 'absolute_position', True, 'commands','writer',
                    source_state_indices=(1,3,5,7,9,0,2,4),stream_indices=(7,6,5,4,3,2,1,0)),))
    return SchemaV3('canary','mixed',streams,(block,),actions,(),30,source,0,100.)


def record(tmp_path, source='next_state', action_dim=8):
    frozen = schema(source,action_dim)
    recorder = ComposableRecorder(frozen.to_dict(),str(tmp_path/'raw'))
    episode = recorder.start('canary')
    t0=1_700_000_000.
    for i in range(91):
        recorder.append_snapshot({'feedback':np.arange(11,dtype=np.float32)+i,
                                  'commands':np.arange(8,dtype=np.float32)*10+i}, t0+i/30)
    recorder.stop()
    align_episode(episode,log=lambda _:None)
    return Path(episode), frozen


@pytest.mark.parametrize('source',['next_state','command'])
def test_11state_8action_numeric_canary(tmp_path,source):
    ep, frozen = record(tmp_path,source)
    with h5py.File(ep/'aligned_data.h5') as file:
        state,action=file['observation/state'][:],file['action'][:]
        assert state.shape==(91,11) and action.shape==(91,8)
        np.testing.assert_array_equal(state[0],np.arange(11)[::-1])
        for i in (0,10,89,90):
            if source=='next_state':
                expected=state[min(i+1,90),[1,3,5,7,9,0,2,4]]
                np.testing.assert_array_equal(action[i,[7,6,5,4,3,2,1,0]],expected)
            else:
                np.testing.assert_array_equal(action[i],np.arange(8)*10+i)
    rep=validate_episode(str(ep),min_duration_s=0,require_aligned=True)
    assert rep.status!='fail',rep.issues
    export=tmp_path/'export'
    convert_session(str(ep.parent),str(export),log=lambda _:None)
    info=json.loads((export/'meta/info.json').read_text())
    assert info['features']['action']['shape']==[8]
    assert info['features']['action']['names']==frozen.action_names()
    import pyarrow.parquet as pq
    table=pq.read_table(next((export/'data').rglob('*.parquet')))
    np.testing.assert_array_equal(np.array(table['action'].to_pylist()),action)
    contract=json.loads((export/'meta/morla_schema.json').read_text())
    assert contract['action_features']==frozen.action_names()
    assert contract['episodes'][0]['provenance']['codecs']['numeric']=='1.0.0'


def test_read_only_no_camera_no_action(tmp_path):
    ep,_=record(tmp_path,action_dim=0)
    with h5py.File(ep/'aligned_data.h5') as file:
        assert file['action'].shape==(91,0)
    export=tmp_path/'export'
    convert_session(str(ep.parent),str(export),log=lambda _:None)
    assert json.loads((export/'meta/info.json').read_text())['features']['action']['shape']==[0]


def test_recorder_rejects_shape_nan_duplicate_timestamp_and_freezes(tmp_path):
    value=schema().to_dict()
    recorder=ComposableRecorder(value,str(tmp_path/'raw'))
    value['profile_id']='mutated'
    ep=Path(recorder.start())
    recorder.append_stream('feedback',np.arange(11),1.)
    for key,values,timestamp in [('feedback',[1],2),('feedback',[np.nan]*11,2),
                                  ('feedback',np.arange(11),1.),('injected',[1],2.)]:
        with pytest.raises(ValueError): recorder.append_stream(key,values,timestamp)
    recorder.stop(cancelled=True)
    meta=json.loads((ep/'meta.json').read_text())
    assert meta['schema']['profile_id']=='canary'
    assert meta['recording_status']=='cancelled'


def test_codec_joint_order_shape_and_no_dynamic_import():
    spec=StreamSpec('j','/j','sensor_msgs/msg/JointState',(2,),'float32','rad','sensor_data',
                    'source_header',True,'state',('a','b'))
    np.testing.assert_array_equal(decode_numeric(SimpleNamespace(position=[3,2],name=['b','a']),spec),[2,3])
    with pytest.raises(ValueError): decode_numeric(SimpleNamespace(position=[3,2],name=['a','a']),spec)
    with pytest.raises(ValueError): DEFAULT_CODECS.get('os.system')
    with pytest.raises(ValueError): DEFAULT_CODECS.register(Codec('numeric','2',lambda _:None))


def test_generic_tensor_sidecar_validity_and_provenance(tmp_path):
    from dataclasses import replace
    from morla_data_collect.pipeline.repair_aligned import repair_episode
    from morla_data_collect.pipeline.verify_aligned import verify_session
    base=schema()
    frozen=replace(base,sensor_specs=({'sensor_key':'pressure_grid','required':False,
                       'fields':{'values':{'shape':[2,3],'dtype':'float32'}}},),hold_frames=2)
    recorder=ComposableRecorder(frozen.to_dict(),str(tmp_path/'raw'))
    ep=Path(recorder.start('sensor canary'))
    t0=1_700_000_000.
    for i in range(91):
        recorder.append_snapshot({'feedback':np.arange(11,dtype=np.float32)+i,
                                  'commands':np.arange(8,dtype=np.float32)+i},t0+i/30)
        recorder.append_sensor('pressure_grid',{'values':np.full((2,3),i,dtype=np.float32),
                                               'valid':np.array(i!=20)},t0+i/30)
    recorder.stop()
    align_episode(str(ep),log=lambda _:None)
    with h5py.File(ep/'aligned_data.h5') as file:
        values=file['sensors/pressure_grid/values'][:]
        assert values.shape==(93,2,3)
        np.testing.assert_array_equal(values[10],np.full((2,3),10))
        assert not file['sensors/pressure_grid/valid'][20]
        assert not file['sensors/pressure_grid/valid'][-1]
    rep=validate_episode(str(ep),min_duration_s=0,require_aligned=True)
    assert rep.status!='fail',rep.issues
    repaired=tmp_path/'repaired'/'episode000000'
    repair_episode(str(ep),str(repaired),None,False)
    assert verify_session(str(ep.parent),str(repaired.parent))['status']=='pass'


@pytest.mark.parametrize('source',['next_state','command'])
def test_repair_and_lerobot_v3_preserve_independent_actions(tmp_path,source):
    from morla_data_collect.pipeline.repair_aligned import repair_episode
    from morla_data_collect.pipeline.verify_aligned import verify_session
    from morla_data_collect.pipeline.convert_to_lerobot_v3 import convert_dataset_v21_to_v30
    import pyarrow.parquet as pq
    ep,frozen=record(tmp_path,source)
    repaired=tmp_path/'repaired'/'episode000000'
    repair_episode(str(ep),str(repaired),None,False)
    assert verify_session(str(ep.parent),str(repaired.parent))['status']=='pass'
    v2,v3=tmp_path/'v2',tmp_path/'v3'
    convert_session(str(repaired.parent),str(v2),log=lambda _:None)
    convert_dataset_v21_to_v30(str(v2),str(v3),log=lambda _:None)
    a=np.asarray(pq.read_table(next((v2/'data').rglob('*.parquet')))['action'].to_pylist())
    b=np.asarray(pq.read_table(next((v3/'data').rglob('*.parquet')))['action'].to_pylist())
    np.testing.assert_array_equal(a,b)
    assert b.shape[1]==8
    info=json.loads((v3/'meta/info.json').read_text())
    assert info['features']['action']['names']==frozen.action_names()


def test_custom_camera_key_and_act_independent_layout(tmp_path):
    from dataclasses import replace
    from conftest import make_jpeg
    from morla_data_collect.pipeline.convert_to_act import run_pipeline, structural_check
    import pyarrow.parquet as pq
    base=schema()
    camera=StreamSpec('camera:overhead_custom','/custom/image','sensor_msgs/msg/CompressedImage',
                       (None,None,3),'uint8','rgb','sensor_data','source_header',True,'camera')
    frozen=replace(base,streams=base.streams+(camera,),cameras=('overhead_custom',))
    recorder=ComposableRecorder(frozen.to_dict(),str(tmp_path/'raw'))
    ep=Path(recorder.start('camera canary'))
    t0=1_700_000_000.
    jpeg=make_jpeg(w=32,h=24)
    for i in range(61):
        recorder.append_snapshot({'feedback':np.arange(11,dtype=np.float32)+i*.01,
                                  'commands':np.arange(8,dtype=np.float32)+i*.01},t0+i/30)
        recorder.append_camera('overhead_custom',jpeg,t0+i/30)
    recorder.stop()
    run_pipeline(str(tmp_path/'act'),session_dir=str(ep.parent),image_size=32,log=lambda _:None)
    assert structural_check(str(tmp_path/'act'))==[]
    info=json.loads((tmp_path/'act/meta/info.json').read_text())
    assert 'observation.images.overhead_custom' in info['features']
    assert info['features']['action']['shape']==[8]
    table=pq.read_table(next((tmp_path/'act/data').rglob('*.parquet')))
    with h5py.File(ep/'aligned_data.h5') as aligned:
        np.testing.assert_array_equal(np.asarray(table['action'].to_pylist()),aligned['action'][:])


def test_missing_optional_generic_sensor_has_invalid_quality(tmp_path):
    from dataclasses import replace
    base=schema()
    frozen=replace(base,sensor_specs=({'sensor_key':'missing','required':False,
                       'fields':{'values':{'shape':[2],'dtype':'float32'}}},),hold_frames=1)
    recorder=ComposableRecorder(frozen.to_dict(),str(tmp_path/'raw'))
    ep=Path(recorder.start())
    for i in range(61):
        recorder.append_stream('feedback',np.arange(11),1_700_000_000.+i/30)
    recorder.stop()
    align_episode(str(ep),log=lambda _:None)
    with h5py.File(ep/'aligned_data.h5') as aligned:
        assert not aligned['sensors/missing/valid'][:].any()
        assert (aligned['sensors/missing/source_index'][:]==-1).all()
        assert ((aligned['quality'][:-1]&8)!=0).all()


def test_empty_recording_is_incomplete_and_provenance_is_explicit(tmp_path):
    value=schema().to_dict()
    recorder=ComposableRecorder(value,str(tmp_path/'empty'))
    ep=Path(recorder.start())
    assert recorder.stop()['status']=='failed'
    assert validate_episode(str(ep),min_duration_s=0).status=='fail'
    recorder=ComposableRecorder(value,str(tmp_path/'source'))
    ep=Path(recorder.start())
    for i in range(61):
        recorder.append_stream('feedback',np.arange(11),1_700_000_000.+i/30,
            provenance={'source':'fake','synthetic':True,'measured':False,'valid':True})
    assert recorder.stop()['status']=='complete'
    with h5py.File(ep/'robot_data.h5') as raw:
        assert raw['streams/feedback/synthetic'][:].all()
        assert not raw['streams/feedback/measured'][:].any()
        assert raw['streams/feedback/source'].asstr()[0]=='fake'
    align_episode(str(ep),log=lambda _:None)
    convert_session(str(ep.parent),str(tmp_path/'export'),log=lambda _:None)
    with h5py.File(tmp_path/'export/meta/provenance/episode_000000.h5') as exported:
        assert exported['streams/feedback/synthetic'][:].all()


@pytest.mark.parametrize('dtype,values',[('int32',[1.5]),('uint8',[-1]),('bool',[2]),('float32',[1e100])])
def test_sensor_codec_rejects_unrepresentable_values(dtype,values):
    from morla_data_collect.codecs import decode_tensor_sensor
    with pytest.raises(ValueError):
        decode_tensor_sensor(SimpleNamespace(data=values),{'fields':{'values':{'shape':[1],'dtype':dtype}}})


def test_async_recorder_queue_never_waits_for_slow_writer_and_overflow_fails(tmp_path):
    from threading import Event
    import time
    from morla_data_collect.composable_recorder import AsyncComposableRecorder
    recorder=AsyncComposableRecorder(schema().to_dict(),str(tmp_path/'async'),queue_size=1)
    ep=Path(recorder.start())
    entered,release=Event(),Event()
    original=recorder._writer.append_stream
    def slow(*args,**kwargs):
        entered.set()
        release.wait(timeout=2)
        return original(*args,**kwargs)
    recorder._writer.append_stream=slow
    assert recorder.append_stream('feedback',np.arange(11),1.)
    assert entered.wait(timeout=1)
    t0=time.monotonic()
    assert recorder.append_stream('feedback',np.arange(11),2.)
    assert not recorder.append_stream('feedback',np.arange(11),3.)
    assert time.monotonic()-t0<0.1
    release.set()
    result=recorder.stop()
    assert result['status']=='failed' and result['dropped_samples']==1
    assert 'OVERFLOW' in result['failure']
    with h5py.File(ep/'robot_data.h5') as raw:
        assert len(raw['streams/feedback/timestamps'])==2


def test_async_recorder_freezes_enqueue_time_and_keeps_source_lag(tmp_path):
    from threading import Event
    import time
    from morla_data_collect.composable_recorder import AsyncComposableRecorder
    recorder=AsyncComposableRecorder(schema().to_dict(),str(tmp_path/'time'),queue_size=8)
    ep=Path(recorder.start())
    entered,release=Event(),Event()
    original=recorder._writer.append_stream
    def slow(*args,**kwargs):
        entered.set(); release.wait(timeout=2)
        return original(*args,**kwargs)
    recorder._writer.append_stream=slow
    enqueue_start=time.time()
    recorder.append_stream('feedback',np.arange(11),provenance={'source_time_ns':100,'received_time_ns':300})
    assert entered.wait(timeout=1)
    recorder.append_stream('commands',np.arange(8),timestamp=1_700_000_000.,
                           provenance={'source_time_ns':150,'received_time_ns':500})
    queued_before=time.time()
    time.sleep(.06)
    release.set()
    assert recorder.stop()['status']=='complete'
    with h5py.File(ep/'robot_data.h5') as raw:
        stamp=raw['streams/feedback/timestamps'][0]
        assert enqueue_start<=stamp<=queued_before
        assert raw['streams/commands/timestamps'][0]==1_700_000_000.
        assert raw['streams/feedback/source_time_ns'][0]==100
        assert raw['streams/commands/received_time_ns'][0]==500
