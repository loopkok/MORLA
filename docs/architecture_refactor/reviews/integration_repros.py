"""Independent integration adversarial probes; run with pytest.

These tests now assert corrected behavior. The original vulnerable observations
and first-run result are retained in integration_review.md.
"""
import copy
import json
from pathlib import Path
from unittest.mock import patch
import pytest
from morla_core.composition import compile_system
from morla_web_manager.composable_service import AssemblyStore, ComposableService
from morla_teleop import BindingController
from morla_input.adapters import InputFrame
from morla_learning import ModelManifest, TrainingSpec, validate_model
from morla_learning.contracts import digest
from morla_data_collect.composable_recorder import ComposableRecorder


def source():
    return {'assembly_version':1,'assembly_id':'review','revision':1,'instances':{'grip':{'plugin':'morla.astral.gripper'},'phone_a':{'plugin':'morla.input.mobile'},'phone_b':{'plugin':'morla.input.mobile'}}}

def start(service,value=None):
    value=value or source();saved=service.save({'assembly':value})
    state=service.start({'assembly_id':value['assembly_id'],'revision':1,'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'start'})
    return state['session_id']


@pytest.mark.parametrize("title",["review","混合机器人系统"])
def test_ir01_crash_between_revision_and_pointer_recovers_retry(tmp_path,title):
    store=AssemblyStore(tmp_path);value=source();value['title']=title;resolved=compile_system(value)
    original=Path.replace
    def crash(path,target):
        if path.name.startswith('.latest-'):raise OSError('injected process death before latest pointer')
        return original(path,target)
    with patch.object(Path,'replace',crash):
        with pytest.raises(OSError):store.save(value,None,resolved,0)
    assert (tmp_path/'review/revision-1.json').exists()
    assert not (tmp_path/'review/latest.json').exists()
    restarted=AssemblyStore(tmp_path)
    assert restarted.save(value,None,resolved,0)['assembly']==value
    assert restarted.get('review')['assembly']==value


def test_ir02_stale_disconnect_invalidates_replacement_pairing(tmp_path):
    service=ComposableService(tmp_path,resource_root=tmp_path/'locks')
    try:
        sid=start(service)
        old=service.pair(sid,'phone_a');old_source,old_connection=service.connect_mobile(sid,'phone_a',old['token'])
        new=service.pair(sid,'phone_a');new_source,new_connection=service.connect_mobile(sid,'phone_a',new['token'])
        assert old_source is not new_source and new_source.connected
        service.disconnect_input('phone_a',connection=old_connection)
        assert new_source.connected
        with pytest.raises(ValueError):service.connect_mobile(sid,'phone_a',old['token'])
        assert new_source.connected
        service.disconnect_input('phone_a',connection=new_connection)
        assert not new_source.connected
    finally:service.close()


def test_ir03_deadman_from_wrong_source_authorizes_mapping():
    value=source();binding={'id':'cross','source':'phone_a.scalar','target':'grip.closure','deadman':'phone_b.hold_to_control','controller':'morla.scalar_map'};value['bindings']=[binding]
    with pytest.raises(ValueError,match='cross-source'):compile_system(value)
    with pytest.raises(ValueError,match='source|deadman'):
        BindingController(binding,['joint1'],clock=lambda:100)



def training(features):
    return TrainingSpec(backend='act',backend_revision='locked-commit',dataset_root='/tmp/dataset',dataset_fingerprint='dataset',output_dir='/tmp/output',features=features,geometry_hash='geometry',normalization_hash='norm',capacity=64,camera_map={'model_left':'cam_left','model_right':'cam_right'},parameters={})


def test_ir04_swapped_model_camera_slots_pass_handshake():
    features={'state_features':[{'name':'q','unit':'rad'}],'action_features':[{'name':'q_target','unit':'rad','semantics':'absolute_position'}],'cameras':['cam_left','cam_right']}
    t=training(features)
    manifest=ModelManifest(version=1,backend='act',backend_revision='locked-commit',feature_hash=t.feature_hash,geometry_hash='geometry',normalization_hash='norm',checkpoint_hash='checkpoint',dataset_fingerprint='dataset',features=features,camera_map={'model_left':'cam_right','model_right':'cam_left'},capacity=64,validation='inference_verified')
    with pytest.raises(ValueError,match='camera'):validate_model({'feature_contract':features,'geometry_hash':'geometry'},manifest,normalization_hash='norm',camera_map=t.camera_map)


def test_ir05_training_parameters_override_validated_capacity_and_camera_map():
    features={'state_features':['q'],'action_features':['q_target'],'cameras':['cam_left','cam_right']}
    t=training(features);raw=t.to_dict();raw['parameters']={'capacity':0,'camera_map':{'wrong':'missing'},'type':'unregistered_backend'}
    with pytest.raises(ValueError,match='override'):TrainingSpec.from_dict(raw).config()


def test_ir06_missing_required_streams_marked_complete(tmp_path):
    value=source();value['instances']['camera']={'plugin':'morla.camera'};value['capture']={'images':{'scene':'camera.rgb'}}
    resolved=compile_system(value)
    recorder=ComposableRecorder(resolved['capture_schema'],str(tmp_path));directory=Path(recorder.start())
    result=recorder.stop()
    assert result['status']=='failed'
    meta=json.loads((directory/'meta.json').read_text())
    assert 'required streams' in meta['failure']
    assert all(count==0 for count in meta['counts'].values())
    assert all(count==0 for count in meta['camera_counts'].values())


def test_same_dimension_feature_unit_or_name_changes_rejected():
    from morla_learning import LearningContractError
    features={'state_features':[{'name':'q','unit':'rad'}],'action_features':[{'name':'q_target','unit':'rad','semantics':'absolute_position'}],'cameras':['cam_left','cam_right']}
    t=training(features)
    manifest=ModelManifest(1,'act','locked-commit',t.feature_hash,'geometry','norm','checkpoint','dataset',features,t.camera_map,64,'inference_verified')
    for key,value in [('name','other_joint'),('unit','m')]:
        changed=copy.deepcopy(features);changed['state_features'][0][key]=value
        with pytest.raises(LearningContractError):validate_model({'feature_contract':changed,'geometry_hash':'geometry'},manifest,normalization_hash='norm')


def test_ir07_installed_blind_arm_web_gateway_recording_canary(tmp_path,monkeypatch):
    from morla_core.composition import default_catalog
    descriptor=default_catalog().get('morla.fake.arm');descriptor['plugin_id']='lab.installed.arm'
    descriptor['joints']=descriptor['joints'][:3]
    for port in descriptor['ports'].values():port['dim']=3
    site=tmp_path/'site';site.mkdir();(site/'arm.json').write_text(json.dumps(descriptor))
    metadata=site/'lab_blind_arm-1.0.0.dist-info';metadata.mkdir()
    (metadata/'METADATA').write_text('Metadata-Version: 2.1\nName: lab-blind-arm\nVersion: 1.0.0\n')
    (metadata/'entry_points.txt').write_text('[morla.component_manifests]\narm = arm.json\n')
    monkeypatch.syspath_prepend(str(site))
    service=ComposableService(tmp_path/'service',resource_root=tmp_path/'locks')
    try:
        value={'assembly_version':1,'assembly_id':'blind','revision':1,'instances':{'new_arm':{'plugin':'lab.installed.arm'}},'capture':{'hold_frames':0}}
        sid=start(service,value)
        assert service.session.prepare('prepare')['status']=='SUCCEEDED'
        assert service.session.activate('activate')['status']=='SUCCEEDED'
        episode=Path(service.start_recording(sid)['episode_dir'])
        permit=service.session.acquire('review',ttl_ns=500_000_000)
        instance=service.session.instances['new_arm'];names=[j['name'] for j in instance['joints']]
        receipt=service.session.submit({'session_id':sid,'producer':'review','token':permit['token'],'epoch':permit['epoch'],'sequence':1,'issued_ns':service.clock(),'ttl_ns':200_000_000,'commands':[{'component_id':'new_arm','names':names,'values':[.1,.2,.3],'mode':'position','units':['rad']*3}]})
        assert receipt['accepted'],receipt
        service.capture_tick()
        result=service.stop_recording()
        assert result['status']=='complete',result
        import h5py
        with h5py.File(episode/'robot_data.h5') as raw:
            streams=raw['streams']
            assert list(streams['new_arm__position_target']['values'][-1])==pytest.approx([.1,.2,.3])
            assert list(streams['new_arm__position']['values'][-1])==pytest.approx([.1,.2,.3])
            assert streams['new_arm__position']['source'].asstr()[-1]=='fake'
            assert bool(streams['new_arm__position']['synthetic'][-1])
            assert not bool(streams['new_arm__position']['measured'][-1])
            assert streams['new_arm__position_target']['source'].asstr()[-1]=='dispatched_command'
    finally:service.close()


def test_missing_resolved_hash_is_never_a_fixture_bypass(tmp_path):
    from morla_runtime import RuntimeSession
    resolved=compile_system(source());resolved.pop('resolved_hash')
    with pytest.raises(RuntimeError,match='HASH_REQUIRED'):RuntimeSession(resolved,resource_root=tmp_path)


def test_wrong_source_frame_and_reconnect_require_new_release():
    binding={'id':'bound','source':'phone_a.scalar','target':'grip.closure','deadman':'phone_a.hold_to_control','controller':'morla.scalar_map'}
    controller=BindingController(binding,['joint1'],clock=lambda:100)
    def frame(source,epoch,seq,down):return InputFrame(source,epoch,seq,100,'input',{'scalar':.3,'hold_to_control':down})
    assert controller.step(frame('phone_a','old',0,False),[0]) is None
    assert controller.step(frame('phone_a','old',1,True),[0])==[.3]
    assert controller.step(frame('phone_a','new',0,True),[0]) is None
    assert controller.step(frame('phone_a','new',1,False),[0]) is None
    assert controller.step(frame('phone_a','new',2,True),[0])==[.3]
    with pytest.raises(ValueError,match='bound source'):controller.step(frame('phone_b','spoof',0,True),[0])


def test_fake_adapter_sdk_harness_matches_runtime_api():
    from morla_core.plugins.conformance import exercise_fake_adapter,validate_adapter_contract
    from morla_runtime.adapters import FakeAdapter
    instance=compile_system(source())['instances'][0]
    adapter=FakeAdapter(instance)
    assert validate_adapter_contract(adapter,instance)['valid']
    assert adapter.calls==[]
    assert exercise_fake_adapter(adapter,instance)['valid']
    assert [call[0] for call in adapter.calls]==['configure','prepare','activate','submit','stop','close']
    with pytest.raises(ValueError,match='fake-only'):exercise_fake_adapter(adapter,{**instance,'backend':'real'})
