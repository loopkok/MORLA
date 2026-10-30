import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import pytest
from morla_core.composition import Catalog, CompilationError, compile_system, default_catalog, import_profile, load_document, verify_resolved
from morla_core.contracts import SchemaV3, to_v3
from morla_core.profile import validate_profile
from morla_core.composition.legacy_import import legacy_profile_view

ROOT=Path(__file__).resolve().parents[5]

def assembly():
    return {'assembly_version':1,'assembly_id':'workcell','revision':1,'instances':{'a':{'plugin':'morla.fake.arm'},'b':{'plugin':'morla.astral.arm'},'pad':{'plugin':'morla.input.gamepad'}}}

def test_deterministic_detached_and_tamper():
    source=assembly(); frozen=copy.deepcopy(source); result=compile_system(source)
    assert source==frozen
    assert compile_system(source)==result
    assert verify_resolved(result)
    result['instances'][0]['joints'][0]['unit']='m'
    with pytest.raises(CompilationError): verify_resolved(result)
    assert compile_system(source)['instances'][0]['joints'][0]['unit']=='rad'

@pytest.mark.parametrize('patch',[{'command':'rm -rf /'},{'assembly_version':True},{'revision':False},{'revision':0}])
def test_reject_unknown_or_invalid_top_fields(patch):
    value=assembly(); value.update(patch)
    with pytest.raises(CompilationError): compile_system(value)

def test_duplicate_and_nonfinite(tmp_path):
    p=tmp_path/'a.yaml';p.write_text('revision: 1\nrevision: 2\n')
    with pytest.raises(CompilationError): load_document(p)
    value=assembly();value['instances']['a']['parameters']={'rate_hz':float('nan')}
    with pytest.raises(CompilationError): compile_system(value)

@pytest.mark.parametrize('namespace',['/bad/../escape','relative','/bad//path','/bad/'])
def test_namespace_injection(namespace):
    value=assembly();value['instances']['a']['namespace']=namespace
    with pytest.raises(CompilationError): compile_system(value)

def test_shared_astral_session_and_resource_conflict():
    value=assembly();value['instances']['a']['plugin']='morla.astral.gripper'
    deployment={'instances':{i:{'session_id':'board','device_id':'one','resource_key':'usb:one'} for i in ('a','b')}}
    result=compile_system(value,deployment)
    assert len([s for s in result['device_sessions'] if s['session_id']=='board'])==1
    deployment['instances']['a']['session_id']='other'
    with pytest.raises(CompilationError,match='collision'): compile_system(value,deployment)

def test_shared_incompatible_sdk_rejected():
    value=assembly();deployment={'instances':{i:{'session_id':'board'} for i in ('a','b')}}
    with pytest.raises(CompilationError,match='incompatible'): compile_system(value,deployment)

def test_binding_capabilities_and_duplicate_writer():
    value=assembly();value['bindings']=[{'id':'jog','source':'pad.axes','target':'a.joint_target','controller':'morla.joint_jog','deadman':'pad.right_shoulder'}]
    assert compile_system(value)['bindings']
    value['bindings'][0]['source']='pad.left_landmarks'
    with pytest.raises(CompilationError,match='unresolved port'): compile_system(value)
    value['bindings'][0]['source']='pad.axes'
    value['bindings'].append({**value['bindings'][0],'id':'second'})
    with pytest.raises(CompilationError,match='multiple input'): compile_system(value)

def test_mount_cycle_and_multiple_parent():
    value=assembly();value['mounts']=[{'child':'a.base','parent':'b.flange'},{'child':'b.base','parent':'a.flange'}]
    with pytest.raises(CompilationError,match='cycle'): compile_system(value)
    value['mounts']=[{'child':'a.base','parent':'world'},{'child':'a.flange','parent':'workcell'}]
    with pytest.raises(CompilationError,match='multiple parents'): compile_system(value)

def test_camera_only_and_zero_component():
    value=assembly();value['instances']={'cam':{'plugin':'morla.camera'}};value['capture']={'images':{'ceiling':'cam.rgb'}}
    result=compile_system(value)
    assert result['feature_contract']['state_dim']==result['feature_contract']['action_dim']==0
    assert result['capture_schema']['cameras']==['ceiling']
    value['instances']={};value['capture']={}
    assert compile_system(value)['capture_schema']['cameras']==[]

def test_state_action_split_and_semantic_hash():
    value=assembly();value['capture']={'state':['a.position','a.velocity'],'action':['a.position_target'],'images':{}}
    result=compile_system(value)
    schema=SchemaV3.from_dict(result['capture_schema'])
    assert (schema.state_dim,schema.action_dim)==(14,7)
    assert schema.action_blocks[0].source_state_indices==tuple(range(7))
    changed=copy.deepcopy(value);changed['instances']['pad']['plugin']='morla.input.mobile'
    assert compile_system(changed)['feature_contract_hash']==result['feature_contract_hash']
    changed=copy.deepcopy(value);changed['instances']['a']['namespace']='/different/arm'
    assert compile_system(changed)['feature_contract_hash']==result['feature_contract_hash']
    changed['capture']['action']=['b.position_target']
    assert compile_system(changed)['feature_contract_hash']!=result['feature_contract_hash']

def test_real_never_fake_fallback():
    value=assembly();value['instances']['a']['backend']='real'
    with pytest.raises(CompilationError,match='device_id'): compile_system(value)
    result=compile_system(value,{'instances':{'a':{'device_id':'physical_one'}}})
    assert result['instances'][0]['backend']=='real'
    assert result['status']['backend_available'] is False
    assert any(d['error_code']=='BACKEND_UNAVAILABLE' for d in result['diagnostics'])

def test_group_mixed_backends_are_compiled():
    value=assembly();value['instances']['b']['backend']='sim';value['control_groups']={'both':{'members':['a','b']}}
    assert compile_system(value)['control_groups'][0]['members']==['a','b']

def test_catalog_ambiguity_and_blind_plugin():
    catalog=default_catalog();desc=catalog.get('morla.fake.arm')
    with pytest.raises(CompilationError,match='ambiguous'): catalog.register(desc)
    desc['plugin_id']='lab.new.arm';desc['joints']=desc['joints'][:3]
    for port in desc['ports'].values(): port['dim']=3
    catalog.register(desc)
    value=assembly();value['instances']={'custom':{'plugin':'lab.new.arm'}}
    assert compile_system(value,registry=catalog)['feature_contract']['action_dim']==3

def test_v3_legacy_explicit_nonmutating():
    files=sorted((ROOT/'src/robots').glob('**/config/profiles/*.yaml'))
    assert len(files)==12
    for path in files:
        profile=validate_profile(load_document(path));before=profile.frozen_schema.to_dict()
        migrated=to_v3(profile.frozen_schema)
        assert migrated.metadata['migration']['source_schema_hash']==profile.frozen_schema.schema_hash
        assert profile.frozen_schema.to_dict()==before
        imported=import_profile(profile);resolved=compile_system(imported)
        assert legacy_profile_view(resolved)==profile.values
        assert resolved['source_assembly']['legacy']['schema_hash']==profile.frozen_schema.schema_hash

def test_hash_seed_independent():
    script="from morla_core.composition import compile_system; print(compile_system("+repr(assembly())+")[\"resolved_hash\"])"
    results=[subprocess.check_output([sys.executable,'-c',script],env={**os.environ,'PYTHONHASHSEED':str(seed)},text=True).strip() for seed in (1,92)]
    assert len(set(results))==1

def test_legacy_golden_hashes_unchanged():
    import hashlib
    rows=json.loads((ROOT/'test/fixtures/legacy_profiles/index.json').read_text())
    assert len(rows)==12
    for row in rows:
        path=ROOT/row['source_path']
        assert hashlib.sha256(path.read_bytes()).hexdigest()==row['source_sha256']
        current=validate_profile(load_document(path)).frozen_schema
        golden=json.loads((ROOT/'test/fixtures/legacy_profiles'/(row['profile_id']+'.json')).read_text())
        assert current.schema_hash==row['schema_hash']==golden['schema_hash']
        assert current.layout_hash==row['layout_hash']==golden['layout_hash']

def test_sensor_raw_only_and_calibration_content_identity():
    value=assembly();value['instances']['touch']={'plugin':'morla.fake.sensor'}
    value['capture']={'state':['a.position'],'action':[],'auxiliary':['a.velocity'],'sensors':[{'sensor_key':'wrist_force','source':'touch.values','fields':{'values':{'shape':[3],'dtype':'float64'}}}]}
    first=compile_system(value,{'calibrations':{'mount':{'translation':[0,0,0]}}})
    second=compile_system(value,{'calibrations':{'mount':{'translation':[1,0,0]}}})
    assert first['geometry_hash']!=second['geometry_hash']
    assert first['feature_contract']['state_dim']==7
    assert first['capture_schema']['sensor_specs'][0]['source_stream']=='touch__values'
    assert first['capture_schema']['metadata']['stream_sources']['a__velocity']=={'instance_id':'a','port_id':'velocity'}


def test_cross_source_deadman_is_compiled():
    value=assembly();value['instances']['other']={'plugin':'morla.input.gamepad'}
    value['bindings']=[{'id':'unsafe','source':'pad.axes','target':'a.joint_target','controller':'morla.joint_jog','deadman':'other.right_shoulder'}]
    assert compile_system(value)['bindings'][0]['deadman']=='other.right_shoulder'

def test_installed_manifest_discovered_without_import(tmp_path,monkeypatch):
    desc=default_catalog().get('morla.fake.arm');desc['plugin_id']='lab.installed.arm'
    for port in desc['ports'].values():port['dim']=3
    desc['joints']=desc['joints'][:3]
    manifests=tmp_path/'manifests';manifests.mkdir();(manifests/'arm.json').write_text(json.dumps(desc))
    dist=tmp_path/'morla_blind_fixture-1.0.0.dist-info';dist.mkdir()
    (dist/'METADATA').write_text('Metadata-Version: 2.1\nName: morla-blind-fixture\nVersion: 1.0.0\n')
    (dist/'entry_points.txt').write_text('[morla.component_manifests]\narm = manifests/arm.json\n')
    monkeypatch.syspath_prepend(str(tmp_path))
    assert default_catalog().get('lab.installed.arm')['kind']=='actuator'
    value=assembly();value['instances']={'third':{'plugin':'lab.installed.arm'}}
    assert compile_system(value)['feature_contract']['action_dim']==3
    (dist/'entry_points.txt').write_text('[morla.component_manifests]\narm = ../../arbitrary.py\n')
    with pytest.raises(ValueError,match='relative JSON/YAML'):default_catalog()

def test_group_identity_and_fault_policy_strict():
    value=assembly();value['control_groups']=[{'group_id':'duplicate','members':['a']},{'group_id':'duplicate','members':['b']}]
    with pytest.raises(CompilationError,match='duplicate control group'):compile_system(value)
    value['control_groups']=[{'group_id':'one','members':['a'],'fault_policy':'continue_unsafe'}]
    assert compile_system(value)['control_groups'][0]['fault_policy']=='continue_unsafe'
    value['control_groups']=[{'group_id':'one','members':['a'],'allow_mixed_backends':'false'}]
    with pytest.raises(CompilationError,match='boolean'):compile_system(value)

@pytest.mark.parametrize('document',['cycle: &cycle [*cycle]', 'cycle: &cycle {self: *cycle}', 'a: '+ '['*65+'0'+']'*65])
def test_bounded_yaml_rejects_cycles_and_depth(tmp_path,document):
    path=tmp_path/'unsafe.yaml';path.write_text(document)
    with pytest.raises(CompilationError):load_document(path)

def test_yaml_alias_limits_and_legitimate_shared_legacy_anchors(tmp_path):
    path=tmp_path/'aliases.yaml';path.write_text('base: &base [1, 2, 3]\na: *base\nb: *base\n')
    assert load_document(path)=={'base':[1,2,3],'a':[1,2,3],'b':[1,2,3]}
    path.write_text('base: &base [1]\nvalues: ['+', '.join(['*base']*129)+']')
    with pytest.raises(CompilationError,match='alias'):load_document(path)
    path.write_text('value: '+('a'*1_000_001))
    with pytest.raises(CompilationError,match='size'):load_document(path)

def test_alias_expansion_bomb_bounded_before_hashing(tmp_path):
    path=tmp_path/'bomb.yaml';rows=['a0: &a0 [0, 1]']
    for index in range(1,20):rows.append(f'a{index}: &a{index} [*a{index-1}, *a{index-1}]')
    path.write_text('\n'.join(rows))
    with pytest.raises(CompilationError,match='node limit'):load_document(path)

def test_conformance_harness_structural_check_has_no_side_effects():
    from morla_core.plugins.conformance import validate_adapter_contract
    class Adapter:
        def configure(self,context):raise AssertionError('must not configure')
        def health(self,now_ns):raise AssertionError('must not read')
        def close(self):raise AssertionError('must not close')
    assert validate_adapter_contract(Adapter(),{'kind':'sensor'})['hardware_touched'] is False
    class Wrong(Adapter):
        def health(self):pass
    with pytest.raises(TypeError,match='SIGNATURE'):validate_adapter_contract(Wrong(),{'kind':'sensor'})
