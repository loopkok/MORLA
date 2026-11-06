"""An unfamiliar plugin must work without editing Web or shared scene code."""
import copy
import json
from pathlib import Path
import pytest
from fastapi.testclient import TestClient
from morla_core.composition import compile_system, CompilationError
from morla_core.plugins.registry import default_catalog
from morla_web_manager.composable_api import create_app

ROOT=Path(__file__).resolve().parents[4]


def external_robot():
    d=json.loads((ROOT/'src/robots/fairino/morla_fr3_driver/descriptor.json').read_text())
    d['plugin_id']='example.new_arm'
    d['web']={'label':'New arm','preferred_controllers':{'pose6d':'morla.cartesian_delta_ik'},
              'templates':[{'id':'example-demo','title':'External demo','package':'example_robot','path':'config/demo.yaml'}]}
    return d


def assembly():
    return {'assembly_version':1,'assembly_id':'external_demo','revision':1,
            'instances':{'robot':{'plugin':'example.new_arm','backend':'sim'},'input':{'plugin':'morla.input.quest'}},
            'mounts':[{'child':'robot.base','parent':'world','transform':[0,0,0,0,0,0,1]}],
            'bindings':[{'id':'follow','source':'input.right_pose','target':'robot.position_target',
                         'controller':'morla.cartesian_delta_ik','input_basis_id':'unity_left_handed_v1','parameters':{}}]}


def test_external_plugin_catalog_template_compile_and_scene(tmp_path,monkeypatch):
    from morla_web_manager import templates
    package=tmp_path/'example_robot';(package/'config').mkdir(parents=True)
    (package/'config/demo.yaml').write_text(json.dumps(assembly()))
    import morla_core.resource_locator as locator
    original=locator.package_share
    monkeypatch.setattr(locator,'package_share',lambda name:package if name=='example_robot' else original(name))
    with TestClient(create_app(tmp_path/'store',resource_root=tmp_path/'locks')) as client:
        service=client.app.state.composable
        service.catalog.register(external_robot())
        assert any(d['plugin_id']=='example.new_arm' for d in client.get('/api/v2/components').json()['components'])
        response=client.get('/api/v2/templates');assert response.status_code==200,response.text
        assert {'id':'example-demo','title':'External demo','assembly_id':'external_demo'} in response.json()
        loaded=client.get('/api/v2/templates/example-demo');assert loaded.status_code==200
        resolved=compile_system(loaded.json(),registry=service.catalog)
        assert any(p[0]=='morla.cartesian_delta_ik@1.0.0' for p in resolved['lockfile']['plugins'])
        pytest.importorskip('mujoco')
        import sys
        sys.path.insert(0,str(ROOT/'src/robot_description/scripts'))
        from composable_scene import build_scene
        scene=build_scene(resolved)
        assert len(scene['joint_map']['robot'])==6
        import mujoco
        model=mujoco.MjModel.from_xml_string(scene['xml'])
        assert model.nq==6
        # Same shape must not bypass a robot-specific controller's target restriction.
        private=json.loads((ROOT/'src/robots/fairino/morla_fr3_driver/controller.json').read_text())
        if not any(d['plugin_id']==private['plugin_id'] for d in service.catalog.list()):service.catalog.register(private)
        bad=loaded.json();bad['bindings'][0]['controller']=private['plugin_id']
        with pytest.raises(CompilationError,match='target plugin'):compile_system(bad,registry=service.catalog)
        descriptor=external_robot();descriptor['plugin_id']='example.other';service.catalog.register(descriptor)
        duplicate=client.get('/api/v2/templates')
        assert duplicate.status_code==422 and 'ambiguous template' in duplicate.text


@pytest.mark.parametrize('path',['../secret.yaml','/tmp/file.yaml','config/../../x.json','file.py'])
def test_template_path_rejected_during_registration(path):
    d=external_robot();d['web']['templates'][0]['path']=path
    with pytest.raises(CompilationError,match='web metadata'):default_catalog().register(d)


def test_missing_optional_package_does_not_advertise_broken_preset(monkeypatch):
    from morla_web_manager import templates
    from morla_core.resource_locator import ResourceLookupError
    catalog=default_catalog();catalog.register(external_robot())
    monkeypatch.setattr(templates,'template_document',lambda entry:(_ for _ in ()).throw(ResourceLookupError('not installed')))
    assert templates.list_templates(catalog)==[]
