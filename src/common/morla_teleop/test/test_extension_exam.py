"""Installed metadata discovery -> two unknown robots -> shared input engine."""
import copy
import json
from pathlib import Path
from morla_core.composition import default_catalog, compile_system
from morla_input.adapters.normalized import InputFrame
from morla_runtime import RuntimeSession
from morla_teleop.session import ControlHost


def test_installed_unknown_plugins_need_only_metadata_factories_and_binding(tmp_path,monkeypatch):
    package=tmp_path/'unified_exam_plugin';package.mkdir()
    (package/'__init__.py').write_text('from morla_runtime.adapters import FakeAdapter\ndef create(instance,context): return FakeAdapter(instance)\n')
    base=default_catalog().get('morla.fake.arm')
    entries=[]
    for suffix in ('alpha','beta'):
        d=copy.deepcopy(base);d['plugin_id']='exam.'+suffix
        (package/(suffix+'.json')).write_text(json.dumps(d))
        entries.append(suffix+' = unified_exam_plugin/'+suffix+'.json')
    d=default_catalog().get('morla.fake.input');d['plugin_id']='exam.input'
    (package/'input.json').write_text(json.dumps(d));entries.append('input = unified_exam_plugin/input.json')
    info=tmp_path/'unified_exam_plugin-1.0.0.dist-info';info.mkdir()
    (info/'METADATA').write_text('Metadata-Version: 2.1\nName: unified-exam-plugin\nVersion: 1.0.0\n')
    (info/'entry_points.txt').write_text('[morla.component_manifests]\n'+'\n'.join(entries)+'\n\n[morla.adapters]\n'+
        '\n'.join('exam.'+name+':fake = unified_exam_plugin:create' for name in ('alpha','beta','input'))+'\n')
    monkeypatch.syspath_prepend(str(tmp_path))
    catalog=default_catalog()
    assembly={'assembly_version':1,'assembly_id':'blind_extension','revision':1,
        'instances':{key:{'plugin':plugin,'backend':'fake'} for key,plugin in
            [('left','exam.alpha'),('right','exam.beta'),('operator','exam.input')]},
        'bindings':[{'id':key,'source':'operator.axes','target':key+'.position_target',
            'controller':'morla.joint_jog','parameters':{'axes_map':['x']+[None]*6,'speed':1}}
            for key in ('left','right')],
        'capture':{'state':['left.position','right.position'],
                   'action':['left.position_target','right.position_target'],'images':{}}}
    clock=[1_000_000_000]
    runtime=RuntimeSession(compile_system(assembly,registry=catalog),registry=catalog,
                           resource_root=tmp_path/'locks',clock=lambda:clock[0])
    host=ControlHost(runtime)
    try:
        runtime.start();runtime.prepare('prepare');runtime.activate('activate')
        assert host.ingest(InputFrame('operator','epoch',0,clock[0],'input',{'axes':{'x':1.}}))['accepted']
        clock[0]+=20_000_000
        assert host.ingest(InputFrame('operator','epoch',1,clock[0],'input',{'axes':{'x':1.}}))['accepted']
        assert runtime.adapters['left'].values[0]==.02
        assert runtime.adapters['right'].values[0]==.02
    finally:host.close();runtime.stop()
