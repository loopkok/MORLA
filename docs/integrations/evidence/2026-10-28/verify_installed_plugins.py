# Run from the MORLA repository root. No physical hardware is opened.
import json, os, pathlib, subprocess, sys, tempfile, zipfile, shutil
import argparse
parser=argparse.ArgumentParser(description='Validate independent device wheels with a substituted ament index')
parser.add_argument('--cr5-wheel',required=True)
parser.add_argument('--loophand-wheel',required=True)
args=parser.parse_args()
wheels=[{'package':'morla_cr5_driver','wheel':args.cr5_wheel},
        {'package':'morla_loophand_driver','wheel':args.loophand_wheel}]
repo=pathlib.Path.cwd()
child=r'''
import sys,os,pathlib,types,json,importlib.util,importlib.metadata
root=pathlib.Path(sys.argv[1]); mode=sys.argv[2];sys.path.insert(0,str(root))
if mode=='cr5':assert importlib.util.find_spec('morla_loophand_driver') is None
if mode=='hand':assert importlib.util.find_spec('morla_cr5_driver') is None
ament=types.ModuleType('ament_index_python');resources=types.ModuleType('ament_index_python.resources')
index=root/'share/ament_index/resource_index/morla_components'
resources.get_resources=lambda kind:{f.name:str(root) for f in index.iterdir()}
resources.get_resource=lambda kind,name:((index/name).read_text(),str(root))
sys.modules['ament_index_python']=ament;sys.modules['ament_index_python.resources']=resources
from morla_core.composition import compile_system,load_document
from morla_core.plugins.registry import default_catalog
from morla_core.plugins.conformance import validate_adapter_contract
from morla_runtime import RuntimeSession
catalog=default_catalog()
if mode=='both':assembly=load_document(pathlib.Path(sys.argv[3])/'examples/cr5_loophand/assembly.fake.yaml');expected=13
else:
 package='morla_cr5_driver' if mode=='cr5' else 'morla_loophand_driver'
 assembly=load_document(root/'share'/package/'config/assembly.fake.yaml');expected=6 if mode=='cr5' else 7
 assert len([e for e in importlib.metadata.entry_points(group='morla.adapters') if e.value.startswith(package+'.')])==2
resolved=compile_system(assembly,registry=catalog)
assert resolved['feature_contract']['state_dim']==expected
assert resolved['feature_contract']['action_dim']==expected
session=RuntimeSession(resolved,registry=catalog,resource_root=root/'locks')
try:
 assert session.start()['status']=='READY_DISARMED'
 for k,a in session.adapters.items():assert validate_adapter_contract(a,session.instances[k])['valid']
 assert session.prepare('p')['status']=='SUCCEEDED'
 assert session.activate('a')['status']=='SUCCEEDED'
finally:session.stop()
assert not session.resources.handles
print(mode+': installed discovery/factory/ABI/runtime PASS; dimensions='+str(expected))
'''
reports=[]
for mode,selected in [('cr5',wheels[:1]),('hand',wheels[1:]),('both',wheels)]:
 with tempfile.TemporaryDirectory(prefix='morla-split-install-') as temp:
  prefix=pathlib.Path(temp)
  for spec in selected:
   with zipfile.ZipFile(spec['wheel']) as archive:
    archive.extractall(prefix)
    base=spec['package']+'-0.1.0.data/data'
    shutil.copytree(prefix/base,prefix,dirs_exist_ok=True)
  env={**os.environ,'PYTHONPATH':str(repo/'src/common/morla_core')+':'+str(repo/'src/common/morla_runtime')}
  result=subprocess.run([sys.executable,'-c',child,str(prefix),mode,str(repo)],env=env,capture_output=True,text=True)
  if result.returncode:print(result.stdout,result.stderr);raise SystemExit(result.returncode)
  reports.append(result.stdout.strip())
print('\n'.join(reports))
