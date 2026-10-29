"""Installed input wheel discovery/bridge/service smoke check, no ROS or hardware.
Run from MORLA root: python verify_install.py /path/to/morla_vader5_input.whl
Only ament index queries are substituted. Common framework loads from source.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile

child=r'''
import sys,types,json,time,importlib.util
from importlib import metadata
from pathlib import Path
prefix=Path(sys.argv[1]).resolve();sys.path.insert(0,str(prefix))
assert importlib.util.find_spec('vader5_sdk') is None
for name in ('morla_cr5_driver','nero_arm_teleop','astral_arm_teleop'):
    assert importlib.util.find_spec(name) is None
ament=types.ModuleType('ament_index_python');resources=types.ModuleType('ament_index_python.resources')
index=prefix/'share/ament_index/resource_index/morla_components'
resources.get_resources=lambda kind:{p.name:str(prefix) for p in index.iterdir()}
resources.get_resource=lambda kind,name:((index/name).read_text(),str(prefix))
sys.modules['ament_index_python']=ament;sys.modules['ament_index_python.resources']=resources
from morla_core.composition import compile_system,default_catalog
from morla_web_manager.composable_service import ComposableService
catalog=default_catalog();catalog.get('morla.input.vader5')
assert 'vader5_sdk' not in sys.modules
factory=next(e for e in metadata.entry_points(group='morla.ros_inputs') if e.name=='morla.input.vader5').load()
a={'assembly_version':1,'assembly_id':'installed_pad','revision':1,
    'instances':{'tool':{'plugin':'morla.astral.gripper'},'pad':{'plugin':'morla.input.vader5'}},
    'bindings':[{'id':'scalar','source':'pad.scalar','target':'tool.closure','controller':'morla.scalar_map','deadman':'pad.hold_to_control'}],
    'capture':{'state':['tool.position'],'action':['tool.closure'],'images':{},'auxiliary':['pad.axes','pad.buttons','pad.gyro','pad.raw_report']}}
service=ComposableService(prefix/'store',resource_root=prefix/'locks');service.catalog=catalog
saved=service.save({'assembly':a,'expected_revision':0});state=service.start({'assembly_id':a['assembly_id'],'revision':1,'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'start'})
sid=state['session_id'];pad=service.session.instances['pad']
bridge=factory(pad,clock=time.monotonic_ns,on_transition=lambda:service.disconnect_input('pad'))
service.inputs['pad']=bridge.source;seq=0

def send(buttons,trigger=0):
    global seq
    seq+=1
    row={'version':1,'epoch':'installed','sequence':seq,'sample_ns':time.monotonic_ns(),
      'connected':True,'backend':'fake','control_tick':True,'sticks':[0,0,0,0],'triggers':[0,trigger],
      'buttons':buttons,'gyro':[0,0,0],'accel':[0,0,9.8],'raw_hex':'','unknown_bits':[0,0,0,0]}
    frame=bridge.consume(json.dumps(row));service.input_event(sid,'pad',frame,bridge.actions)
    return service.ingest_frame(sid,frame)

try:
    send([]);send(['START']);assert service.session.status=='ACTIVE'
    send([]);assert send(['M1'],.4)['accepted']
    assert service.session.adapters['tool'].values==[.4]
    assert service.session.adapters['pad'].read(time.monotonic_ns())['synthetic']
    send(['B']);assert service.session.status=='READY_DISARMED'
finally:service.close()
assert not service.session.resources.handles
print('PASS installed input wheel: descriptor without SDK/HID import, selected ROS-source entry point, normalized bridge, service button activation, runtime command, synthetic provenance, disarm/cleanup')
print('Ament query substitution only; no ROS transport or device/robot acceptance claimed')
'''
with tempfile.TemporaryDirectory(prefix='morla-vader5-installed-') as temp:
    prefix=Path(temp)
    with zipfile.ZipFile(sys.argv[1]) as archive:archive.extractall(prefix)
    shutil.copytree(next(prefix.glob('*.data'))/'data',prefix,dirs_exist_ok=True)
    common=[str(p) for p in (Path.cwd()/'src/common').iterdir() if (p/'package.xml').exists() and p.name!='morla_vader5_input']
    result=subprocess.run([sys.executable,'-c',child,str(prefix)],env={**os.environ,'PYTHONPATH':os.pathsep.join(common)},capture_output=True,text=True)
    print(result.stdout,end='')
    if result.returncode:print(result.stderr);raise SystemExit(result.returncode)
