"""Run from MORLA root with --input-wheel and --driver-wheel. No hardware opens."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input-wheel',required=True)
parser.add_argument('--driver-wheel',required=True)
args=parser.parse_args()
child=r'''
import importlib.util,json,sys,time,types
from pathlib import Path
from fastapi.testclient import TestClient
prefix=Path(sys.argv[1]);sys.path.insert(0,str(prefix))
assert importlib.util.find_spec('morla_loophand_driver') is None
assert importlib.util.find_spec('nero_arm_teleop') is None
assert importlib.util.find_spec('astral_arm_teleop') is None
ament=types.ModuleType('ament_index_python');resources=types.ModuleType('ament_index_python.resources')
index=prefix/'share/ament_index/resource_index/morla_components'
resources.get_resources=lambda kind:{p.name:str(prefix) for p in index.iterdir()}
resources.get_resource=lambda kind,name:((index/name).read_text(),str(prefix))
sys.modules['ament_index_python']=ament;sys.modules['ament_index_python.resources']=resources
from morla_core.composition import load_document
from morla_web_manager.composable_api import create_app
from morla_input.quest_right import QuestRightSource
from morla_cr5_driver.vr_session import attach_control_page
app=create_app(prefix/'store',resource_root=prefix/'locks',mount_web=False)
service=app.state.composable
assembly=load_document(prefix/'share/morla_cr5_driver/config/quest_right.fake.yaml')
saved=service.save({'assembly':assembly,'expected_revision':0})
assert 'morla.cr5.cartesian_delta@1.0.0' in dict(saved['resolved']['lockfile']['plugins'])
state=service.start({'assembly_id':assembly['assembly_id'],'revision':1,
                    'assembly_hash':saved['resolved']['assembly_hash'],'request_id':'installed'})
sid=state['session_id'];attach_control_page(app,service,sid)
source=QuestRightSource('quest',on_transition=lambda:service.disconnect_input('quest'))
service.inputs['quest']=source;sequence=0

def send(kind,values):
    global sequence
    sequence+=1
    frame=source.event({'version':1,'side':'right','kind':kind,'source':'controller',
        'epoch':'installed-test','sequence':sequence,'sent_ns':time.time_ns(),
        'frame_id':'quest_canonical','values':values})
    return service.ingest_frame(sid,frame) if source.control_event({'kind':kind},frame) else None

with TestClient(app) as client:
    assert client.get('/').status_code==200
    assert client.get('/cr5-vr/session').json()['backends']['arm']=='fake'
    for operation in ('prepare','activate'):
        reply=client.post(f'/api/v2/sessions/{sid}/operations',json={'operation':operation,'request_id':operation})
        assert reply.status_code==200,reply.text
    send('pose',[0,0,0,0,0,0,1]);send('grip',0.);send('grip',1.);send('pose',[0,0,0,0,0,0,1])
    controller=service.bindings['right_vr'].registry.plugins['morla.cr5.cartesian_delta']
    deadline=time.monotonic()+1
    while controller.result is None and time.monotonic()<deadline:time.sleep(.002)
    assert controller.result is not None
    deadline=time.monotonic()+1
    while service.session.adapters['arm'].values[0]<=0 and time.monotonic()<deadline:
        time.sleep(.01);send('pose',[.002,0,0,0,0,0,1]);send('grip',1.)
    assert service.session.adapters['arm'].values[0]>0
    send('grip',0.)
    assert service.session.lease is None and service.session.status=='READY_DISARMED'
assert not controller.thread.is_alive()
assert not service.session.resources.handles
print('Installed input/CR5 wheels: static discovery, controller entry point, UI HTTP, right-controller motion, release and cleanup PASS')
print('Ament index queries substituted; all runtime/controller/adapter code loaded normally; no ROS or hardware acceptance claimed')
'''
with tempfile.TemporaryDirectory(prefix='morla-cr5-vr-installed-') as temp:
    prefix=Path(temp)
    for wheel in (args.input_wheel,args.driver_wheel):
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(prefix)
        data=prefix/(Path(wheel).name.split('-py3-')[0]+'.data')/'data'
        shutil.copytree(data,prefix,dirs_exist_ok=True)
    common=[str(p) for p in (Path.cwd()/'src/common').iterdir() if (p/'package.xml').exists()
            and p.name!='morla_input']
    env={**os.environ,'PYTHONPATH':os.pathsep.join(common)}
    result=subprocess.run([sys.executable,'-c',child,str(prefix)],env=env,capture_output=True,text=True)
    print(result.stdout,end='')
    if result.returncode:print(result.stderr);raise SystemExit(result.returncode)
