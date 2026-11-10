#!/usr/bin/env python3
"""Local Web acceptance host with explicitly synthetic Quest input.

All configuration, activation and recording remain normal Web operations.
Only fake Quest instances in an already UI-created MMK2 session get frames.
No physical input, ROS service, private command endpoint or auto-activation.
"""
import argparse
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
for pattern in ('src/common/*','src/robots/*/*'):
    for package in ROOT.glob(pattern):
        if (package/'package.xml').is_file():
            sys.path.insert(0,str(package))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store',default='/tmp/morla-mmk2-web-20261109')
    parser.add_argument('--port',type=int,default=8080)
    args=parser.parse_args()
    os.environ.setdefault('AMENT_PREFIX_PATH',sys.prefix)
    from morla_input.adapters.normalized import InputStream
    from morla_web_manager.composable_api import create_app
    import uvicorn
    store=Path(args.store);store.mkdir(parents=True,exist_ok=True)
    control=store/'quest_fixture.json'
    if not control.exists():
        control.write_text(json.dumps(dict(enabled=True,hold=True,amplitude_m=.006)))
    app=create_app(store,resource_root=store/'locks')
    service=app.state.composable
    stop=threading.Event()
    def feed():
        identity=None;sequence=0;started=time.monotonic();counts={}
        while not stop.wait(1/30):
            try:
                settings=json.loads(control.read_text())
                if not settings.get('enabled',True):continue
                with service.lock:
                    session=service.session
                    if session is None or session.status in ('STOPPED','STOPPING','FAULTED','ESTOP_LATCHED'):
                        continue
                    if not any(r['plugin_id'].startswith('morla.airbot.mmk2.') for r in session.instances.values()):continue
                    for key,row in session.instances.items():
                        if row['plugin_id']!='morla.input.quest' or row['backend']!='fake':continue
                        current=(session.session_id,key)
                        if current!=identity:
                            source=InputStream(key,clock=service.clock);source.reconnect()
                            service.inputs[key]=source;identity=current;sequence=0;counts={};started=time.monotonic()
                        source=service.inputs[key]
                        sequence+=1;t=time.monotonic()-started
                        ports={};hold=bool(settings.get('hold',True));amp=float(settings.get('amplitude_m',.006))
                        for side,sign in [('left',1),('right',-1)]:
                            ports[side+'_pose']={'position':[sign*.25+amp*math.sin(t*.7),1.2+amp*math.sin(t*.5),.4+amp*math.sin(t*.9)],'quaternion':[0.,0.,0.,1.]}
                            ports[side+'_hold']=hold
                            ports[side+'_scalar']=.5+.2*math.sin(t*.6)
                            ports[side+'_axes']={'x':.15*math.sin(t*.5),'y':.15*math.sin(t*.4)}
                        frame=source.accept(ports,sequence,epoch=source.epoch,input_basis_id='unity_left_handed_v1')
                        frame=replace(frame,synthetic=True);source.latest=frame
                        service.input_event(session.session_id,key,frame,[])
                        result=service.ingest_frame(session.session_id,frame)
                        reason='accepted' if result.get('accepted') else result.get('reason','held')
                        counts[reason]=counts.get(reason,0)+1
                        if sequence%30==0:
                            (store/'quest_status.json').write_text(json.dumps(dict(session_id=session.session_id,sequence=sequence,synthetic=True,hold=hold,counts=counts,last_result=result)))
            except Exception as exc:
                (store/'quest_error.txt').write_text(repr(exc))
    thread=threading.Thread(target=feed,name='mmk2-synthetic-quest',daemon=True);thread.start()
    print(json.dumps(dict(url=f'http://127.0.0.1:{args.port}',store=str(store),input='synthetic Quest, 30 Hz; no auto activation')),flush=True)
    try:uvicorn.run(app,host='127.0.0.1',port=args.port,log_level='warning')
    finally:stop.set();thread.join(timeout=2)


if __name__=='__main__':main()
