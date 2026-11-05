"""Installed FR3 sim + actual renderer + synthetic normalized Quest input.

MUJOCO_GL=egl python3 docs/testing/fr3_sim_camera_acceptance.py --output-root /tmp/fr3-sim-camera
No hardware backend or vendor SDK is used. Requires installed MORLA and TestClient.
"""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,required=True)
    args=parser.parse_args();root=args.output_root.resolve()
    if (root/'store').exists():parser.error('Use a new output-root to preserve previous run evidence')
    root.mkdir(parents=True,exist_ok=True)
    from fastapi.testclient import TestClient
    from morla_web_manager.composable_api import create_app
    from morla_input.adapters.normalized import InputStream
    app=create_app(root/'store',resource_root=root/'locks',diagnostic_root=root/'logs')
    with TestClient(app) as client:
        a=client.get('/api/v2/templates/fr3-sim').json()
        assert a['instances']['arm']['backend']=='sim'
        a['metadata']['simulation']={'mode':'physics','display':'web'}
        saved=client.post('/api/v2/assemblies',json={'assembly':a,'expected_revision':0})
        assert saved.status_code==200,saved.text
        started=client.post('/api/v2/sessions',json={'assembly_id':a['assembly_id'],'revision':1,
            'assembly_hash':saved.json()['resolved']['assembly_hash'],'request_id':'start'})
        assert started.status_code==200,started.text
        sid=started.json()['session_id'];svc=app.state.composable
        source=InputStream('quest');source.reconnect();sequence=0
        def send(x):
            nonlocal sequence
            sequence+=1
            return svc.ingest_frame(sid,source.accept({'right_pose':{'position':[x,0,0],
                'quaternion':[0,0,0,1]},'right_hold':True},sequence,epoch=source.epoch,
                input_basis_id='unity_left_handed_v1'))
        try:
            until=time.monotonic()+10
            while not svc.session.adapters['scene_camera'].health(svc.clock()).get('ready'):
                assert time.monotonic()<until,'actual camera did not become ready'
                time.sleep(.02)
            send(0)
            for name in ('prepare','activate'):
                r=client.post(f'/api/v2/sessions/{sid}/operations',json={'operation':name,'request_id':name})
                until=time.monotonic()+3
                while r.json()['status']=='RUNNING' and time.monotonic()<until:
                    time.sleep(.01);r=client.get(f'/api/v2/sessions/{sid}/operations/{name}')
                assert r.json()['status']=='SUCCEEDED',r.text
            send(0)
            r=client.post(f'/api/v2/sessions/{sid}/recordings',json={'task':'fr3_sim_synthetic_quest_camera'})
            assert r.status_code==200,r.text
            episode=Path(r.json()['episode_dir']);accepted=0
            for k in range(120):
                accepted+=bool(send(min(k*.0001,.01))['accepted']);time.sleep(.02)
            stopped=client.post(f'/api/v2/sessions/{sid}/recordings/stop',json={})
            assert stopped.status_code==200,stopped.text
            meta=json.loads((episode/'meta.json').read_text())
            summary={'status':meta['recording_status'],'accepted_frames':accepted,
                'counts':meta['counts'],'camera_counts':meta['camera_counts'],
                'source':'simulation_with_synthetic_Quest','episode_dir':str(episode),
                'vendor_sdk_loaded':any('vendor.Robot' in name for name in sys.modules)}
            (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
            print(json.dumps(summary,indent=2))
            assert meta['recording_status']=='complete',meta.get('failure')
            assert accepted>0 and meta['counts']['arm__position_target']>0
            assert meta['camera_counts']['scene']>0 and not summary['vendor_sdk_loaded']
        finally:
            svc.stop_recording()
            svc.operation(sid,{'operation':'stop','request_id':'cleanup'})


if __name__=='__main__':main()
