#!/usr/bin/env python3
"""Read-only audit of the MMK2 Web acceptance recordings and lifecycle logs."""
import argparse
import io
import json
from pathlib import Path
import h5py
import numpy as np
from PIL import Image


def audit_episode(path):
    meta=json.loads((path/'meta.json').read_text())
    assert meta['recording_status']=='complete',meta.get('failure')
    schema=meta['schema']
    assert sum(b['dim'] for b in schema['state_blocks'])==19
    assert sum(b['dim'] for b in schema['action_blocks'])==19
    base=next(b for b in schema['action_blocks'] if b['writer_id']=='base')
    assert base['unit']=='rad/s' and base['semantics']=='velocity'
    groups={}
    with h5py.File(path/'robot_data.h5','r') as f:
        for role,dim in [('base',2),('head',2),('lift',1),('left_arm',6),('right_arm',6),('left_gripper',1),('right_gripper',1)]:
            state=f['streams/'+role+'__position'];values=state['values'][:]
            action=f['streams/'+role+('__velocity_target' if role=='base' else '__position_target')]
            targets=action['values'][:]
            assert values.shape[1]==dim and targets.shape[1]==dim
            assert len(values)>10 and len(targets)>10
            assert np.isfinite(values).all() and np.isfinite(targets).all()
            assert np.all(state['synthetic'][:]) and not np.any(state['measured'][:])
            assert np.all(np.diff(state['timestamps'][:])>0)
            movement=float(np.ptp(values,axis=0).max())
            assert movement>1e-5,(role,'no observed movement')
            groups[role]=dict(state_samples=len(values),action_samples=len(targets),max_joint_peak_to_peak=movement)
        sensors={}
        for name,dim in [('odometry',7),('gyro',3),('acceleration',3),('lidar',180)]:
            stream=f['streams/sensors__'+name];values=stream['values'][:]
            assert values.shape[1]==dim and len(values)>5 and np.isfinite(values).all()
            times=stream['source_time_ns'][:]
            assert np.all(np.diff(times)>0)
            sensors[name]=dict(samples=len(values),dim=dim)
    cameras={}
    with h5py.File(path/'camera_data.h5','r') as f:
        assert set(f)=={'head','left_wrist','right_wrist'}
        for name in f:
            images=f[name]['images'];times=f[name]['timestamps'][:]
            assert len(images)>10 and np.all(np.diff(times)>0)
            for index in (0,len(images)-1):
                blob=np.asarray(images[index],dtype=np.uint8).tobytes()
                with Image.open(io.BytesIO(blob)) as image:
                    assert image.size==(320,240)
                    image.verify()
            cameras[name]=dict(jpeg_frames=len(images),observed_fps=float((len(times)-1)/(times[-1]-times[0])))
    return dict(episode=str(path),task=meta['task'],status='complete',duration_s=meta['duration_s'],state_dim=19,action_dim=19,groups=groups,sensors=sensors,cameras=cameras)


def audit_run(path):
    summary=json.loads((path/'summary.json').read_text())
    assert summary['outcome']['status']=='STOPPED'
    assert not summary['outcome']['operation']['result']['errors']
    assert not summary.get('error') and not any(summary.get('dropped',{}).values())
    release=None;residual=[];accepted=0;all_seven=0;components={};mode_changes=[];events=set()
    for file in sorted(path.glob('trace-*.jsonl')):
        for line in file.open():
            row=json.loads(line);data=row.get('data',{})
            if row['kind']=='input' and data.get('ports',{}).get('right_hold') is False and release is None:
                release=row['monotonic_ns']
            if row['kind']=='feedback' and row.get('component_id')=='base' and release and row['monotonic_ns']>release+600_000_000:
                velocity=data.get('ports',{}).get('velocity',{}).get('values')
                if velocity:residual.append(max(map(abs,velocity)))
            if row['kind']=='command_receipt' and data.get('receipt',{}).get('accepted'):
                accepted+=1
                dispatched=set(data['receipt']['dispatched'])
                if dispatched=={'base','head','lift','left_arm','right_arm','left_gripper','right_gripper'}:all_seven+=1
                for key in dispatched:components[key]=components.get(key,0)+1
            if row['kind']=='event':
                events.add(data.get('event'))
                if data.get('event')=='simulation_options':mode_changes.append(data.get('mode'))
    result=dict(session_id=summary['outcome']['operation'].get('session_id',path.name.split('_')[-1]),status='STOPPED',accepted_commands=accepted,accepted_all_seven_commands=all_seven,command_receipts_by_component=components,dropped=summary.get('dropped',{}),mode_changes=mode_changes,cleanup_errors=[])
    assert all_seven>0 and len(components)==7
    if residual:
        result['grip_release_wheel_max_rad_s_after_600ms']=max(residual)
        assert max(residual)<1e-3
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store',default='/tmp/morla-mmk2-web-20261109')
    parser.add_argument('--output',default=None)
    args=parser.parse_args();root=Path(args.store)
    result=dict(episodes=[audit_episode(p.parent) for p in sorted((root/'recordings').rglob('meta.json'))],
                runs=[audit_run(p.parent) for p in sorted((root/'logs').rglob('summary.json'))])
    text=json.dumps(result,indent=2)
    if args.output:Path(args.output).write_text(text+'\n')
    print(text)


if __name__=='__main__':main()
