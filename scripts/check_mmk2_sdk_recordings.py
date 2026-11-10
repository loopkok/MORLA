#!/usr/bin/env python3
"""Read-only audit of SDK fake Web recordings; never contacts the controller."""
import argparse
import io
import json
from pathlib import Path
import h5py
import numpy as np
from PIL import Image


def audit_episode(path):
    meta=json.loads((path/'meta.json').read_text());assert meta['recording_status']=='complete',meta.get('failure')
    schema=meta['schema'];assert sum(b['dim'] for b in schema['state_blocks'])==sum(b['dim'] for b in schema['action_blocks'])==19
    base=next(b for b in schema['action_blocks'] if b['writer_id']=='base')
    assert base['unit']=='m_rad/s' and base['semantics']=='velocity' and not base['source_state_indices']
    groups={}
    with h5py.File(path/'robot_data.h5','r') as f:
        for role,dim in [('base',2),('head',2),('lift',1),('left_arm',6),('right_arm',6),('left_gripper',1),('right_gripper',1)]:
            state=f['streams/'+role+('__velocity' if role=='base' else '__position')]
            values=state['values'][:];targets=f['streams/'+role+('__velocity_target' if role=='base' else '__position_target')]['values'][:]
            assert values.shape[1]==targets.shape[1]==dim and len(values)>10 and len(targets)>10
            assert np.isfinite(values).all() and np.isfinite(targets).all()
            assert np.all(state['synthetic'][:]) and not np.any(state['measured'][:])
            assert np.all(np.diff(state['source_time_ns'][:])>0)
            movement=float(np.ptp(values,axis=0).max());assert movement>1e-5,(role,'unchanged')
            groups[role]=dict(state_samples=len(values),action_samples=len(targets),native_value_peak_to_peak=movement)
    cameras={}
    with h5py.File(path/'camera_data.h5','r') as f:
        assert set(f)=={'head','left_wrist','right_wrist'}
        for name in f:
            images=f[name]['images'];assert len(images)>10 and np.all(np.diff(f[name]['timestamps'][:])>0)
            for index in [0,len(images)-1]:
                with Image.open(io.BytesIO(np.asarray(images[index],dtype=np.uint8).tobytes())) as image:
                    assert image.size==(16,16);image.verify()
            cameras[name]=len(images)
    return dict(episode=str(path),status='complete',duration_s=meta['duration_s'],state_dim=19,action_dim=19,groups=groups,camera_jpeg_frames=cameras,synthetic=True,measured=False)


def audit_run(path):
    summary=json.loads((path/'summary.json').read_text());assert summary['outcome']['status']=='STOPPED'
    errors=summary['outcome']['operation']['result']['errors'];assert not errors
    accepted=0;rejected=[];faults=[]
    for file in sorted(path.glob('trace-*.jsonl')):
        for line in file.open():
            row=json.loads(line);data=row.get('data',{})
            if row['kind']=='command_receipt':
                receipt=data.get('receipt',{})
                if receipt.get('accepted') and set(receipt.get('dispatched',[]))=={'base','head','lift','left_arm','right_arm','left_gripper','right_gripper'}:accepted+=1
            if row['kind']=='event' and data.get('event')=='command_rejected':rejected.append(data.get('reason'))
            if row['kind']=='device_fault':faults.append(data)
    return dict(session=path.name,final_status='STOPPED',accepted_all_seven=accepted,rejected=rejected,device_faults=faults,cleanup_errors=errors,dropped=summary.get('dropped',{}))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--store',required=True);parser.add_argument('--output')
    args=parser.parse_args();root=Path(args.store)
    result={'validation':'synthetic SDK protocol only; no MMK2 or Quest hardware / ROS DDS',
            'episodes':[audit_episode(p.parent) for p in sorted((root/'recordings').rglob('meta.json'))],
            'runs':[audit_run(p.parent) for p in sorted((root/'logs').rglob('summary.json'))]}
    text=json.dumps(result,ensure_ascii=False,indent=2)
    if args.output:Path(args.output).write_text(text+'\n')
    print(text)


if __name__=='__main__':main()
