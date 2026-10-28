#!/usr/bin/env python3
"""Reproducible pure compile + actual fake gateway latency sample (not RT proof)."""
import argparse
import hashlib
import inspect
import json
import platform
import statistics
import sys
import tempfile
import time
from pathlib import Path
from morla_core.composition import compile_system,default_catalog
from morla_runtime import RuntimeSession


def stats(samples):
    values=sorted(samples)
    def percentile(q):return values[round((len(values)-1)*q)]/1e6
    return {'iterations':len(values),'unit':'ms','p50':percentile(.50),'p95':percentile(.95),'p99':percentile(.99),'max':max(values)/1e6,'mean':statistics.fmean(values)/1e6}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--iterations',type=int,default=10_000);parser.add_argument('--output',default='docs/architecture_refactor/performance.json');args=parser.parse_args()
    if args.iterations<1:raise ValueError('iterations must be positive')
    assembly={'assembly_version':1,'assembly_id':'benchmark','revision':1,'instances':{'arm':{'plugin':'morla.fake.arm'}},'capture':{'state':['arm.position','arm.velocity'],'action':['arm.position_target']}}
    start=time.perf_counter_ns();catalog=default_catalog();discovery=time.perf_counter_ns()-start
    for _ in range(100):compile_system(assembly,registry=catalog)
    compilation=[]
    for _ in range(args.iterations):
        start=time.perf_counter_ns();resolved=compile_system(assembly,registry=catalog);compilation.append(time.perf_counter_ns()-start)
    with tempfile.TemporaryDirectory(prefix='morla-bench-') as root:
        session=RuntimeSession(resolved,resource_root=root,registry=catalog)
        try:
            session.start();session.prepare('prepare');session.activate('activate')
            permit=session.acquire('benchmark',ttl_ns=5_000_000_000)
            names=[joint['name'] for joint in resolved['instances'][0]['joints']]
            command={'session_id':session.session_id,'producer':'benchmark','token':permit['token'],'epoch':permit['epoch'],'ttl_ns':1_000_000_000,'commands':[{'component_id':'arm','names':names,'values':[0.0]*7,'mode':'position','units':['rad']*7}]}
            latency=[]
            for index in range(args.iterations+100):
                if index and index%1000==0:session.renew('benchmark',permit['token'],5_000_000_000)
                command.update(sequence=index,issued_ns=time.monotonic_ns())
                start=time.perf_counter_ns();receipt=session.submit(command);elapsed=time.perf_counter_ns()-start
                if not receipt['accepted']:raise RuntimeError(receipt)
                if index>=100:latency.append(elapsed)
        finally:session.stop()
    from morla_core.composition import compiler as compiler_module
    source_files={'compiler':inspect.getsourcefile(compiler_module),'runtime_session':inspect.getsourcefile(RuntimeSession),'benchmark':__file__}
    source_hashes={key:hashlib.sha256(Path(path).read_bytes()).hexdigest() for key,path in source_files.items()}
    evidence={'source_sha256':source_hashes,'catalog_hash':catalog.to_dict()['catalog_hash'],'benchmark_version':1,'measured_at_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'environment':{'python':sys.version,'executable':sys.executable,'platform':platform.platform(),'processor':platform.processor()},'source':'fake','hardware_touched':False,'ros_used':False,'hard_realtime_claim':False,'long_duration_claim':False,'warmup_iterations':100,'catalog_discovery_ms':discovery/1e6,'compile':stats(compilation),'gateway_submit':stats(latency),'workload':{'state_dim':14,'action_dim':7,'actuators':1,'capture_disk_write_included':False,'installed_catalog_discovery_in_compile_samples':False,'clock':'perf_counter_ns','command_ttl_ns':1_000_000_000,'resource_locking':'RuntimeSession.start host locks held during gateway samples'},'limits':['Single process/macOS/Python fake backend; does not measure SDK/device latency or ROS transport.','No background camera/network/disk contention workload included.','Percentiles describe this run and do not establish deadline or worst-case guarantees.']}
    target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(evidence,indent=2)+'\n');print(json.dumps({'compile':evidence['compile'],'gateway_submit':evidence['gateway_submit'],'output':str(target)},indent=2))
if __name__=='__main__':main()
