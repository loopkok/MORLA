import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import pytest
from morla_core.composition import compile_system
from morla_input.adapters.normalized import InputFrame
from morla_runtime import RuntimeSession
from morla_runtime.diagnostic_log import DiagnosticLog, snapshot
from morla_teleop.session import ControlHost


def resolved(options=None):
    return compile_system({'assembly_version':1,'assembly_id':'diagnostic_test','revision':1,
        'instances':{'arm':{'plugin':'morla.fake.arm'},'pad':{'plugin':'morla.input.gamepad'}},
        'bindings':[{'id':'move','source':'pad.axes','target':'arm.position_target','controller':'morla.joint_jog',
                    'parameters':{'axes_map':['left_x']*7,'speed':.2}}],
        'metadata':{'diagnostic_logging': options or {}}})


def records(directory):
    return [json.loads(line) for path in sorted(Path(directory).glob('trace-*.jsonl')) for line in path.read_text().splitlines()]


def test_run_diagnostics_without_dataset_recorder_and_after_restart(tmp_path):
    system=resolved();runtime=RuntimeSession(system,resource_root=tmp_path/'locks',diagnostic_root=tmp_path/'logs')
    assert not (tmp_path/'logs').exists() # Preflight/constructor do not create a run.
    runtime.start();host=ControlHost(runtime)
    directory=Path(runtime.state()['diagnostic_log']['directory'])
    try:
        runtime.activate('activate')
        for seq in range(3):
            frame=InputFrame('pad','epoch',seq,runtime.clock(),'operator',{'axes':{'left_x':.5}})
            assert host.ingest(frame)['accepted']
            runtime.tick();time.sleep(.021)
        runtime.trace('adapter_extra',{'measured':False,'temperature_c':32.},component_id='arm')
    finally:host.close();runtime.stop()
    rows=records(directory);kinds={row['kind'] for row in rows}
    assert {'run_started','input','control','command_request','command_receipt','feedback','event','adapter_extra','run_ended'}<=kinds
    assert len([r for r in rows if r['kind']=='input'])==3 # Host and engine share dedup.
    assert all(r['session_id']==runtime.session_id for r in rows)
    assert [r['record_sequence'] for r in rows]==sorted(r['record_sequence'] for r in rows)
    control=next(r for r in rows if r['kind']=='control')
    assert control['binding_id']=='move' and control['component_id']=='arm'
    assert control['data']['feedback']['source']=='fake'
    assert not control['data']['feedback']['measured']
    assert (directory/'manifest.json').is_file()
    summary=json.loads((directory/'summary.json').read_text())
    assert summary['clean_shutdown'] and summary['outcome']['status']=='STOPPED'
    assert summary['written']['command_receipt']==3
    second=RuntimeSession(system,resource_root=tmp_path/'locks',diagnostic_root=tmp_path/'logs')
    second.start();second.stop()
    assert second.diagnostics.directory!=directory
    assert len(list((tmp_path/'logs').iterdir()))==2


def test_failed_start_is_preserved_and_logging_can_be_disabled(tmp_path):
    runtime=RuntimeSession(resolved(),resource_root=tmp_path/'locks',diagnostic_root=tmp_path/'logs')
    class Broken:
        def configure(self, context):raise RuntimeError('broken adapter fixture')
        def stop(self):return True
        def close(self):return True
    runtime.adapters['arm']=Broken()
    with pytest.raises(RuntimeError,match='broken'):runtime.start()
    rows=records(runtime.diagnostics.directory)
    assert any(r['kind']=='start_failed' and 'broken' in r['data']['reason'] for r in rows)
    disabled=RuntimeSession(resolved({'enabled':False}),resource_root=tmp_path/'other-lock',diagnostic_root=tmp_path/'disabled')
    disabled.start();disabled.stop()
    assert not (tmp_path/'disabled').exists()
    assert disabled.state()['diagnostic_log']['state']=='disabled'


def test_redaction_binary_omission_and_bounded_snapshots(tmp_path):
    value={'token':'auth','password':'pwd','device':{'secret_ref':'vault','position':[1,2]},'image':b'pixels'}
    result=snapshot(value)
    assert result['token']==result['device']['secret_ref']=='[redacted]'
    assert result['image']=={'omitted':'binary','bytes':6}
    assert result['device']['position']==[1,2]
    nested={};nested['self']=nested
    assert 'snapshot_limit' in json.dumps(snapshot(nested))
    assert len(snapshot(list(range(10000))))==513
    logger=DiagnosticLog('a'*32,resolved(),root=tmp_path);logger.start()
    logger.emit('extension',value);logger.close()
    content=''.join(p.read_text() for p in logger.directory.iterdir() if p.is_file())
    assert 'pixels' not in content and 'vault' not in content and '"pwd"' not in content


def test_slow_disk_and_queue_overflow_do_not_block_motion(tmp_path):
    entered,release=threading.Event(),threading.Event()
    runtime=RuntimeSession(resolved({'queue_size':16}),resource_root=tmp_path/'locks',diagnostic_root=tmp_path/'logs')
    original=runtime.diagnostics._open_segment
    def blocked(index):entered.set();release.wait(3);return original(index)
    runtime.diagnostics._open_segment=blocked
    runtime.start();assert entered.wait(.5)
    runtime.activate('activate')
    began=time.perf_counter()
    for index in range(100):
        assert runtime.submit({'sequence':index,'commands':[{'component_id':'arm','values':[.1]*7} ]})['accepted']
    assert time.perf_counter()-began<.5
    assert sum(runtime.diagnostics.status()['dropped'].values())>0
    release.set();runtime.stop()
    summary=json.loads((runtime.diagnostics.directory/'summary.json').read_text())
    assert not summary['clean_shutdown'] and summary['state']=='degraded'


def test_unwritable_root_never_prevents_motion(tmp_path):
    path=tmp_path/'not_a_directory';path.write_text('blocked')
    runtime=RuntimeSession(resolved(),resource_root=tmp_path/'locks',diagnostic_root=path)
    runtime.start();runtime.activate('activate')
    assert runtime.submit({'commands':[{'component_id':'arm','values':[.1]*7}]})['accepted']
    assert runtime.state()['diagnostic_log']['state']=='degraded'
    assert 'LOG_START_FAILED' in runtime.diagnostics.error
    runtime.stop()


def test_rotation_budget_and_writer_failure_are_explicit(tmp_path):
    logger=DiagnosticLog('b'*32,resolved({'segment_mb':1,'max_run_mb':3}),root=tmp_path)
    logger.start()
    for index in range(650):
        logger.emit('large',{'values':['x'*4000]*2})
        if index%20==0: logger.queue.join()
    logger.close()
    assert len(logger.segments)>=2
    assert logger.status()['error']=='LOG_RUN_BUDGET_EXHAUSTED'
    assert logger.status()['dropped']['large']>0
    broken=DiagnosticLog('c'*32,resolved(),root=tmp_path)
    def fail(_):raise OSError('disk full fixture')
    broken._open_segment=fail;broken.start()
    broken.thread.join(.5)
    assert 'LOG_WRITE_FAILED' in broken.status()['error']
    assert json.loads((broken.directory/'summary.json').read_text())['clean_shutdown'] is False


def test_feedback_sampling_deduplicates_samples_but_preserves_fault_changes(tmp_path):
    logger=DiagnosticLog('d'*32,resolved({'feedback_hz':10}),root=tmp_path)
    logger.start()
    sample={'sample_ns':100,'values':[1.],'source':'hardware','measured':True,'synthetic':False,'ready':True}
    logger.feedback('arm',sample,now_ns=100)
    logger.feedback('arm',sample,now_ns=200_000_000)
    logger.feedback('arm',{**sample,'sample_ns':200},now_ns=200_000_001)
    logger.feedback('arm',{**sample,'sample_ns':201},now_ns=200_000_002)
    logger.feedback('arm',{**sample,'ready':False,'fault':True},now_ns=200_000_003)
    logger.close()
    rows=[r for r in records(logger.directory) if r['kind']=='feedback']
    assert len(rows)==3
    assert rows[-1]['data']['fault']
    assert rows[0]['data']['measured'] is True


def test_new_robot_without_diagnostic_plugin_gets_base_trace(tmp_path):
    from morla_core.plugins.registry import default_catalog
    catalog=default_catalog();descriptor=catalog.get('morla.fake.arm')
    descriptor['plugin_id']='vendor.new_robot';catalog.register(descriptor)
    source=resolved()['source_assembly'];source['instances']['arm']['plugin']='vendor.new_robot'
    runtime=RuntimeSession(compile_system(source,registry=catalog),registry=catalog,resource_root=tmp_path/'locks',diagnostic_root=tmp_path/'logs')
    runtime.start();host=ControlHost(runtime)
    try:
        runtime.activate('activate')
        assert host.ingest(InputFrame('pad','epoch',0,runtime.clock(),'operator',{'axes':{'left_x':.5}}))['accepted']
        runtime.tick()
    finally:host.close();runtime.stop()
    rows=records(runtime.diagnostics.directory)
    assert {'input','control','command_receipt','feedback'}<={r['kind'] for r in rows}
    manifest=json.loads((runtime.diagnostics.directory/'manifest.json').read_text())
    assert any(i['plugin_id']=='vendor.new_robot' for i in manifest['resolved_system']['instances'])
