import sys
import pytest
from morla_runtime.supervisor import ProcessSupervisor


def test_real_owned_process_exit_and_close():
    supervisor=ProcessSupervisor(factories={'new.input':lambda instance:[[sys.executable,'-c','import time; time.sleep(60)']]})
    supervisor.start([{'instance_id':'input','plugin_id':'new.input','parameters':{'process_mode':'managed'}}])
    child=supervisor.rows[0]['child']
    assert supervisor.state()[0]['exit_code'] is None
    supervisor.close()
    assert child.poll() is not None


def test_external_never_creates_or_kills_process():
    supervisor=ProcessSupervisor(factories={})
    supervisor.start([{'instance_id':'external','plugin_id':'unknown','parameters':{'process_mode':'external'}}])
    assert supervisor.state()==[{'instance_id':'external','mode':'external'}]
    supervisor.close()


def test_failure_cleans_earlier_children():
    supervisor=ProcessSupervisor(factories={'ok':lambda instance:[[sys.executable,'-c','import time; time.sleep(60)']]})
    with pytest.raises(ValueError):
        supervisor.start([{'instance_id':key,'plugin_id':key,'parameters':{'process_mode':'managed'}} for key in ('ok','missing')])
    assert supervisor.executor.children==[]


def test_shared_process_started_once_for_two_logical_components():
    supervisor=ProcessSupervisor(factories={'device':lambda instance:[{'key':'one-board',
        'argv':[sys.executable,'-c','import time; time.sleep(60)'],'env':{'MORLA_TEST':'yes'}}]})
    try:
        supervisor.start([{'instance_id':name,'plugin_id':'device','parameters':{}} for name in ('arm','gripper')])
        assert len(supervisor.executor.children)==1
        assert supervisor.state()[0]['pid']==supervisor.state()[1]['pid']
    finally:supervisor.close()


def test_conflicting_shared_process_plan_is_not_silently_reused():
    supervisor=ProcessSupervisor(factories={'device':lambda instance:[{'key':'shared',
        'argv':[sys.executable,'-c','import time; time.sleep(60)',instance['instance_id']]}]})
    with pytest.raises(ValueError,match='conflicting'):
        supervisor.start([{'instance_id':name,'plugin_id':'device','parameters':{}} for name in ('first','second')])
    assert supervisor.executor.children==[]
