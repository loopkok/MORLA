"""Compiled ROS C++ node with intercepted SDK, isolated domain 75, no hardware.

Set MORLA_TEST_XHAND_EXECUTABLE to the just-built node and
MORLA_TEST_XHAND_INCLUDE to its vendor SDK include directory.
"""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

EXECUTABLE = os.environ.get('MORLA_TEST_XHAND_EXECUTABLE')
pytestmark = pytest.mark.skipif(not EXECUTABLE, reason='explicit isolated native-test binary required')


@pytest.fixture(scope='module')
def stub(tmp_path_factory):
    assert os.environ.get('ROS_DOMAIN_ID') == '75' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    path = tmp_path_factory.mktemp('xhand-stub') / 'libtest_xhand.so'
    subprocess.run(['g++','-shared','-fPIC','-std=c++14','-I'+os.environ['MORLA_TEST_XHAND_INCLUDE'],
                    str(Path(__file__).with_name('piano_xhand_sdk_stub.cpp')),'-o',str(path)], check=True)
    return path


@pytest.fixture
def native(stub,tmp_path):
    processes = []
    def start(fail=False, diagnostic=False):
        env=dict(os.environ,LD_PRELOAD=str(stub),PIANO_XHAND_TEST_DIR=str(tmp_path))
        if fail: env['PIANO_XHAND_FAIL_INIT']='1'
        if diagnostic: env['PIANO_XHAND_DIAGNOSTIC_SAMPLE']='1'
        p = subprocess.Popen([EXECUTABLE,'--ros-args','-r','__ns:=/piano_native_audit',
             '-p','port_name:=/dev/morla-piano-audit-DOES-NOT-EXIST','-p','update_rate:=50.0',
             '-p','piano_feedback_mode:=true'],
             env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        processes.append(p)
        return p
    yield start,tmp_path
    for p in processes:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try: p.wait(timeout=3)
            except subprocess.TimeoutExpired: p.kill(); p.wait(timeout=2)
    assert not (tmp_path/'UNEXPECTED_COMMAND').exists()
    assert not (tmp_path/'UNEXPECTED_FORCE_UPDATE').exists()


def until(predicate,seconds=5):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(.02)
    raise AssertionError('native condition timed out')


def start_feedback(node):
    import rclpy
    from std_srvs.srv import Trigger
    client=node.create_client(Trigger,'/piano_native_audit/start_feedback')
    assert client.wait_for_service(timeout_sec=3)
    future=client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node,future,timeout_sec=3)
    assert future.done() and future.result().success
    return client


def test_native_sends_no_command_before_explicit_passive_start_and_closes(native):
    import rclpy
    start,path=native
    p=start()
    until(lambda:(path/'open_stub').exists())
    time.sleep(.1)
    assert not (path/'PASSIVE_COMMAND').exists() and not (path/'cached_read').exists()
    rclpy.init();node=rclpy.create_node('explicit_passive_init_test')
    try:
        client=start_feedback(node)
        until(lambda:(path/'PASSIVE_COMMAND').exists() and (path/'cached_read').exists())
        p.send_signal(signal.SIGINT)
        assert p.wait(timeout=3)==0
        assert (path/'closed').exists()
    finally:
        node.destroy_node();rclpy.shutdown()


def test_native_init_failure_exits_nonzero_and_closes(native):
    start,path=native
    p=start(True)
    assert p.wait(timeout=3)==1
    assert (path/'open_stub').exists() and (path/'closed').exists()
    assert not (path/'PASSIVE_COMMAND').exists()


def test_native_device_timeout_stops_publishing_fresh_states(native):
    import rclpy
    from xhand_control_interfaces.msg import XHandStateArray
    rclpy.init()
    node=rclpy.create_node('native_audit_observer')
    samples=[]
    from rclpy.qos import qos_profile_sensor_data
    sub=node.create_subscription(XHandStateArray,'/piano_native_audit/xhand_state',samples.append,qos_profile_sensor_data)
    start,path=native
    p=start()
    def read():
        rclpy.spin_once(node,timeout_sec=.02)
        return bool(samples)
    try:
        client=start_feedback(node)
        until(read)
        assert abs(samples[-1].hand_states[0].position[3]-.13)<1e-5
        (path/'fail_read').touch()
        # Drain DDS samples already in flight, then require silence.
        end=time.monotonic()+.4
        while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.01)
        count=len(samples)
        end=time.monotonic()+.3
        while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.01)
        assert len(samples)==count and p.poll() is None
        (path/'fail_read').unlink()
        until(lambda:(rclpy.spin_once(node,timeout_sec=.02) is None and len(samples)>count))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_piano_target_requires_lease_and_reverts_to_passive_when_lease_stops(native):
    import json
    import rclpy
    from std_msgs.msg import String
    from xhand_control_interfaces.msg import XHandCommand
    from rclpy.qos import QoSProfile,DurabilityPolicy
    rclpy.init();node=rclpy.create_node('piano_hand_lease_test')
    start,path=native
    p=start()
    pub=node.create_publisher(XHandCommand,'/piano_native_audit/xhand_command',1)
    lease=node.create_publisher(String,'/policy_inference/state',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    names=['thumb_bend_joint','thumb_rota_joint1','thumb_rota_joint2','index_bend_joint','index_joint1','index_joint2',
           'mid_joint1','mid_joint2','ring_joint1','ring_joint2','pinky_joint1','pinky_joint2']
    command=XHandCommand(hand_id=0,name=names,position=[.1]*12,kp=[80.]*12,ki=[0.]*12,kd=[0.]*12,effort_limit=[400.]*12,mode=3)
    try:
        client=start_feedback(node)
        until(lambda:(path/'PASSIVE_COMMAND').exists())
        for _ in range(10):pub.publish(command);rclpy.spin_once(node,timeout_sec=.02)
        assert not (path/'ACTIVE_COMMAND').exists()
        (path/'allow_active').touch()
        deadline=time.monotonic()+3
        while not (path/'ACTIVE_COMMAND').exists() and time.monotonic()<deadline:
            lease.publish(String(data=json.dumps(dict(activity='playback',sent_at=time.time()))))
            pub.publish(command);rclpy.spin_once(node,timeout_sec=.02)
        assert (path/'ACTIVE_COMMAND').exists()
        # Stop renewing; a cached target may not be replayed indefinitely.
        time.sleep(.8)
        until(lambda:(path/'last_mode').read_text()=='0')
    finally:
        node.destroy_node();rclpy.shutdown()


def test_sdk_diagnostics_report_device_fields_without_treating_transport_as_motor_health(native):
    import json
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('piano_raw_hand_diagnostics_test')
    samples=[]
    sub=node.create_subscription(String,'/piano_native_audit/sdk_status',
        lambda msg:samples.append(json.loads(msg.data)),1)
    start,path=native
    p=start(diagnostic=True)
    try:
        client=start_feedback(node)
        def ready():
            rclpy.spin_once(node,timeout_sec=.02)
            return bool(samples and samples[-1].get('device_diagnostics'))
        until(ready)
        status=samples[-1]
        assert status['feedback_valid'] is True and status['send_ack'] is True
        assert status['passive'] is True
        joint=status['device_diagnostics'][0]['joints'][4]
        assert joint['name']=='index_joint1' and joint['raw_position']==1234
        assert joint['current_ma']==-12 and joint['temperature_raw']==31
        assert (joint['commboard_err'],joint['jointboard_err'],joint['tipboard_err'])==(1,2,4)
        assert joint['command_mode']==0 and joint['kp']==0 and joint['current_limit_ma']==0
        assert not (path/'ACTIVE_COMMAND').exists()
    finally:
        node.destroy_node();rclpy.shutdown()
