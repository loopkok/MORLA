"""A late vendor call must precede piano stop, which must precede disconnect."""
import threading
import time

import pytest

pytest.importorskip('rclpy')
import rclpy

from nero_robot_control.driver_node import NeroRobotDriverNode
from test_enable_confirmation import spin_until


@pytest.mark.parametrize('piano', [True,False])
def test_piano_shutdown_serializes_stop_after_inflight_call_and_preserves_default(piano):
    rclpy.init(args=['--ros-args','-p','sdk_backend:=fake',
        '-p','initial_position:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]',
        '-p','stop_on_shutdown:='+str(piano).lower()])
    node=None
    destroyed=False
    release=threading.Event()
    try:
        node=NeroRobotDriverNode()
        spin_until(rclpy,node,lambda:node.core.connected)
        disconnect=node.sdk.disconnect
        def observed_disconnect():
            node.sdk.calls.append(('disconnect',))
            disconnect()
        node.sdk.disconnect=observed_disconnect
        node._position_active=True
        def late_command():
            assert release.wait(3)
            node.sdk.calls.append(('late_move',))
        node._position_future=node._worker.submit(late_command)
        node.destroy_node()
        destroyed=True
        release.set()
        deadline=time.monotonic()+3
        while ('disconnect',) not in node.sdk.calls and time.monotonic()<deadline:
            time.sleep(.01)
        sequence=[call[0] for call in node.sdk.calls if call[0] in {'late_move','estop','disconnect'}]
        assert sequence == (['late_move','estop','disconnect'] if piano else ['late_move','disconnect'])
    finally:
        release.set()
        if node is not None and not destroyed: node.destroy_node()
        rclpy.shutdown()
