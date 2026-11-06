"""Loopback controller emulator: real V2.1.5 SDK, no physical robot or SSH."""
import ctypes
import socket
import threading
import time
import xmlrpc.client
from xmlrpc.server import SimpleXMLRPCServer

import pytest

from morla_fr3_driver.sdk_backend import SDKBackend
from morla_fr3_driver.vendor import Robot


def test_sdk215_actual_ctor_servo_feedback_and_readonly_cleanup(monkeypatch):
    state={'enabled':0,'count':0,'calls':[],'drop':False,'accepts':0}
    stop=threading.Event()
    listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen()
    listener.settimeout(.1)
    def stream():
        conn=None
        try:
            while not stop.is_set():
                try:conn,_=listener.accept();break
                except socket.timeout:continue
            if conn is None:return
            state['accepts']+=1
            while not stop.is_set() and not state['drop']:
                packet=Robot.RobotStatePkg()
                packet.frame_head=0x5a5a;packet.data_len=ctypes.sizeof(packet)-7
                count=state['count']%256;packet.frame_cnt=count if count<128 else count-256
                state['count']+=1;packet.rbtEnableState=state['enabled']
                packet.jt_cur_pos[:]=[0,-90,90,-90,-90,0]
                raw=bytes(packet);raw=raw[:-2]+(sum(raw[:-2])&0xffff).to_bytes(2,'little')
                # Exercise actual TCP stream fragments, not cached SDK getters.
                conn.sendall(raw[:3]);conn.sendall(raw[3:])
                stop.wait(.008)
        except OSError:
            pass
        finally:
            if conn is not None:conn.close()
    receiver=threading.Thread(target=stream,daemon=True);receiver.start()
    server=SimpleXMLRPCServer(('127.0.0.1',0),logRequests=False,allow_none=True)
    server.register_function(lambda:[0,'127.0.0.1'],'GetControllerIP')
    def enable(value):
        state['calls'].append(('enable',value));state['enabled']=value;return 0
    server.register_function(enable,'RobotEnable')
    for name in ('ServoMoveStart','ServoMoveEnd','StopMotion'):
        def operation(name=name):state['calls'].append((name,));return 0
        server.register_function(operation,name)
    def servo(*args):state['calls'].append(('ServoJ',args));return 0
    server.register_function(servo,'ServoJ')
    server.register_function(lambda q:[0,1,2,3,4,5,6],'GetForwardKin')
    rpc_thread=threading.Thread(target=server.serve_forever,daemon=True);rpc_thread.start()
    original=xmlrpc.client.ServerProxy
    def redirected(url,*args,**kwargs):
        return original(url.replace(':20003',':'+str(server.server_address[1])),*args,**kwargs)
    monkeypatch.setattr(xmlrpc.client,'ServerProxy',redirected)
    monkeypatch.setattr(Robot.RPC,'ROBOT_REALTIME_PORT',listener.getsockname()[1])
    parameters={'host':'127.0.0.1','rpc_timeout_s':.5,'feedback_timeout_s':.2,
                'operation_timeout_s':2,'rate_hz':125}
    backend=None
    try:
        backend=SDKBackend(parameters)
        assert backend.read()['values'][1]==pytest.approx(-1.5707963267948966)
        assert state['calls']==[]  # construction/read must not enable/move
        assert backend.forward_kinematics([0]*6)==[1,2,3,4,5,6]
        backend.activate();backend.write([0]*6);backend.stop()
        assert state['calls'][0:2]==[('enable',1),('ServoMoveStart',)]
        assert state['calls'][2][0]=='ServoJ'
        assert state['calls'][-3:]==[('StopMotion',),('ServoMoveEnd',),('enable',0)]
        state['drop']=True
        until=time.monotonic()+1
        while time.monotonic()<until and backend.sdk.SDK_state:time.sleep(.01)
        with pytest.raises(ConnectionError,match='EOF'):backend.read()
        assert state['accepts']==1
        calls=state['calls'].copy();backend.close()
        assert not backend.sdk.morla_thread.is_alive()
        assert state['calls']==calls
    finally:
        if backend is not None:backend.close()
        stop.set();listener.close();receiver.join(timeout=1)
        server.shutdown();server.server_close();rpc_thread.join(timeout=1)


def test_sdk215_version_has_no_tls_dependency():
    assert Robot.RPC.GetSDKVersion(object.__new__(Robot.RPC)) == (0,['SDK:V2.1.5','Robot:V3.8.5'])
