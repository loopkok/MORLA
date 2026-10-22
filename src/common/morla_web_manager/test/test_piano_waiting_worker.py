"""Exercise actual worker IPC with absent joints; no ROS or device is opened."""
import json
import os
from pathlib import Path
import queue
import shutil
import sys
import threading
import time

from morla_web_manager.piano_manager import PianoManager
from test_piano_lane import store

STUB = '''
import time
from .engine import FakeBackend
class Node:
 def create_client(self,*args): return object()
 def count_publishers(self,*args): return 0
class RosBackend(FakeBackend):
 def __init__(self,c,mode):
  super().__init__(c)
  self._hand_values={s:list(self.measured[s+'_hand']) for s in ('left','right')}
  self.measured={};self.received={};self.node=Node();self.Trigger=object
  self.clients={};self.input_errors={};self.sdk_statuses={};self.sdk_status_at={}
  self.sdk_statuses={s:dict(feedback_generation=0,feedback_valid=False,error='not initialized') for s in ('left','right')}
  self.statuses={s+'_arm':dict(connected=True,backend='hardware',reset_seq=0,can_init_seq=0,
    reset_status='idle',controller_arm_status=1,controller_status_fresh=True) for s in ('left','right')}
  self.status_at={k:time.monotonic() for k in self.statuses}
  self.expected_backend='hardware'
 def _graph_check(self): pass
 def pump(self):
  self.status_at={k:time.monotonic() for k in self.statuses}
  self.sdk_status_at={s:time.monotonic() for s in self.sdk_statuses}
 def check_connection(self): raise RuntimeError('missing all four joint streams')
 def connect(self): raise AssertionError('blocking connect must not run in Web worker')
 def _call(self,side,verb):
  if verb=='start_hand_feedback':
   self.sdk_statuses[side].update(feedback_generation=self.sdk_statuses[side]['feedback_generation']+1,feedback_valid=True,error='')
   self.measured[side+'_hand']=self._hand_values[side];self.received[side+'_hand']=time.monotonic()
   return
  assert verb=='reset',verb
  s=self.statuses[side+'_arm'];s['reset_seq']+=1;s['reset_status']='cleared';s['controller_arm_status']=0
 def emergency_stop(self): pass
'''


def test_missing_four_streams_does_not_block_refresh_or_reset(store,tmp_path):
    project=tmp_path/'external'
    shutil.copytree(store.project/'morla_piano',project/'morla_piano',ignore=shutil.ignore_patterns('__pycache__'))
    (project/'morla_piano/ros_backend.py').write_text(STUB)
    config=tmp_path/'config.json';config.write_text(json.dumps(store.config()['config']))
    env=dict(os.environ)
    package=Path(__file__).resolve().parents[1]
    env['PYTHONPATH']=str(package)+os.pathsep+env.get('PYTHONPATH','')
    proc=PianoManager._spawn([sys.executable,'-m','morla_web_manager.piano_worker','--project',str(project),
         '--config',str(config),'--mode','hardware','--log',str(tmp_path/'events.jsonl')],env=env)
    messages=queue.Queue();stopped=threading.Event();write_lock=threading.Lock()
    def read():
        for line in proc.stdout:
            try: messages.put(json.loads(line))
            except ValueError: pass
    def send(value):
        with write_lock: proc.stdin.write(json.dumps(value)+'\n');proc.stdin.flush()
    def heartbeat():
        while not stopped.wait(.2):
            try: send(dict(operation='heartbeat'))
            except (OSError,ValueError): return
    threading.Thread(target=read,daemon=True).start()
    threading.Thread(target=heartbeat,daemon=True).start()
    def wait(predicate,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            try: item=messages.get(timeout=.1)
            except queue.Empty: continue
            if predicate(item): return item
        raise AssertionError('worker response timed out')
    try:
        state=wait(lambda m:m.get('kind')=='snapshot' and m.get('maintenance_allowed'))
        assert state['state']=='WAITING_FEEDBACK' and state['connected'] is False and state['positions']=={}
        began=time.monotonic()
        send(dict(operation='refresh',job_id='refresh'))
        assert wait(lambda m:m.get('kind')=='result' and m.get('job_id')=='refresh')['ok']
        assert time.monotonic()-began<1
        send(dict(operation='reset',job_id='reset',calibration_confirmed=True))
        assert wait(lambda m:m.get('kind')=='result' and m.get('job_id')=='reset')['ok']
        assert wait(lambda m:m.get('kind')=='snapshot' and m.get('reset_required') is False)['motion_claimed'] is False
        send(dict(operation='start_hand_feedback',job_id='hands',calibration_confirmed=True))
        assert wait(lambda m:m.get('kind')=='result' and m.get('job_id')=='hands')['ok']
        state=wait(lambda m:m.get('kind')=='snapshot' and len(m.get('positions',{}))==2)
        assert set(state['positions'])=={'left_hand','right_hand'}
        assert state['connected'] is False and state['motion_claimed'] is False
        send(dict(operation='shutdown',job_id='close'))
        assert proc.wait(timeout=3)==0
    finally:
        stopped.set()
        PianoManager._kill_group(proc)
