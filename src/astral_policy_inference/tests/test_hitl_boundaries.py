"""No ROS node creation: execute the real arbitration methods with fake IO."""
import threading
from types import SimpleNamespace
import numpy as np
from astral_policy_inference.node import PolicyNode
from astral_policy_inference.controller import Controller
from astral_policy_inference.executor import SafeExecutor
from astral_policy_inference.robot_io import CmdTarget

class TickProbe:
    _tick = PolicyNode._tick
    def __init__(self, pending=False, initiate=False):
        self._lock = threading.RLock()
        self._takeover_pending = {'pending': object()} if pending else None
        self.controller = Controller()
        self.controller.request('policy')
        self._engine = object()
        self._ctrl_rate = 60
        self.drained = self.dispatched = False
        self.initiate = initiate
    def _begin_control_diag(self, *a, **k): pass
    def _finish_control_diag(self, *a, **k): pass
    def _mark_control_diag(self, *a, **k): pass
    def _maybe_publish_state(self, *a): pass
    def _poll_takeover(self, now): return True
    def _drain_cmds(self):
        self.drained = True
        if self.initiate:
            self._takeover_pending = {'pending': object()}
        else:
            self._takeover_pending = None
            self.controller.request('stop')
    def _policy_tick(self, dt): self.dispatched = True

def test_stop_is_drained_while_takeover_pending():
    p = TickProbe(pending=True)
    p._tick()
    assert p.drained and p.controller.state == 'IDLE'

def test_no_policy_dispatch_on_new_takeover_tick():
    p = TickProbe(initiate=True)
    p._tick()
    assert not p.dispatched

def test_seeded_executor_limits_first_policy_action():
    ex = SafeExecutor(max_joint_vel=1.0)
    zero = CmdTarget('left_arm_cmd', '/left_arm/joint_commands', 'joint', np.zeros(7))
    ex.seed([zero])
    goal = CmdTarget(zero.stream, zero.topic, zero.kind, np.ones(7)*.5)
    result = ex.step([goal], 1/60)[0].values
    assert np.max(result) <= 1/60 + 1e-12
