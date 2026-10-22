"""Exercise ROS message validation without opening a ROS/CAN connection."""
import json
import math
from types import SimpleNamespace
import time

import pytest

from test_piano_lane import store


def test_bad_sample_blocks_motion_until_same_input_recovers(store, monkeypatch):
    monkeypatch.syspath_prepend(str(store.project))
    from morla_piano.ros_backend import RosBackend
    b = RosBackend.__new__(RosBackend)
    b.c = store.config()['config']
    b.names = {'left_arm': tuple('left_joint'+str(i) for i in range(1,8))}
    b.measured, b.received, b.statuses, b.status_at, b.input_errors = {}, {}, {}, {}, {}
    b.fault = None
    b.claimed = False
    now = time.time()
    b.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(now*1e9))))
    def sample(stamp):
        return SimpleNamespace(name=list(b.names['left_arm']),position=[0.]*7,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=int(stamp),nanosec=int(stamp%1*1e9))))
    b._feedback('left_arm', sample(now-3))
    assert b.input_errors and not b.measured
    b._status('left_arm', SimpleNamespace(data='broken json'))
    assert len(b.input_errors) == 2
    b._feedback('left_arm',sample(now))
    assert 'left_arm/feedback' not in b.input_errors
    assert 'left_arm/status' in b.input_errors
    b._status('left_arm', SimpleNamespace(data=json.dumps(dict(backend='hardware',connected=True,feedback_fresh=True))))
    assert not b.input_errors
    b.expected_backend='hardware'
    b.health(require_enabled=False)
    b._feedback('left_arm',sample(now-3))
    with pytest.raises(RuntimeError,match='stale'):
        b.health(require_enabled=False)


def test_refresh_waits_for_new_samples_instead_of_failing_on_old_dictionary(store, monkeypatch):
    monkeypatch.syspath_prepend(str(store.project))
    from morla_piano.ros_backend import RosBackend
    b=RosBackend.__new__(RosBackend)
    b.c={'ros':{'connect_timeout_s':.2}}
    b.measured=b.statuses=dict.fromkeys(('left_arm','right_arm','left_hand','right_hand'))
    ticks=[]
    b.pump=lambda:ticks.append(1)
    b._graph_check=lambda:None
    def health(require_enabled=True):
        assert require_enabled is False
        if len(ticks)<3: raise RuntimeError('waiting for fresh feedback')
    b.health=health
    b.connect()
    assert len(ticks)>=3


def test_nonblocking_inspection_reports_all_missing_parts_and_sdk_reason(store,monkeypatch):
    monkeypatch.syspath_prepend(str(store.project))
    from morla_piano.ros_backend import RosBackend
    b=RosBackend.__new__(RosBackend)
    b.names=dict.fromkeys(('left_arm','right_arm','left_hand','right_hand'))
    b.measured={'left_arm':[],'right_arm':[]};b.statuses=dict(b.names);b.input_errors={}
    b.sdk_statuses={'left':dict(feedback_valid=False,error_code=1501070,error='CRC')}
    b.sdk_status_at={'left':time.monotonic()}
    with pytest.raises(RuntimeError,match='left_hand SDK 1501070: CRC') as error:b.check_connection()
    assert 'left_hand' in str(error.value) and 'right_hand' in str(error.value)


def feedback_backend(store, monkeypatch, key='left_hand'):
    monkeypatch.syspath_prepend(str(store.project))
    from morla_piano.model import HAND_NAMES
    from morla_piano.ros_backend import RosBackend
    b = RosBackend.__new__(RosBackend)
    b.c = store.config()['config']
    b.c['calibrated'] = True
    b.names = {key: HAND_NAMES if key.endswith('hand') else tuple(f'left_joint{i}' for i in range(1, 8))}
    b.measured, b.received, b.statuses, b.status_at, b.input_errors = {}, {}, {}, {}, {}
    b.feedback_warnings = {}
    b.claimed = False
    b.expected_backend = 'hardware'
    now = time.time()
    # Any accidental ownership publisher or device operation fails the test.
    b.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(now*1e9))))
    def sample(values):
        return SimpleNamespace(name=list(b.names[key]), position=values,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=int(now), nanosec=int(now%1*1e9))))
    return b, sample


@pytest.mark.parametrize('edge,offset', [(0, -math.radians(1.5)), (0, -0.05), (1, 0.05)])
def test_nexus_hand_read_margin_preserves_real_value_but_blocks_command_seed(store, monkeypatch, edge, offset):
    b, sample = feedback_backend(store, monkeypatch)
    value = b.c['hands']['left']['hand_limits_rad'][0][edge] + offset
    q = [0.] * 12
    q[0] = value
    b._feedback('left_hand', sample(q))
    assert not b.input_errors
    assert b.measured['left_hand'][0] == value  # No zeroing/clamping of measurements.
    assert 'thumb_bend_joint' in b.feedback_warnings['left_hand']
    b._graph_check = lambda: None
    b.health = lambda **kwargs: None
    with pytest.raises(RuntimeError, match='未发送目标'):
        b.claim()
    assert b.claimed is False
    # Targets remain strictly rejected even within the measurement margin.
    from morla_piano.model import check_limits
    with pytest.raises(ValueError):
        check_limits(q, b.c['hands']['left']['hand_limits_rad'], 'left_hand')
    b._feedback('left_hand', sample([0.] * 12))
    assert not b.feedback_warnings and not b.input_errors


@pytest.mark.parametrize('edge,offset', [(0, -0.05001), (1, 0.05001), (0, float('nan'))])
def test_hand_read_margin_still_rejects_invalid_or_large_values(store, monkeypatch, edge, offset):
    b, sample = feedback_backend(store, monkeypatch)
    value = b.c['hands']['left']['hand_limits_rad'][0][edge] + offset
    q = [0.] * 12
    q[0] = value
    b._feedback('left_hand', sample(q))
    assert 'left_hand/feedback' in b.input_errors
    assert not b.measured
    if math.isfinite(value):
        assert 'thumb_bend_joint' in b.input_errors['left_hand/feedback']


def test_hand_margin_does_not_widen_arm_feedback_margin(store, monkeypatch):
    b, sample = feedback_backend(store, monkeypatch, 'left_arm')
    q = [0.] * 7
    q[0] = b.c['hands']['left']['arm_limits_rad'][0][0] - math.radians(.2)
    b._feedback('left_arm', sample(q))
    assert b.input_errors and not b.measured


@pytest.mark.parametrize('side,deg', [
    ('left', [-.75, 59.0816, 30.6658, -1.4602, .0833, .6666, 2.0833, .25, 1.1666, -1.25, 1., 1.1666]),
    ('right', [.25, 59.3316, 28.9158, .0764, -1., .4167, 1.3333, -.75, 1.9166, -.5833, 1.8333, -1.5]),
])
def test_recorded_passive_hand_samples_are_visible_without_zeroing(store, monkeypatch, side, deg):
    key=side+'_hand'
    b,sample=feedback_backend(store,monkeypatch,key)
    q=[math.radians(v) for v in deg]
    b._feedback(key,sample(q))
    assert not b.input_errors
    assert b.measured[key]==q
    assert b.received[key]>0 and key in b.feedback_warnings
