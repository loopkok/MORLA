import json
from types import SimpleNamespace

from morla_input.quest3_udp_mocap import Quest3UDPMocap


class Pub(list):
    def publish(self, message):
        self.append(json.loads(message.data))


def test_events_are_published_per_side_with_their_own_sequence():
    node = SimpleNamespace(
        _teleop_events_enabled=True, _teleop_epoch='epoch-1',
        _teleop_sequence={'left': 0, 'right': 0},
        coordinate_basis_id='unity_left_handed_v1',
        _teleop_events_pub={'left': Pub(), 'right': Pub()})
    Quest3UDPMocap._publish_teleop_event(node, 'left', 'pose', 'hand', 'f', [0]*7, 1.0)
    Quest3UDPMocap._publish_teleop_event(node, 'right', 'pose', 'controller', 'f', [0]*7, 1.0)
    Quest3UDPMocap._publish_teleop_event(node, 'left', 'grip', 'controller', 'f', .8, 1.0)
    left = node._teleop_events_pub['left']
    right = node._teleop_events_pub['right']
    assert [row['sequence'] for row in left] == [1, 2]
    assert [row['sequence'] for row in right] == [1]
    assert [row['side'] for row in left] == ['left', 'left'] and right[0]['side'] == 'right'
    assert left[0]['epoch'] == right[0]['epoch'] == 'epoch-1'
    assert {row['version'] for row in left + right} == {2}
    assert {row['basis'] for row in left + right} == {'unity_left_handed_v1'}


def test_events_stay_off_unless_enabled_and_side_is_known():
    node = SimpleNamespace(
        _teleop_events_enabled=True, _teleop_epoch='epoch-1',
        _teleop_sequence={'left': 0, 'right': 0},
        coordinate_basis_id='unity_left_handed_v1',
        _teleop_events_pub={'left': Pub(), 'right': Pub()})
    Quest3UDPMocap._publish_teleop_event(node, 'middle', 'pose', 'hand', 'f', [0]*7, 1.0)
    assert not node._teleop_events_pub['left'] and not node._teleop_events_pub['right']
    node._teleop_events_enabled = False
    Quest3UDPMocap._publish_teleop_event(node, 'left', 'pose', 'hand', 'f', [0]*7, 1.0)
    assert not node._teleop_events_pub['left']
