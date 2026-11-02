import pytest
from morla_input.quest_events import QuestEventSource
from morla_input.adapters.normalized import InputError


def points(closed):
    p = [[0., 0., 0.] for _ in range(21)]
    p[17] = [.08, 0, 0]
    p[8] = [.005 if closed else .08, 0, 0]
    return p


def setup_source(sides=('right',)):
    clock = [1_000_000_000]
    transitions = []
    source = QuestEventSource('quest', clock=lambda: clock[0], wall_clock=lambda: clock[0],
                              on_transition=lambda: transitions.append('changed'), sides=sides)
    sequence = [0]

    def event(kind, source_kind='controller', values=None, epoch='first', side='right',
              basis='unity_left_handed_v1'):
        sequence[0] += 1
        clock[0] += 20_000_000
        return {'version': 2, 'basis': basis, 'side': side, 'kind': kind,
                'source': source_kind,
                'epoch': epoch, 'sequence': sequence[0], 'sent_ns': clock[0],
                'frame_id': 'quest_world',
                'values': values if values is not None else [0, 0, 0, 0, 0, 0, 1]}
    return source, event, clock, transitions


def test_event_carries_the_basis_it_was_measured_in():
    source, event, _, _ = setup_source()
    frame = source.event(event('pose'))
    assert frame.input_basis_id == 'unity_left_handed_v1'
    canonical = source.event(event('pose', basis='quest3_canonical_legacy_v1'))
    assert canonical.input_basis_id == 'quest3_canonical_legacy_v1'


def test_v1_rows_are_rejected_rather_than_reinterpreted():
    source, event, _, _ = setup_source()
    row = event('pose')
    del row['basis']
    row['version'] = 1
    with pytest.raises(InputError):
        source.event(row)
    with pytest.raises(InputError):
        source.event({**event('pose'), 'basis': ''})


def test_controller_and_hand_have_distinct_epochs_and_independent_deadman():
    source, event, clock, transitions = setup_source()
    f = source.event(event('pose'))
    first = f.source_epoch
    assert 'right_hold' not in f.ports
    assert source.event(event('grip', values=0.)).ports['right_hold'] is False
    assert source.event(event('grip', values=1.)).ports['right_hold'] is True
    switched = source.event(event('pose', 'hand'))
    assert switched.source_epoch != first and 'right_hold' not in switched.ports
    assert source.event(event('grip', values=1.)) is None
    assert source.event(event('landmarks', 'hand', points(False))).ports['right_hold'] is False
    assert source.event(event('landmarks', 'hand', points(True))).ports['right_hold'] is True
    assert len(transitions) == 2


def test_side_parameter_selects_the_hand():
    right, event, _, _ = setup_source(('right',))
    assert right.event(event('pose', side='left')) is None
    assert 'left_pose' not in right.event(event('pose')).ports
    left, event, _, _ = setup_source(('left',))
    assert left.event(event('pose', side='right')) is None
    frame = left.event(event('pose', side='left'))
    assert 'left_pose' in frame.ports and 'right_pose' not in frame.ports
    assert 'scalar' not in frame.ports


def test_both_hands_keep_independent_poses_holds_and_scalars():
    source, event, clock, _ = setup_source(('left', 'right'))
    source.event(event('pose', side='left', values=[.1, 0, 0, 0, 0, 0, 1]))
    source.event(event('pose', side='right', values=[.2, 0, 0, 0, 0, 0, 1]))
    frame = source.event(event('trigger', values=.25, side='left'))
    frame = source.event(event('trigger', values=.75, side='right'))
    assert frame.ports['left_pose']['position'][0] == .1
    assert frame.ports['right_pose']['position'][0] == .2
    assert frame.ports['left_scalar'] == .25 and frame.ports['right_scalar'] == .75
    assert frame.ports['scalar'] == .75  # right hand drives the unsided alias
    assert frame.port_metadata['left_scalar']['received_ns'] < frame.port_metadata['right_scalar']['received_ns']


def test_one_hand_mode_switch_does_not_reset_the_other_hands_mode():
    source, event, clock, transitions = setup_source(('left', 'right'))
    source.event(event('pose', side='left'))
    source.event(event('pose', side='right'))
    before = len(transitions)
    source.event(event('pose', 'hand', side='right'))
    assert len(transitions) == before + 1
    # Left stays in controller mode; it must not look like another switch.
    frame = source.event(event('pose', side='left', values=[.1, 0, 0, 0, 0, 0, 1]))
    assert len(transitions) == before + 1 and frame.ports['left_pose']['position'][0] == .1
    # Left controller buttons still work; right controller events are stale.
    assert source.event(event('grip', values=1., side='left')).ports['left_hold'] is True
    assert source.event(event('grip', values=1., side='right')) is None


def test_intermediate_pinch_does_not_invent_release():
    source, event, _, _ = setup_source()
    source.event(event('pose', 'hand'))
    p = points(False)
    p[8] = [.028, 0, 0]
    assert 'right_hold' not in source.event(event('landmarks', 'hand', p)).ports
    assert source.event(event('landmarks', 'hand', points(True))).ports['right_hold'] is True


def test_buttons_cannot_refresh_old_pose():
    source, event, clock, _ = setup_source()
    source.event(event('pose'))
    clock[0] += 300_000_000
    frame = source.event(event('grip', values=1.))
    assert 'right_pose' not in frame.ports
    assert frame.ports['right_hold'] is True


def test_old_epochs_stamps_duplicates_and_bad_shapes_are_rejected():
    source, event, clock, _ = setup_source(('left', 'right'))
    first = event('pose')
    source.event(first)
    with pytest.raises(InputError):
        source.event(first)
    source.event(event('pose', epoch='second'))
    with pytest.raises(InputError):
        source.event(event('pose', epoch='first'))
    late = event('pose', epoch='second')
    late['sent_ns'] -= 300_000_000
    with pytest.raises(InputError):
        source.event(late)
    bad = event('pose', epoch='second')
    bad['values'] = [0]*7
    with pytest.raises(InputError):
        source.event(bad)
    with pytest.raises(InputError):
        source.event({**event('pose', epoch='second'), 'side': 'middle'})


def test_source_age_is_not_refreshed_at_ros_arrival():
    source, event, clock, _ = setup_source()
    row = event('pose')
    row['sent_ns'] -= 200_000_000
    frame = source.event(row)
    assert clock[0]-frame.port_metadata['right_pose']['received_ns'] == 200_000_000
    clock[0] += 60_000_000
    assert 'right_pose' not in source.event(event('grip', values=1.)).ports


def test_pinch_gap_cannot_turn_old_open_into_new_release():
    source, event, clock, _ = setup_source()
    source.event(event('pose', 'hand'))
    source.event(event('landmarks', 'hand', points(False)))
    clock[0] += 300_000_000
    source.event(event('pose', 'hand'))
    p = points(False)
    p[8] = [.028, 0, 0]
    frame = source.event(event('landmarks', 'hand', p))
    assert 'right_hold' not in frame.ports


def test_motion_ticks_only_on_pose_but_release_is_immediate():
    source, event, _, _ = setup_source()
    row = event('pose')
    frame = source.event(row)
    assert source.control_event(row, frame)
    row = event('grip', values=1.)
    frame = source.event(row)
    assert not source.control_event(row, frame)
    assert source.latest.ports['right_hold'] is True
    row = event('grip', values=0.)
    frame = source.event(row)
    assert source.control_event(row, frame)


def test_trigger_scalar_is_available_to_any_gripper_binding():
    source, event, _, _ = setup_source()
    source.event(event('pose'))
    frame = source.event(event('trigger', values=.75))
    assert frame.ports['scalar'] == .75
    assert source.control_event({'kind': 'trigger'}, frame)
    assert frame.port_metadata['scalar']['received_ns'] == frame.received_ns


def test_hand_and_controller_expose_same_scalar_capability():
    source, event, _, _ = setup_source()
    source.event(event('pose'))
    assert source.event(event('trigger', values=1.)).ports['scalar'] == 1
    source.event(event('pose', 'hand'))
    closed = source.event(event('landmarks', 'hand', points(True)))
    assert closed.ports['scalar'] == 1
    assert len(closed.ports['right_landmarks']) == 21
    assert source.control_event({'kind': 'landmarks'}, closed)
    opened = source.event(event('landmarks', 'hand', points(False)))
    assert opened.ports['scalar'] == 0


def test_thumbstick_is_axes_only_and_never_refreshes_pose():
    source, event, clock, _ = setup_source(('left', 'right'))
    source.reconnect()
    frame = source.axes('right', {'x': .2, 'y': -.4}, 'joy')
    assert frame.ports['right_axes'] == {'x': .2, 'y': -.4}
    assert frame.ports['axes'] == {'x': .2, 'y': -.4}
    assert 'right_pose' not in frame.ports and 'right_hold' not in frame.ports
    clock[0] += 300_000_000
    frame = source.axes('left', {'x': 0., 'y': 1.}, 'joy')
    assert frame.ports['left_axes'] == {'x': 0., 'y': 1.} and 'right_axes' not in frame.ports
    source.disconnect()
    assert source.axes('right', {'x': 0., 'y': 0.}) is None
