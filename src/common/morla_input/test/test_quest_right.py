import copy
import pytest
from morla_input.quest_right import QuestRightSource
from morla_input.adapters.normalized import InputError


def points(closed):
    p=[[0.,0.,0.] for _ in range(21)]
    p[17]=[.08,0,0];p[8]=[.005 if closed else .08,0,0]
    return p


def setup_source():
    clock=[1_000_000_000];transitions=[]
    source=QuestRightSource('quest',clock=lambda:clock[0],wall_clock=lambda:clock[0],
                            on_transition=lambda:transitions.append('changed'))
    sequence=[0]
    def event(kind,source_kind='controller',values=None,epoch='first'):
        sequence[0]+=1;clock[0]+=20_000_000
        return {'version':1,'side':'right','kind':kind,'source':source_kind,'epoch':epoch,
                'sequence':sequence[0],'sent_ns':clock[0],'frame_id':'quest_world',
                'values':values if values is not None else [0,0,0,0,0,0,1]}
    return source,event,clock,transitions


def test_controller_and_hand_have_distinct_epochs_and_independent_deadman():
    source,event,clock,transitions=setup_source()
    f=source.event(event('pose'));first=f.source_epoch
    assert 'right_hold' not in f.ports
    assert source.event(event('grip',values=0.)).ports['right_hold'] is False
    assert source.event(event('grip',values=1.)).ports['right_hold'] is True
    switched=source.event(event('pose','hand'))
    assert switched.source_epoch!=first and 'right_hold' not in switched.ports
    assert source.event(event('grip',values=1.)) is None
    assert source.event(event('landmarks','hand',points(False))).ports['right_hold'] is False
    assert source.event(event('landmarks','hand',points(True))).ports['right_hold'] is True
    assert len(transitions)==2


def test_intermediate_pinch_does_not_invent_release():
    source,event,_,_=setup_source();source.event(event('pose','hand'))
    p=points(False);p[8]=[.028,0,0]
    assert 'right_hold' not in source.event(event('landmarks','hand',p)).ports
    assert source.event(event('landmarks','hand',points(True))).ports['right_hold'] is True


def test_buttons_cannot_refresh_old_pose():
    source,event,clock,_=setup_source();source.event(event('pose'))
    clock[0]+=300_000_000
    frame=source.event(event('grip',values=1.))
    assert 'right_pose' not in frame.ports
    assert frame.ports['right_hold'] is True


def test_old_epochs_stamps_duplicates_and_bad_shapes_are_rejected():
    source,event,clock,_=setup_source()
    first=event('pose');source.event(first)
    with pytest.raises(InputError):source.event(first)
    source.event(event('pose',epoch='second'))
    with pytest.raises(InputError):source.event(event('pose',epoch='first'))
    late=event('pose',epoch='second');late['sent_ns']-=300_000_000
    with pytest.raises(InputError):source.event(late)
    bad=event('pose',epoch='second');bad['values']=[0]*7
    with pytest.raises(InputError):source.event(bad)
    left=event('pose',epoch='second');left['side']='left'
    with pytest.raises(InputError):source.event(left)


def test_source_age_is_not_refreshed_at_ros_arrival():
    source,event,clock,_=setup_source()
    row=event('pose');row['sent_ns']-=200_000_000
    frame=source.event(row)
    assert clock[0]-frame.port_metadata['right_pose']['received_ns']==200_000_000
    clock[0]+=60_000_000
    assert 'right_pose' not in source.event(event('grip',values=1.)).ports


def test_pinch_gap_cannot_turn_old_open_into_new_release():
    source,event,clock,_=setup_source()
    source.event(event('pose','hand'))
    source.event(event('landmarks','hand',points(False)))
    clock[0]+=300_000_000
    source.event(event('pose','hand'))
    p=points(False);p[8]=[.028,0,0]
    frame=source.event(event('landmarks','hand',p))
    assert 'right_hold' not in frame.ports


def test_motion_ticks_only_on_pose_but_release_is_immediate():
    source,event,_,_=setup_source()
    row=event('pose');frame=source.event(row)
    assert source.control_event(row,frame)
    row=event('grip',values=1.);frame=source.event(row)
    assert not source.control_event(row,frame)
    assert source.latest.ports['right_hold'] is True
    row=event('grip',values=0.);frame=source.event(row)
    assert source.control_event(row,frame)
