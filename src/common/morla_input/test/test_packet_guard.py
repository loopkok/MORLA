from morla_input.packet_guard import PacketGuard


def packet(seq, stamp=100, session='session', payload='Left controller:, 0, 0, 0, 0, 0, 0, 1'):
    return f'@q3 {session} {seq} {stamp*1000}|{payload}'


def test_expired_buffered_duplicate_and_old_session_rejected():
    guard = PacketGuard()
    assert guard.accept(packet(1), 100.1)
    assert guard.accept(packet(1), 100.1) is None
    assert guard.accept(packet(0), 100.1) is None
    assert guard.accept(packet(2), 101) is None
    assert guard.accept(packet(1, session='new'), 100.1)
    assert guard.accept(packet(3), 100.1) is None


def test_legacy_requires_explicit_compatibility_and_future_clock_is_rejected():
    assert PacketGuard().accept('Left buttons:,0,0,0,0,0', 100) is None
    assert PacketGuard(allow_legacy=True).accept('Left buttons:,0,0,0,0,0', 100)
    assert PacketGuard().accept(packet(1, 101), 100) is None


def test_debug_frame_counter_is_not_a_new_stream():
    guard = PacketGuard()
    for i in range(1000):
        assert guard.accept(packet(i, payload=f'body iobt | fid={i}:0,0,0'), 100.1)
    assert len(guard._sequences) == 1


def test_new_session_is_accepted_with_epoch_change_and_retires_old_session():
    guard = PacketGuard()
    assert guard.accept(packet(1, session='old'), 100.1)
    assert guard.last_session_change is None
    assert guard.accept(packet(1, session='new'), 100.1)
    assert guard.last_session_change == {
        'old_session': 'old', 'new_session': 'new', 'stream': 'left controller'
    }
    assert guard.accept(packet(2, session='new'), 100.1)
    assert guard.last_session_change is None
    assert guard.accept(packet(2, session='old'), 100.1) is None
    assert guard.last_session_change is None
