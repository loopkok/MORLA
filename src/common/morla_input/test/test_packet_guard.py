from morla_input.packet_guard import PacketGuard


def packet(
    seq, source_us=4_100_000, session="session",
    payload="Left controller:, 0, 0, 0, 0, 0, 0, 1",
):
    return f"@q3m {session} {seq} {source_us}|{payload}"


def sync(
    guard, session="session", peer="tcp:127.0.0.1:12345",
    pc_send_ns=10_000_000_000, quest_receive_us=4_000_000,
    quest_send_us=4_001_000, pc_receive_ns=10_020_000_000,
):
    challenge = guard.make_sync_challenge(peer, now_ns=pc_send_ns, force=True)
    _, nonce, _ = challenge.strip().split()
    ack = f"@q3sync_ack {session} {nonce} {quest_receive_us} {quest_send_us}"
    assert guard.handle_sync_ack(ack, peer, received_ns=pc_receive_ns)


def test_monotonic_handshake_removes_wall_clock_dependency_and_rejects_stale_packets():
    guard = PacketGuard(max_age=0.5)
    sync(guard)

    # UTC can be any value; freshness is calculated only from monotonic clocks.
    accepted = guard.accept(
        packet(1), received=1_900_000_000.0, received_monotonic=10.11
    )
    assert accepted is not None
    assert accepted[0].startswith("Left controller:")
    assert 1_899_999_999.9 < accepted[1] <= 1_900_000_000.0
    assert guard.accept(
        packet(1), received=1_900_000_000.0, received_monotonic=10.11
    ) is None
    assert guard.accept(
        packet(0), received=1_900_000_000.0, received_monotonic=10.11
    ) is None

    # 600 ms old after offset correction.
    assert guard.accept(
        packet(2, source_us=3_500_000),
        received=1_900_000_000.0,
        received_monotonic=10.11,
    ) is None
    # A future timestamp beyond measured sync uncertainty is rejected.
    assert guard.accept(
        packet(3, source_us=4_200_000),
        received=1_900_000_000.0,
        received_monotonic=10.11,
    ) is None


def test_packets_require_a_fresh_peer_bound_clock_challenge():
    guard = PacketGuard()
    assert guard.accept(
        packet(1), received=1_900_000_000.0, received_monotonic=10.11
    ) is None
    challenge = guard.make_sync_challenge(
        "tcp:127.0.0.1:12345", now_ns=10_000_000_000
    )
    _, nonce, _ = challenge.strip().split()
    ack = f"@q3sync_ack session {nonce} 4000000 4001000"
    assert not guard.handle_sync_ack(
        ack, "tcp:127.0.0.1:9999", received_ns=10_020_000_000
    )
    assert not guard.is_synced("session", now_ns=10_020_000_000)
    assert guard.handle_sync_ack(
        ack, "tcp:127.0.0.1:12345", received_ns=10_020_000_000
    )
    assert guard.is_synced("session", now_ns=10_020_000_000)
    assert not guard.is_synced("session", now_ns=20_020_000_001)


def test_session_switch_retires_old_session_and_preserves_epoch_event():
    guard = PacketGuard()
    sync(guard, session="old")
    sync(
        guard, session="new", peer="tcp:127.0.0.1:12346",
        pc_send_ns=20_000_000_000, quest_receive_us=1_000_000,
        quest_send_us=1_001_000, pc_receive_ns=20_020_000_000,
    )
    assert guard.accept(
        packet(10, session="old"),
        received=1_900_000_000.0,
        received_monotonic=10.11,
    )
    assert guard.last_session_change is None
    assert guard.accept(
        packet(1, source_us=1_100_000, session="new"),
        received=1_900_000_000.0,
        received_monotonic=20.11,
    )
    assert guard.last_session_change == {
        "old_session": "old", "new_session": "new", "stream": "left controller"
    }
    assert guard.accept(
        packet(2, source_us=1_101_000, session="new"),
        received=1_900_000_000.0,
        received_monotonic=20.111,
    )
    assert guard.last_session_change is None
    assert guard.accept(
        packet(11, session="old"),
        received=1_900_000_000.0,
        received_monotonic=10.11,
    ) is None


def test_legacy_requires_explicit_compatibility_but_old_utc_envelope_stays_rejected():
    assert PacketGuard().accept("Left buttons:,0,0,0,0,0", 100) is None
    assert PacketGuard(allow_legacy=True).accept("Left buttons:,0,0,0,0,0", 100)
    assert PacketGuard(allow_legacy=True).accept(
        "@q3 session 1 1790000000000|Left buttons:,0,0,0,0,0", 100
    ) is None


def test_debug_frame_counter_is_not_a_new_stream():
    guard = PacketGuard()
    sync(guard)
    for i in range(1000):
        assert guard.accept(
            packet(
                i, source_us=4_100_000 + i * 1000,
                payload=f"body iobt | fid={i}:0,0,0",
            ),
            received=1_900_000_000.0,
            received_monotonic=10.11 + i * 0.001,
        )
    assert len(guard._sequences) == 1


def test_session_extraction_and_periodic_challenge():
    guard = PacketGuard(sync_interval_s=2.0)
    peer = "udp:127.0.0.1:50000"
    challenge = guard.make_sync_challenge(peer, now_ns=10_000_000_000)
    assert challenge.startswith("@q3sync ")
    assert guard.make_sync_challenge(peer, now_ns=11_000_000_000) is None
    assert guard.make_sync_challenge(peer, now_ns=12_000_000_000)
    assert PacketGuard.session_from_packet(packet(1)) == "session"
    assert PacketGuard.session_from_packet("old packet") is None
