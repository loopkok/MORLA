from morla_input.stream_health import StreamHealth


def test_independent_quest_sockets_do_not_invalidate_each_other():
    health = StreamHealth()
    assert not health.observe("Left wrist:,0", "hands", 1.0)
    assert not health.observe("Head:,0", "head", 1.1)
    assert not health.observe("Right controller:,0", "controller", 1.2)
    assert not health.expired(1.49)
    assert health.disconnected("head") == ["head"]
    assert set(health.disconnected("hands")) == {"left wrist"}


def test_silent_motion_stream_expires_even_when_buttons_and_head_continue():
    health = StreamHealth()
    health.observe("Left wrist:,0", "hands", 1.0)
    health.observe("Left buttons:,0", "buttons", 1.59)
    health.observe("Head:,0", "head", 1.59)
    assert health.expired(1.6) == ["left wrist"]
    health.clear()
    assert not health.expired(50.0)


def test_landmark_loss_cannot_be_hidden_by_continuing_wrist_pose():
    health = StreamHealth()
    health.observe("Left landmarks:,0", "hands", 1.0)
    health.observe("Left wrist:,0", "hands", 1.59)
    assert health.expired(1.6) == ["left landmarks"]
    assert not health.observe("Left controller buttons:,0", "buttons", 1.6)


def test_replacement_socket_invalidates_anchor_but_old_disconnect_is_ignored():
    health = StreamHealth()
    health.observe("Left wrist:,0", "old", 1.0)
    assert health.observe("Left wrist:,0", "new", 1.1)
    assert not health.disconnected("old")
    assert health.disconnected("new") == ["left wrist"]


def test_resume_detects_gap_even_before_watchdog_timer_runs():
    health = StreamHealth()
    health.observe("Left:,0", "hands", 1.0)
    assert health.observe("Left:,0", "hands", 1.6)
