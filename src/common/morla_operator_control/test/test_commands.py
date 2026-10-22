import base64
import json
import pytest

from morla_operator_control.commands import PHRASES, PedalEdges, SeenRequests, request, validate_request, voice_command
from morla_operator_control.audio_protocol import decode_pcm


@pytest.mark.parametrize("phrase,command", PHRASES.items())
def test_exact_wake_phrase_matches_full_instruction(phrase, command):
    assert voice_command("机 器 人 " + " ".join(phrase) + "。", .95) == command


@pytest.mark.parametrize("text", ["开始跟随", "不要机器人开始跟随", "机器人不要开始跟随", "机器人开始跟随后回家", "机器人", "打开电机", "机器人使能", "机器人归零"])
def test_negation_missing_wake_and_unlisted_motion_commands_do_not_execute(text):
    assert voice_command(text, .99) is None


@pytest.mark.parametrize("confidence", [.2, float("nan"), float("inf"), None])
def test_low_or_invalid_confidence_never_executes(confidence):
    assert voice_command("机器人开始跟随", confidence) is None


def test_request_validation_and_replay_and_dual_microphone_dedup():
    row = request("teleop_start", "voice_host")
    assert validate_request(json.dumps(row), now=row["sent_at"]) == row
    seen = SeenRequests()
    assert seen.accept(row, 1.)
    assert not seen.accept(row, 10.)
    assert not seen.accept(request("teleop_start", "voice_quest"), 1.8)
    assert seen.accept(request("teleop_start", "voice_quest"), 4.)
    assert seen.accept(request("record_start", "pedal"), 4.1)
    with pytest.raises(ValueError):
        validate_request(json.dumps(row), now=row["sent_at"] + 3)
    row["command"] = "enable"
    with pytest.raises(ValueError):
        validate_request(json.dumps(row))


def test_pedal_startup_held_key_repeat_and_reconnect_do_not_trigger():
    edge = PedalEdges({1: "teleop_toggle"})
    assert edge.update(1, 1, 1.) is None
    assert edge.update(1, 0, 2.) is None
    assert edge.update(1, 1, 3.) == "teleop_toggle"
    assert edge.update(1, 1, 3.1) is None
    assert edge.update(1, 2, 3.2) is None
    edge.reset()
    assert edge.update(1, 1, 4.) is None


def test_pedal_discard_requires_hold_and_release_and_ignores_short_press():
    edge = PedalEdges({1: "record_discard"})
    edge.released.add(1)
    assert edge.update(1, 1, 1.) is None
    assert edge.update(1, 0, 1.2) is None
    assert edge.update(1, 1, 2.) is None
    assert edge.update(1, 2, 4.) is None
    assert edge.update(1, 0, 4.) == "record_discard"


def test_pcm_framing_is_bounded_and_rejects_wrong_rate_or_encoding():
    payload = "VoicePCM:16000:" + base64.b64encode(bytes(1280)).decode()
    assert decode_pcm(payload) == bytes(1280)
    for bad in (payload.replace("16000", "48000"), "VoicePCM:16000:???", "VoicePCM:16000:AA=="):
        with pytest.raises(ValueError):
            decode_pcm(bad)
