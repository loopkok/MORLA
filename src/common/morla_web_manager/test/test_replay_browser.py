"""Playback browsing is read-only and confined to the configured data roots."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from morla_web_manager.replay_browser import browse_replay_sources  # noqa: E402


def test_browse_saved_raw_aligned_and_robot_file_sources(tmp_path):
    episode = tmp_path / "raw/My Task/episode000005"
    episode.mkdir(parents=True)
    for name in ("robot_data.h5", "meta.json", "aligned_data.h5", "camera_data.h5"):
        (episode / name).write_bytes(b"sample")
    before = {p.name: (p.read_bytes(), p.stat().st_mtime_ns) for p in episode.iterdir()}
    root = browse_replay_sources([tmp_path])
    assert root["parent"] is None
    session = browse_replay_sources([tmp_path], str(episode.parent))
    entry = session["entries"][0]
    assert entry["is_dir"] and entry["selectable"] and entry["kind"] == "raw"
    contents = browse_replay_sources([tmp_path], entry["path"])
    assert contents["selectable"] and contents["kind"] == "raw"
    assert {item["name"] for item in contents["entries"]} == {"robot_data.h5", "aligned_data.h5"}
    assert {item["kind"] for item in contents["entries"]} == {"raw", "aligned"}
    robot_file = browse_replay_sources([tmp_path], str(episode / "robot_data.h5"))
    assert robot_file["path"] == str(episode)
    assert before == {p.name: (p.read_bytes(), p.stat().st_mtime_ns) for p in episode.iterdir()}


def test_unfinished_recording_and_unsupported_dataset_are_not_selectable(tmp_path):
    recording = tmp_path / "episode000001"
    recording.mkdir()
    (recording / "robot_data.h5").touch()
    for version in ("v2.1", "v3.0"):
        info = tmp_path / version / "meta/info.json"
        info.parent.mkdir(parents=True)
        info.write_text(json.dumps({"codebase_version": version}), encoding="utf-8")
    entries = {entry["name"]: entry for entry in browse_replay_sources([tmp_path])["entries"]}
    assert entries["episode000001"]["kind"] == "recording"
    assert not entries["episode000001"]["selectable"]
    assert entries["v2.1"]["kind"] == "lerobot" and entries["v2.1"]["selectable"]
    assert entries["v3.0"]["kind"] == "unsupported" and not entries["v3.0"]["selectable"]
    assert not browse_replay_sources([tmp_path], str(recording))["entries"][0]["selectable"]


def test_paths_and_symlinks_cannot_escape_configured_roots(tmp_path):
    root = tmp_path / "data"
    root.mkdir()
    external = tmp_path / "private"
    external.mkdir()
    (root / "escape").symlink_to(external, target_is_directory=True)
    for value in (str(external), str(root / "escape"), str(root / "..")):
        with pytest.raises(PermissionError):
            browse_replay_sources([root], value)
    assert browse_replay_sources([root])["entries"] == []


def test_multiple_roots_missing_roots_and_hidden_files(tmp_path):
    extra = tmp_path / "extra"
    extra.mkdir()
    (extra / ".hidden").mkdir()
    (extra / "notes.txt").touch()
    missing = tmp_path / "missing"
    result = browse_replay_sources([missing, extra, extra])
    assert len(result["roots"]) == 1
    assert result["path"] == str(extra)
    assert result["entries"] == []
    with pytest.raises(FileNotFoundError):
        browse_replay_sources([extra], str(extra / "absent"))
    with pytest.raises(FileNotFoundError):
        browse_replay_sources([missing])


def test_web_endpoint_uses_configured_roots_without_ros_node(monkeypatch, tmp_path):
    from morla_web_manager import web_server

    episode = tmp_path / "session/episode000003"
    episode.mkdir(parents=True)
    (episode / "robot_data.h5").touch()
    (episode / "meta.json").touch()
    monkeypatch.setattr(web_server.config, "REPLAY_BROWSE_ROOTS", [tmp_path])
    response = web_server.infer_replay_sources(str(episode.parent))
    assert response.ok
    assert response.data["path"] == str(episode.parent)
    assert response.data["entries"][0]["kind"] == "raw"
