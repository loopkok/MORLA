"""Exercise the public CLI without any legacy module or script entry points."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from conftest import write_raw_episode

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]


def run(*args):
    env = {**os.environ, "PYTHONPATH": str(PACKAGE), "ASTRAL_DATA_PYTHON": sys.executable}
    result = subprocess.run(["bash", str(WORKSPACE / "scripts/data_pipeline.sh"), *map(str, args)],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


def test_cli_help_needs_no_optional_dependencies():
    # -S removes site packages: even numpy/h5py are unavailable here.
    env = {**os.environ, "PYTHONPATH": str(PACKAGE)}
    result = subprocess.run([sys.executable, "-S", "-m", "astral_data_collect.pipeline", "--help"],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "process-openpi" in result.stdout and "repair" in result.stdout


def test_unified_cli_pipeline_with_spaces(tmp_path):
    source, cleaned = tmp_path / "source session", tmp_path / "clean session"
    write_raw_episode(str(source / "episode000000"))
    raw_before = (source / "episode000000/robot_data.h5").read_bytes()
    report = tmp_path / "quantify.json"
    run("quantify", "--h5", source, "--out", report)
    assert json.loads(report.read_text())
    run("align", "--session", source)
    run("repair", "--session", source, "--dry-run")
    run("repair", "--session", source, "--out-session", cleaned)
    verified = json.loads(run("verify-aligned", "--source", source, "--session", cleaned))
    assert verified["episodes"] == 1 and verified["frames"] > 0
    output = tmp_path / "openpi output"
    run("process-openpi", cleaned, output, "--no-quarantine", "--image-size", "32")
    assert (output / "meta/info.json").exists()
    assert (output / "meta/processing_manifest.json").exists()
    act_output, intermediate = tmp_path / "act output", tmp_path / "v21 intermediate"
    run("process-act", cleaned, act_output, "--no-quarantine", "--image-size", "32",
        "--keep-v21", intermediate)
    assert (act_output / "meta/info.json").exists()
    assert (intermediate / "meta/info.json").exists()
    assert (source / "episode000000/robot_data.h5").read_bytes() == raw_before


def test_process_does_not_quarantine_or_export_by_default(tmp_path):
    source = tmp_path / 'short raw session'
    episode = Path(write_raw_episode(str(source / 'episode000000'), duration_s=1.0))
    output = tmp_path / 'must not export'
    env = {**os.environ, "PYTHONPATH": str(PACKAGE), "ASTRAL_DATA_PYTHON": sys.executable}
    result = subprocess.run(
        ["bash", str(WORKSPACE / "scripts/data_pipeline.sh"), "process-openpi",
         str(source), str(output), "--image-size", "32"],
        env=env, capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert episode.is_dir()
    assert not (source / 'quarantine').exists()
    assert not output.exists()

    quarantined_source = tmp_path / 'explicit quarantine session'
    quarantined_episode = Path(write_raw_episode(
        str(quarantined_source / 'episode000000'), duration_s=1.0
    ))
    quarantined_output = tmp_path / 'quarantined output'
    result = subprocess.run(
        ["bash", str(WORKSPACE / "scripts/data_pipeline.sh"), "process-openpi",
         str(quarantined_source), str(quarantined_output), "--quarantine", "--image-size", "32"],
        env=env, capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert not quarantined_episode.exists()
    assert (quarantined_source / 'quarantine/episode000000').is_dir()
    assert not quarantined_output.exists()


def test_verify_aligned_rejects_empty_session(tmp_path):
    from astral_data_collect.pipeline.verify_aligned import verify_session
    with pytest.raises(ValueError, match="no episodes"):
        verify_session(str(tmp_path), str(tmp_path / "empty"))
