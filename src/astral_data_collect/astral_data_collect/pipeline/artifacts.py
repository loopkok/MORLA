"""Write derived artifacts completely before replacing a previous version."""
from contextlib import contextmanager
import os
import shutil
import tempfile


def reject_overlap(source: str, output: str) -> None:
    source, output = os.path.realpath(source), os.path.realpath(output)
    if os.path.commonpath([source, output]) in (source, output):
        raise ValueError('source and output directories must not overlap')


@contextmanager
def atomic_file(path: str):
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    fd, staged = tempfile.mkstemp(prefix='.staged-', dir=parent)
    os.close(fd)
    try:
        yield staged
        os.replace(staged, path)
    finally:
        if os.path.exists(staged):
            os.unlink(staged)


@contextmanager
def atomic_directory(path: str):
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    os.makedirs(parent, exist_ok=True)
    staged = tempfile.mkdtemp(prefix='.dataset-', dir=parent)
    backup = None
    try:
        yield staged
        if os.path.exists(path):
            backup = tempfile.mkdtemp(prefix='.previous-', dir=parent)
            os.rmdir(backup)
            os.replace(path, backup)
        try:
            os.replace(staged, path)
        except BaseException:
            if backup is not None:
                os.replace(backup, path)
                backup = None
            raise
        if backup is not None:
            shutil.rmtree(backup)
    finally:
        if os.path.isdir(staged):
            shutil.rmtree(staged)


def source_signature(episode_dir: str) -> str:
    import hashlib
    import json
    files = {}
    for name in ("robot_data.h5", "camera_data.h5", "meta.json"):
        path = os.path.join(episode_dir, name)
        stat = os.stat(path)
        files[name] = [stat.st_size, stat.st_mtime_ns]
    with open(os.path.join(episode_dir, "meta.json"), "rb") as f:
        files["meta_sha256"] = hashlib.sha256(f.read()).hexdigest()
    return json.dumps(files, sort_keys=True)


def require_source_signature(aligned, episode_dir: str) -> None:
    """Refuse to consume aligned data without proof that it matches its raw inputs."""
    recorded = aligned.attrs.get("raw_signature")
    if isinstance(recorded, bytes):
        recorded = recorded.decode("utf-8", errors="replace")
    if not recorded:
        raise ValueError("aligned has no raw_signature; rebuild from raw")
    if recorded != source_signature(episode_dir):
        raise ValueError("aligned raw source changed since alignment; rebuild")
