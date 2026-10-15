"""Read-only discovery of playback sources on the robot host, without ROS/HDF5."""
from __future__ import annotations

import json
from pathlib import Path


def _source_kind(path: Path) -> str:
    if path.is_dir():
        if (path / "robot_data.h5").is_file():
            return "raw" if (path / "meta.json").is_file() else "recording"
        info = path / "meta/info.json"
        if info.is_file():
            try:
                meta = json.loads(info.read_text(encoding="utf-8"))
                return "lerobot" if meta.get("codebase_version") == "v2.1" else "unsupported"
            except (OSError, ValueError, AttributeError):
                return "folder"
        return "folder"
    if path.name == "robot_data.h5":
        return "raw" if (path.parent / "meta.json").is_file() else "recording"
    if path.name == "aligned_data.h5":
        return "aligned"
    return "hdf5"


def browse_replay_sources(roots: list[Path], value: str = "") -> dict:
    """List one folder; resolve symlinks before enforcing configured roots."""
    roots = list(dict.fromkeys(Path(root).expanduser().resolve() for root in roots))
    available = [root for root in roots if root.is_dir()]
    if not available:
        raise FileNotFoundError("尚未找到数据目录，请先录制数据或配置数据根目录")
    path = Path(value).expanduser().resolve() if value else available[0]

    def allowed(candidate: Path) -> bool:
        return any(candidate.is_relative_to(root) for root in available)

    if not allowed(path):
        raise PermissionError("请选择数据根目录内的回放源")
    if path.is_file():
        path = path.parent
    if not path.is_dir():
        raise FileNotFoundError(f"目录不存在：{path}")

    entries = []
    for child in path.iterdir():
        if child.name.startswith("."):
            continue
        try:
            if not allowed(child.resolve()):
                continue
            directory = child.is_dir()
            if not directory and (
                not child.is_file() or child.suffix.lower() != ".h5"
                or child.name == "camera_data.h5"
            ):
                continue
            kind = _source_kind(child)
            entries.append({
                "name": child.name, "path": str(child), "is_dir": directory,
                "kind": kind,
                "selectable": kind in {"raw", "aligned", "hdf5", "lerobot"},
            })
        except (OSError, RuntimeError):
            continue
    entries.sort(key=lambda entry: (not entry["is_dir"], entry["name"].casefold()))
    kind = _source_kind(path)
    return {
        "path": str(path),
        "parent": str(path.parent) if allowed(path.parent) and path.parent != path else None,
        "kind": kind, "selectable": kind in {"raw", "lerobot"}, "entries": entries,
        "roots": [{"path": str(root), "name": str(root)} for root in available],
    }
