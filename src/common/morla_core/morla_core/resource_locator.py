"""Package-share and source-checkout resource lookup with traversal guards."""
from __future__ import annotations

from pathlib import Path


class ResourceLookupError(RuntimeError):
    pass


def package_share(package_name: str) -> Path:
    """Return an installed ament package share path (imports ROS index lazily)."""
    if not package_name or "/" in package_name or package_name in {".", ".."}:
        raise ResourceLookupError(f"invalid package name {package_name!r}")
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package_name)).resolve()
    except Exception as exc:  # import and index errors become a stable core error
        raise ResourceLookupError(f"ament package {package_name!r} is not installed") from exc


def resolve_package_resource(package_name: str, relative_path: str, *, must_exist: bool = True) -> Path:
    """Resolve a relative path inside a package share, rejecting path escape."""
    relative = Path(relative_path)
    if relative.is_absolute() or ".." in relative.parts:
        raise ResourceLookupError(f"resource path must stay within package share: {relative_path!r}")
    base = package_share(package_name)
    target = (base / relative).resolve()
    if not target.is_relative_to(base):
        raise ResourceLookupError(f"resource symlink escapes package share: {relative_path!r}")
    if must_exist and not target.exists():
        raise ResourceLookupError(f"resource does not exist: {package_name}/{relative_path}")
    return target


def morla_source_root(start: str | Path | None = None) -> Path:
    """Find this source checkout without referring to Astral/XNero workspaces."""
    candidates = [Path(start).resolve()] if start is not None else [Path(__file__).resolve()]
    for candidate in candidates:
        for parent in (candidate, *candidate.parents):
            if (parent / "docs" / "MIGRATION_PLAN.md").is_file() and (parent / "src").is_dir():
                return parent
    raise ResourceLookupError("MORLA source root not found; set MORLA_WS for installed tools")


def resolve_source_resource(package_path: str | Path, relative_path: str) -> Path:
    """Resolve a path within a source package, rejecting path traversal."""
    base = Path(package_path).resolve()
    relative = Path(relative_path)
    if relative.is_absolute() or ".." in relative.parts:
        raise ResourceLookupError(f"resource path must be relative: {relative_path!r}")
    target = (base / relative).resolve()
    if not target.is_relative_to(base):
        raise ResourceLookupError(f"resource symlink escapes package source: {relative_path!r}")
    if not target.exists():
        raise ResourceLookupError(f"source resource does not exist: {target}")
    return target
