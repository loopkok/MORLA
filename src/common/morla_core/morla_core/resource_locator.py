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


def resolve_robot_asset(model_id: str) -> Path:
    """Resolve a model ID through robot_description's installed asset manifest.

    Runtime consumers use stable model IDs from robot profiles. The manifest is
    the only place that maps those IDs to files, so consumers do not invent a
    second set of robot-specific paths.
    """
    if not model_id or not isinstance(model_id, str):
        raise ResourceLookupError("robot asset model_id must be a non-empty string")
    share = package_share("robot_description")
    manifest_relative = "config/asset_manifest.yaml"
    source_share: Path | None = None
    try:
        import yaml

        try:
            manifest_path = resolve_package_resource("robot_description", manifest_relative)
        except ResourceLookupError:
            # colcon --symlink-install places the resource tree outside install/share.
            # Permit that only when the manifest symlink resolves into the matching
            # robot_description source package; arbitrary escaping symlinks stay blocked.
            manifest_path = (share / manifest_relative).resolve(strict=True)
            source_share = manifest_path.parent.parent
            if source_share.name != "robot_description" or not (source_share / "CMakeLists.txt").is_file():
                raise
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except ResourceLookupError:
        raise
    except Exception as exc:  # keep optional parser errors behind a stable API
        raise ResourceLookupError("failed to load robot_description asset manifest") from exc
    # Supplemental catalogs add independent models without rewriting the frozen
    # manifest revision referenced by existing robot profiles.
    models = list(manifest.get("models", []))
    catalog_root = source_share if source_share is not None else share
    for supplement in sorted((catalog_root / "config" / "asset_manifests").glob("*.yaml")):
        resolved = supplement.resolve()
        if not resolved.is_relative_to(catalog_root):
            raise ResourceLookupError("supplemental model manifest escapes package share")
        try:
            extra = yaml.safe_load(resolved.read_text(encoding="utf-8"))
            if extra.get("package") != "robot_description" or extra.get("schema_version") != 1:
                raise ValueError("unsupported supplemental manifest")
            models.extend(extra.get("models", []))
        except Exception as exc:
            raise ResourceLookupError("failed to load supplemental model manifest") from exc
    matches = [
        item
        for item in models
        if isinstance(item, dict) and item.get("model_id") == model_id
    ]
    if len(matches) != 1:
        raise ResourceLookupError(
            f"asset manifest must contain exactly one model_id {model_id!r}; found {len(matches)}"
        )
    relative = matches[0].get("path")
    if not relative:
        raise ResourceLookupError(f"robot model {model_id!r} has no standalone asset path")
    try:
        return resolve_package_resource("robot_description", str(relative))
    except ResourceLookupError:
        if source_share is None:
            raise
        rel_path = Path(str(relative))
        if rel_path.is_absolute() or ".." in rel_path.parts:
            raise ResourceLookupError(f"asset path escapes source package: {relative!r}")
        target = (source_share / rel_path).resolve()
        if not target.is_relative_to(source_share) or not target.is_file():
            raise ResourceLookupError(f"source asset is missing or escapes package: {relative!r}")
        return target
