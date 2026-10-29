import pytest

from morla_core import resource_locator
from morla_core.resource_locator import ResourceLookupError, resolve_package_resource


def make_tree(root, name, marker="package.xml"):
    base = root / name
    (base / "config").mkdir(parents=True)
    (base / marker).write_text("x")
    return base


def use_share(monkeypatch, share):
    monkeypatch.setattr(resource_locator, "package_share", lambda name: share.resolve())


def test_resolves_file_inside_the_share(tmp_path, monkeypatch):
    share = make_tree(tmp_path / "install", "demo")
    (share / "config" / "a.yaml").write_text("ok")
    use_share(monkeypatch, share)

    assert resolve_package_resource("demo", "config/a.yaml").read_text() == "ok"


def test_accepts_symlink_install_target_inside_the_source_package(tmp_path, monkeypatch):
    share = make_tree(tmp_path / "install", "demo")
    source = make_tree(tmp_path / "src", "demo")
    (source / "descriptor.json").write_text("{}")
    (share / "descriptor.json").symlink_to(source / "descriptor.json")
    use_share(monkeypatch, share)

    target = resolve_package_resource("demo", "descriptor.json")

    assert target == (source / "descriptor.json").resolve()
    assert target.read_text() == "{}"


def test_rejects_symlink_to_an_unrelated_location(tmp_path, monkeypatch):
    share = make_tree(tmp_path / "install", "demo")
    outside = tmp_path / "elsewhere"
    outside.mkdir()
    (outside / "secret").write_text("no")
    (share / "descriptor.json").symlink_to(outside / "secret")
    use_share(monkeypatch, share)

    with pytest.raises(ResourceLookupError, match="escapes package share"):
        resolve_package_resource("demo", "descriptor.json")


def test_rejects_symlink_to_a_same_named_directory_without_build_metadata(tmp_path, monkeypatch):
    share = make_tree(tmp_path / "install", "demo")
    imposter = tmp_path / "elsewhere" / "demo"
    imposter.mkdir(parents=True)
    (imposter / "secret").write_text("no")
    (share / "descriptor.json").symlink_to(imposter / "secret")
    use_share(monkeypatch, share)

    with pytest.raises(ResourceLookupError, match="escapes package share"):
        resolve_package_resource("demo", "descriptor.json")


def test_rejects_parent_traversal(tmp_path, monkeypatch):
    share = make_tree(tmp_path / "install", "demo")
    (tmp_path / "install" / "escape").write_text("no")
    use_share(monkeypatch, share)

    with pytest.raises(ResourceLookupError, match="within package share"):
        resolve_package_resource("demo", "../escape")


def test_reports_a_missing_resource(tmp_path, monkeypatch):
    share = make_tree(tmp_path / "install", "demo")
    use_share(monkeypatch, share)

    with pytest.raises(ResourceLookupError, match="does not exist"):
        resolve_package_resource("demo", "missing.yaml")
