"""Discover installed component data without importing the plugin."""
from importlib import metadata
import os
from pathlib import Path, PurePosixPath

from .registry import Catalog
from morla_core.composition.parser import load_document


def load_catalog(paths):
    return Catalog(row for path in sorted(map(Path, paths)) for row in _rows(path))


def _rows(path):
    value = load_document(path)
    return value if isinstance(value, list) else [value]


def _relative(value):
    if not isinstance(value, str) or not value or '\\' in value:
        raise ValueError('expected a package-relative path')
    path = PurePosixPath(value)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError('expected a package-relative path')
    return path


def _within(root, relative):
    path = (root / _relative(relative)).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError('missing resource or path outside package: ' + relative)
    return path


class WheelResources:
    def __init__(self, distribution, marker):
        self.directory = PurePosixPath(str(marker)).parent
        self.site = Path(distribution.locate_file('')).resolve()
        self.root = self.site / self.directory
        self.files = {str(path) for path in distribution.files or ()}
        index = load_document(self.resource('morla_plugin.json'))
        if not isinstance(index, dict) or type(index.get('version')) is not int or index['version'] != 1:
            raise ValueError('unsupported morla_plugin.json version')
        self.package = index.get('resource_package')
        if not isinstance(self.package, str) or not self.package.isidentifier() or not self.package.isascii():
            raise ValueError('invalid resource_package')
        self.manifests = index.get('manifests')
        if not isinstance(self.manifests, list) or not self.manifests:
            raise ValueError('expected nonempty manifests list')

    def resource(self, relative):
        recorded = self.directory / _relative(relative)
        if str(recorded) not in self.files:
            raise ValueError('resource is missing from wheel RECORD: ' + relative)
        path = _within(self.root.resolve(), relative)
        if not path.is_relative_to(self.site) or path != self.site / recorded:
            raise ValueError('resource symlinks are not supported in wheels')
        return path


def _wheel_markers(distribution):
    return [path for path in distribution.files or ()
            if path.name == 'morla_plugin.json' and len(path.parts) > 1
            and not path.is_absolute() and '..' not in path.parts]


def wheel_resource(package, relative):
    resources = []
    for distribution in metadata.distributions():
        for marker in _wheel_markers(distribution):
            try:
                wheel = WheelResources(distribution, marker)
            except (OSError, ValueError):
                continue
            if wheel.package == package:
                resources.append(wheel)
    if len(resources) > 1:
        raise ValueError('ambiguous wheel resource package: ' + package)
    return resources[0].resource(relative) if resources else None


def _ament_manifests():
    try:
        from ament_index_python.resources import get_resources, get_resource
    except ModuleNotFoundError as exc:
        if not (exc.name or '').startswith('ament_index_python'):
            raise
        return
    for package in sorted(get_resources('morla_components')):
        contents, prefix = get_resource('morla_components', package)
        root = Path(prefix).resolve()
        for line in contents.splitlines():
            if line.strip():
                yield package, root, line.strip()


def discover_ament():
    return load_catalog(root / _relative(relative) for _, root, relative in _ament_manifests())


def register_installed(catalog):
    """Report discovery failures by origin and quarantine ambiguous identities."""
    for package, root, relative in _ament_manifests():
        try:
            # Ament symlink installs can point back to their source checkout.
            catalog.add_installed(_rows(root / _relative(relative)), 'ament:' + package)
        except (OSError, ValueError) as exc:
            catalog.report('ament:' + package, exc)
    for distribution in metadata.distributions():
        for marker in _wheel_markers(distribution):
            name = distribution.name
            origin = 'wheel:' + (name or str(marker))
            try:
                if not name:
                    raise ValueError('wheel distribution name is missing')
                wheel = WheelResources(distribution, marker)
                for manifest in wheel.manifests:
                    catalog.add_installed(_rows(wheel.resource(manifest)), origin, name)
            except (OSError, ValueError) as exc:
                catalog.report(origin, exc)
        for entry in distribution.entry_points:
            if entry.group == 'morla.component_manifests':
                origin = 'legacy:' + (distribution.name or entry.name)
                try:
                    relative = _relative(entry.value)
                    if relative.suffix.lower() not in {'.json', '.yaml', '.yml'}:
                        raise ValueError('expected relative JSON/YAML manifest path')
                    root = Path(distribution.locate_file('')).resolve()
                    catalog.add_installed(_rows(_within(root, entry.value)), origin)
                except (OSError, ValueError) as exc:
                    catalog.report(origin, exc)


def discover_descriptors():
    catalog = Catalog()
    register_installed(catalog)
    return catalog.list()
