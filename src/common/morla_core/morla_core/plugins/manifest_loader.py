"""Manifest loading is data-only. Package installation is a separate trusted action."""
import os
from pathlib import Path
from .registry import Catalog
from morla_core.composition.parser import load_document

def load_catalog(paths):
    descriptors=[]
    for path in sorted(map(Path,paths)):
        value=load_document(path)
        descriptors.extend(value if isinstance(value,list) else [value])
    return Catalog(descriptors)

def discover_ament():
    """Read registered static manifest paths without importing entry-point modules."""
    from ament_index_python.resources import get_resources, get_resource
    manifests=[]
    for package in sorted(get_resources('morla_components')):
        contents,prefix=get_resource('morla_components',package)
        root=Path(prefix).resolve()
        for relative in contents.splitlines():
            candidate=root/relative.strip()
            # Check lexically: symlink installs point into the source tree, so
            # resolving the file itself would legitimately leave the prefix.
            check=Path(os.path.abspath(candidate))
            if root not in check.parents: raise ValueError('manifest escaped installed package prefix')
            manifests.append(candidate)
    return load_catalog(manifests)


def discover_descriptors():
    """Discover installed data manifests without importing/loading entry points.

    Python distribution entry-point values in ``morla.component_manifests`` are
    relative JSON/YAML resource paths, never Python module:function references.
    Entry points are installation metadata and only select static data files.
    """
    import importlib.metadata
    descriptors=[]
    try:
        ament=discover_ament()
    except ModuleNotFoundError as exc:
        if not (exc.name or '').startswith('ament_index_python'):raise
    else:
        descriptors.extend(ament.list())
    for distribution in importlib.metadata.distributions():
        for entry in distribution.entry_points:
            if entry.group!='morla.component_manifests':continue
            relative=Path(entry.value)
            if relative.is_absolute() or '..' in relative.parts or relative.suffix.lower() not in {'.json','.yaml','.yml'}:
                raise ValueError('component manifest entry point must name a distribution-relative JSON/YAML data file')
            root=Path(distribution.locate_file('')).resolve()
            path=Path(distribution.locate_file(relative)).resolve()
            if root not in path.parents:raise ValueError('component manifest escaped installed distribution')
            value=load_document(path)
            descriptors.extend(value if isinstance(value,list) else [value])
    return descriptors
