"""Trusted fr3_description assets, installed-first with a source-only fallback."""
from morla_core.resource_locator import package_share, ResourceLookupError, morla_source_root


def urdf_path():
    try:
        share=package_share('fr3_description')
    except ResourceLookupError:
        path=morla_source_root()/'src/robots/fairino/fr3_description/urdf/fairino3_v6.urdf'
    else:
        path=share/'urdf/fairino3_v6.urdf'
    if not path.is_file():
        raise FileNotFoundError('FR3 URDF missing from fr3_description: '+str(path))
    return path
