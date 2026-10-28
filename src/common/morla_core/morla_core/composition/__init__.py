"""ROS-free composable system API. Loading this module never opens hardware."""
from .diagnostics import CompilationError
from .parser import load_document
from morla_core.plugins.registry import Catalog, default_catalog

def compile_system(*args, **kwargs):
    from .compiler import compile_system as compile_impl
    return compile_impl(*args, **kwargs)

def verify_resolved(*args, **kwargs):
    from .compiler import verify_resolved as verify_impl
    return verify_impl(*args, **kwargs)

def import_profile(*args, **kwargs):
    from .legacy_import import import_profile as import_impl
    return import_impl(*args, **kwargs)
