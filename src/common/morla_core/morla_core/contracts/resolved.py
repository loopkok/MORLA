from dataclasses import dataclass
from copy import deepcopy

@dataclass(frozen=True)
class ResolvedSystem:
    """Owns a detached verified snapshot; callers receive copies of its payload."""
    _values: dict
    @classmethod
    def from_dict(cls,value,registry=None):
        from morla_core.composition import verify_resolved
        verify_resolved(value,registry=registry)
        return cls(deepcopy(value))
    def to_dict(self): return deepcopy(self._values)
