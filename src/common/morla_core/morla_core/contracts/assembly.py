"""Typed source wrappers delegate to the single compiler validator."""
from dataclasses import dataclass
from copy import deepcopy
from typing import Any

@dataclass(frozen=True)
class AssemblySpec:
    values: dict[str, Any]
    @classmethod
    def from_dict(cls, value):
        from morla_core.composition import compile_system
        compile_system(value)
        return cls(deepcopy(value))
    def to_dict(self): return deepcopy(self.values)

@dataclass(frozen=True)
class DeploymentBinding:
    values: dict[str, Any]
    def to_dict(self): return deepcopy(self.values)
