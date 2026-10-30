"""Adapter SDK matches the runtime ABI, independent of ROS and vendor SDKs.

Descriptor discovery is static. The selected adapter factory receives
AdapterFactoryContext only after runtime resource locks are owned. configure()
receives the smaller AdapterContext. Gateway authority is checked centrally;
adapter.activate() is called only after that gate, with no forged permit argument.
"""
from dataclasses import dataclass
from typing import Any, Callable, Protocol, TypedDict

class AdapterContext(TypedDict):
    session_id: str
    instance: dict

class AdapterFactoryContext(TypedDict, total=False):
    session_id: str
    clock: Callable[[],int]
    resource_root: str
    lane_fd: int
    shared_device_sessions: dict
    instances: dict[str, dict]
    device_session: dict | None

class ManagedAdapter(Protocol):
    def configure(self, context: AdapterContext) -> None: ...
    def health(self, now_ns: int) -> dict: ...
    def close(self) -> None: ...

class StateProvider(Protocol):
    def read(self, now_ns: int) -> dict: ...

class ActuatorAdapter(ManagedAdapter, Protocol):
    def prepare(self) -> bool | dict: ...
    def activate(self) -> bool | dict: ...
    def submit(self, command: dict) -> dict: ...
    def stop(self) -> bool | dict: ...

class InputAdapter(StateProvider, Protocol):
    """Input sampling is separate from actuator command submission."""

class InputFeedbackProvider(Protocol):
    def feedback(self, event: Any) -> dict: ...

class OperationProvider(Protocol):
    def operate(self, operation: str) -> bool | dict: ...

class AdapterFactory(Protocol):
    def __call__(self, instance: dict, context: AdapterFactoryContext) -> ManagedAdapter: ...

@dataclass(frozen=True)
class ComponentDescriptor:
    values: dict
    @classmethod
    def from_dict(cls,value):
        from copy import deepcopy
        from morla_core.plugins.registry import Catalog
        Catalog([value]);return cls(deepcopy(value))
    def to_dict(self):
        from copy import deepcopy
        return deepcopy(self.values)
