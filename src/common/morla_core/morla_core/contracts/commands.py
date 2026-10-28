"""Wire-neutral command contracts matching the runtime and MORLA ROS IDL.

issued_ns/ttl_ns use the session host monotonic clock. Authority fields are
validated by the gateway; constructing these values never grants permission.
"""
from dataclasses import dataclass
from enum import Enum
from typing import Mapping

class ControlMode(str, Enum):
    POSITION='position'; VELOCITY='velocity'; EFFORT='effort'; POSE='pose'; TRAJECTORY='trajectory'; GRIPPER_RATIO='gripper_ratio'
class ReceiptStatus(str, Enum):
    REJECTED='rejected'; ACCEPTED='accepted'; DISPATCHED='dispatched'; DEVICE_ACKNOWLEDGED='device_acknowledged'; COMPLETED='completed'
@dataclass(frozen=True)
class ComponentTarget:
    component_id: str
    mode: str
    names: tuple[str,...]
    values: tuple[float,...]
    units: tuple[str,...]
@dataclass(frozen=True)
class CommandEnvelope:
    session_id: str
    producer: str
    token: str
    epoch: int
    sequence: int
    issued_ns: int
    ttl_ns: int
    commands: tuple[ComponentTarget,...]
    assembly_revision: int | None = None
    group_sequence: int | None = None
    desired_execute_at_ns: int | None = None
@dataclass(frozen=True)
class ActivationPermit:
    session_id: str
    token: str
    producer: str
    resources: tuple[str,...]
    epoch: int
    expires_ns: int
@dataclass(frozen=True)
class StopPolicy:
    mode: str
    timeout_ns: int
    independently_verified: bool = False
@dataclass(frozen=True)
class CommandReceipt:
    status: ReceiptStatus
    command_id: str
    reason: str | None = None
    device_acknowledged: bool | None = None
