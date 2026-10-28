from dataclasses import dataclass
from typing import Any

@dataclass(frozen=True)
class SampleEnvelope:
    session_id: str
    instance_id: str
    port_id: str
    source_epoch: str
    sequence: int
    source_time_ns: int
    received_monotonic_ns: int
    record_time_ns: int
    clock_domain: str
    sync_uncertainty_ns: int
    frame_id: str
    valid_mask: tuple[bool,...]
    provenance: str
    payload: Any
    quality_reasons: tuple[str,...] = ()
    def __post_init__(self):
        if self.provenance not in {'measured','estimated','command_echo','synthetic','fake','sim'}: raise ValueError('unknown sample provenance')
        if type(self.sequence) is not int or self.sequence<0: raise ValueError('invalid sequence')
        if self.sync_uncertainty_ns<0: raise ValueError('negative clock uncertainty')
