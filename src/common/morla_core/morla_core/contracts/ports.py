from dataclasses import dataclass

@dataclass(frozen=True)
class PortSpec:
    direction: str
    semantic: str
    dim: int
    unit: str
    codec: str='numeric'
    dtype: str='float64'
    def __post_init__(self):
        if self.direction not in {'input','output'}:raise ValueError('invalid port direction')
        if type(self.dim) is not int or self.dim<1:raise ValueError('invalid port dimension')
        if not self.semantic or not self.unit:raise ValueError('port semantic and unit required')
