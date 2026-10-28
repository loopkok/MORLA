"""Schema v3 keeps legacy layouts intact while allowing camera-free, independent actions."""
from dataclasses import dataclass, field
from copy import deepcopy
import json
from morla_core.schema import StreamSpec, StateBlockSpec
from morla_core.composition.hashing import digest
from morla_core.composition.parser import finite_json, fields

@dataclass(frozen=True)
class ActionBlockV3:
    block_id: str
    dim: int
    feature_names: tuple
    unit: str
    action_indices: tuple
    semantics: str
    writable: bool
    command_stream: str | None = None
    writer_id: str | None = None
    limits: tuple = ()
    source_state_indices: tuple = ()
    stream_indices: tuple = ()
    def __post_init__(self):
        if type(self.dim) is not int or self.dim < 1: raise ValueError("action dim must be positive integer")
        if len(self.feature_names)!=self.dim or len(self.action_indices)!=self.dim: raise ValueError("action dimensions mismatch")
        if self.semantics not in {"absolute_position","next_state","command","delta","velocity","effort","closure_ratio","delta_pose"}: raise ValueError("unknown action semantics")
        if self.writable and (not self.command_stream or not self.writer_id): raise ValueError("writable action requires stream and writer")
        for indexes in (self.source_state_indices,self.stream_indices):
            if indexes and (len(indexes)!=self.dim or any(type(i) is not int or i<0 for i in indexes)): raise ValueError("invalid action source indices")
    @classmethod
    def from_dict(cls, value):
        fields(value, cls.__dataclass_fields__, ("block_id","dim","feature_names","unit","action_indices","semantics","writable"))
        kwargs=dict(value)
        for key in ("feature_names","action_indices","limits","source_state_indices","stream_indices"): kwargs[key]=tuple(kwargs.get(key,()))
        return cls(**kwargs)
    def to_dict(self):
        from dataclasses import asdict
        return json.loads(json.dumps(asdict(self)))

@dataclass(frozen=True)
class SchemaV3:
    profile_id: str
    robot_id: str
    streams: tuple
    state_blocks: tuple
    action_blocks: tuple
    cameras: tuple = ()
    dataset_fps: int = 30
    action_source: str = "command"
    hold_frames: int = 10
    max_gap_ms: float = 100.0
    schema_version: int = 3
    profile_version: int = 1
    sensor_specs: tuple = ()
    metadata: dict = field(default_factory=dict)
    def __post_init__(self):
        finite_json(self.to_dict())
        if self.schema_version!=3 or type(self.dataset_fps) is not int or self.dataset_fps<1: raise ValueError("invalid v3 version/fps")
        if self.action_source not in {"command","next_state","requested","authorized","dispatched","feedback"}: raise ValueError("invalid action source")
        if type(self.hold_frames) is not int or self.hold_frames<0 or self.max_gap_ms<=0: raise ValueError("invalid alignment parameters")
        streams={s.stream_key:s for s in self.streams}
        if len(streams)!=len(self.streams): raise ValueError("duplicate stream key")
        if len(set(self.cameras))!=len(self.cameras): raise ValueError("duplicate camera")
        for side,blocks,attr in (("state",self.state_blocks,"state_indices"),("action",self.action_blocks,"action_indices")):
            indexes=[i for b in blocks for i in getattr(b,attr)]
            names=[n for b in blocks for n in b.feature_names]
            if any(type(i) is not int or i<0 for i in indexes) or sorted(indexes)!=list(range(len(indexes))): raise ValueError(side+" indices must densely partition vector")
            if len(set(names))!=len(names): raise ValueError(side+" names must be unique")
        for block in self.state_blocks:
            if block.stream_key not in streams: raise ValueError("unresolved state stream")
            if any(i>=streams[block.stream_key].shape[-1] for i in block.stream_indices): raise ValueError("state stream index out of range")
        for block in self.action_blocks:
            if block.command_stream and block.command_stream not in streams: raise ValueError("unresolved action stream")
            if block.source_state_indices and any(i>=self.state_dim for i in block.source_state_indices): raise ValueError("action state index out of range")
            if block.command_stream and block.stream_indices and any(i>=streams[block.command_stream].shape[-1] for i in block.stream_indices): raise ValueError("action stream index out of range")
        for camera in self.cameras:
            if "camera:"+camera not in streams: raise ValueError("unresolved camera stream")
    @property
    def state_dim(self): return sum(b.dim for b in self.state_blocks)
    @property
    def action_dim(self): return sum(b.dim for b in self.action_blocks)
    def state_names(self):
        result=[None]*self.state_dim
        for b in self.state_blocks:
            for i,n in zip(b.state_indices,b.feature_names): result[i]=n
        return result
    def action_names(self):
        result=[None]*self.action_dim
        for b in self.action_blocks:
            for i,n in zip(b.action_indices,b.feature_names): result[i]=n
        return result
    def to_dict(self):
        return {"schema_version":3,"profile_version":self.profile_version,"profile_id":self.profile_id,"robot_id":self.robot_id,
                "streams":[s.to_dict() for s in self.streams],"state_blocks":[s.to_dict() for s in self.state_blocks],"action_blocks":[s.to_dict() for s in self.action_blocks],
                "cameras":list(self.cameras),"dataset_fps":self.dataset_fps,"action_source":self.action_source,"hold_frames":self.hold_frames,"max_gap_ms":self.max_gap_ms,
                "sensor_specs":deepcopy(list(self.sensor_specs)),"metadata":deepcopy(self.metadata)}
    def to_json(self): return json.dumps(self.to_dict(),sort_keys=True,allow_nan=False)
    @classmethod
    def from_dict(cls,value):
        value=deepcopy(value)
        fields(value,cls.__dataclass_fields__,("profile_id","robot_id","streams","state_blocks","action_blocks"))
        value["streams"]=tuple(StreamSpec.from_dict(s) for s in value["streams"])
        value["state_blocks"]=tuple(StateBlockSpec.from_dict(s) for s in value["state_blocks"])
        value["action_blocks"]=tuple(ActionBlockV3.from_dict(s) for s in value["action_blocks"])
        for key in ("cameras","sensor_specs"): value[key]=tuple(value.get(key,()))
        return cls(**value)
    @classmethod
    def from_json(cls,value): return cls.from_dict(json.loads(value))
    @property
    def schema_hash(self): return digest(self.to_dict())
    @property
    def layout_hash(self): return digest({"state":self.state_names(),"action":self.action_names(),"cameras":self.cameras})

def to_v3(schema):
    """Explicit, nonmutating migration; legacy identity is recorded, never redefined."""
    value=schema.to_dict()
    value.pop("schema_hash",None); value.pop("layout_hash",None)
    for key in ("state_dim","action_dim","kinematics_hash"): value.pop(key,None)
    source=value.get("schema_version",2); value["schema_version"]=3
    value.setdefault("metadata",{}).update({"migration":{"source_version":source,"source_schema_hash":schema.schema_hash,"migrator_version":"1.0.0","differences":["camera requirement removed","action source indices explicit"]}})
    return SchemaV3.from_dict(value)
