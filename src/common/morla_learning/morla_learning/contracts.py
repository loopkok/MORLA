"""Learning identity is explicit and independent of transport addresses."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any


class LearningContractError(ValueError):
    pass


def digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def feature_contract(resolved: dict) -> dict:
    """Normalize a compiler snapshot or frozen capture schema without guessing names."""
    schema = resolved.get("capture_schema", resolved)
    contract = resolved.get("feature_contract")
    if contract is not None:
        return json.loads(json.dumps(contract, allow_nan=False))
    def features(kind):
        blocks = schema.get(kind + "_blocks", [])
        count = sum(int(b["dim"]) for b in blocks)
        result = [None] * count
        for block in blocks:
            indices = block.get(kind + "_indices", [])
            if len(indices) != block["dim"] or len(block["feature_names"]) != block["dim"]:
                raise LearningContractError(f"incomplete {kind} feature mapping")
            for index, name in zip(indices, block["feature_names"]):
                if type(index) is not int or not 0 <= index < count or result[index] is not None:
                    raise LearningContractError(f"invalid {kind} feature index")
                result[index] = {"name": name, "unit": block["unit"],
                                 "semantics": block.get("semantics", "observation")}
        if any(item is None for item in result):
            raise LearningContractError(f"incomplete {kind} layout")
        return result
    return {"state_features": features("state"), "action_features": features("action"),
            "cameras": list(schema.get("cameras", [])), "dataset_fps": schema.get("dataset_fps", 30)}


def feature_rows(contract: dict, kind: str) -> list:
    rows = contract.get(kind + "_features", contract.get(kind, []))
    if not isinstance(rows, list):
        raise LearningContractError(f"{kind}_features must be ordered list")
    return rows


def camera_keys(contract: dict) -> list[str]:
    rows = contract.get("cameras", [])
    if isinstance(rows, dict):
        return list(rows)
    return [x if isinstance(x, str) else x.get("key", x.get("label")) for x in rows]


@dataclass(frozen=True)
class TrainingSpec:
    backend: str
    backend_revision: str
    dataset_root: str
    dataset_fingerprint: str
    output_dir: str
    features: dict
    geometry_hash: str
    normalization_hash: str
    capacity: int
    camera_map: dict
    parameters: dict
    version: int = 1

    def __post_init__(self):
        if self.backend not in {"act", "openpi"}:
            raise LearningContractError("backend must be act or openpi")
        for field in ("backend_revision", "dataset_root", "dataset_fingerprint", "output_dir",
                      "normalization_hash", "geometry_hash"):
            if not isinstance(getattr(self, field), str) or not getattr(self, field).strip():
                raise LearningContractError(f"{field} is required")
        if self.version != 1 or type(self.capacity) is not int or self.capacity < 1:
            raise LearningContractError("invalid version/capacity")
        state, action = feature_rows(self.features, "state"), feature_rows(self.features, "action")
        if not action or max(len(state), len(action)) > self.capacity:
            raise LearningContractError("model capacity exceeded or no trainable actions")
        for rows in (state, action):
            names = [r if isinstance(r, str) else r.get("name", r.get("feature_id")) for r in rows]
            if any(not n for n in names) or len(set(names)) != len(names):
                raise LearningContractError("features require unique explicit names")
        cameras = camera_keys(self.features)
        if set(self.camera_map.values()) != set(cameras) or len(self.camera_map) != len(cameras):
            raise LearningContractError("camera_map must cover every camera exactly once")
        if self.backend == "openpi" and not cameras:
            raise LearningContractError("OpenPI vision backend requires cameras")
        if not isinstance(self.parameters, dict):
            raise LearningContractError("parameters must be a mapping")
        if set(self.parameters)&{'type','capacity','camera_map','features','contract','output_dir'}:
            raise LearningContractError("training parameters cannot override frozen contract fields")
        digest(self.to_dict())

    @classmethod
    def from_dict(cls, raw: dict):
        allowed = set(cls.__dataclass_fields__)
        if not isinstance(raw, dict) or set(raw) - allowed:
            raise LearningContractError("unknown training fields")
        try:
            return cls(**raw)
        except TypeError as exc:
            raise LearningContractError(str(exc)) from exc

    @property
    def feature_hash(self):
        return digest(self.features)

    def to_dict(self):
        return {field: getattr(self, field) for field in self.__dataclass_fields__}

    def config(self) -> dict:
        """Return a complete backend-neutral bridge configuration, never an exec string."""
        return {"version": 1, "backend": self.backend, "backend_revision": self.backend_revision,
                "dataset": {"root": self.dataset_root, "fingerprint": self.dataset_fingerprint,
                            "features": self.features},
                "policy": {"type": self.backend, "capacity": self.capacity,
                           "camera_map": self.camera_map, **self.parameters},
                "output_dir": self.output_dir,
                "contract": {"feature_hash": self.feature_hash, "geometry_hash": self.geometry_hash,
                             "normalization_hash": self.normalization_hash}}

    def argv(self, executable: str) -> list[str]:
        if not executable or "\x00" in executable:
            raise LearningContractError("invalid training executable")
        if self.backend == "act":
            args = [executable, f"--dataset.root={self.dataset_root}", "--policy.type=act",
                    "--policy.push_to_hub=false", f"--output_dir={self.output_dir}"]
            allowed = {"steps", "batch_size", "num_workers", "policy.chunk_size", "policy.n_action_steps",
                       "policy.optimizer_lr", "dataset.repo_id", "wandb.enable"}
            for key, value in self.parameters.items():
                if key not in allowed or isinstance(value, (dict, list)):
                    raise LearningContractError(f"unsupported ACT parameter {key}")
                args.append(f"--{key}={str(value).lower() if isinstance(value, bool) else value}")
            return args
        raise LearningContractError("OpenPI requires the version-locked MORLA bridge config; use write_config")

    def write_config(self, directory: str | Path):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / "training_spec.json"
        if path.exists() and json.loads(path.read_text()) != self.to_dict():
            raise LearningContractError("output contains a different frozen training spec")
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        tmp.replace(path)
        (directory / "backend_config.json").write_text(json.dumps(self.config(), ensure_ascii=False, indent=2) + "\n")
        return path


@dataclass(frozen=True)
class ModelManifest:
    version: int
    backend: str
    backend_revision: str
    feature_hash: str
    geometry_hash: str
    normalization_hash: str
    checkpoint_hash: str
    dataset_fingerprint: str
    features: dict
    camera_map: dict
    capacity: int
    validation: str

    def __post_init__(self):
        if self.version != 1 or self.validation not in {"config_valid", "loader_verified", "inference_verified"}:
            raise LearningContractError("unsupported manifest status/version")
        if digest(self.features) != self.feature_hash:
            raise LearningContractError("manifest feature hash mismatch")
        if self.backend not in {'act','openpi'}:
            raise LearningContractError('unsupported manifest backend')
        cameras=camera_keys(self.features)
        if (not isinstance(self.camera_map,dict) or len(self.camera_map)!=len(cameras)
                or set(self.camera_map.values())!=set(cameras)):
            raise LearningContractError('manifest camera map must exactly cover cameras')
        if type(self.capacity) is not int or self.capacity < max(len(feature_rows(self.features, "state")),
                                                               len(feature_rows(self.features, "action"))):
            raise LearningContractError("manifest capacity exceeded")
        for field in ("backend", "backend_revision", "geometry_hash", "normalization_hash",
                      "checkpoint_hash", "dataset_fingerprint"):
            if not isinstance(getattr(self, field), str) or not getattr(self, field):
                raise LearningContractError(f"missing {field}")

    @classmethod
    def from_dict(cls, raw):
        if set(raw) != set(cls.__dataclass_fields__):
            raise LearningContractError("invalid manifest fields")
        return cls(**raw)

    def to_dict(self):
        return {key: getattr(self, key) for key in self.__dataclass_fields__}


def validate_model(resolved: dict, manifest: ModelManifest, *, normalization_hash: str,
                   backend_revision: str | None = None, camera_map: dict | None = None) -> None:
    expected = feature_contract(resolved)
    if digest(expected) != manifest.feature_hash or expected != manifest.features:
        raise LearningContractError("model feature identity/order/unit/action semantics mismatch")
    if resolved.get("geometry_hash", resolved.get("kinematics_hash")) != manifest.geometry_hash:
        raise LearningContractError("model geometry mismatch")
    if not normalization_hash or manifest.normalization_hash != normalization_hash:
        raise LearningContractError("model normalization mismatch")
    if backend_revision is not None and manifest.backend_revision != backend_revision:
        raise LearningContractError("model backend revision mismatch")
    expected_map=camera_map
    if expected_map is None:
        # Compiler cameras can freeze model slots. Without explicit slots the
        # only implicit mapping permitted is the identity, never a permutation.
        cameras=expected.get('cameras',{})
        expected_map={}
        for label in camera_keys(expected):
            row=cameras.get(label,{}) if isinstance(cameras,dict) else {}
            expected_map[row.get('model_slot') or label]=label
    if manifest.camera_map!=expected_map:
        raise LearningContractError('model camera slot mapping mismatch')
    if manifest.validation != "inference_verified":
        raise LearningContractError("checkpoint has not passed inference verification")


def project_features(values: list, source: list[str], target: list[str]) -> list[float]:
    """Explicit named projection; no silent padding or truncation."""
    if len(values) != len(source) or len(set(source)) != len(source) or len(set(target)) != len(target):
        raise LearningContractError("ambiguous feature projection")
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
        raise LearningContractError("nonfinite feature")
    mapping = dict(zip(source, values))
    try:
        return [float(mapping[name]) for name in target]
    except KeyError as exc:
        raise LearningContractError(f"missing feature {exc}") from exc
