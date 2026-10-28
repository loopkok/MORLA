import copy
import pytest
from morla_learning.contracts import (TrainingSpec, ModelManifest, LearningContractError,
                                      digest, validate_model, project_features)


def spec(**overrides):
    values = dict(backend="act", backend_revision="locked-test-revision", dataset_root="/tmp/data with spaces",
                  dataset_fingerprint="dataset-sha", output_dir="/tmp/model", geometry_hash="geometry",
                  normalization_hash="norm", capacity=32, camera_map={}, parameters={"steps": 2},
                  features={"state_features": [{"name": "q", "unit": "rad"}, {"name": "dq", "unit": "rad/s"}],
                            "action_features": [{"name": "q_target", "unit": "rad", "semantics": "absolute_position"}],
                            "cameras": [], "dataset_fps": 30})
    values.update(overrides)
    return TrainingSpec(**values)


def manifest(s):
    return ModelManifest(1, s.backend, s.backend_revision, s.feature_hash, s.geometry_hash,
                         s.normalization_hash, "checkpoint-sha", s.dataset_fingerprint, s.features,
                         s.camera_map, s.capacity, "inference_verified")


def test_training_roundtrip_and_safe_argv(tmp_path):
    s = spec(); assert TrainingSpec.from_dict(s.to_dict()) == s
    assert s.argv("lerobot-train")[1] == "--dataset.root=/tmp/data with spaces"
    s.write_config(tmp_path)
    with pytest.raises(LearningContractError): spec(normalization_hash="other").write_config(tmp_path)


def test_capacity_unknown_features_and_missing_camera_rejected():
    with pytest.raises(LearningContractError): spec(capacity=1)
    with pytest.raises(LearningContractError): spec(backend="openpi")
    with pytest.raises(LearningContractError): TrainingSpec.from_dict({**spec().to_dict(), "ignore_hash": True})


def test_same_dimension_wrong_unit_or_order_is_incompatible():
    s = spec(); m = manifest(s); resolved = {"feature_contract": s.features, "geometry_hash": "geometry"}
    validate_model(resolved, m, normalization_hash="norm")
    wrong = copy.deepcopy(resolved); wrong["feature_contract"]["state_features"][0]["unit"] = "degree"
    with pytest.raises(LearningContractError): validate_model(wrong, m, normalization_hash="norm")
    with pytest.raises(LearningContractError): validate_model(resolved, m, normalization_hash="other")
    with pytest.raises(LearningContractError): validate_model({**resolved, "geometry_hash": "other"}, m, normalization_hash="norm")


def test_projection_is_named_finite_and_explicit():
    assert project_features([1,2,3], ["c","a","b"], ["b","a"]) == [3,2]
    for values, names, target in [([1], ["a"], ["missing"]), ([float('nan')], ["a"], ["a"]), ([1,2], ["a","a"], ["a"])]:
        with pytest.raises(LearningContractError): project_features(values, names, target)
