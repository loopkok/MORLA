"""Bounded, revisioned storage for the external piano project; no ROS imports."""
from __future__ import annotations

import copy
from contextlib import contextmanager
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import threading
import uuid

from .piano_calibration import WORK_POSE_DEG, validate_work_poses


class PianoError(ValueError):
    pass


class PianoConflict(PianoError):
    pass


def strict_json(text):
    def reject(value):
        raise PianoError(f"不允许非有限数值: {value}")
    return json.loads(text, parse_constant=reject)


def canonical(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))


def revision(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def number(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise PianoError(f"{label} 必须在 {low} 至 {high} 之间")
    return value


class PianoStore:
    def __init__(self, project: Path, root: Path):
        self.project, self.root = project.resolve(), root.resolve()
        self.lock = threading.RLock()
        self._model = None

    @contextmanager
    def file_lock(self):
        self.root.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.root / ".store.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            yield
        finally:
            os.close(fd)

    def model(self):
        if self._model is None:
            path = self.project / "morla_piano/model.py"
            if not path.is_file():
                raise PianoError(f"未找到外部钢琴项目：{self.project}，请配置 MORLA_PIANO_PROJECT")
            name = "_morla_web_piano_model_" + hashlib.sha256(str(path).encode()).hexdigest()[:12]
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
            self._model = module
        return self._model

    def atomic(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=".piano-", dir=path.parent)
        try:
            with os.fdopen(fd, "w") as stream:
                stream.write(canonical(value) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def validate(self, config):
        config = copy.deepcopy(config)
        if isinstance(config, dict):
            config.setdefault("calibration", {"work_pose_deg": copy.deepcopy(WORK_POSE_DEG)})
            if isinstance(config["calibration"], dict):
                config["calibration"].setdefault("work_pose_deg", copy.deepcopy(WORK_POSE_DEG))
        canonical(config)
        model = self.model()
        model.validate_config(config)
        validate_work_poses(config)
        # User-editable limits may narrow the actual contract, never widen it.
        reference = model.load_config(self.project / "config/piano.placeholder.json")
        for side in model.SIDES:
            h = config["hands"][side]
            for kind in ("arm", "hand"):
                for limit, hard in zip(h[kind + "_limits_rad"], reference["hands"][side][kind + "_limits_rad"]):
                    if limit[0] < hard[0] or limit[1] > hard[1]:
                        raise PianoError(f"{side} {kind} 不能放宽 MORLA 关节限位")
            lo, hi = h["shift_range_white"]
            number(lo, "最小移位", -35, 0)
            number(hi, "最大移位", 0, 35)
            if int(lo) != lo or int(hi) != hi:
                raise PianoError("白键移位必须为整数")
            for key, pose in h.get("arm_positions_rad", {}).items():
                if not re.fullmatch(r"-?(0|[1-9][0-9]?)", key) or not lo <= int(key) <= hi:
                    raise PianoError("实测姿态索引必须为移位范围内的整数")
            # Force all generated targets to be bounded before a session can enable a driver.
            for shift in range(int(lo), int(hi) + 1):
                try:
                    model.arm_target(config, side, shift)
                except ValueError as exc:
                    raise PianoError(f"{side} 移位 {shift}: {exc}") from exc
        bounds = {
            "control_hz": (20, 125), "feedback_timeout_s": (.1, .5),
            "arrival_tolerance_rad": (.001, math.radians(3)), "arrival_timeout_s": (.2, 40),
            "arrival_dwell_s": (.04, 2), "max_arm_velocity_rad_s": (.01, .3),
            "prepare_duration_s": (1, 60), "move_duration_s": (.1, 30),
            "settle_s": (.06, 5), "release_settle_s": (.04, 2),
            "lead_in_s": (0, 60), "max_lateness_s": (.01, .1),
            "max_tracking_error_rad": (.01, math.radians(30)),
        }
        for key, (low, high) in bounds.items():
            number(config["timing"][key], key, low, high)
        number(config["ros"]["connect_timeout_s"], "连接超时", .5, 15)
        number(config["score_defaults"]["tempo_bpm"], "默认速度", 20, 300)
        model.pitch(config["score_defaults"]["tonic"])
        return copy.deepcopy(config)

    def config(self):
        with self.lock:
            path = self.root / "config.json"
            value = strict_json(path.read_text()) if path.exists() else self.model().load_config(self.project / "config/piano.placeholder.json")
            value.setdefault("calibration", {"work_pose_deg": copy.deepcopy(WORK_POSE_DEG)})
            if isinstance(value["calibration"], dict):
                value["calibration"].setdefault("work_pose_deg", copy.deepcopy(WORK_POSE_DEG))
            self.validate(value)
            return {"config": value, "revision": revision(value)}

    def save_config(self, value, expected_revision, certify=False):
        with self.lock, self.file_lock():
            if self.config()["revision"] != expected_revision:
                raise PianoConflict("配置已被其他页面修改，请重新载入")
            value = copy.deepcopy(value)
            value["calibrated"] = certify is True
            self.validate(value)
            self.atomic(self.root / "config.json", value)
            return self.config()

    def _score_path(self, score_id):
        if not re.fullmatch(r"[a-f0-9]{32}", score_id):
            raise PianoError("无效曲谱 ID")
        path = self.root / "scores" / (score_id + ".json")
        if path.is_symlink() or path.parent.resolve() != (self.root / "scores").resolve():
            raise PianoError("无效曲谱路径")
        return path

    def list_scores(self):
        with self.lock:
            result = []
            directory = self.root / "scores"
            if directory.exists():
                for path in sorted(directory.glob("*.json")):
                    if not re.fullmatch(r"[a-f0-9]{32}", path.stem) or path.is_symlink():
                        continue
                    score = strict_json(path.read_text())
                    result.append({"id": path.stem, "name": score["title"], "tempo_bpm": score["tempo_bpm"], "events": len(score["events"])})
            return result

    def validate_score(self, score):
        canonical(score)
        if not isinstance(score, dict) or not isinstance(score.get("events"), list) or not 1 <= len(score["events"]) <= 5000:
            raise PianoError("曲谱需要 1 至 5000 个时序事件")
        number(score.get("tempo_bpm"), "曲谱速度", 20, 300)
        self.model().pitch(score.get("tonic", "C4"))
        for event in score["events"]:
            if not isinstance(event, dict):
                raise PianoError("曲谱事件必须是对象")
            for key in ("start", "duration"):
                number(event.get(key), key, 0 if key == "start" else .001, 14400)
            if len(str(event.get("note", ""))) > 24 or event.get("hand") not in ("left", "right"):
                raise PianoError("无效音符或左右手")
        # Existing planner performs key/finger/overlap/shift/gap checks against the active config.
        return score

    def import_score(self, name, content, format="json", tempo_bpm=60, tonic="C4"):
        if not isinstance(content, str) or len(content.encode()) > 512_000:
            raise PianoError("曲谱文件不得超过 512KB")
        name = str(name).strip()
        if not name or len(name) > 80:
            raise PianoError("曲谱名称需为 1 至 80 个字符")
        if format == "json":
            score = strict_json(content)
        elif format == "csv":
            import csv
            import io
            score = {"tempo_bpm": tempo_bpm, "tonic": tonic, "events": list(csv.DictReader(io.StringIO(content.lstrip("\ufeff"))))}
            for event in score["events"]:
                for key in ("start", "duration"):
                    event[key] = float(event[key])
        else:
            raise PianoError("仅支持 JSON/CSV 时序表")
        if not isinstance(score, dict):
            raise PianoError("曲谱根节点必须为对象")
        score["title"] = name
        self.validate_score(score)
        with self.lock, self.file_lock():
            if len(self.list_scores()) >= 200:
                raise PianoError("曲谱库已满，请先删除曲谱")
            score_id = uuid.uuid4().hex
            self.atomic(self._score_path(score_id), score)
        return {"id": score_id, "name": name}

    def score(self, score_id):
        with self.lock:
            path = self._score_path(score_id)
            if not path.is_file():
                raise PianoError("曲谱不存在")
            score = strict_json(path.read_text())
            self.validate_score(score)
            return score

    def plan(self, score_id, tempo_bpm=None, config=None):
        config = config or self.config()["config"]
        self.validate(config)
        score = self.score(score_id)
        if tempo_bpm is not None:
            score["tempo_bpm"] = number(tempo_bpm, "演奏速度", 20, 300)
        self.root.mkdir(parents=True, exist_ok=True)
        fd, filename = tempfile.mkstemp(suffix=".json", dir=self.root)
        try:
            with os.fdopen(fd, "w") as stream:
                stream.write(canonical(score))
            plan = self.model().plan_score(filename, config)
            press_at = None
            for action in plan["actions"]:
                if action["kind"] == "press":
                    press_at = action["at_s"]
                elif action["kind"] == "release" and press_at is not None:
                    if action["at_s"] - press_at < max(.04, 2 / config["timing"]["control_hz"]) - 1e-9:
                        raise PianoError(f"第 {action['row']} 行按键时值太短，无法取得手指实测确认；请降低速度或延长时值")
            if plan["duration_s"] > 7200:
                raise PianoError("单首演奏不得超过两小时")
            return score, plan
        finally:
            os.unlink(filename)

    def seed(self):
        with self.lock:
            if self.list_scores():
                return
            for name in ("demo.json", "template.csv"):
                path = self.project / "scores" / name
                if path.is_file():
                    self.import_score("示例 · " + path.stem, path.read_text(), path.suffix[1:])
