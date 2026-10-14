"""HDF5 写入器：原始采集（raw）的落盘层。

每个 episode 一个目录：
  robot_data.h5   /streams/{name}/values (N, D) float32 + timestamps (N,) float64
  camera_data.h5  /cam_{i}/images (N,) vlen uint8(JPEG) + timestamps (N,) float64

写入模型：chunk 预分配 + 末尾裁剪（与 nero_dual_data_collect 同款，实测稳定）。
所有写调用来自写盘线程；采集线程只往 deque 里放，不在此层竞争。
"""

from __future__ import annotations

import os

import h5py
import numpy as np

CHUNK_SIZE = 1000  # HDF5 预分配粒度（行）

ROBOT_H5 = "robot_data.h5"
CAMERA_H5 = "camera_data.h5"
META_JSON = "meta.json"


class StreamDataWriter:
    """Numeric streams and fixed-shape sensor arrays in one raw HDF5 file."""

    def __init__(self, h5_path: str, streams: dict[str, int], sensor_specs=()) -> None:
        self.h5_path = h5_path
        self._streams = dict(streams)
        self._sensor_specs = {str(s["sensor_key"]): dict(s) for s in sensor_specs}
        self._file: h5py.File | None = None
        self._values: dict[str, h5py.Dataset] = {}
        self._ts: dict[str, h5py.Dataset] = {}
        self._sizes: dict[str, int] = {}
        self._idx: dict[str, int] = {}
        self._sensors: dict[str, dict[str, h5py.Dataset]] = {}
        self._sensor_idx: dict[str, int] = {}
        self._sensor_valid_counts: dict[str, np.ndarray] = {}
        self._sensor_temperature_valid_counts: dict[str, np.ndarray] = {}

    def open(self) -> "StreamDataWriter":
        os.makedirs(os.path.dirname(self.h5_path), exist_ok=True)
        self._file = h5py.File(self.h5_path, "w")
        grp = self._file.create_group("streams")
        for name, dim in self._streams.items():
            g = grp.create_group(name)
            self._values[name] = g.create_dataset(
                "values", shape=(0, dim), maxshape=(None, dim), dtype="f4",
                chunks=(min(CHUNK_SIZE, 256), dim),
            )
            self._ts[name] = g.create_dataset(
                "timestamps", shape=(0,), maxshape=(None,), dtype="f8",
                chunks=(min(CHUNK_SIZE, 1024),),
            )
            self._sizes[name] = 0
            self._idx[name] = 0
        if self._sensor_specs:
            root = self._file.create_group("sensors")
            for key, spec in self._sensor_specs.items():
                fingers = len(spec.get("finger_order", ("thumb", "index", "middle", "ring", "pinky")))
                points = int(spec.get("point_count_per_finger", 120))
                group = root.create_group(key)
                group.attrs["finger_order"] = __import__("json").dumps(
                    list(spec.get("finger_order", ("thumb", "index", "middle", "ring", "pinky")))
                )
                shapes = {
                    "force_raw": (fingers, points, 3),
                    "force_sum": (fingers, 3),
                    "temperature_raw": (fingers, 20),
                    "temperature_mean": (fingers,),
                    "valid": (fingers,),
                    "temperature_valid": (fingers,),
                }
                datasets = {}
                for field, shape in shapes.items():
                    dtype = "bool" if field.endswith("valid") else ("i4" if field == "temperature_raw" else "f4")
                    datasets[field] = group.create_dataset(
                        field, shape=(0,) + shape, maxshape=(None,) + shape,
                        dtype=dtype, chunks=(1,) + shape,
                    )
                for field in ("timestamps", "arrival_timestamps"):
                    datasets[field] = group.create_dataset(
                        field, shape=(0,), maxshape=(None,), dtype="f8", chunks=(1024,)
                    )
                self._sensors[key] = datasets
                self._sensor_idx[key] = 0
                self._sensor_valid_counts[key] = np.zeros(fingers, dtype=np.int64)
                self._sensor_temperature_valid_counts[key] = np.zeros(fingers, dtype=np.int64)
        return self

    def write(self, name: str, values: np.ndarray, timestamp: float) -> None:
        if self._file is None or name not in self._values:
            return
        i = self._idx[name]
        if i >= self._sizes[name]:
            self._sizes[name] += CHUNK_SIZE
            dim = self._values[name].shape[1]
            self._values[name].resize((self._sizes[name], dim))
            self._ts[name].resize((self._sizes[name],))
        self._values[name][i] = np.asarray(values, dtype=np.float32).reshape(-1)
        self._ts[name][i] = timestamp
        self._idx[name] = i + 1

    def count(self, name: str) -> int:
        if name.startswith("sensor:"):
            return self._sensor_idx.get(name.removeprefix("sensor:"), 0)
        return self._idx.get(name, 0)

    def counts(self) -> dict[str, int]:
        return {**self._idx, **{f"sensor:{key}": value for key, value in self._sensor_idx.items()}}

    def write_sensor(self, key: str, sample: dict, timestamp: float, arrival_timestamp: float) -> None:
        """Append one decoded tactile sample; invalid fingers stay explicitly invalid."""
        if self._file is None or key not in self._sensors:
            return
        i = self._sensor_idx[key]
        fields = self._sensors[key]
        for name, value in sample.items():
            if name not in fields or name in {"timestamps", "arrival_timestamps"}:
                continue
            dataset = fields[name]
            dataset.resize((i + 1,) + dataset.shape[1:])
            dataset[i] = np.asarray(value, dtype=dataset.dtype)
        for name, value in (("timestamps", timestamp), ("arrival_timestamps", arrival_timestamp)):
            dataset = fields[name]
            dataset.resize((i + 1,))
            dataset[i] = value
        self._sensor_idx[key] = i + 1
        if "valid" in sample:
            valid = np.asarray(sample["valid"], dtype=bool)
            if valid.shape == self._sensor_valid_counts[key].shape:
                self._sensor_valid_counts[key] += valid.astype(np.int64)
        if "temperature_valid" in sample:
            valid = np.asarray(sample["temperature_valid"], dtype=bool)
            if valid.shape == self._sensor_temperature_valid_counts[key].shape:
                self._sensor_temperature_valid_counts[key] += valid.astype(np.int64)

    def sensor_valid_counts(self) -> dict[str, list[int]]:
        return {key: values.astype(int).tolist() for key, values in self._sensor_valid_counts.items()}

    def sensor_temperature_valid_counts(self) -> dict[str, list[int]]:
        return {key: values.astype(int).tolist()
                for key, values in self._sensor_temperature_valid_counts.items()}

    def close(self) -> dict[str, int]:
        if self._file is None:
            return {}
        for name in self._streams:
            n = self._idx[name]
            dim = self._values[name].shape[1]
            self._values[name].resize((n, dim))
            self._ts[name].resize((n,))
        for key, fields in self._sensors.items():
            n = self._sensor_idx[key]
            for dataset in fields.values():
                dataset.resize((n,) + dataset.shape[1:])
        self._file.close()
        self._file = None
        return self.counts()


class CameraDataWriter:
    """图像流写入：/cam_{i}/images(vlen JPEG bytes) + timestamps。"""

    def __init__(self, h5_path: str, cam_ids: list[str]) -> None:
        self.h5_path = h5_path
        self.cam_ids = list(cam_ids)
        self._file: h5py.File | None = None
        self._images: dict[str, h5py.Dataset] = {}
        self._ts: dict[str, h5py.Dataset] = {}
        self._arrival: dict[str, h5py.Dataset] = {}
        self._sizes: dict[str, int] = {}
        self._idx: dict[str, int] = {}

    def open(self) -> "CameraDataWriter":
        os.makedirs(os.path.dirname(self.h5_path), exist_ok=True)
        self._file = h5py.File(self.h5_path, "w")
        vlen_u8 = h5py.special_dtype(vlen=np.dtype("uint8"))
        for cam in self.cam_ids:
            g = self._file.create_group(cam)
            self._images[cam] = g.create_dataset(
                "images", shape=(0,), maxshape=(None,), dtype=vlen_u8,
                chunks=(64,),
            )
            self._ts[cam] = g.create_dataset(
                "timestamps", shape=(0,), maxshape=(None,), dtype="f8",
                chunks=(min(CHUNK_SIZE, 1024),),
            )
            self._arrival[cam] = g.create_dataset("arrival_timestamps", shape=(0,), maxshape=(None,),
                                                   dtype="f8", chunks=(min(CHUNK_SIZE, 1024),))
            self._sizes[cam] = 0
            self._idx[cam] = 0
        return self

    def write_image(self, cam: str, jpeg_bytes: bytes, timestamp: float, arrival_timestamp: float | None = None) -> None:
        if self._file is None or cam not in self._images:
            return
        i = self._idx[cam]
        if i >= self._sizes[cam]:
            self._sizes[cam] += 256
            self._images[cam].resize((self._sizes[cam],))
            self._ts[cam].resize((self._sizes[cam],))
            self._arrival[cam].resize((self._sizes[cam],))
        self._images[cam][i] = np.frombuffer(jpeg_bytes, dtype="uint8")
        self._ts[cam][i] = timestamp
        self._arrival[cam][i] = timestamp if arrival_timestamp is None else arrival_timestamp
        self._idx[cam] = i + 1

    def count(self, cam: str) -> int:
        return self._idx.get(cam, 0)

    def counts(self) -> dict[str, int]:
        return dict(self._idx)

    def close(self) -> dict[str, int]:
        if self._file is None:
            return {}
        for cam in self.cam_ids:
            n = self._idx[cam]
            self._images[cam].resize((n,))
            self._ts[cam].resize((n,))
            self._arrival[cam].resize((n,))
        self._file.close()
        self._file = None
        return dict(self._idx)
