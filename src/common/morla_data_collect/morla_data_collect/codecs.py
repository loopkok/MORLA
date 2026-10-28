"""Trusted, versioned stream codecs; native protocols stay outside collection logic."""
from dataclasses import dataclass
from typing import Callable
import numpy as np

@dataclass(frozen=True)
class Codec:
    codec_id: str
    version: str
    decode: Callable

class CodecRegistry:
    def __init__(self): self._codecs = {}
    def register(self, codec):
        if codec.codec_id in self._codecs: raise ValueError("duplicate codec ID")
        self._codecs[codec.codec_id] = codec
    def get(self, codec_id):
        if codec_id not in self._codecs: raise ValueError(f"unregistered codec {codec_id!r}")
        return self._codecs[codec_id]
    def manifest(self): return {key: value.version for key, value in sorted(self._codecs.items())}


def checked_array(raw, dtype, shape=None):
    """Reject silent integer truncation, wrapping and float overflow at codec ingress."""
    target = np.dtype(dtype)
    values = np.asarray(raw)
    if values.dtype.kind not in 'biuf': raise ValueError('non-numeric codec input')
    numeric = values.astype(np.float64)
    if not np.isfinite(numeric).all(): raise ValueError('nonfinite codec input')
    if target.kind in 'iu':
        limits = np.iinfo(target)
        if np.any(numeric != np.floor(numeric)) or np.any(values.astype(object) < limits.min) or np.any(values.astype(object) > limits.max):
            raise ValueError('integer codec input is not representable')
    elif target.kind == 'b':
        if np.any((numeric != 0) & (numeric != 1)): raise ValueError('boolean codec input must be binary')
    elif target.kind == 'f':
        if np.any(np.abs(numeric) > np.finfo(target).max): raise ValueError('float codec input overflows storage dtype')
    else: raise ValueError('unsupported storage dtype')
    result = values.astype(target)
    if shape is not None and result.shape != tuple(shape): raise ValueError('codec shape mismatch')
    return result


def decode_numeric(message, spec):
    """Reject shape, name-order and finite-value errors before buffering."""
    kind = spec.message_type
    if kind == "sensor_msgs/msg/JointState":
        field = "velocity" if spec.unit in {"rad/s", "m/s", "ratio/s"} else "effort" if spec.unit in {"Nm", "N.m"} else "position"
        values = checked_array(getattr(message, field), np.float32)
        if spec.joint_names:
            names = list(message.name)
            if len(set(names)) != len(names) or set(names) != set(spec.joint_names):
                raise ValueError("joint name contract mismatch")
            values = values[[names.index(n) for n in spec.joint_names]]
    else:
        values = checked_array(message.data, np.float32).reshape(-1)
    if values.shape != tuple(spec.shape) or not np.isfinite(values).all():
        raise ValueError("numeric codec shape or finite-value mismatch")
    return values


def decode_xhand_native(msg, *, hand_id, finger_order, point_count=120, runtime_mode="unknown"):
    ids = [int(value) for value in msg.hand_id]
    if len(set(ids)) != len(ids) or hand_id not in ids:
        raise ValueError("invalid native hand IDs or sensor count")
    hand_index = ids.index(hand_id)
    if len(msg.sensor_states) != len(ids):
        raise ValueError("invalid native hand IDs or sensor count")
    sensor_state = msg.sensor_states[hand_index]
    by_finger = {}
    for finger_state in sensor_state.finger_sensor_states:
        label = str(finger_state.location).strip().lower()
        if label == "mid":
            label = "middle"
        if label in by_finger:
            raise ValueError("duplicate native finger")
        by_finger[label] = finger_state

    forces = np.zeros((len(finger_order), point_count, 3), dtype=np.float32)
    sums = np.zeros((len(finger_order), 3), dtype=np.float32)
    temperatures = np.zeros((len(finger_order), 20), dtype=np.int32)
    temperature_means = np.zeros(len(finger_order), dtype=np.float32)
    valid = np.zeros(len(finger_order), dtype=np.bool_)
    temperature_valid = np.zeros(len(finger_order), dtype=np.bool_)
    for index, finger in enumerate(finger_order):
        item = by_finger.get(str(finger))
        if item is None:
            continue
        raw = np.asarray([(v.x, v.y, v.z) for v in item.raw_force], dtype=np.float64)
        calc = np.asarray((item.calc_force.x, item.calc_force.y, item.calc_force.z), dtype=np.float64)
        if raw.shape == (point_count, 3) and np.isfinite(raw).all() and np.isfinite(calc).all():
            forces[index] = checked_array(raw, np.float32)
            sums[index] = checked_array(calc, np.float32)
            valid[index] = True
        raw_temp = np.asarray(item.raw_temperature, dtype=np.float64)
        calc_temp = float(item.calc_temperature)
        if (raw_temp.shape == (20,) and np.isfinite(raw_temp).all()
                and np.equal(raw_temp, np.rint(raw_temp)).all()
                and raw_temp.min(initial=0) >= np.iinfo(np.int32).min
                and raw_temp.max(initial=0) <= np.iinfo(np.int32).max
                and np.isfinite(calc_temp)):
            temperatures[index] = raw_temp.astype(np.int32)
            temperature_means[index] = checked_array(calc_temp, np.float32)
            temperature_valid[index] = runtime_mode != "sim"

    sample = {
        "force_raw": forces, "force_sum": sums,
        "temperature_raw": temperatures, "temperature_mean": temperature_means,
        "valid": valid, "temperature_valid": temperature_valid,
    }
    return sample

DEFAULT_CODECS = CodecRegistry()
DEFAULT_CODECS.register(Codec("numeric", "1.0.0", decode_numeric))
DEFAULT_CODECS.register(Codec("xhand_native", "1.0.0", decode_xhand_native))


def sensor_fields(spec):
    """A plugin declares standard fixed-shape tensors, with explicit validity.

    Legacy tactile specs retain their historical storage without teaching the
    writer a vendor message format. Generic specs use ``fields`` mapping names
    to {shape, dtype}; all samples include a validity mask (scalar by default).
    """
    if 'fields' in spec:
        result = {}
        for key, value in spec['fields'].items():
            if not key or '/' in key or key in {'timestamps','arrival_timestamps','source_index','source_offsets','source_timestamps'}:
                raise ValueError('unsafe or reserved sensor field')
            shape = tuple(value.get('shape', ()))
            if any(type(d) is not int or d < 1 for d in shape): raise ValueError('invalid sensor field shape')
            dtype = np.dtype(value.get('dtype','float32'))
            if dtype.kind not in 'biuf': raise ValueError('sensor fields must be numeric or bool')
            result[key] = (shape, dtype)
        result.setdefault('valid', ((), np.dtype('bool')))
        return result
    fingers = len(spec.get('finger_order', ('thumb','index','middle','ring','pinky')))
    points = int(spec.get('point_count_per_finger',120))
    return {name:(shape,np.dtype(dtype)) for name,shape,dtype in (
        ('force_raw',(fingers,points,3),'float32'),('force_sum',(fingers,3),'float32'),
        ('temperature_raw',(fingers,20),'int32'),('temperature_mean',(fingers,),'float32'),
        ('valid',(fingers,),'bool'),('temperature_valid',(fingers,),'bool'))}


def decode_tensor_sensor(message, spec):
    result = {}
    for field, (shape, dtype) in sensor_fields(spec).items():
        raw = getattr(message, field, None)
        if raw is None and field == 'values': raw = getattr(message, 'data', getattr(message, 'position', None))
        if raw is None and field == 'valid': raw = True
        if raw is None: raise ValueError(f'missing sensor field {field}')
        values = checked_array(raw, dtype)
        if values.size != int(np.prod(shape) if shape else 1): raise ValueError('sensor codec shape mismatch')
        values = values.reshape(shape)
        if dtype.kind in 'iuf' and not np.isfinite(values).all(): raise ValueError('nonfinite sensor values')
        result[field] = values
    return result

DEFAULT_CODECS.register(Codec('tensor_sensor', '1.0.0', decode_tensor_sensor))
