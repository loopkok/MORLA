"""Small wire protocol shared by the standalone robot and OpenPI processes."""

import msgpack
import numpy as np


def _encode(value):
    if isinstance(value, np.ndarray):
        if value.dtype.kind in "VOc":
            raise TypeError(f"unsupported ndarray dtype {value.dtype}")
        return {b"__ndarray__": True, b"data": value.tobytes(),
                b"dtype": value.dtype.str, b"shape": value.shape}
    if isinstance(value, np.generic):
        return {b"__npgeneric__": True, b"data": value.item(),
                b"dtype": value.dtype.str}
    raise TypeError(f"cannot serialize {type(value).__name__}")


def _decode(value):
    if b"__ndarray__" in value:
        return np.ndarray(buffer=value[b"data"], dtype=np.dtype(value[b"dtype"]),
                          shape=value[b"shape"]).copy()
    if b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value


def pack(value):
    return msgpack.packb(value, default=_encode, use_bin_type=True)


def unpack(value):
    return msgpack.unpackb(value, object_hook=_decode, raw=False)
