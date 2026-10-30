"""Stateful input mappings independent of robot brands and ROS transports."""
from __future__ import annotations
import math
import time


class ControllerError(ValueError):
    pass


def _vector(values, dim):
    if len(values) != dim or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ControllerError("invalid named vector")
    return list(map(float, values))


class ControllerRegistry:
    def __init__(self):
        self.plugins = {}

    def register(self, plugin_id, fn):
        if plugin_id in self.plugins:
            raise ControllerError("duplicate controller")
        self.plugins[plugin_id] = fn

    def apply(self, plugin_id, value, measured, dt, parameters):
        if plugin_id in self.plugins:
            return self.plugins[plugin_id](value, measured, dt, parameters)
        if plugin_id in {"morla.scalar_map", "scalar_map"}:
            if len(measured) != 1:
                raise ControllerError("scalar mapping requires scalar target")
            x = _vector([value], 1)[0]
            lo, hi = parameters.get("input_range", [0.0, 1.0])
            out_lo, out_hi = parameters.get("output_range", [0.0, 1.0])
            if not lo < hi or not lo <= x <= hi:
                raise ControllerError("scalar outside declared range")
            target=out_lo + (x-lo)/(hi-lo)*(out_hi-out_lo)
            return [target]
        if plugin_id in {"morla.joint_jog", "joint_jog"}:
            axes = parameters.get("axes_map")
            if not isinstance(value, dict) or not isinstance(axes, list) or len(axes) != len(measured):
                raise ControllerError("explicit per-joint axes_map required")
            speed = parameters.get("speed", 0.25)
            if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not speed > 0:
                raise ControllerError("invalid jog speed")
            if any(axis is not None and axis not in value for axis in axes):
                raise ControllerError('axes_map references an unavailable axis')
            delta = _vector([value[axis] if axis is not None else 0.0 for axis in axes], len(measured))
            if any(abs(v) > 1 for v in delta):
                raise ControllerError("jog axes out of range")
            return [q + x*speed*dt for q, x in zip(measured, delta)]
        raise ControllerError(f"controller {plugin_id!r} requires an installed mapping/IK plugin")


class BindingController:
    """Applies one bound input source when its frame is fresh and in sequence."""
    def __init__(self, binding, names, *, registry=None, max_age_ns=250_000_000, clock=time.monotonic_ns):
        self.binding, self.names = binding, tuple(names)
        self.registry = registry or ControllerRegistry()
        self.max_age_ns, self.clock = max_age_ns, clock
        self.identity = None
        self.last_ns = None
        self.last_sequence = -1
        self.last_received_ns = None

    def reset(self):
        self.identity = None
        self.last_ns = None
        self.last_sequence = -1
        self.last_received_ns = None
        for plugin in self.registry.plugins.values():
            if hasattr(plugin,'reset'): plugin.reset()

    def close(self):
        self.reset()
        for plugin in self.registry.plugins.values():
            if hasattr(plugin,'close'): plugin.close()

    def input_fresh(self, frame):
        now=self.clock()
        if (frame is None or not frame.valid or
                frame.source_id!=self.binding['source'].split('.',1)[0] or
                not 0<=now-frame.received_ns<=self.max_age_ns):return False
        metadata=frame.port_metadata or {}
        port=self.binding['source'].split('.',1)[1]
        if port not in frame.ports:return False
        stamp=metadata.get(port,{}).get('received_ns',frame.received_ns)
        return type(stamp) is int and 0<=now-stamp<=self.max_age_ns

    @property
    def requires_fresh_input(self):
        return any(getattr(plugin,'requires_fresh_input',False) for plugin in self.registry.plugins.values())

    def step(self, frame, measured):
        now = self.clock()
        if frame is None or not frame.valid or not 0 <= now - frame.received_ns <= self.max_age_ns:
            self.reset()
            return None
        if frame.source_id!=self.binding['source'].split('.',1)[0]:
            self.reset()
            raise ControllerError('input frame does not match bound source instance')
        if self.last_received_ns is not None and frame.received_ns-self.last_received_ns>self.max_age_ns:
            self.reset()
        source_port=self.binding['source'].split('.',1)[1]
        metadata=frame.port_metadata or {}
        stamp=metadata.get(source_port,{}).get('received_ns',frame.received_ns)
        if not 0<=now-stamp<=self.max_age_ns:
            self.reset(); return None
        source_frame=metadata.get(source_port,{}).get('frame_id',frame.frame_id)
        identity = (frame.source_id, frame.source_epoch, source_frame)
        if identity != self.identity:
            self.reset()
            self.identity = identity
        if frame.sequence <= self.last_sequence:
            raise ControllerError("input sequence not increasing")
        self.last_sequence = frame.sequence
        self.last_received_ns = frame.received_ns
        measured = _vector(measured, len(self.names))
        source = self.binding["source"].split(".", 1)[-1]
        if source not in frame.ports:
            raise ControllerError("missing bound input port")
        dt = min(0.05, max(0.0, (now-self.last_ns)/1e9)) if self.last_ns is not None else 0.0
        self.last_ns = now
        return self.registry.apply(self.binding["controller"], frame.ports[source], measured, dt,
                                   self.binding.get("parameters", {}))
