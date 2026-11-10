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
        if plugin_id=='morla.pinch_scalar':
            import numpy as np
            points=np.asarray(value,dtype=float).reshape(21,3)
            if not np.isfinite(points).all() or len(measured)!=1:
                raise ControllerError('pinch requires 21 finite xyz points and one target')
            distance=float(np.linalg.norm(points[4]-points[8]))
            closed,opened=parameters.get('pinch_range_m',[.015,.08])
            if not closed<opened: raise ControllerError('pinch_range_m must be increasing')
            ratio=1-max(0.,min(1.,(distance-closed)/(opened-closed)))
            lower,upper=parameters.get('output_range',[0.,1.])
            return [lower+ratio*(upper-lower)]
        if plugin_id=='morla.identity':
            return _vector(value if isinstance(value,(list,tuple)) else [value],len(measured))
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


class AnchoredScalarController:
    """Scalar mapping that never steps the joint when the deadman engages.

    ``morla.scalar_map`` writes the absolute position the trigger currently
    names, so with a hold deadman every grip press snaps the joint to whatever
    the trigger carried at that instant. On the MMK2 whole-body preset the
    left trigger rests at 0, that map starts at 0.1 m, and the vendor URDF
    slide joint points +z *down* (origin rpy 3.1416 0 1.5708): both arms
    dropped 10 cm the moment grip was pressed, before any hand motion.

    This controller latches the measured position and the trigger value on the
    first frame after engage and applies only the change since then, so
    pressing grip moves nothing and pulling the trigger moves the joint
    relative to where it was left. ``input_range``/``output_range`` keep the
    scalar_map meaning; ``output_range`` is now the travel across the whole
    input range rather than an absolute position.
    """
    synchronous_reanchor = True

    def __init__(self):
        self.origin = None

    def reset(self):
        self.origin = None

    def __call__(self, value, measured, dt, parameters):
        if len(measured) != 1:
            raise ControllerError("anchored scalar requires one target")
        x = _vector([value], 1)[0]
        lo, hi = parameters.get("input_range", [0.0, 1.0])
        if "output_range" not in parameters:
            # Defaulting to [0,1] means one metre of travel. On the MMK2 lift
            # (slide_joint positive = down) that drove the body to the bottom
            # stop as soon as the binding lost its range, so refuse instead.
            raise ControllerError("anchored scalar requires an explicit output_range")
        out = parameters["output_range"]
        if (not isinstance(out, (list, tuple)) or len(out) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in out)
                or float(out[0]) == float(out[1])):
            raise ControllerError("anchored scalar requires two differing finite output_range values")
        out_lo, out_hi = float(out[0]), float(out[1])
        if not lo < hi or not lo <= x <= hi:
            raise ControllerError("scalar outside declared range")
        if self.origin is None:
            origin = float(measured[0])
            if not math.isfinite(origin):
                raise ControllerError("invalid measured scalar target")
            self.origin = (origin, x)
        origin, x0 = self.origin
        return [origin + (x - x0) / (hi - lo) * (out_hi - out_lo)]


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
        self._last_port_sample = None
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

    def reanchor(self, frame, measured):
        """Stage a controller; the caller commits it only after every binding ACKs."""
        if not self.input_fresh(frame):raise ControllerError('reanchor requires fresh bound input')
        measured=_vector(measured,len(self.names))
        port=self.binding['source'].split('.',1)[1]
        plugin=self.registry.plugins.get(self.binding['controller'])
        hook=getattr(plugin,'reanchor',None)
        if hook is not None:
            ack=hook(frame.ports[port],measured,self.binding.get('parameters',{}))
            if not isinstance(ack,dict) or ack.get('acknowledged') is not True:
                raise ControllerError('controller did not acknowledge reanchor')
        else:
            # Non-Cartesian mappings are stateless or resettable. Validate their
            # input/output without dispatching the candidate to any device.
            if plugin is not None and not getattr(plugin,'synchronous_reanchor',False):
                raise ControllerError('controller needs an explicit reanchor acknowledgement hook')
            self.reset()
            result=self.registry.apply(self.binding['controller'],frame.ports[port],measured,0.,self.binding.get('parameters',{}))
            if result is None:raise ControllerError('mapping did not acknowledge initialization')
            _vector(result,len(self.names))
            ack={'acknowledged':True,'kind':'mapping'}
        metadata=(frame.port_metadata or {}).get(port,{})
        source_frame=metadata.get('frame_id',frame.frame_id)
        self.identity=(frame.source_id,frame.source_epoch,source_frame)
        self.last_sequence=frame.sequence;self.last_received_ns=frame.received_ns;self.last_ns=self.clock()
        self._last_port_sample=(frame.source_epoch,metadata.get('received_ns'),source_frame,
                                metadata.get('sample_sequence'))
        return ack

    def step(self, frame, measured, now=None):
        # ``now`` is the frame's arrival-evaluation time from the caller. Left
        # out, the clock is read here; either way staleness means "this sample
        # was already old when the frame arrived", not "our own pipeline spent
        # a while on it" (a first-frame controller build can take hundreds of
        # ms and would otherwise discard that frame for every later binding).
        now = self.clock() if now is None else now
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
