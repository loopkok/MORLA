"""Quest 3 intent from the guarded teleop event stream, both hands.

The receiver publishes one v1 JSON event stream per side.  A single input
instance selects ``left``, ``right`` or ``both``; there is no per-hand plugin.
Pose, deadman, scalar and landmarks all come from the event stream, which
carries an explicit source (controller or hand), a receiver epoch and a
per-side sequence, so a queued/replayed frame is rejected instead of moving
the arm.  The only remaining ROS subscription for this input is the
controller joystick: the event stream does not carry the thumbstick.
"""
import math
import time
from dataclasses import replace
import numpy as np
from .adapters.normalized import InputStream, InputError, finite

SIDES = ('left', 'right')
EVENT_VERSION = 2


class QuestEventSource(InputStream):
    def __init__(self, source_id, *, clock=time.monotonic_ns, wall_clock=time.time_ns,
                 on_transition=None, sides=SIDES, max_age_ns=250_000_000,
                 pinch_range_m=(.015, .08)):
        super().__init__(source_id, clock=clock, max_age_ns=max_age_ns)
        sides = tuple(sides)
        if not sides or len(set(sides)) != len(sides) or any(side not in SIDES for side in sides):
            raise ValueError('sides must be a non-empty subset of left/right')
        self.sides = sides
        self.wall_clock = wall_clock
        self.on_transition = on_transition or (lambda: None)
        self.upstream_epoch = None
        self.retired_epochs = set()
        self.event_sequence = {side: -1 for side in SIDES}
        self.mode = {side: None for side in SIDES}
        self.cache = {}
        self.pinch = {side: None for side in SIDES}
        self.input_basis_id = None
        self.pinch_range_m = tuple(pinch_range_m)
        if len(self.pinch_range_m) != 2 or not self.pinch_range_m[0] < self.pinch_range_m[1]:
            raise ValueError('pinch_range_m must contain increasing close/open distances')

    def disconnect(self):
        super().disconnect()
        self.cache = {}

    @staticmethod
    def control_event(row, frame):
        # All continuous capabilities reach bindings; BindingEngine processes only
        # fresh samples of each bound port, so a landmark cannot replay a cached pose.
        return frame is not None and (
            row['kind'] in ('pose', 'trigger', 'landmarks')
            or frame.ports.get(f"{row['side']}_hold") is not True)

    def event(self, row):
        required = {'version', 'side', 'kind', 'source',
                    'epoch', 'sequence', 'sent_ns', 'frame_id', 'values', 'basis'}
        if not isinstance(row, dict) or set(row) != required or not finite(row):
            raise InputError('Invalid Quest teleop event')
        if row['version'] != EVENT_VERSION:
            raise InputError('Unsupported Quest event version')
        # The basis is opaque here on purpose: morla_input must not depend on
        # morla_core. Consumers compare it with the basis their binding declares.
        basis = row['basis']
        if not isinstance(basis, str) or not 1 <= len(basis) <= 64:
            raise InputError('Invalid Quest input basis')
        self.input_basis_id = basis
        side = row['side']
        if (side not in SIDES or row['source'] not in ('controller', 'hand')
                or row['kind'] not in ('pose', 'grip', 'trigger', 'landmarks')):
            raise InputError('Unknown Quest event')
        if side not in self.sides:
            return None  # Other hand is not selected by this instance.
        epoch = row['epoch']
        seq = row['sequence']
        if (not isinstance(epoch, str) or not 1 <= len(epoch) <= 128
                or epoch in self.retired_epochs):
            raise InputError('Retired/invalid Quest epoch')
        if type(seq) is not int or seq < 0:
            raise InputError('Invalid Quest sequence')
        source_age = self.wall_clock()-row['sent_ns'] if type(row['sent_ns']) is int else -1
        if not 0 <= source_age <= self.max_age_ns:
            raise InputError('Queued/stale Quest event')
        if not isinstance(row['frame_id'], str) or not 1 <= len(row['frame_id']) <= 128:
            raise InputError('Invalid Quest frame')
        changed_epoch = epoch != self.upstream_epoch
        if not changed_epoch and seq <= self.event_sequence[side]:
            raise InputError('Replayed Quest event')
        if changed_epoch:
            if self.upstream_epoch is not None:
                self.retired_epochs.add(self.upstream_epoch)
            # Bounded history: a restarted input session should use a new source object.
            if len(self.retired_epochs) > 64:
                raise InputError('Quest epoch history exhausted; reconnect input host')
            self.upstream_epoch = epoch
            self.event_sequence = {name: -1 for name in SIDES}
            self.mode = {name: None for name in SIDES}
        self.event_sequence[side] = seq
        # First observation of a hand only adopts its mode; a real switch
        # (controller <-> hand) re-anchors, which also keeps one hand from
        # being re-anchored by the other hand's first frame.
        mode_change = (row['kind'] == 'pose' and self.mode[side] is not None
                       and row['source'] != self.mode[side])
        if changed_epoch or mode_change or not self.connected:
            # Source identity is part of the binding frame identity: switching
            # controller/hand must re-anchor, never bridge two anchors.
            self.on_transition()
            self.reconnect()
            self.cache = {}
            self.pinch = {name: None for name in SIDES}
        if row['kind'] == 'pose':
            self.mode[side] = row['source']
        if row['source'] != self.mode[side]:
            return None  # Old controller buttons cannot arm hand tracking.
        now = self.clock()
        stamp = now-source_age
        kind = row['kind']
        values = row['values']
        if stamp < 0:
            raise InputError('Invalid mapped Quest monotonic sample time')
        if kind == 'pose':
            if (not isinstance(values, list) or len(values) != 7
                    or any(type(v) not in (int, float) for v in values)):
                raise InputError('Quest pose requires xyz/xyzw')
            if not math.isclose(sum(v*v for v in values[3:]), 1, abs_tol=1e-3):
                raise InputError('Quest quaternion must be normalized')
            frame = row['frame_id'] + '/' + side + '/' + self.mode[side]
            self.cache[side+'_pose'] = (
                {'position': values[:3], 'quaternion': values[3:]}, stamp, frame)
        elif kind == 'trigger':
            if self.mode[side] != 'controller' or type(values) not in (int, float) \
                    or not 0 <= values <= 1:
                raise InputError('Invalid controller trigger')
            self._scalar(side, float(values), stamp, row['frame_id'])
        elif kind == 'grip':
            if self.mode[side] != 'controller' or type(values) not in (int, float) \
                    or not 0 <= values <= 1:
                raise InputError('Invalid controller grip')
            self._hold(side, values > .5, stamp, row['frame_id'])
        else:
            if self.mode[side] != 'hand':
                return None
            previous = self.cache.get(side+'_hold')
            if previous is not None and now-previous[1] > self.max_age_ns:
                self.pinch[side] = None
            points = np.asarray(values, dtype=float)
            if points.shape != (21, 3) or not np.isfinite(points).all():
                raise InputError('Invalid hand topology')
            width = float(np.linalg.norm(points[5]-points[17]))
            if width < .005 or width > .20:
                raise InputError('Invalid hand scale; metres required')
            distance = float(np.linalg.norm(points[4]-points[8]))
            ratio = distance/width
            if ratio < .25:
                self.pinch[side] = True
            elif ratio > .45:
                self.pinch[side] = False
            self.cache[side+'_landmarks'] = (points.tolist(), stamp, row['frame_id'])
            closed = ((distance-self.pinch_range_m[0])
                      / (self.pinch_range_m[1]-self.pinch_range_m[0]))
            self._scalar(side, 1-max(0., min(1., closed)), stamp, row['frame_id'])
            if self.pinch[side] is not None:
                self._hold(side, self.pinch[side], stamp, row['frame_id'])
        return self._accept(row['frame_id'], basis)

    def axes(self, side, values, frame_id='input'):
        """Controller thumbstick; the event stream carries no axes."""
        if side not in self.sides or not self.connected:
            return None
        if not isinstance(values, dict) or set(values) != {'x', 'y'} \
                or not finite(values) or any(type(v) not in (int, float) for v in values.values()):
            raise InputError('Invalid Quest axes')
        now = self.clock()
        self.cache[side+'_axes'] = (dict(values), now, frame_id)
        self.cache['axes'] = (dict(values), now, frame_id)
        return self._accept(frame_id)

    def _scalar(self, side, value, stamp, frame_id):
        self.cache[side+'_scalar'] = (value, stamp, frame_id)
        # ``scalar`` mirrors the right hand, or the only selected hand, so a
        # single-gripper binding keeps working without naming a side.
        if side == 'right' or self.sides == ('left',):
            self.cache['scalar'] = (value, stamp, frame_id)

    def _hold(self, side, value, stamp, frame_id):
        self.cache[side+'_hold'] = (value, stamp, frame_id)
        self.cache[side+'_grip'] = (value, stamp, frame_id)

    def _accept(self, frame_id, input_basis_id=None):
        now = self.clock()
        current = {key: value for key, (value, stamp, _) in self.cache.items()
                   if 0 <= now-stamp <= self.max_age_ns}
        metadata = {key: {'received_ns': stamp, 'frame_id': frame}
                    for key, (_, stamp, frame) in self.cache.items() if key in current}
        frame = self.accept(current, self.sequence+1, epoch=self.epoch, frame_id=frame_id,
                            input_basis_id=input_basis_id)
        frame = replace(frame, port_metadata=metadata)
        self.latest = frame
        return frame
