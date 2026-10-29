"""Source-aware right-side Quest intent, independent of robot brands and ROS."""
import math
import time
from dataclasses import replace
import numpy as np
from .adapters.normalized import InputStream, InputError, finite


class QuestRightSource(InputStream):
    def __init__(self, source_id, *, clock=time.monotonic_ns, wall_clock=time.time_ns,
                 on_transition=None, max_age_ns=250_000_000):
        super().__init__(source_id, clock=clock, max_age_ns=max_age_ns)
        self.wall_clock = wall_clock
        self.on_transition = on_transition or (lambda: None)
        self.upstream_epoch = None
        self.retired_epochs = set()
        self.event_sequence = -1
        self.mode = None
        self.cache = {}
        self.pinch = None

    def disconnect(self):
        super().disconnect()
        self.cache = {}

    @staticmethod
    def control_event(row, frame):
        # Grip/landmarks update cached clutch state; do not overwrite a pose target
        # with a tiny-dt command from the next line in the same device packet.
        return frame is not None and (
            row['kind'] == 'pose' or frame.ports.get('right_hold') is not True)

    def event(self, row):
        required = {'version', 'side', 'kind', 'source',
                    'epoch', 'sequence', 'sent_ns', 'frame_id', 'values'}
        if not isinstance(row, dict) or set(row) != required or not finite(row):
            raise InputError('Invalid Quest teleop event')
        if row['version'] != 1 or row['side'] != 'right':
            raise InputError('Right-side v1 event required')
        if (row['source'] not in ('controller', 'hand')
                or row['kind'] not in ('pose', 'grip', 'landmarks')):
            raise InputError('Unknown Quest event')
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
        if not changed_epoch and seq <= self.event_sequence:
            raise InputError('Replayed Quest event')
        if changed_epoch:
            if self.upstream_epoch is not None:
                self.retired_epochs.add(self.upstream_epoch)
            # Bounded history: a restarted input session should use a new source object.
            if len(self.retired_epochs) > 64:
                raise InputError('Quest epoch history exhausted; reconnect input host')
            self.upstream_epoch = epoch
            self.event_sequence = -1
        self.event_sequence = seq
        mode_change = row['kind'] == 'pose' and row['source'] != self.mode
        if changed_epoch or mode_change or not self.connected:
            self.on_transition()
            self.reconnect()
            self.cache = {}
            self.pinch = None
            if changed_epoch:
                self.mode = None
        if row['kind'] == 'pose':
            self.mode = row['source']
        if row['source'] != self.mode:
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
            # Source identity is part of the binding frame identity; never bridges two anchors.
            frame = row['frame_id'] + '/' + self.mode
            self.cache['right_pose'] = (
                {'position': values[:3], 'quaternion': values[3:]}, stamp, frame)
        elif kind == 'grip':
            if (self.mode != 'controller' or type(values) not in (int, float)
                    or not 0 <= values <= 1):
                raise InputError('Invalid controller grip')
            self.cache['right_hold'] = (values > .5, stamp, row['frame_id'])
        else:
            if self.mode != 'hand':
                return None
            previous = self.cache.get('right_hold')
            if previous is not None and now-previous[1] > self.max_age_ns:
                self.pinch = None
            points = np.asarray(values, dtype=float)
            if points.shape != (21, 3) or not np.isfinite(points).all():
                raise InputError('Invalid hand topology')
            width = float(np.linalg.norm(points[5]-points[17]))
            if width < .005 or width > .20:
                raise InputError('Invalid hand scale; metres required')
            ratio = float(np.linalg.norm(points[4]-points[8]))/width
            if ratio < .25:
                self.pinch = True
            elif ratio > .45:
                self.pinch = False
            if self.pinch is not None:
                self.cache['right_hold'] = (self.pinch, stamp, row['frame_id'])
        current = {k: v for k, (v, stamp, _) in self.cache.items()
                   if 0 <= now-stamp <= self.max_age_ns}
        metadata = {k: {'received_ns': stamp, 'frame_id': frame}
                    for k, (_, stamp, frame) in self.cache.items() if k in current}
        frame = self.accept(current, self.sequence+1, epoch=self.epoch, frame_id=row['frame_id'])
        frame = replace(frame, port_metadata=metadata)
        self.latest = frame
        return frame
