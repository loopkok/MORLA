"""
ROS input adapters; the device drivers are independent of robot models.

Quest sources subscribe to the guarded morla_input outputs, never raw network
packets. The gamepad source expects joy/game_controller_node's SDL layout.
"""
from __future__ import annotations
import threading
import time
from dataclasses import replace
from .adapters.normalized import InputStream, GamepadAdapter, InputError


class RosInputManager:
    def __init__(self, instances, on_frame, on_disconnect, *,
                 clock=time.monotonic_ns, on_event=None):
        self.instances = instances
        self.on_frame = on_frame
        self.on_disconnect = on_disconnect
        self.on_event = on_event or (lambda iid, frame, actions: None)
        self.clock = clock
        self.sources = {}
        self.cache = {}
        self.node = None
        self.thread = None
        self.executor = None
        self.supervisor = None

    def start(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Joy
        from geometry_msgs.msg import PoseStamped, PoseArray
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('morla_composed_inputs_'+str(id(self)))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        for instance in self.instances:
            iid = instance['instance_id']
            plugin = instance['plugin_id']
            from importlib import metadata
            factories = [e for e in metadata.entry_points(
                group='morla.ros_inputs') if e.name == plugin]
            if len(factories) > 1:
                raise InputError('Ambiguous ROS input source '+plugin)
            if factories:
                bridge = factories[0].load()(instance, clock=self.clock,
                                             on_transition=lambda iid=iid: self.on_disconnect(iid))
                self.sources[iid] = bridge.source

                def external(msg, iid=iid, bridge=bridge):
                    def action():
                        frame = bridge.consume(msg.data)
                        if frame is not None:
                            self.on_event(iid, frame, bridge.actions)
                            if bridge.control_event(frame):
                                self.on_frame(frame)
                    self._guarded(iid, action)
                from rclpy.qos import QoSProfile, ReliabilityPolicy
                self.node.create_subscription(
                    String, bridge.topic, external,
                    QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE))
            else:
                from .builtin_sources import subscribe
                subscribe(self, instance)
        from morla_runtime.supervisor import ProcessSupervisor
        self.supervisor = ProcessSupervisor()
        self.supervisor.start(self.instances)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _guarded(self, iid, fn):
        try:
            fn()
        except Exception as exc:
            self.on_disconnect(iid)
            self.node.get_logger().error(f'input {iid} rejected: {exc}')

    def _joy(self, iid, message):
        def action():
            source = self.sources[iid]
            if source.latest is not None and source.read() is None:
                source.reconnect()
            self.on_frame(source.joy(list(message.axes), list(message.buttons)))
        self._guarded(iid, action)

    def _epoch(self, iid):
        self.sources[iid].reconnect()
        self.cache[iid] = {}
        self.on_disconnect(iid)

    def _emit(self, iid, values, frame_id):
        source = self.sources[iid]
        now = self.clock()
        # Keep separate per-port ages; a fresh button cannot refresh an old pose.
        for key, value in values.items():
            self.cache[iid][key] = (value, now, frame_id)
        current = {key: value for key, (value, stamp, _)
                   in self.cache[iid].items() if now-stamp <= 250_000_000}
        frame = source.accept(current, source.sequence+1, epoch=source.epoch, frame_id=frame_id)
        metadata = {key: {'received_ns': stamp, 'frame_id': fid}
                    for key, (_, stamp, fid) in self.cache[iid].items() if key in current}
        frame = replace(frame, port_metadata=metadata)
        source.latest = frame
        self.on_frame(frame)

    def _pose(self, iid, side, msg):
        def action():
            p, q = msg.pose.position, msg.pose.orientation
            self._emit(iid, {
                side + '_pose': {
                    'position': [p.x, p.y, p.z],
                    'quaternion': [q.x, q.y, q.z, q.w]}},
                msg.header.frame_id)
        self._guarded(iid, action)

    def _quest_joy(self, iid, side, msg):
        def action():
            if len(msg.axes) < 4:
                raise InputError('Quest Joy shape mismatch')
            values = {side+'_grip': msg.axes[1] > .5, 'scalar': float(msg.axes[0]),
                      side+'_scalar': float(msg.axes[0]),
                      side+'_axes': {'x':float(msg.axes[2]),'y':float(msg.axes[3])},
                      'axes': {'x': float(msg.axes[2]), 'y': float(msg.axes[3])}}
            self._emit(iid, values, msg.header.frame_id or 'input')
        self._guarded(iid, action)

    def _landmarks(self, iid, side, msg):
        def action():
            if len(msg.poses) != 21:
                raise InputError('landmark topology must contain 21 points')
            points = [[p.position.x, p.position.y, p.position.z] for p in msg.poses]
            distance=sum((points[4][i]-points[8][i])**2 for i in range(3))**.5
            scalar=1-max(0.,min(1.,(distance-.015)/(.08-.015)))
            self._emit(iid, {side+'_landmarks': points,side+'_scalar':scalar}, msg.header.frame_id)
        self._guarded(iid, action)

    def close(self):
        if self.supervisor is not None: self.supervisor.close()
        if self.node is None:
            return
        if self.executor is not None: self.executor.shutdown(timeout_sec=1)
        if self.thread is not None: self.thread.join(timeout=1)
        self.node.destroy_node()
        self.node = None
