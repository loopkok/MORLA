"""Built-in device transport bindings; isolated from the generic ROS host."""
from .adapters.normalized import InputStream, GamepadAdapter, InputError


def subscribe(manager, instance):
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import PoseStamped, PoseArray
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    self=manager
    iid=instance['instance_id']; plugin=instance['plugin_id']
    if plugin == 'morla.input.quest_right':
        import json
        from .quest_right import QuestRightSource
        source = QuestRightSource(iid, clock=self.clock,
                                  pinch_range_m=instance.get('parameters',{}).get('pinch_range_m',[.015,.08]),
                                  on_transition=lambda iid=iid: self.on_disconnect(iid))
        self.sources[iid] = source
        topic = instance.get('parameters', {}).get(
            'input_topic', '/quest3/right_teleop_events')

        def event(msg, iid=iid):
            def action():
                if len(msg.data) > 16384:
                    raise InputError('Quest event oversized')
                row = json.loads(msg.data)
                source = self.sources[iid]
                frame = source.event(row)
                if source.control_event(row, frame):
                    self.on_frame(frame)
            self._guarded(iid, action)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        event_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=16)
        self.node.create_subscription(String, topic, event, event_qos)
    elif 'gamepad' in plugin or 'xbox' in plugin:
        source = GamepadAdapter(iid, clock=self.clock)
        source.reconnect()
        self.sources[iid] = source
        topic = instance.get('parameters', {}).get('input_topic', '/joy')
        self.node.create_subscription(
            Joy, topic, lambda msg, iid=iid: self._joy(iid, msg), qos_profile_sensor_data)
    elif plugin in {'morla.input.quest', 'morla.input.glove'}:
        source = InputStream(iid, clock=self.clock)
        source.reconnect()
        self.sources[iid] = source
        self.cache[iid] = {}
        if plugin == 'morla.input.quest':
            self.node.create_subscription(
                String, '/quest3/input_epoch', lambda msg, iid=iid: self._epoch(iid), 10)
            for side in ('left', 'right'):
                self.node.create_subscription(
                    PoseStamped, f'/quest3/{side}_wrist_pose',
                    lambda msg, iid=iid, side=side: self._pose(iid, side, msg),
                    qos_profile_sensor_data)
                self.node.create_subscription(
                    Joy, f'/quest3/{side}_controller_joy',
                    lambda msg, iid=iid, side=side: self._quest_joy(iid, side, msg),
                    qos_profile_sensor_data)
        for side in ('left', 'right'):
            self.node.create_subscription(
                PoseArray, f'/hand_landmarks/{side}',
                lambda msg, iid=iid, side=side: self._landmarks(iid, side, msg),
                qos_profile_sensor_data)
