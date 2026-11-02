"""Built-in device transport bindings; isolated from the generic ROS host."""
from .adapters.normalized import InputStream, GamepadAdapter, InputError


def quest_sides(instance):
    """``side`` selects which hands this input instance accepts."""
    side = instance.get('parameters', {}).get('side', 'both')
    if side == 'both':
        return ('left', 'right')
    if side in ('left', 'right'):
        return (side,)
    raise InputError('quest side must be left, right or both')


def subscribe(manager, instance):
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import PoseArray
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    self=manager
    iid=instance['instance_id']; plugin=instance['plugin_id']
    if 'gamepad' in plugin or 'xbox' in plugin:
        source = GamepadAdapter(iid, clock=self.clock)
        source.reconnect()
        self.sources[iid] = source
        topic = instance.get('parameters', {}).get('input_topic', '/joy')
        self.node.create_subscription(
            Joy, topic, lambda msg, iid=iid: self._joy(iid, msg), qos_profile_sensor_data)
    elif plugin == 'morla.input.quest':
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from .quest_events import QuestEventSource
        parameters = instance.get('parameters', {})
        sides = quest_sides(instance)
        # One guarded event stream per hand; same receiver, same epoch.
        template = parameters.get('input_topic', '/quest3/{side}_teleop_events')
        if len(sides) > 1 and '{side}' not in template:
            raise InputError('quest input_topic needs {side} when both hands are selected')
        source = QuestEventSource(iid, clock=self.clock, sides=sides,
                                  pinch_range_m=parameters.get('pinch_range_m', [.015, .08]),
                                  on_transition=lambda iid=iid: self.on_disconnect(iid))
        source.reconnect()
        self.sources[iid] = source
        self.cache[iid] = {}

        # Newest event only: an event older than max_age_ns is rejected by
        # QuestEventSource, so a queued backlog turns one slow tick into a
        # disconnect instead of a skipped frame. Same rule as the Vader input.
        event_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        for side in sides:
            self.node.create_subscription(
                String, template.replace('{side}', side),
                lambda msg, iid=iid: self._quest_event(iid, msg), event_qos)
            # The event stream has no thumbstick; axes stay on the Joy topic.
            self.node.create_subscription(
                Joy, f'/quest3/{side}_controller_joy',
                lambda msg, iid=iid, side=side: self._quest_joy(iid, side, msg),
                qos_profile_sensor_data)
    elif plugin == 'morla.input.glove':
        source = InputStream(iid, clock=self.clock)
        source.reconnect()
        self.sources[iid] = source
        self.cache[iid] = {}
        for side in ('left', 'right'):
            self.node.create_subscription(
                PoseArray, f'/hand_landmarks/{side}',
                lambda msg, iid=iid, side=side: self._landmarks(iid, side, msg),
                qos_profile_sensor_data)
