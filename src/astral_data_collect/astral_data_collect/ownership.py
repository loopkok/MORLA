"""Shared, ROS-free policy ownership lease used by all teleop writers."""
from dataclasses import dataclass, field
import json
import math
import time


def fresh_header(msg, now, timeout=0.5):
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    # Unstamped CLI/simulator input remains supported; network input must pass
    # PacketGuard first and always carries a nonzero source timestamp.
    return stamp == 0 or -0.05 <= now - stamp <= timeout


@dataclass
class PolicyLease:
    timeout: float = 2.0
    payload: dict = field(default_factory=dict)
    received: float = 0.0
    seen: bool = False
    invalid: bool = False

    def bind(self, node, writer: str, callback=None):
        """Acknowledge ownership only after the writer's serial callback gates IO."""
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(String, '/teleop/ownership_ack', 10)
        def receive(msg):
            if callback is None:
                self.update(msg.data)
            else:
                callback(msg)
            transition = self.payload.get('transition_id')
            if self.fresh and transition:
                pub.publish(String(data=json.dumps({'writer': writer, 'transition_id': transition})))
        node.create_subscription(String, '/policy_inference/state', receive, qos)

    def update(self, text: str) -> bool:
        self.seen = True
        try:
            value = json.loads(text)
            if not isinstance(value, dict) or value.get('activity') not in (None, 'human', 'policy', 'playback'):
                raise ValueError('invalid activity')
            sent = float(value.get('sent_at', time.time()))
            if not math.isfinite(sent) or abs(time.time() - sent) > self.timeout:
                raise ValueError('expired state')
            if (value.get('epoch') == self.payload.get('epoch') and 'sequence' in value
                    and int(value['sequence']) <= int(self.payload.get('sequence', -1))):
                return False
            self.payload = value
            self.received = time.monotonic()
            self.invalid = False
            return True
        except (ValueError, TypeError, OverflowError):
            self.invalid = True
            return False

    @property
    def fresh(self) -> bool:
        return self.seen and not self.invalid and time.monotonic() - self.received <= self.timeout

    @property
    def manual_allowed(self) -> bool:
        if not self.seen:
            return True
        # A confirmed IDLE releases ownership even after the policy process
        # exits. Active/HUMAN ownership must keep renewing its heartbeat.
        if (not self.invalid and self.payload.get('activity') is None
                and not self.payload.get('takeover_pending')
                and not self.payload.get('policy_start_pending')):
            return True
        return (self.fresh and self.payload.get('activity') in (None, 'human')
                and not self.payload.get('takeover_pending')
                and not self.payload.get('policy_start_pending'))

    @property
    def activity(self):
        return self.payload.get('activity') if self.fresh or self.manual_allowed else 'unknown'
