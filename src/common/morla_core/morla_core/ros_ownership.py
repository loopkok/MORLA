"""Optional ROS bridge for :class:`PolicyLease` (kept out of pure imports)."""
from __future__ import annotations

import json
from typing import Callable


def bind_ownership(lease, node, writer: str, callback: Callable | None = None):
    """Subscribe to policy ownership and ACK only after the writer callback.

    Importing ``morla_core`` never imports rclpy or generated ROS messages. This
    bridge is loaded only when a ROS writer binds its lease.
    """
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    ack_pub = node.create_publisher(String, "/teleop/ownership_ack", 10)

    def receive(msg):
        if callback is None:
            lease.update(msg.data)
        else:
            callback(msg)
        transition = lease.payload.get("transition_id")
        if lease.fresh and transition:
            ack_pub.publish(String(data=json.dumps({
                "writer": writer,
                "transition_id": transition,
            })))

    subscription = node.create_subscription(String, "/policy_inference/state", receive, qos)
    return subscription, ack_pub
