#!/usr/bin/env python3
"""Test node that directly publishes XHandCommand with cycling joint angles."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from xhand_control_interfaces.msg import XHandCommand


XHAND_JOINT_NAMES = [
    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
    "index_bend_joint", "index_joint1", "index_joint2",
    "mid_joint1", "mid_joint2",
    "ring_joint1", "ring_joint2",
    "pinky_joint1", "pinky_joint2",
]

# Home position (open, from XHandConfig)
HOME_RAD = [
    math.radians(d) for d in (
        0.0, 80.66, 33.2,    # thumb
        0.0, 5.11, 5.0,       # index
        6.53, 5.0,            # mid
        6.76, 5.0,            # ring
        10.13, 5.0,           # pinky
    )
]

# Closed/fist position (much more curled)
CLOSE_RAD = [
    math.radians(d) for d in (
        60.0, 60.0, 80.0,       # thumb: bend in, rota more
        10.0, 90.0, 90.0,       # index: bend side + curl
        90.0, 90.0,             # mid: curl
        90.0, 90.0,             # ring: curl
        90.0, 80.0,             # pinky: curl
    )
]


class TestDirectControl(Node):
    def __init__(self):
        super().__init__("test_direct_control")

        self.declare_parameter("hand_side", "right")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("cycle_period", 4.0)

        self.hand_side = self.get_parameter("hand_side").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.cycle_period = self.get_parameter("cycle_period").value

        topic = f"/{self.hand_side}_hand/xhand_command"
        self.cmd_pub = self.create_publisher(XHandCommand, topic, 10)
        self.js_pub = self.create_publisher(JointState, f"/{self.hand_side}_hand/joint_states", 10)

        self.start_time = time.time()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Direct control test — side: {self.hand_side}, "
            f"cycle: {self.cycle_period}s, topic: {topic}"
        )

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        phase = (elapsed % self.cycle_period) / self.cycle_period

        # Triangle wave: 0→1→0 (open→close→open)
        if phase < 0.5:
            t = phase * 2.0
        else:
            t = 2.0 - phase * 2.0

        # Ease-in-out cubic
        t = t * t * (3.0 - 2.0 * t)

        positions = [home + (close - home) * t for home, close in zip(HOME_RAD, CLOSE_RAD)]

        msg = XHandCommand()
        msg.hand_id = 0
        msg.name = XHAND_JOINT_NAMES
        msg.position = [float(p) for p in positions]
        msg.kp = [80.0] * 12
        msg.ki = [0.0] * 12
        msg.kd = [0.0] * 12
        msg.effort_limit = [400.0] * 12
        msg.mode = 3
        self.cmd_pub.publish(msg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = XHAND_JOINT_NAMES
        js.position = [float(p) for p in positions]
        self.js_pub.publish(js)

        if int(elapsed * 2) % 2 == 0 and int((elapsed + 0.03) * 2) % 2 != 0:
            self.get_logger().info(
                f"[{phase:.1f}] t={t:.2f} thumb_bend={math.degrees(positions[0]):.1f}° "
                f"index_bend={math.degrees(positions[3]):.1f}°"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestDirectControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
