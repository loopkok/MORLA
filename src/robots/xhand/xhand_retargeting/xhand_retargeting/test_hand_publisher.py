#!/usr/bin/env python3
"""Test node that publishes simulated hand landmarks to drive the XHand pipeline."""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from rclpy.node import Node


def make_open_hand() -> np.ndarray:
    """Generate an open-hand pose (21, 3) in MANO coordinate frame.

    MANO convention: wrist at origin, palm normal = +Y, fingers point in -Z,
    thumb points in +X for right hand.
    Scale is matched to XHand URDF dimensions (~0.04m finger segments).
    """
    landmarks = np.zeros((21, 3))

    landmarks[0] = [0.0, 0.0, 0.0]                    # WRIST

    # Thumb (1-4): CMC, MCP, IP, TIP — extends +X, slight -Y
    landmarks[1] = [0.008, -0.002, -0.004]
    landmarks[2] = [0.016, -0.004, -0.006]
    landmarks[3] = [0.024, -0.005, -0.005]
    landmarks[4] = [0.032, -0.006, -0.003]

    # Index (5-8): MCP, PIP, DIP, TIP — extends -Z
    landmarks[5] = [0.004, 0.002, -0.018]
    landmarks[6] = [0.003, 0.002, -0.036]
    landmarks[7] = [0.002, 0.002, -0.048]
    landmarks[8] = [0.001, 0.001, -0.056]

    # Middle (9-12): longest finger
    landmarks[9] = [0.000, 0.001, -0.020]
    landmarks[10] = [-0.001, 0.000, -0.040]
    landmarks[11] = [-0.002, 0.000, -0.052]
    landmarks[12] = [-0.003, 0.000, -0.060]

    # Ring (13-16)
    landmarks[13] = [-0.004, -0.001, -0.017]
    landmarks[14] = [-0.006, -0.001, -0.035]
    landmarks[15] = [-0.007, -0.001, -0.046]
    landmarks[16] = [-0.008, -0.001, -0.054]

    # Pinky (17-20)
    landmarks[17] = [-0.007, -0.002, -0.014]
    landmarks[18] = [-0.010, -0.003, -0.028]
    landmarks[19] = [-0.012, -0.003, -0.036]
    landmarks[20] = [-0.014, -0.003, -0.042]

    return landmarks


def interpolate_pose(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    return a + (b - a) * t


class TestHandPublisher(Node):
    def __init__(self):
        super().__init__("test_hand_publisher")

        self.declare_parameter("hand_side", "right")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("cycle_period", 4.0)  # seconds for open->close->open

        self.hand_side = self.get_parameter("hand_side").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.cycle_period = self.get_parameter("cycle_period").value

        self.publisher = self.create_publisher(PoseArray, "hand_landmarks", 10)

        self.start_time = time.time()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Test hand publisher started — side: {self.hand_side}, "
            f"rate: {self.publish_rate} Hz, cycle: {self.cycle_period}s"
        )

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        # Phase 0-1: open → close → open (triangle wave)
        phase = (elapsed % self.cycle_period) / self.cycle_period

        # Map to 0→1→0 triangle
        if phase < 0.5:
            t = phase * 2.0  # 0→1
        else:
            t = 2.0 - phase * 2.0  # 1→0

        t = t ** 2  # ease-in-out

        # Generate landmarks: interpolate between open and "closed" (fingers curled)
        landmarks = self._generate_pose(t)

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"hand_{self.hand_side}"
        msg.poses = [
            Pose(
                position=Point(x=float(lm[0]), y=float(lm[1]), z=float(lm[2])),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            for lm in landmarks
        ]
        self.publisher.publish(msg)

    def _generate_pose(self, t: float) -> np.ndarray:
        """Generate a (21,3) landmark pose at interpolation factor t.
        t=0: open, t=1: maximum curl.
        """
        base = make_open_hand()
        curl = t

        # Curl each finger by folding PIP/DIP/TIP toward palm (+Z)
        finger_ranges = [
            (5, 8),   # index: MCP=5, TIP=8
            (9, 12),  # middle
            (13, 16), # ring
            (17, 20), # pinky
        ]

        for mcp, tip in finger_ranges:
            mcp_pos = base[mcp].copy()
            for i in range(mcp + 1, tip + 1):
                orig = base[i].copy()
                vec = orig - mcp_pos
                # Rotate the segment: curl = move +Z (toward palm) and -X (wrap in)
                # Simulate bending at each joint
                curl_amount = curl * (i - mcp) / (tip - mcp) * 0.8
                # Shrink Z extension
                new_z = mcp_pos[2] + vec[2] * (1.0 - curl_amount)
                # Pull toward palm surface (+Y)
                new_y = orig[1] + curl_amount * 0.02
                # Slight wrap toward center
                sign = -1 if mcp > 10 else 1
                new_x = orig[0] + curl_amount * 0.005 * sign
                base[i] = [new_x, new_y, new_z]

        # Thumb: folds in X-Z plane
        thumb_base = base[2].copy()
        for i in range(3, 5):
            orig = base[i].copy()
            vec = orig - thumb_base
            curl_amount = curl * (i - 2) / 3 * 0.6
            base[i][0] = thumb_base[0] + vec[0] * (1.0 - curl_amount * 0.3)
            base[i][2] = thumb_base[2] + vec[2] * (1.0 - curl_amount * 0.5)

        return base


def main(args=None):
    rclpy.init(args=args)
    node = TestHandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
