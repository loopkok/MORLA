from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from nero_robot_control.driver_node import NeroRobotDriverNode


def test_flange_pose_is_canonical_and_legacy_tcp_topic_is_same_payload():
    rclpy.init(args=[
        "--ros-args",
        "-p", "initial_position:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
        "-p", "position_pose:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]",
    ])
    driver = None
    probe = None
    try:
        driver = NeroRobotDriverNode()
        probe = Node("nero_flange_pose_topic_test")
        received: dict[str, PoseStamped] = {}
        probe.create_subscription(
            PoseStamped, "/left_arm/flange_pose",
            lambda msg: received.__setitem__("flange", msg), 10,
        )
        probe.create_subscription(
            PoseStamped, "/left_arm/tcp_pose",
            lambda msg: received.__setitem__("legacy", msg), 10,
        )
        q = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])
        sdk_flange = np.array([0.31, -0.22, 0.43, 0.1, -0.2, 0.3])
        driver._publish_feedback(q, sdk_flange)
        for _ in range(30):
            rclpy.spin_once(probe, timeout_sec=0.02)
            if len(received) == 2:
                break

        assert set(received) == {"flange", "legacy"}
        canonical = received["flange"]
        legacy = received["legacy"]
        assert canonical.header.frame_id == "left_base_link"
        assert legacy.header.frame_id == canonical.header.frame_id
        assert driver.flange_frame_id == "left_gripper_flange"
        assert canonical.pose.position.x == legacy.pose.position.x
        assert canonical.pose.position.y == legacy.pose.position.y
        assert canonical.pose.position.z == legacy.pose.position.z
        assert canonical.pose.orientation == legacy.pose.orientation
        assert driver.flange_topic == "/left_arm/flange_pose"
        assert driver.tcp_topic == "/left_arm/tcp_pose"
    finally:
        if probe is not None:
            probe.destroy_node()
        if driver is not None:
            driver.destroy_node()
        rclpy.try_shutdown()
