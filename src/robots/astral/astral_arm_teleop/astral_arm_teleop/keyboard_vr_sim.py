#!/usr/bin/env python3
"""Keyboard-based VR hand pose simulator for Astral arm teleop testing.

Publishes simulated Quest3 wrist PoseStamped messages so you can verify
VR-to-arm coordinate mapping and IK correctness without a real Quest3.

Coordinate system (robot_world, convert_to_robot:=true):
  X left, Y back, Z up — same as Quest3 after axis remap.

Keys (left hand unless shifted):
  W/S    — hand forward / backward   (-Y / +Y)
  A/D    — hand left / right          (+X / -X)
  Q/E    — hand down / up             (-Z / +Z)
  I/K    — pitch (rotate around VR forward axis)
  J/L    — yaw   (rotate around VR vertical axis)
  U/O    — roll  (rotate around VR outward axis)
  R      — reset pose to zero
  ESC    — quit

Hold SHIFT for right hand control.
"""

import os
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.node import Node
from scipy.spatial.transform import Rotation


STEP_POS = 0.02       # meters per key press
STEP_ROT = 0.05       # radians per key press (~3 deg)


class KeyboardVRSim(Node):
    def __init__(self):
        super().__init__("keyboard_vr_sim")

        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("arm_side", "both")  # "left", "right", or "both"
        rate = self.get_parameter("publish_rate").value
        self.arm_side = self.get_parameter("arm_side").value

        # State for each hand
        self.pos = {"left": np.zeros(3), "right": np.zeros(3)}
        self.rot = {
            "left": Rotation.identity(),
            "right": Rotation.identity(),
        }

        # Publishers
        if self.arm_side in ("left", "both"):
            self.pub_left = self.create_publisher(
                PoseStamped, "quest3/left_wrist_pose", 10
            )
            self.get_logger().info("Publishing: quest3/left_wrist_pose")
        if self.arm_side in ("right", "both"):
            self.pub_right = self.create_publisher(
                PoseStamped, "quest3/right_wrist_pose", 10
            )
            self.get_logger().info("Publishing: quest3/right_wrist_pose")

        # Thread for keyboard input
        self._running = True
        self._keys_pressed = set()
        self._shift = False
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        # Timer for publishing
        self.dt = 1.0 / rate
        self._timer = self.create_timer(self.dt, self._publish)

        self.get_logger().info("Keyboard VR Sim ready.")
        self._print_help()

    def _print_help(self):
        msg = (
            "\n========================================\n"
            "  Keyboard VR Simulator\n"
            "========================================\n"
            "  Hand position (robot_world):\n"
            "    W/S : forward / backward (-Y/+Y)\n"
            "    A/D : left / right       (+X/-X)\n"
            "    Q/E : down / up          (-Z/+Z)\n"
            "  Hand rotation:\n"
            "    I/K : pitch\n"
            "    J/L : yaw\n"
            "    U/O : roll\n"
            "  Other:\n"
            "    R   : reset pose\n"
            "    ESC : quit\n"
            "  Hold SHIFT for right hand.\n"
            "========================================\n"
        )
        self.get_logger().info(msg)

    # ------------------------------------------------------------------
    def _keyboard_loop(self):
        """Read single keystrokes in a background thread."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running:
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                if ch == "\x1b":  # ESC
                    self._running = False
                    break
                # Check for shift (uppercase)
                if ch.isupper():
                    self._shift = True
                    ch = ch.lower()
                else:
                    self._shift = False

                if ch in "wsadqerijkluo":
                    side = "right" if self._shift else "left"
                    self._apply_key(side, ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ------------------------------------------------------------------
    def _apply_key(self, side: str, key: str):
        """Apply key press to hand state."""
        p = self.pos[side]
        r = self.rot[side]

        # Position — robot_world: X left, Y back, Z up
        if key == "w":
            p[1] -= STEP_POS  # forward (-Y)
        elif key == "s":
            p[1] += STEP_POS  # back (+Y)
        elif key == "a":
            p[0] += STEP_POS  # left (+X)
        elif key == "d":
            p[0] -= STEP_POS  # right (-X)
        elif key == "e":
            p[2] += STEP_POS  # up (+Z)
        elif key == "q":
            p[2] -= STEP_POS  # down (-Z)

        # Rotation (incremental, robot_world local)
        elif key == "i":
            r = r * Rotation.from_euler("X", STEP_ROT)
        elif key == "k":
            r = r * Rotation.from_euler("X", -STEP_ROT)
        elif key == "l":
            r = r * Rotation.from_euler("Z", STEP_ROT)
        elif key == "j":
            r = r * Rotation.from_euler("Z", -STEP_ROT)
        elif key == "o":
            r = r * Rotation.from_euler("Y", STEP_ROT)
        elif key == "u":
            r = r * Rotation.from_euler("Y", -STEP_ROT)

        # Reset
        elif key == "r":
            p[:] = 0.0
            r = Rotation.identity()

        self.pos[side] = p
        self.rot[side] = r

    # ------------------------------------------------------------------
    def _build_msg(self, side: str) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "robot_world"

        p = self.pos[side]
        r = self.rot[side]

        msg.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        quat = r.as_quat()
        msg.pose.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]),
            z=float(quat[2]), w=float(quat[3]),
        )
        return msg

    # ------------------------------------------------------------------
    def _publish(self):
        if not self._running:
            return

        side_str = (
            "L" if self._shift else "R"
            if self.arm_side == "both"
            else self.arm_side[:1].upper()
        )

        if self.arm_side in ("left", "both"):
            self.pub_left.publish(self._build_msg("left"))
        if self.arm_side in ("right", "both"):
            self.pub_right.publish(self._build_msg("right"))

        # Print position every ~1s (20 frames at 50Hz)
        ts = time.monotonic()
        if not hasattr(self, "_last_status") or ts - self._last_status > 2.0:
            p = self.pos["left"]
            rpy = self.rot["left"].as_euler("xyz", degrees=True)
            self.get_logger().info(
                f"Left: pos=[{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}] "
                f"rpy=[{rpy[0]:.1f},{rpy[1]:.1f},{rpy[2]:.1f}]deg"
            )
            self._last_status = ts

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardVRSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
