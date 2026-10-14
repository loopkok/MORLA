#!/usr/bin/env python3
"""Keyboard driver for /policy_inference/cmd control verbs.

  KEY   VERB        EFFECT
  ─────────────────────────────────────────────────
  s     policy      start policy inference (must have fresh obs)
  y     playback    replay current episode (or `y <path> <ep>`)
  t     task:<txt>  set language instruction
  space pause       hold current target
  n     resume      unpause
  h     takeover    human-in-the-loop: VR takeover (re-anchor)
  g     release     return control to policy / playback
  x     stop        -> IDLE (also disarm teleop)
  q                 quit

Uses raw terminal read; requires the same ros environment as the node.
"""

from __future__ import annotations

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_HELP = """\
astral_policy_inference keyboard:
  s           start POLICY
  y           start PLAYBACK (uses replay_source/replay_episode params)
  y <path>    start PLAYBACK from an explicit source (h5 / lerobot)
  y <path>:<ep>  start PLAYBACK episode <ep> of <path> (colon syntax)
  t <text>    set task / language instruction
  SPACE       pause        n   resume
  h           HUMAN takeover (VR teleop re-anchor)
  g           release -> back to policy/playback
  x           stop -> IDLE
  q           quit
> """


class PolicyKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("policy_keyboard")
        self.declare_parameter("cmd_topic", "/policy_inference/cmd")
        self.declare_parameter("state_topic", "/policy_inference/state")
        self.declare_parameter("task_topic", "/policy_inference/task")
        self._cmd_pub = self.create_publisher(
            String, self.get_parameter("cmd_topic").value, 10
        )
        self._task_pub = self.create_publisher(
            String, self.get_parameter("task_topic").value, 10
        )
        self._state_sub = self.create_subscription(
            String,
            self.get_parameter("state_topic").value,
            lambda msg: sys.stdout.write("\rstate: " + msg.data + "\n> "),
            _LATCHED,
        )
        self.get_logger().info("keyboard online; send 'h' anytime for human takeover")

    def send(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._cmd_pub.publish(msg)

    def send_task(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._task_pub.publish(msg)


def _read_line(prompt: str) -> str:
    sys.stdout.write(prompt)
    sys.stdout.flush()
    line = ""
    while True:
        ch = sys.stdin.read(1)
        if not ch:  # EOF
            return "q"
        if ch == "\n":
            return line
        if ch == "\x1b":  # ignore escape sequences (arrows)
            continue
        if ord(ch) == 32:  # space
            return "pause"
        line += ch


def main(args=None) -> None:
    rclpy.init(args=remove_ros_args(args))
    node = PolicyKeyboard()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        while rclpy.ok():
            line = _read_line("> ").strip()
            if not line:
                continue
            if line in ("q", "quit"):
                break
            if line == "s":
                node.send("policy")
            elif line == "pause":
                node.send("pause")
            elif line in ("y", "y "):
                node.send("playback")
            elif line.startswith("y ") or line.startswith("y:"):
                rest = line[1:].strip()  # ":path:ep" or " path[:ep]"
                if rest.startswith(" "):
                    rest = rest[1:]
                node.send("playback:" + rest.lstrip(":"))
            elif line == "h":
                node.send("takeover")
            elif line == "g":
                node.send("release")
            elif line == "n":
                node.send("resume")
            elif line == "x":
                node.send("stop")
            elif line.startswith("t "):
                node.send_task(line[2:].strip())
            else:
                print(_HELP)
    except (KeyboardInterrupt, EOFError):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
