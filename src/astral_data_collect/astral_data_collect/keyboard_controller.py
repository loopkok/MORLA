"""键盘控制器：终端热键 → /data_collect/control 话题。

按键（与 nero_dual_data_collect 一致，保持操作习惯）：
  s   start       开始新段
  q   stop        结束并保存当前段
  d   discard     丢弃当前段（删除文件）
  n   next        保存当前段并立即开新段
  p   pause/resume 暂停/继续（同一段内）
  t   task        输入下一段的任务文本
  ESC quit        退出控制器（不影响采集节点）

用法：
  ros2 run astral_data_collect keyboard_controller
"""

from __future__ import annotations

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_KEY_CMDS = {
    "s": "start",
    "q": "stop",
    "d": "discard",
    "n": "next",
    "p": None,  # pause/resume 二义：依据 /data_collect/state 决定
}

_HELP = """\
[data_collect keyboard] 热键：
  s=start  q=stop&save  d=discard  n=next  p=pause/resume  t=task text  ESC=quit
"""


class KeyboardController(Node):
    def __init__(self) -> None:
        super().__init__("data_collect_keyboard")
        self._ctrl_pub = self.create_publisher(String, "/data_collect/control", _CONTROL_QOS)
        self._task_pub = self.create_publisher(String, "/data_collect/task", _LATCHED_QOS)
        self._last_state = "IDLE"
        self.create_subscription(
            String, "/data_collect/state", self._on_state, _LATCHED_QOS
        )
        self._stdin_stop = threading.Event()
        self._stdin_thread = threading.Thread(
            target=self._stdin_loop, name="data-collect-kbd", daemon=True
        )

    def _on_state(self, msg: String) -> None:
        import json

        try:
            self._last_state = json.loads(msg.data).get("state", "IDLE")
        except Exception:
            pass

    def start(self) -> None:
        self._stdin_thread.start()

    def stop(self) -> None:
        self._stdin_stop.set()

    def send(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._ctrl_pub.publish(msg)
        self.get_logger().info(f"-> {cmd}")

    def send_task(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._task_pub.publish(msg)
        self.get_logger().info(f"-> task {text!r}")

    # -- stdin 读键 ------------------------------------------------------------

    def _stdin_loop(self) -> None:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            print(_HELP, flush=True)
            while not self._stdin_stop.is_set() and rclpy.ok():
                ch = sys.stdin.read(1)
                if not ch:
                    break
                if ch == "\x1b":  # ESC
                    self.get_logger().info("ESC — controller exit (采集节点不受影响)")
                    rclpy.shutdown()
                    return
                if ch in _KEY_CMDS:
                    cmd = _KEY_CMDS[ch]
                    if cmd is None:  # p
                        cmd = "resume" if self._last_state == "PAUSED" else "pause"
                    self.send(cmd)
                elif ch == "t":
                    # 临时恢复 cooked 模式读一整行
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)
                    try:
                        text = input("task for NEXT episode> ").strip()
                    finally:
                        tty.setcbreak(fd)
                    if text:
                        self.send_task(text)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KeyboardController()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
