"""One explicitly selected USB input device; no desktop focus or keyboard scan."""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .commands import COMMANDS, PedalEdges, request
from .router import TOPIC


class PedalInput(Node):
    def __init__(self):
        super().__init__("morla_pedal_input")
        self.declare_parameter("device", "")
        self.declare_parameter("bindings", ["KEY_F13:teleop_toggle", "KEY_F14:record_toggle", "KEY_F15:record_discard"])
        self.declare_parameter("long_press_s", 1.5)
        self.declare_parameter("exclusive", True)
        self.declare_parameter("dry_run", False)
        self._pub = self.create_publisher(String, TOPIC, 10)
        self._sources = self.create_publisher(String, "/morla/operator/sources", 10)
        self._stop = threading.Event()
        self._device = None
        self._status = "waiting"
        self._detail = str(self.get_parameter("device").value)
        self.create_timer(1.0, self._publish_source)
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _publish_source(self):
        self._sources.publish(String(data=json.dumps(dict(source="pedal", state=self._status,
            detail=self._detail, sent_at=time.time()))))

    def _read(self):
        try:
            from evdev import InputDevice, ecodes
            bindings = {}
            for item in self.get_parameter("bindings").value:
                key, command = item.split(":", 1)
                if command not in COMMANDS or key not in ecodes.ecodes:
                    raise ValueError(f"invalid pedal binding: {item}")
                bindings[ecodes.ecodes[key]] = command
            edges = PedalEdges(bindings, float(self.get_parameter("long_press_s").value))
            path = str(self.get_parameter("device").value)
            if not path:
                raise ValueError("set device to the pedal's /dev/input/by-id/...-event-kbd path")
            while not self._stop.is_set():
                connected = False
                try:
                    with InputDevice(path) as device:
                        self._device = device
                        if self.get_parameter("exclusive").value:
                            device.grab()
                        monotonic_events = hasattr(device, "set_clockid")
                        if monotonic_events:
                            device.set_clockid(time.CLOCK_MONOTONIC)
                        edges.reset()
                        edges.released.update(set(bindings) - set(device.active_keys()))
                        self._status, self._detail = "connected", device.name
                        connected = True
                        for event in device.read_loop():
                            if self._stop.is_set():
                                break
                            if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_DROPPED:
                                raise OSError("pedal input queue overflow; release keys before reconnect")
                            if event.type == ecodes.EV_KEY:
                                event_time = event.timestamp()
                                now = time.monotonic() if monotonic_events else time.time()
                                if not -0.05 <= now - event_time <= 0.5:
                                    edges.reset()
                                    edges.released.update(set(bindings) - set(device.active_keys()))
                                    continue
                                command = edges.update(event.code, event.value, event_time)
                                if command:
                                    row = request(command, "pedal", long_press=command == "record_discard")
                                    if self.get_parameter("dry_run").value:
                                        self._detail = "dry_run: " + command
                                    else:
                                        self._pub.publish(String(data=json.dumps(row)))
                except OSError as exc:
                    self._status, self._detail = "disconnected", str(exc)
                    if connected and not self._stop.is_set() and not self.get_parameter("dry_run").value:
                        self._pub.publish(String(data=json.dumps(request("teleop_pause", "pedal_disconnect"))))
                    self._stop.wait(1.0)
        except (ImportError, ValueError) as exc:
            self._status, self._detail = "error", str(exc)
            self.get_logger().error(str(exc))

    def destroy_node(self):
        self._stop.set()
        if self._device:
            self._device.close()
        self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PedalInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
