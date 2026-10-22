"""Native XHand DDS feedback must reach raw HDF5 with either publisher QoS.

Use an isolated ROS domain: this publishes synthetic sensor feedback only and
does not launch drivers, enable motors, or publish robot commands.
"""
from pathlib import Path
import time

import h5py
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from xhand_control_interfaces.msg import FingerSensorState, XHandSensorState, XHandStateArray

from morla_data_collect.data_collect_node import DataCollectNode


@pytest.mark.parametrize("profile_id", ["nero_left_xhand", "nero_right_xhand", "nero_dual_xhand"])
@pytest.mark.parametrize("reliability", [ReliabilityPolicy.BEST_EFFORT, ReliabilityPolicy.RELIABLE])
def test_native_tactile_dds_is_recorded_with_both_publisher_reliabilities(tmp_path, profile_id, reliability):
    rclpy.init()
    collector = probe = None
    try:
        collector = DataCollectNode(parameter_overrides=[
            Parameter("save_root", value=str(tmp_path)),
            Parameter("session", value="native-qos"),
            Parameter("profile_id", value=profile_id),
            Parameter("runtime_mode", value="real"),
        ])
        probe = Node("native_tactile_qos_probe")
        sources = []
        for sensor in collector._sensor_specs:
            stream = collector._stream_specs[sensor["source_stream"]]
            assert stream.qos == "best_effort_keep_last_1", "frozen sensor metadata must match the subscriber"
            publisher = probe.create_publisher(XHandStateArray, stream.topic,
                                             QoSProfile(depth=1, reliability=reliability))
            message = XHandStateArray()
            side = sensor["side"]
            message.hand_id = [int(collector._profile_values["hardware"]["xhand"][side]["hand_id"])]
            hand = XHandSensorState()
            value = 1.0 if side == "left" else 2.0
            for finger in sensor["finger_order"]:
                item = FingerSensorState()
                item.location = "mid" if finger == "middle" else finger
                item.raw_force = [Vector3(x=value, y=2.0, z=3.0) for _ in range(120)]
                item.calc_force = Vector3(x=value, y=2.0, z=3.0)
                item.raw_temperature = [25] * 20
                item.calc_temperature = 25
                hand.finger_sensor_states.append(item)
            message.sensor_states = [hand]
            sources.append((publisher, message, sensor, value))

        collector._begin_episode()
        deadline = time.monotonic() + 4.0
        matched = False
        while time.monotonic() < deadline:
            for publisher, message, _, _ in sources:
                message.header.stamp = probe.get_clock().now().to_msg()
                publisher.publish(message)
            rclpy.spin_once(probe, timeout_sec=0.01)
            # Drain both hands before publishing again; spin_once executes only
            # one callback, so a continuously ready left stream can starve right.
            for _ in range(len(sources) + 2):
                rclpy.spin_once(collector, timeout_sec=0.01)
            matched = all(publisher.get_subscription_count() > 0 for publisher, *_ in sources)
            with collector._state_lock:
                collector._drain_buffers()
                counts = collector._robot_writer.sensor_valid_counts()
                received = all(all(count > 0 for count in counts[sensor["sensor_key"]])
                               for _, _, sensor, _ in sources)
            if matched and received:
                break
        assert matched, "native feedback publisher did not match collector QoS"
        assert received, f"matched native feedback did not produce valid tactile samples: {counts}"
        episode = Path(collector._episode_dir)
        collector._end_episode(save=True)
        with h5py.File(episode / "robot_data.h5", "r") as raw:
            for _, _, sensor, value in sources:
                group = raw[f"sensors/{sensor['sensor_key']}"]
                assert group["force_raw"].shape[0] > 0, "DDS tactile feedback was not recorded"
                assert group["force_raw"].shape[1:] == (5, 120, 3)
                np.testing.assert_array_equal(group["force_raw"][0, :, :, 0], np.full((5, 120), value))
                assert group["valid"][0].all()
                assert group["temperature_valid"][0].all()
    finally:
        if probe is not None:
            probe.destroy_node()
        if collector is not None:
            collector.destroy_node()
        rclpy.shutdown()
