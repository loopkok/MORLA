"""采集节点进程内冒烟测试：假发布者 + 话题控制 start/stop/discard，校验落盘。

运行（需要 ROS 环境）：
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 -m pytest test_collect_smoke.py -p no:anyio
"""

import json
import os
import sys
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")  # noqa: E402 环境无 ROS 时跳过

from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from astral_data_collect.data_collect_node import DataCollectNode, _to_bool  # noqa: E402
from conftest import make_jpeg  # noqa: E402


def test_to_bool():
    assert _to_bool(True) is True
    assert _to_bool(False) is False
    assert _to_bool("true") is True
    assert _to_bool("false") is False   # launch 字符串回归：bool("false") 恒真
    assert _to_bool("1") is True
    assert _to_bool("0") is False

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class _FakeTeleop(Node):
    """模拟最小遥操数据面：双臂反馈/指令 + 左夹爪 + 两路相机。"""

    def __init__(self) -> None:
        super().__init__("fake_teleop")
        from sensor_msgs.msg import CompressedImage, JointState
        from std_msgs.msg import Float64

        self._t0 = time.time()
        self._pubs = {
            "left_arm_state": self.create_publisher(JointState, "/left_arm/joint_states", _QOS),
            "right_arm_state": self.create_publisher(JointState, "/right_arm/joint_states", _QOS),
            "left_arm_cmd": self.create_publisher(JointState, "/left_arm/joint_commands", _QOS),
            "right_arm_cmd": self.create_publisher(JointState, "/right_arm/joint_commands", _QOS),
            "left_gripper_ratio": self.create_publisher(Float64, "/left_gripper/command", _QOS),
            "left_gripper_rad": self.create_publisher(JointState, "/left_gripper/joint_states", _QOS),
            "right_gripper_ratio": self.create_publisher(Float64, "/right_gripper/command", _QOS),
            "right_gripper_rad": self.create_publisher(JointState, "/right_gripper/joint_states", _QOS),
            "cam0": self.create_publisher(
                CompressedImage, "/quest3_video_streamer/collect/wrist_left", _QOS
            ),
            "cam1": self.create_publisher(
                CompressedImage, "/quest3_video_streamer/collect/wrist_right", _QOS
            ),
        }
        self._jpeg = make_jpeg()
        self.create_timer(0.02, self._tick_joints)   # 50Hz
        self.create_timer(1.0 / 30.0, self._tick_cams)  # 30Hz

    def _tick_joints(self) -> None:
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64

        now = self.get_clock().now().to_msg()
        t = time.time() - self._t0
        joints = [0.1 * i + 0.01 * t for i in range(7)]
        for name in ("left_arm_state", "right_arm_state", "left_arm_cmd", "right_arm_cmd"):
            msg = JointState()
            msg.header.stamp = now
            msg.position = joints
            self._pubs[name].publish(msg)
        for name in ("left_gripper_rad", "right_gripper_rad"):
            msg = JointState()
            msg.header.stamp = now
            msg.position = [0.5]
            self._pubs[name].publish(msg)
        for name in ("left_gripper_ratio", "right_gripper_ratio"):
            msg = Float64()
            msg.data = 0.5
            self._pubs[name].publish(msg)

    def _tick_cams(self) -> None:
        from sensor_msgs.msg import CompressedImage

        now = self.get_clock().now().to_msg()
        for name in ("cam0", "cam1"):
            msg = CompressedImage()
            msg.header.stamp = now
            msg.format = "jpeg"
            msg.data = self._jpeg
            self._pubs[name].publish(msg)


@pytest.fixture()
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wait_state(node: DataCollectNode, want: str, timeout: float = 5.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if node._state == want:
            return
        time.sleep(0.05)
    raise TimeoutError(f"state stuck at {node._state}, want {want}")


def test_record_save_and_discard(ros_context, tmp_path):
    from std_msgs.msg import String

    collect = DataCollectNode(
        parameter_overrides=[
            Parameter("save_root", value=str(tmp_path)),
            Parameter("session", value="smoke"),
            Parameter("cameras", value=["wrist_left", "wrist_right"]),
            Parameter("default_task", value="smoke task"),
        ]
    )
    fake = _FakeTeleop()
    ctrl = fake.create_publisher(
        String, "/data_collect/control",
        QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10),
    )

    def spin_all():
        while rclpy.ok():
            try:
                rclpy.spin_once(collect, timeout_sec=0.01)
                rclpy.spin_once(fake, timeout_sec=0.01)
            except Exception:
                break

    spin_thread = threading.Thread(target=spin_all, daemon=True)
    spin_thread.start()
    time.sleep(0.5)  # 等发现 + latched 握手（此间 IDLE 数据应被门控丢弃）

    def send(cmd: str) -> None:
        msg = String()
        msg.data = cmd
        ctrl.publish(msg)

    # -- 段 0：start → 1s → stop 保存 ------------------------------------------
    send("start")
    _wait_state(collect, "RECORDING")
    time.sleep(1.0)
    send("stop")
    _wait_state(collect, "IDLE")

    ep0 = tmp_path / "smoke" / "episode000000"
    assert ep0.is_dir()
    import h5py

    meta = json.loads((ep0 / "meta.json").read_text())
    meta_start_wall = meta["start_time_wall"]
    with h5py.File(ep0 / "robot_data.h5") as f:
        assert f["streams/left_arm_state/values"].shape[1] == 7
        assert f["streams/left_arm_state/values"].shape[0] > 30  # ≥30 条@50Hz·1s
        assert f["streams/left_gripper_ratio/values"].shape[0] > 30
        ts = f["streams/left_arm_state/timestamps"][:]
        assert (ts[1:] > ts[:-1]).all()
        # IDLE 期（启动前 0.5s 假数据）不得混入段内
        assert ts[0] >= meta_start_wall - 0.05
    with h5py.File(ep0 / "camera_data.h5") as f:
        assert f["wrist_left/images"].shape[0] > 10
    assert meta["task"] == "smoke task"
    assert meta["schema"]["end_effector_left"] == "gripper"
    assert meta["streams"]["left_arm_state"]["count"] > 30
    assert any(e["name"] == "episode_start" for e in meta["events"])

    # -- 段 1：start → discard 删除 ----------------------------------------------
    send("start")
    _wait_state(collect, "RECORDING")
    time.sleep(0.4)
    send("discard")
    _wait_state(collect, "IDLE")
    assert not (tmp_path / "smoke" / "episode000001").exists()

    # -- pause/resume 拆成两段，不制造跨暂停时间空洞 --------------------------
    send("start")
    _wait_state(collect, "RECORDING")
    send("pause")
    _wait_state(collect, "PAUSED")
    time.sleep(0.4)  # PAUSED 期间数据应被真正丢弃
    send("resume")
    _wait_state(collect, "RECORDING")
    time.sleep(0.3)
    send("stop")
    _wait_state(collect, "IDLE")
    ep2 = tmp_path / "smoke" / "episode000001"
    assert ep2.is_dir()
    meta2 = json.loads((ep2 / "meta.json").read_text())
    names = [e["name"] for e in meta2["events"]]
    assert "pause" in names
    with h5py.File(ep2 / "robot_data.h5") as f:
        ts = f["streams/left_arm_state/timestamps"][:]
        assert len(ts) >= 1
        if len(ts) > 1:
            assert float(np.diff(ts).max()) < 0.25
    resumed = tmp_path / "smoke" / "episode000002"
    assert (resumed / "meta.json").exists()
    with h5py.File(resumed / "robot_data.h5") as f:
        ts_resume = f["streams/left_arm_state/timestamps"][:]
        assert len(ts_resume) > 5
        assert np.max(np.diff(ts_resume)) < 0.25
        assert ts_resume[0] - ts[-1] > 0.25
    with h5py.File(ep0 / "camera_data.h5") as f:
        assert len(f["wrist_left/arrival_timestamps"]) == len(f["wrist_left/images"])
        assert np.all(f["wrist_left/arrival_timestamps"][:] >= f["wrist_left/timestamps"][:] - 0.01)

    collect.destroy_node()
    fake.destroy_node()
