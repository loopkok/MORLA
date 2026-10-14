"""Nero VR teleoperation publisher; it never imports or calls the arm SDK."""

from __future__ import annotations

import json
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

from morla_core.coordinates import (
    fixed_transform_between_urdf, solver_tip_to_tcp, tcp_to_solver_tip,
    validate_rotation, validate_transform,
)
from morla_core.ownership import PolicyLease
from morla_core.resource_locator import resolve_robot_asset
from .fps_counter import FPSCounter
from .ik_solver import IKSolver, NeroParams
from .pose_processor import PoseProcessor
from .safety_filter import SafetyFilter


class NeroArmTeleopNode(Node):
    """Nero IK and VR mapping connected to standard named ROS interfaces."""

    def __init__(self) -> None:
        super().__init__("nero_arm_teleop")
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("control_rate_hz", 125.0)
        self.declare_parameter("require_start_signal", True)
        self.declare_parameter("wrist_topic", "")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("command_topic", "")
        # A floating sentinel infers DOUBLE_ARRAY; shape validation below
        # rejects a missing profile matrix instead of accepting an empty byte array.
        self.declare_parameter("vr_to_arm_rot_after_unity_to_robot", [0.0])
        self.declare_parameter("pos_smoothing", 0.8)
        self.declare_parameter("rot_smoothing", 0.8)
        self.declare_parameter("motion_scale", 0.65)
        self.declare_parameter("flip_pitch", False)
        self.declare_parameter("max_joint_delta_rad_per_step", 0.065)
        self.declare_parameter("workspace_radius", 0.58)
        self.declare_parameter("workspace_x_min", float("-inf"))
        self.declare_parameter("workspace_x_max", float("inf"))
        self.declare_parameter("workspace_y_min", float("-inf"))
        self.declare_parameter("workspace_y_max", float("inf"))
        self.declare_parameter("workspace_z_min", float("-inf"))
        self.declare_parameter("workspace_z_max", float("inf"))
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter(
            "solver_tip_to_tcp_transform",
            [1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter("use_solver_tip_to_tcp_transform", False)
        self.declare_parameter("data_timeout_s", 1.5)
        self.declare_parameter("description_model_id", "nero_left_arm")
        self.declare_parameter("coordinate_contract_version", 0)
        self.declare_parameter("kinematics_hash", "")
        self.declare_parameter("solver_base_frame", "")
        self.declare_parameter("solver_tip_frame", "")
        self.declare_parameter("tool_frame", "")
        self.declare_parameter("solver_adapter_id", "nero_analytic_mdh_link7_v1")

        self.side = str(self.get_parameter("arm_side").value).strip().lower()
        if self.side not in ("left", "right"):
            raise ValueError("arm_side must be left or right")
        self.joint_names = [f"{self.side}_joint{i}" for i in range(1, 8)]
        defaults = {
            # Input is a shared root-level stream from morla_input. Keeping
            # this absolute avoids resolving it under the arm namespace.
            "wrist_topic": f"/quest3/{self.side}_wrist_pose",
            "state_topic": f"/{self.side}_arm/joint_states",
            "command_topic": f"/{self.side}_arm/joint_commands",
        }
        self.topics = {
            name: str(self.get_parameter(name).value).strip() or fallback
            for name, fallback in defaults.items()
        }
        matrix = np.asarray(
            self.get_parameter("vr_to_arm_rot_after_unity_to_robot").value,
            dtype=float,
        )
        if matrix.shape != (9,):
            raise ValueError("vr_to_arm_rot_after_unity_to_robot must have 9 values")
        self.rotation_to_arm = validate_rotation(matrix.reshape(3, 3))

        model_id = str(self.get_parameter("description_model_id").value).strip()
        model_path = resolve_robot_asset(model_id)
        params = NeroParams.default()
        self.ik = IKSolver(nero_params=params)
        self.pose_processor = PoseProcessor(
            vr_to_arm_rot=self.rotation_to_arm,
            pos_smoothing=float(self.get_parameter("pos_smoothing").value),
            rot_smoothing=float(self.get_parameter("rot_smoothing").value),
            motion_scale=float(self.get_parameter("motion_scale").value),
            flip_pitch=bool(self.get_parameter("flip_pitch").value),
        )
        tcp = np.asarray(self.get_parameter("tcp_offset").value, dtype=float)
        if tcp.shape != (6,) or not np.isfinite(tcp).all():
            raise ValueError("tcp_offset must contain [x,y,z,roll,pitch,yaw]")
        self.solver_base_frame = str(self.get_parameter("solver_base_frame").value).strip() or f"{self.side}_base_link"
        self.solver_tip_frame = str(self.get_parameter("solver_tip_frame").value).strip() or f"{self.side}_link7"
        configured_tool_frame = str(self.get_parameter("tool_frame").value).strip()
        self.tool_frame = configured_tool_frame or self.solver_tip_frame
        # Profile launches pass the full fixed transform resolved from URDF.
        # Direct node launches default to the solver tip (link7). A named
        # physical flange/tool frame can still be selected explicitly.
        # ``tcp_offset`` remains an explicit legacy override.
        tool_transform = np.asarray(
            self.get_parameter("solver_tip_to_tcp_transform").value, dtype=float
        )
        if bool(self.get_parameter("use_solver_tip_to_tcp_transform").value):
            if tool_transform.size != 16:
                raise ValueError("solver_tip_to_tcp_transform must contain 16 row-major values")
            self._T_solver_tip_to_tcp = validate_transform(tool_transform.reshape(4, 4))
        elif not np.allclose(tcp, np.zeros(6), atol=1e-12, rtol=0.0):
            self._T_solver_tip_to_tcp = np.eye(4)
            self._T_solver_tip_to_tcp[:3, :3] = Rotation.from_euler("xyz", tcp[3:]).as_matrix()
            self._T_solver_tip_to_tcp[:3, 3] = tcp[:3]
        else:
            try:
                self._T_solver_tip_to_tcp = fixed_transform_between_urdf(
                    model_path, self.solver_tip_frame, self.tool_frame
                )
            except ValueError as exc:
                raise ValueError(
                    f"cannot resolve default/configured tool frame {self.tool_frame!r} "
                    f"from solver tip {self.solver_tip_frame!r}: {exc}"
                ) from exc
        self.coordinate_contract_version = int(self.get_parameter("coordinate_contract_version").value)
        self.kinematics_hash = str(self.get_parameter("kinematics_hash").value)
        self.solver_adapter_id = str(self.get_parameter("solver_adapter_id").value)

        self.data_timeout_s = float(self.get_parameter("data_timeout_s").value)
        self._require_start = bool(self.get_parameter("require_start_signal").value)
        self._q: np.ndarray | None = None
        self._q_received_at = 0.0
        self._wrist: PoseStamped | None = None
        self._wrist_received_at = 0.0
        self._armed = False
        self._active_frame: str | None = None
        self._prepared_token: str | None = None
        self._last_loop_at = time.monotonic()
        self._fps = FPSCounter(window=200, print_interval=3.0)

        self.safety = SafetyFilter(
            joint_lower_limits=self.ik.lower_limits,
            joint_upper_limits=self.ik.upper_limits,
            max_joint_vel=float(
                self.get_parameter("max_joint_delta_rad_per_step").value
            ),
            workspace_radius=float(self.get_parameter("workspace_radius").value),
            workspace_x_min=float(self.get_parameter("workspace_x_min").value),
            workspace_x_max=float(self.get_parameter("workspace_x_max").value),
            workspace_y_min=float(self.get_parameter("workspace_y_min").value),
            workspace_y_max=float(self.get_parameter("workspace_y_max").value),
            workspace_z_min=float(self.get_parameter("workspace_z_min").value),
            workspace_z_max=float(self.get_parameter("workspace_z_max").value),
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._cmd_pub = self.create_publisher(JointState, self.topics["command_topic"], 1)
        self._target_pub = self.create_publisher(
            PoseStamped, f"nero_teleop/{self.side}_target_ee", 1
        )
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(PoseStamped, self.topics["wrist_topic"], self._on_wrist, qos)
        self.create_subscription(JointState, self.topics["state_topic"], self._on_state, 10)
        self.create_subscription(Bool, "/teleop/start", self._on_start, 10)
        self.create_subscription(Bool, "/teleop/disarm", self._on_disarm, 10)
        self.create_subscription(String, "quest3/input_epoch", self._on_epoch, 10)
        self._policy_lease = PolicyLease()
        self._ownership_subscription, self._ownership_ack_pub = self._policy_lease.bind(
            self, writer=f"{self.side}_arm", callback=self._on_policy_state
        )
        self.create_service(Trigger, "~/prepare_reanchor", self._on_prepare_reanchor)
        self.create_service(Trigger, "~/reanchor", self._on_reanchor)
        rate = float(self.get_parameter("control_rate_hz").value)
        if rate <= 0:
            raise ValueError("control_rate_hz must be positive")
        self._timer = self.create_timer(1.0 / rate, self._control_tick)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"Nero teleop side={self.side} rate={rate:g}Hz start_required="
            f"{self._require_start}; standard output={self.topics['command_topic']}"
        )

    def _on_state(self, msg: JointState) -> None:
        if len(msg.name) != 7 or len(msg.position) != 7 or set(msg.name) != set(self.joint_names):
            self.get_logger().error("rejecting feedback: expected exactly named Nero joints")
            return
        values = dict(zip(msg.name, msg.position))
        q = np.asarray([values[name] for name in self.joint_names], dtype=float)
        if not np.isfinite(q).all():
            self.get_logger().error("rejecting non-finite measured joint feedback")
            return
        self._q = q
        self._q_received_at = time.monotonic()

    def _on_wrist(self, msg: PoseStamped) -> None:
        values = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            dtype=float,
        )
        if not np.isfinite(values).all():
            self.get_logger().warning("rejecting non-finite Quest pose")
            return
        quat = values[3:]
        norm = float(np.linalg.norm(quat))
        if norm < 1e-8 or abs(norm - 1.0) > 0.1:
            self.get_logger().warning("rejecting invalid Quest quaternion")
            return
        self._wrist = msg
        self._wrist_received_at = time.monotonic()
        if self._armed and self._active_frame != msg.header.frame_id:
            self._disarm_local("Quest pose frame changed during active teleop")
            return
        if self._armed:
            self.pose_processor.update_vr_pose(values[:3], Rotation.from_quat(quat / norm))
        elif not self._require_start and self._fresh_feedback():
            if self._policy_lease.manual_allowed and self._policy_lease.activity != "human":
                self._capture_anchor(msg, arm=True)

    def _fresh_feedback(self) -> bool:
        return self._q is not None and time.monotonic() - self._q_received_at <= 0.5

    def _fresh_wrist(self) -> bool:
        return self._wrist is not None and time.monotonic() - self._wrist_received_at <= self.data_timeout_s

    def _on_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        if not self._policy_lease.manual_allowed:
            self.get_logger().warning("start rejected: policy/playback owns the command stream")
            return
        if self._policy_lease.activity == "human":
            self.get_logger().warning(
                "start rejected during HUMAN ownership; a prepared re-anchor token is required"
            )
            return
        self._capture_anchor(self._wrist, arm=True)

    def _capture_anchor(self, wrist: PoseStamped | None, *, arm: bool) -> bool:
        if not self._fresh_feedback() or not self._fresh_wrist():
            self.get_logger().warning(
                "start rejected: fresh measured joint state and Quest wrist pose are required"
            )
            return False
        if wrist is None:
            return False
        pose = wrist.pose
        pos = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        quat = np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            dtype=float,
        )
        if np.linalg.norm(quat) < 1e-8:
            return False
        self.ik.sync_state(self._q)
        reference = solver_tip_to_tcp(
            self.ik.fk(self._q), self._T_solver_tip_to_tcp
        )
        self.pose_processor.reset()
        self.pose_processor.set_vr_zero_point(pos, Rotation.from_quat(quat / np.linalg.norm(quat)))
        self.robot_init_pos = reference[:3, 3].copy()
        self.robot_init_rot = reference[:3, :3].copy()
        self.safety.set_initial_state(self._q)
        self._active_frame = wrist.header.frame_id
        self._armed = bool(arm)
        self.get_logger().info(
            f"[{self.side}] anchor captured from measured q and frame={self._active_frame!r}; "
            f"armed={self._armed}"
        )
        return True

    def _on_policy_state(self, msg: String) -> None:
        accepted = self._policy_lease.update(msg.data)
        payload = self._policy_lease.payload
        if not self._policy_lease.manual_allowed:
            if not payload.get("takeover_pending"):
                self._prepared_token = None
            self._disarm_local("policy/playback owns the command stream")
            return
        if not accepted:
            return
        if payload.get("activity") != "human":
            self._prepared_token = None
            return
        tokens = payload.get("reanchor_tokens", [])
        token = self._prepared_token
        if token and isinstance(tokens, list) and token in tokens:
            if self._fresh_feedback() and self._fresh_wrist():
                self._armed = True
                self._prepared_token = None
                self.get_logger().info(
                    f"[{self.side}] HUMAN ownership committed with prepared re-anchor"
                )
            else:
                self._prepared_token = None
                self.get_logger().error(
                    f"[{self.side}] HUMAN commit rejected: wrist or measured state became stale"
                )

    def _on_prepare_reanchor(self, _request, response):
        activity = self._policy_lease.activity
        if self._policy_lease.seen and (
            not self._policy_lease.fresh or activity in ("human", "unknown")
        ):
            response.success = False
            response.message = "ownership is stale, unknown, or already HUMAN"
            return response
        if not self._capture_anchor(self._wrist, arm=False):
            response.success = False
            response.message = "fresh wrist pose and measured joint state are required"
            return response
        self._prepared_token = uuid.uuid4().hex
        response.success = True
        response.message = json.dumps({"token": self._prepared_token})
        return response

    def _on_reanchor(self, _request, response):
        if not self._policy_lease.manual_allowed:
            response.success = False
            response.message = "policy/playback owns the command stream"
            return response
        ok = self._capture_anchor(self._wrist, arm=True)
        response.success = ok
        response.message = "re-anchored and armed" if ok else "fresh input/state required"
        return response

    def _on_disarm(self, msg: Bool) -> None:
        if msg.data:
            self._disarm_local("/teleop/disarm received")

    def _on_epoch(self, msg: String) -> None:
        self._wrist = None
        self._wrist_received_at = 0.0
        self._prepared_token = None
        self._disarm_local(f"Quest input epoch changed: {msg.data}")

    def _disarm_local(self, reason: str) -> None:
        was_armed = self._armed
        self._armed = False
        self._active_frame = None
        self.pose_processor.reset()
        if was_armed:
            self.get_logger().warning(f"[{self.side}] disarmed: {reason}; no automatic home")

    def _control_tick(self) -> None:
        now = time.monotonic()
        dt = now - self._last_loop_at
        self._last_loop_at = now
        fps = self._fps.tick()
        if self._fps.should_print():
            self.get_logger().info(f"[{self.side}] teleop control loop {fps:.1f}Hz")
        if not self._armed:
            return
        if not self._fresh_feedback() or not self._fresh_wrist():
            self._disarm_local("feedback or wrist input timed out")
            return

        delta_pos, delta_rot = self.pose_processor.process()
        target_tcp = self.pose_processor.compute_target_pose(
            delta_pos, delta_rot, self.robot_init_pos, self.robot_init_rot
        )
        # SafetyFilter's radial workspace is centered at the solver base, so
        # evaluate it on the solver-tip pose after removing the tool transform.
        target_solver_tip = tcp_to_solver_tip(target_tcp, self._T_solver_tip_to_tcp)
        target_solver_tip[:3, 3] = self.safety.check_workspace(target_solver_tip[:3, 3])
        target_tcp = solver_tip_to_tcp(target_solver_tip, self._T_solver_tip_to_tcp)
        solution = self.ik.solve(target_solver_tip)
        if solution is None:
            self.get_logger().warning("Nero analytic IK did not converge", throttle_duration_sec=1.0)
            return
        safe, info = self.safety.filter(solution, dt)
        if info.get("collision"):
            self.get_logger().warning("Nero safety filter rejected self-collision")
            return

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = f"{self.side}_base_link"
        command.name = list(self.joint_names)
        command.position = [float(value) for value in safe]
        self._cmd_pub.publish(command)

        target = PoseStamped()
        target.header = command.header
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(
            float, target_tcp[:3, 3]
        )
        target_quat = Rotation.from_matrix(target_tcp[:3, :3]).as_quat()
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = map(float, target_quat)
        self._target_pub.publish(target)

    def _publish_status(self) -> None:
        payload = {
            "side": self.side,
            "armed": self._armed,
            "ownership": self._policy_lease.activity,
            "prepared_reanchor": self._prepared_token is not None,
            "feedback_fresh": self._fresh_feedback(),
            "wrist_fresh": self._fresh_wrist(),
            "frame_id": self._active_frame,
            "command_topic": self.topics["command_topic"],
            "solver": "analytic_dh",
            "coordinate_contract_version": self.coordinate_contract_version,
            "kinematics_hash": self.kinematics_hash,
            "solver_base_frame": self.solver_base_frame,
            "solver_tip_frame": self.solver_tip_frame,
            "solver_adapter_id": self.solver_adapter_id,
            "tool_frame": self.tool_frame,
        }
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NeroArmTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
