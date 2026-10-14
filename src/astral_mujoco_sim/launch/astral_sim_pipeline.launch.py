"""Quest3 → Astral dual teleop → MuJoCo sim.

Do NOT start astral_robot_control alongside this launch.

Nodes: quest3_udp_mocap + 2× astral_arm_teleop_node + mujoco_sim_node
      + optional astral_gripper_teleop (left pinch → /left_gripper/command;
        sim echoes to /left_gripper/joint_states, no MJCF gripper joint yet).

Node config lives in yaml (not launch defaults):
  astral_arm_teleop_{left,right}.yaml  — solver_type, smoothing, …
  quest3_mocap.yaml                — protocol, convert_to_robot, …
  astral_mujoco_sim.yaml           — viewer, mjcf, gripper echo, …

Launch arguments are optional overrides (empty → yaml).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_astral_urdf() -> str:
    """Installed share → source tree → empty (teleop uses its own default)."""
    try:
        share = get_package_share_directory("astral_robot_description")
        p = os.path.join(share, "urdf", "astral_robot.pin.urdf")
        if os.path.isfile(p):
            return p
    except Exception:  # noqa: BLE001
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(
            os.path.join(
                here, "..", "..", "astral_robot_description", "urdf", "astral_robot.pin.urdf"
            )
        ),
        os.path.normpath(
            os.path.join(
                here,
                "..",
                "..",
                "..",
                "src",
                "astral_robot_description",
                "urdf",
                "astral_robot.pin.urdf",
            )
        ),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return ""


def _opt(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _launch_setup(context, *args, **kwargs):
    teleop_pkg = get_package_share_directory("astral_arm_teleop")
    sim_pkg = get_package_share_directory("astral_mujoco_sim")
    quest_pkg = get_package_share_directory("quest3_hand_mocap")

    cfg_l = os.path.join(teleop_pkg, "config", "astral_arm_teleop_left.yaml")
    cfg_r = os.path.join(teleop_pkg, "config", "astral_arm_teleop_right.yaml")
    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")
    sim_cfg = os.path.join(sim_pkg, "config", "astral_mujoco_sim.yaml")

    teleop_extra = {"dry_run": False}
    solver_type = _opt(context, "solver_type")
    if solver_type:
        teleop_extra["solver_type"] = solver_type
    urdf_arg = _opt(context, "urdf_path") or _resolve_astral_urdf()
    if urdf_arg:
        teleop_extra["urdf_path"] = urdf_arg
    require_start = _opt(context, "require_start_signal")
    if require_start:
        teleop_extra["require_start_signal"] = require_start.lower() in ("true", "1", "yes")

    mocap_extra = {"arm_side": "both"}
    protocol = _opt(context, "protocol")
    if protocol:
        mocap_extra["protocol"] = protocol
    convert = _opt(context, "convert_to_robot")
    if convert:
        mocap_extra["convert_to_robot"] = convert.lower() in ("true", "1", "yes")

    mocap = Node(
        package="quest3_hand_mocap",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        parameters=[quest_cfg, mocap_extra],
    )

    # Left Touch controller grip button (mask bit 5) → /teleop/start external
    # start gate — same as full_teleop. Lets you test the external start switch
    # in sim by pressing the left controller grip.
    start_gate = Node(
        package="astral_teleop",
        executable="controller_start_gate",
        name="controller_start_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{"joy_topic": "quest3/left_controller_joy", "button_index": 5}],
        condition=IfCondition(LaunchConfiguration("with_start_gate")),
    )

    # Left Touch controller X button → 段间回位（/teleop/disarm + /teleop/init_direct），
    # 与 start_gate 同开关：sim 里也能按 X 测回位。
    workpos_gate = Node(
        package="astral_teleop",
        executable="controller_workpos_gate",
        name="controller_workpos_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{"joy_topic": "quest3/left_controller_joy", "button_index": 0}],
        condition=IfCondition(LaunchConfiguration("with_start_gate")),
    )

    # Right Touch thumbstick → head yaw/pitch (absolute, spring-return). Sim has
    # no head MJCF joint, but mujoco_sim_node echoes /head/joint_commands →
    # /head/joint_states + logs so the stick mapping can be verified.
    astral_teleop_pkg = get_package_share_directory("astral_teleop")
    head_cfg = os.path.join(astral_teleop_pkg, "config", "head_teleop.yaml")
    head_extra = {}
    if require_start:
        head_extra["require_start_signal"] = require_start.lower() in ("true", "1", "yes")
    head_teleop = Node(
        package="astral_teleop",
        executable="head_teleop_node",
        name="head_teleop_node",
        output="screen",
        emulate_tty=True,
        parameters=[head_cfg, head_extra],
        condition=IfCondition(LaunchConfiguration("with_head_teleop")),
    )

    sim_extra = {}
    viewer = _opt(context, "enable_viewer")
    if viewer:
        sim_extra["enable_viewer"] = viewer.lower() in ("true", "1", "yes")

    sim_params = [sim_cfg]
    if sim_extra:
        sim_params.append(sim_extra)

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astral_gripper_teleop"),
                "launch",
                "gripper_teleop.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("with_gripper")),
        launch_arguments={
            "hand_side": "left",
            "controller_joy_topic": "quest3/left_controller_joy",
        }.items(),
    )

    return [
        mocap,
        start_gate,
        workpos_gate,
        head_teleop,
        gripper,
        Node(
            package="astral_arm_teleop",
            executable="astral_arm_teleop_node",
            name="astral_arm_teleop_left",
            output="screen",
            parameters=[cfg_l, teleop_extra],
        ),
        Node(
            package="astral_arm_teleop",
            executable="astral_arm_teleop_node",
            name="astral_arm_teleop_right",
            output="screen",
            parameters=[cfg_r, teleop_extra],
        ),
        Node(
            package="astral_mujoco_sim",
            executable="astral_mujoco_sim_node",
            name="astral_mujoco_sim_node",
            output="screen",
            parameters=sim_params,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "protocol",
                default_value="",
                description="empty → quest3_mocap.yaml; else udp | tcp_wired | tcp_wireless",
            ),
            DeclareLaunchArgument(
                "convert_to_robot",
                default_value="",
                description="empty → yaml; true=Unity → robot_world",
            ),
            DeclareLaunchArgument(
                "enable_viewer",
                default_value="",
                description="empty → astral_mujoco_sim.yaml",
            ),
            DeclareLaunchArgument(
                "solver_type",
                default_value="",
                description="empty → astral_arm_teleop_{left,right}.yaml",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value="",
                description="empty → astral_robot.pin.urdf (still passed so DH↔URDF hot-switch works)",
            ),
            DeclareLaunchArgument(
                "with_gripper",
                default_value="true",
                description="Quest3 left pinch/trigger → /left_gripper/command (sim echoes, no MJCF gripper joint yet)",
            ),
            DeclareLaunchArgument(
                "with_start_gate",
                default_value="true",
                description="Left controller grip button → /teleop/start external start gate",
            ),
            DeclareLaunchArgument(
                "with_head_teleop",
                default_value="true",
                description=(
                    "Right thumbstick → head yaw/pitch (absolute, spring-return; "
                    "gated by /teleop/start). Sim echoes via mujoco_sim_node."
                ),
            ),
            DeclareLaunchArgument(
                "require_start_signal",
                default_value="",
                description=(
                    "empty → yaml (默认 true). true → wait for /teleop/start to "
                    "capture vr_init and arm (test the external start gate in sim)."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
