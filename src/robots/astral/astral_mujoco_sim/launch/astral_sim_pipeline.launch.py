"""Quest3 → Astral dual teleop → MuJoCo sim.

Do NOT start astral_robot_control alongside this launch.

Nodes: quest3_udp_mocap + selected astral_arm_teleop_node(s) + mujoco_sim_node
      + optional astral_gripper_teleop (left pinch → /left_gripper/command;
        sim echoes to /left_gripper/joint_states, no MJCF gripper joint yet).
Profile launches also add the measured-state bridge and robot_state_publisher;
the Profile supplies input, solver and tool transforms to the selected arms.

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
from morla_core.resource_locator import resolve_robot_asset
from morla_core.profile import ProfileRegistry
from morla_core.coordinate_contract import build_coordinate_contract
from morla_bringup.profile_state import profile_description_actions


def _resolve_astral_urdf() -> str:
    """Resolve the sole installed Astral IK model from robot_description."""
    return str(resolve_robot_asset("astral_left_arm"))


def _opt(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _launch_setup(context, *args, **kwargs):
    teleop_pkg = get_package_share_directory("astral_arm_teleop")
    sim_pkg = get_package_share_directory("astral_mujoco_sim")
    quest_pkg = get_package_share_directory("morla_input")

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

    control_profile_id = _opt(context, "control_profile_id")
    arm_side = _opt(context, "arm_side").lower() or "both"
    if arm_side not in ("left", "right", "both"):
        raise RuntimeError("arm_side must be left|right|both")
    arm_sides = ["left", "right"] if arm_side == "both" else [arm_side]
    gripper_sides_arg = _opt(context, "gripper_sides")
    gripper_sides = [side for side in gripper_sides_arg.split(",") if side]
    coordinator = []
    description_actions = []
    profile_hands = []
    profile_has_head = True
    quest_basis_id = "quest3_canonical_legacy_v1"
    quest_kinematics_hash = ""
    quest_coordinate_version = 0
    side_coordinate_extra = {}
    if control_profile_id:
        profile = ProfileRegistry.from_index().load(control_profile_id)
        if profile.robot_id != "astral":
            raise RuntimeError(f"profile {control_profile_id!r} is not an Astral profile")
        profile_sides = sorted({item["side"] for item in profile.values["components"]
                                if item.get("enabled") and item.get("type") == "arm"})
        if profile_sides != sorted(arm_sides):
            raise RuntimeError(
                f"profile arms {profile_sides} do not match arm_side {arm_side!r}"
            )
        profile_grippers = sorted({item["side"] for item in profile.values["components"]
                                   if item.get("enabled") and item.get("type") == "gripper"})
        profile_hands = [item for item in profile.values["components"]
                         if item.get("enabled") and item.get("type") == "wuji_hand"]
        unsupported_hands = [item for item in profile.values["components"]
                             if item.get("enabled") and item.get("type") not in {
                                 "arm", "gripper", "wuji_hand"}]
        profile_has_head = bool(profile.values.get("capabilities", {}).get("head", False))
        if profile_grippers != sorted(gripper_sides) or unsupported_hands:
            raise RuntimeError(
                "Astral MuJoCo profile grippers must match gripper_sides and only "
                "Wuji Hand is supported as a simulated dexterous hand"
            )
        coordinate_contract = build_coordinate_contract(profile.values)
        description_actions = profile_description_actions(profile)
        quest_basis_id = coordinate_contract.input_basis_id
        quest_kinematics_hash = coordinate_contract.kinematics_hash
        quest_coordinate_version = coordinate_contract.version
        solver_type_value = (solver_type or "geometric").strip().lower()
        expected_adapters = {
            "geometric": "astral_geometric_poe_link7_v1",
            "urdf_numerical": "astral_urdf_link7_v1",
            "urdf": "astral_urdf_link7_v1",
            "numerical": "astral_urdf_link7_v1",
            "analytic_dh": "astral_analytic_dh_legacy_v1",
            "analytic": "astral_analytic_dh_legacy_v1",
            "dh": "astral_analytic_dh_legacy_v1",
        }
        for component in profile.values.get("components", []):
            if not component.get("enabled") or component.get("type") != "arm":
                continue
            side = component["side"]
            model_path = str(resolve_robot_asset(str(component["description_model_id"])))
            arm_contract = coordinate_contract.arm(side)
            if expected_adapters.get(solver_type_value) != arm_contract["solver_adapter_id"]:
                raise RuntimeError(
                    f"solver_type={solver_type_value!r} does not match Profile adapter "
                    f"{arm_contract['solver_adapter_id']!r} for {side}"
                )
            input_rotation = coordinate_contract.resolve_arm_input_rotation(side, model_path)
            tool_transform = coordinate_contract.resolve_arm_tool_transform(side, model_path)
            side_coordinate_extra[side] = {
                "vr_to_arm_rot": input_rotation.reshape(-1).tolist(),
                "solver_tip_to_tcp_transform": tool_transform.reshape(-1).tolist(),
                "use_solver_tip_to_tcp_transform": True,
                "coordinate_contract_version": coordinate_contract.version,
                "kinematics_hash": coordinate_contract.kinematics_hash,
                "solver_base_frame": arm_contract["solver_base_frame"],
                "solver_tip_frame": arm_contract["solver_tip_frame"],
                "solver_adapter_id": arm_contract["solver_adapter_id"],
                "tool_frame": arm_contract["tool"]["frame"],
            }
        coordinator.append(Node(
            package="morla_bringup", executable="control_coordinator_node",
            name="morla_control_coordinator", output="screen",
            parameters=[{"profile_id": control_profile_id, "operating_mode": "sim"}],
        ))
    elif not gripper_sides and _opt(context, "with_gripper").lower() in ("true", "1", "yes"):
        gripper_sides = ["left"]
    mocap_extra = {
        "arm_side": arm_side,
        "coordinate_basis_id": quest_basis_id,
        "coordinate_contract_version": quest_coordinate_version,
        "kinematics_hash": quest_kinematics_hash,
        "landmark_preprocess": "raw",
        "allow_legacy_packets": _opt(context, "allow_legacy_packets").lower()
        in ("true", "1", "yes"),
    }
    protocol = _opt(context, "protocol")
    if protocol:
        mocap_extra["protocol"] = protocol
    convert = _opt(context, "convert_to_robot")
    if convert:
        mocap_extra["convert_to_robot"] = convert.lower() in ("true", "1", "yes")

    mocap = Node(
        package="morla_input",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        parameters=[quest_cfg, mocap_extra],
    )
    with_input = _opt(context, "with_input").lower() not in ("false", "0", "no")

    # Left Touch controller grip button (mask bit 5) → /teleop/start external
    # start gate — same as full_teleop. Lets you test the external start switch
    # in sim by pressing the left controller grip.
    start_gate = Node(
        package="morla_bringup",
        executable="controller_start_gate",
        name="controller_start_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{"joy_topic": "quest3/left_controller_joy", "button_index": 5,
                      "use_control_coordinator": bool(control_profile_id)}],
        condition=IfCondition(LaunchConfiguration("with_start_gate")),
    )

    # Left Touch controller X button → 段间回位（/teleop/disarm + /teleop/init_direct），
    # 与 start_gate 同开关：sim 里也能按 X 测回位。
    workpos_gate = Node(
        package="morla_bringup",
        executable="controller_workpos_gate",
        name="controller_workpos_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{"joy_topic": "quest3/left_controller_joy", "button_index": 0,
                      "use_control_coordinator": bool(control_profile_id)}],
        condition=IfCondition(LaunchConfiguration("with_start_gate")),
    )

    # Right Touch thumbstick → head yaw/pitch (absolute, spring-return). Sim has
    # no head MJCF joint, but mujoco_sim_node echoes /head/joint_commands →
    # /head/joint_states + logs so the stick mapping can be verified.
    morla_bringup_pkg = get_package_share_directory("morla_bringup")
    head_cfg = os.path.join(morla_bringup_pkg, "config", "head_teleop.yaml")
    head_extra = {}
    if require_start:
        head_extra["require_start_signal"] = require_start.lower() in ("true", "1", "yes")
    head_teleop = Node(
        package="morla_bringup",
        executable="head_teleop_node",
        name="head_teleop_node",
        output="screen",
        emulate_tty=True,
        parameters=[head_cfg, head_extra],
        condition=IfCondition(
            LaunchConfiguration("with_head_teleop") if profile_has_head else "false"
        ),
    )

    sim_extra = {}
    viewer = _opt(context, "enable_viewer")
    if viewer:
        sim_extra["enable_viewer"] = viewer.lower() in ("true", "1", "yes")

    sim_params = [sim_cfg]
    if sim_extra:
        sim_params.append(sim_extra)

    grippers = []
    for side in gripper_sides:
        if side not in ("left", "right"):
            raise RuntimeError(f"invalid gripper side {side!r}")
        grippers.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("astral_gripper_teleop"),
                "launch", "gripper_teleop.launch.py")),
            launch_arguments={
                "hand_side": side,
                "controller_joy_topic": f"quest3/{side}_controller_joy",
            }.items(),
        ))

    wuji_hands = []
    for item in profile_hands:
        side = item["side"]
        hand_pkg = get_package_share_directory("wujihand_mujoco_sim")
        retarget_pkg = get_package_share_directory(
            item.get("retarget_package", "wujihand_retargeting")
        )
        retarget_config = os.path.join(
            retarget_pkg, item.get("retarget_params_file", "config/retarget_dexpilot_right.yml")
        )
        default_retarget_params = os.path.join(
            retarget_pkg, "config", "wujihand_retarget_params.yaml"
        )
        model_id = str(item.get("description_model_id", f"wuji_{side}_hand"))
        urdf_path = str(resolve_robot_asset(model_id))
        wuji_hands.extend([
            Node(
                package="wujihand_retargeting",
                executable="wujihand_retarget_node",
                name=f"wujihand_retarget_{side}",
                output="screen",
                parameters=[default_retarget_params, {
                    "hand_side": side,
                    "retarget_backend": "dexpilot",
                    "input_topic": "/hand_landmarks",
                    f"dexpilot_config_{side}": retarget_config,
                    f"dexpilot_urdf_{side}": urdf_path,
                    "smoothing_alpha": 0.7,
                    "max_joint_rate": 1.5,
                }],
            ),
            Node(
                package="wujihand_mujoco_sim",
                executable="wujihand_mujoco_sim_node",
                name=f"wujihand_mujoco_sim_{side}",
                output="screen",
                parameters=[os.path.join(hand_pkg, "config", "wujihand_mujoco_sim.yaml"), {
                    "hand_side": side,
                    "hand_name": f"{side}_hand",
                    "cmd_topic": str(item.get("command_topic", f"/{side}_hand/joint_commands")),
                    "state_topic": str(item.get("state_topic", f"/{side}_hand/joint_states")),
                    "enable_viewer": False,
                    "realtime": True,
                    "state_rate_hz": 50.0,
                }],
            ),
        ])

    arm_nodes = []
    for side in arm_sides:
        parameters = [cfg_l if side == "left" else cfg_r, teleop_extra]
        if side in side_coordinate_extra:
            parameters.append(side_coordinate_extra[side])
        arm_nodes.append(Node(
            package="astral_arm_teleop", executable="astral_arm_teleop_node",
            name=f"astral_arm_teleop_{side}", output="screen",
            parameters=parameters,
        ))

    return [
        *description_actions,
        *([mocap] if with_input else []),
        start_gate,
        workpos_gate,
        head_teleop,
        *grippers,
        *wuji_hands,
        *arm_nodes,
        Node(
            package="astral_mujoco_sim",
            executable="astral_mujoco_sim_node",
            name="astral_mujoco_sim_node",
            output="screen",
            parameters=sim_params,
        ),
        *coordinator,
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
                "allow_legacy_packets",
                default_value="false",
                description=(
                    "Explicit old-APK compatibility; bypasses source timestamp, "
                    "sequence and session checks."
                ),
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
            DeclareLaunchArgument("control_profile_id", default_value="",
                                  description="MORLA profile; enables coordinated controls"),
            DeclareLaunchArgument("with_input", default_value="true",
                                  description="launch the shared Quest input receiver"),
            DeclareLaunchArgument("arm_side", default_value="both", description="left|right|both"),
            DeclareLaunchArgument("gripper_sides", default_value="",
                                  description="Comma-separated configured gripper sides"),
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
