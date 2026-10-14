"""Profile-driven Nero/XHand stack with mutually exclusive sim and real modes."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from morla_core.profile import ProfileRegistry


def _bool(context, name, default="false"):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    return value in ("true", "1", "yes") if value else default == "true"


def _setup(context, *args, **kwargs):
    profile_id = LaunchConfiguration("profile_id").perform(context).strip()
    profile = ProfileRegistry.from_index().load(profile_id)
    if profile.robot_id != "nero":
        raise RuntimeError(f"profile {profile_id!r} is not a Nero robot profile")
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in ("sim", "real"):
        raise RuntimeError("mode must be 'sim' or 'real'")
    sdk_backend = LaunchConfiguration("sdk_backend").perform(context).strip().lower()
    if mode == "real" and sdk_backend != "hardware":
        raise RuntimeError("real mode requires sdk_backend:=hardware; use mode:=sim for software tests")

    components = [item for item in profile.values.get("components", []) if item.get("enabled")]
    arms = [item for item in components if item.get("type") == "nero_arm"]
    hands = [item for item in components if item.get("type") == "xhand"]
    if not arms and not hands:
        raise RuntimeError(f"profile {profile_id!r} has no enabled Nero/XHand components")

    actions = []
    if _bool(context, "with_input", "true"):
        allow_legacy = _bool(context, "allow_legacy_packets", "false")
        actions.append(
            Node(
                package="morla_input",
                executable="quest3_udp_mocap",
                name="quest3_udp_mocap",
                output="screen",
                parameters=[{
                    "arm_side": "both" if {item["side"] for item in arms} == {"left", "right"}
                    else (arms[0]["side"] if arms else "both"),
                    "landmark_preprocess": "raw",
                    "allow_legacy_packets": allow_legacy,
                }],
            )
        )

    actions.append(
        Node(
            package="morla_bringup",
            executable="controller_start_gate",
            name="controller_start_gate",
            output="screen",
            parameters=[{
                "joy_topic": "quest3/left_controller_joy",
                "right_joy_topic": "quest3/right_controller_joy",
                "button_index": 5,
                "use_control_coordinator": True,
            }],
        )
    )
    actions.append(
        Node(
            package="morla_bringup",
            executable="controller_workpos_gate",
            name="controller_workpos_gate",
            output="screen",
            parameters=[{
                "joy_topic": "quest3/left_controller_joy",
                "button_index": 0,
                "use_control_coordinator": True,
            }],
        )
    )

    for item in arms:
        side = item["side"]
        teleop_config = os.path.join(
            get_package_share_directory(item["teleop_package"]), item["teleop_params_file"]
        )
        actions.append(
            Node(
                package=item["teleop_package"],
                executable="nero_arm_teleop_node",
                name="nero_arm_teleop",
                namespace=f"/{side}_arm",
                output="screen",
                parameters=[teleop_config],
            )
        )
        if mode == "real":
            driver_config = os.path.join(
                get_package_share_directory(item["driver_package"]), item["driver_params_file"]
            )
            actions.append(
                Node(
                    package=item["driver_package"],
                    executable="nero_robot_driver",
                    name="nero_robot_driver",
                    namespace=f"/{side}_arm",
                    output="screen",
                    parameters=[
                        driver_config,
                        {"arm_side": side, "sdk_backend": "hardware"},
                    ],
                )
            )

    if mode == "sim":
        sim_parameters = {
            "enable_viewer": _bool(context, "enable_viewer", "false"),
            "publish_tactile": True,
            "state_rate": 50.0,
            "tactile_rate": 50.0,
        }
        for item in hands:
            sim_parameters[f"hand_id_{item['side']}"] = int(item.get("hand_id", 0))
        actions.append(
            Node(
                package="xnero_mujoco_sim",
                executable="xnero_mujoco_sim_node",
                name="xnero_mujoco_sim_node",
                output="screen",
                parameters=[sim_parameters],
            )
        )

    for item in hands:
        side = item["side"]
        native = item.get("native_command", {})
        hand_limits = item.get("limits")
        if not isinstance(hand_limits, list) or len(hand_limits) != 12:
            raise RuntimeError(f"profile component {item.get('component_id')} must define 12 joint limits")
        lower_limits = [float(pair[0]) for pair in hand_limits]
        upper_limits = [float(pair[1]) for pair in hand_limits]
        adapter_parameters = {
            "side": side,
            "hand_id": int(item.get("hand_id", 0)),
            "mode": int(native.get("mode", 3)),
            "kp": float(native.get("kp", 80.0)),
            "ki": float(native.get("ki", 0.0)),
            "kd": float(native.get("kd", 0.0)),
            "effort_limit_ma": float(native.get("effort_limit_ma", 400.0)),
            "joint_lower_limits": lower_limits,
            "joint_upper_limits": upper_limits,
            "require_armed": True,
            "state_topic": item.get("state_topic", f"/{side}_hand/joint_states"),
            "command_topic": item.get("command_topic", f"/{side}_hand/joint_commands"),
            "native_state_topic": item.get("native_state_topic", f"/{side}_hand/driver/xhand_state"),
            "native_command_topic": item.get("native_command_topic", f"/{side}_hand/driver/xhand_command"),
            "status_topic": f"/{side}_hand/adapter/status",
        }
        actions.append(
            Node(
                package="xhand_adapter",
                executable="xhand_adapter",
                name=f"xhand_adapter_{side}",
                output="screen",
                parameters=[adapter_parameters],
            )
        )

        retarget_share = get_package_share_directory(item.get("retarget_package", "xhand_retargeting"))
        retarget_config = os.path.join(retarget_share, item["retarget_params_file"])
        compatibility = item.get("retarget_compatibility", {})
        actions.append(
            Node(
                package=item.get("retarget_package", "xhand_retargeting"),
                executable="xhand_dex_retargeting_node",
                name=f"xhand_dex_retargeting_{side}",
                output="screen",
                parameters=[{
                    "side": side,
                    "input_topic": "/hand_landmarks",
                    "retarget_config_file": retarget_config,
                    "description_model_id": item.get("description_model_id", f"xhand_{side}"),
                    "preprocess_mode": compatibility.get("preprocess_mode", "legacy_xhand_pinky"),
                    "index_bend_sign": float(compatibility.get("index_bend_right_sign",
                                                                   compatibility.get("index_bend_sign", 1.0))),
                    "joint_lower_limits": lower_limits,
                    "joint_upper_limits": upper_limits,
                    "smoothing_alpha": 0.7,
                    "enable_thumb_fix": True,
                    "require_armed": True,
                    "command_topic": item.get("command_topic", f"/{side}_hand/joint_commands"),
                    "target_topic": f"/{side}_hand/retarget_targets",
                }],
            )
        )

        if mode == "real":
            hardware = profile.values.get("hardware", {}).get("xhand", {}).get(side, {})
            launch_port = LaunchConfiguration(f"{side}_serial_port").perform(context).strip()
            serial_port = launch_port or str(hardware.get("serial_port", "")).strip()
            if not serial_port or serial_port == "configure_on_host":
                raise RuntimeError(
                    f"real XHand mode requires profile hardware.xhand.{side}.serial_port "
                    f"or {side}_serial_port:=/dev/tty..."
                )
            vendor_config = os.path.join(
                get_package_share_directory("xhand_control_ros2"), "config", "xhand_config.yaml"
            )
            actions.append(
                Node(
                    package="xhand_control_ros2",
                    executable="xhand_control_ros2_node",
                    name=f"xhand_control_{side}",
                    namespace=f"{side}_hand/driver",
                    output="screen",
                    parameters=[vendor_config, {"port_name": serial_port, "update_rate": 50.0}],
                )
            )
    if arms:
        actions.append(
            Node(
                package="morla_bringup",
                executable="control_coordinator_node",
                name="morla_control_coordinator",
                output="screen",
                parameters=[{
                    "profile_id": profile_id,
                    "operating_mode": mode,
                }],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile_id", default_value="nero_left_xhand"),
        DeclareLaunchArgument("mode", default_value="sim", description="sim | real (mutually exclusive)"),
        DeclareLaunchArgument("sdk_backend", default_value="fake", description="real mode requires hardware"),
        DeclareLaunchArgument("with_input", default_value="true"),
        DeclareLaunchArgument("allow_legacy_packets", default_value="false"),
        DeclareLaunchArgument("enable_viewer", default_value="false"),
        DeclareLaunchArgument("left_serial_port", default_value=""),
        DeclareLaunchArgument("right_serial_port", default_value=""),
        OpaqueFunction(function=_setup),
    ])
