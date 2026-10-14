"""Profile-based selection of the existing robot-specific launch assemblies."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RobotStack:
    package: str
    launch_file: str
    arguments: dict[str, str]


def select_robot_stack(profile: Any, mode: str, sdk_backend: str = "fake") -> RobotStack:
    """Resolve one validated profile to its robot-specific launch assembly.

    This function is ROS-free so the Web profile catalog and the CLI launch
    entrypoint make the same capability decision before starting any nodes.
    """
    mode = str(mode).strip().lower()
    sdk_backend = str(sdk_backend).strip().lower()
    if mode not in {"sim", "real"}:
        raise ValueError("mode must be sim or real")
    if sdk_backend not in {"fake", "hardware"}:
        raise ValueError("sdk_backend must be fake or hardware")
    if mode == "sim" and sdk_backend != "fake":
        raise ValueError("mode=sim requires sdk_backend=fake")
    if mode == "real" and sdk_backend != "hardware":
        raise ValueError("mode=real requires sdk_backend=hardware")

    components = [
        row for row in profile.values.get("components", []) if row.get("enabled")
    ]
    if profile.robot_id == "nero":
        arms = [row for row in components if row.get("type") == "nero_arm"]
        hands = [row for row in components if row.get("type") == "xhand"]
        unsupported = [row for row in components if row.get("type") not in {"nero_arm", "xhand"}]
        if not arms:
            raise ValueError(f"Nero profile {profile.profile_id!r} has no enabled arm")
        if unsupported:
            raise ValueError(
                f"Nero profile {profile.profile_id!r} has unsupported components: "
                f"{[row.get('component_id') for row in unsupported]}"
            )
        if hands:
            return RobotStack(
                "morla_bringup",
                "nero_xhand_stack.launch.py",
                {
                    "profile_id": profile.profile_id,
                    "mode": mode,
                    "sdk_backend": sdk_backend,
                },
            )
        return RobotStack(
            "morla_bringup",
            "nero_arm_stack.launch.py",
            {"profile_id": profile.profile_id, "sdk_backend": sdk_backend},
        )

    if profile.robot_id != "astral":
        raise ValueError(f"unsupported robot_id={profile.robot_id!r}")

    arms = [row for row in components if row.get("type") == "arm"]
    grippers = [row for row in components if row.get("type") == "gripper"]
    hands = [row for row in components if row.get("type") == "wuji_hand"]
    unsupported = [
        row for row in components
        if row.get("type") not in {"arm", "gripper", "wuji_hand"}
    ]
    if not arms:
        raise ValueError(f"Astral profile {profile.profile_id!r} has no enabled arm")
    if unsupported:
        raise ValueError(
            f"Astral profile {profile.profile_id!r} has unsupported components: "
            f"{[row.get('component_id') for row in unsupported]}"
        )
    sides = sorted({str(row["side"]) for row in arms})
    if not set(sides) <= {"left", "right"}:
        raise ValueError(f"Astral profile {profile.profile_id!r} has invalid arm sides {sides}")
    arm_side = "both" if sides == ["left", "right"] else sides[0]
    gripper_sides = sorted({str(row["side"]) for row in grippers})
    wuji_sides = sorted({str(row["side"]) for row in hands})

    if mode == "sim":
        return RobotStack(
            "astral_mujoco_sim",
            "astral_sim_pipeline.launch.py",
            {
                "control_profile_id": profile.profile_id,
                "arm_side": arm_side,
                "gripper_sides": ",".join(gripper_sides),
            },
        )

    if any(side != "right" for side in wuji_sides) or len(wuji_sides) > 1:
        raise ValueError("Astral real launch currently supports one right-side Wuji hand")
    right_source = (
        "quest3" if "right" in wuji_sides
        else "gripper" if "right" in gripper_sides
        else "none"
    )
    return RobotStack(
        "morla_bringup",
        "full_teleop.launch.py",
        {
            "control_profile_id": profile.profile_id,
            "arm_side": arm_side,
            "with_arm_driver": "true",
            "with_gripper": "true" if "left" in gripper_sides else "false",
            "with_hand_driver": "true" if "right" in wuji_sides else "false",
            "right_hand_source": right_source,
        },
    )
