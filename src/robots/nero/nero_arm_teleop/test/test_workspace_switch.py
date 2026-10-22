"""Workspace bypass must not disable joint-space protections."""

import numpy as np

from nero_arm_teleop.safety_filter import SafetyFilter


def make_filter(**kwargs):
    return SafetyFilter(-np.ones(7), np.ones(7), **kwargs)


def test_disabled_workspace_preserves_outside_target_and_input():
    safety = make_filter(
        workspace_enabled=False, workspace_radius=0.58,
        workspace_x_min=-0.1, workspace_x_max=0.1,
        workspace_y_min=-0.1, workspace_y_max=0.1,
        workspace_z_min=0.0, workspace_z_max=0.1,
    )
    # Current HOME exceeds the configured radial limit and these box limits.
    target = np.array([-0.5777929249, -0.0235, 0.1885503308])
    result = safety.check_workspace(target)
    np.testing.assert_array_equal(result, target)
    assert not np.shares_memory(result, target)


def test_enabled_workspace_still_projects_sphere_and_box():
    safety = make_filter(
        workspace_enabled=True, workspace_radius=0.5,
        workspace_x_min=-0.2, workspace_x_max=0.2,
        workspace_y_min=-0.1, workspace_y_max=0.1,
        workspace_z_min=0.0, workspace_z_max=0.15,
    )
    np.testing.assert_allclose(
        safety.check_workspace(np.ones(3)), [0.2, 0.1, 0.15]
    )


def test_disabled_workspace_keeps_joint_limits_and_step_limit():
    safety = make_filter(workspace_enabled=False, max_joint_vel=0.05)
    target = np.full(7, 2.0)
    clamped, info = safety.filter(target)
    np.testing.assert_allclose(clamped, np.ones(7))
    assert info["clamped"]
    safety.set_initial_state(np.zeros(7))
    limited, info = safety.filter(target)
    np.testing.assert_allclose(limited, np.full(7, 0.05))
    assert info["clamped"] and info["velocity_limited"]


def test_disabled_workspace_keeps_configured_collision_check():
    safety = make_filter(workspace_enabled=False, collision_check_fn=lambda q: True)
    safety.set_initial_state(np.zeros(7))
    result, info = safety.filter(np.full(7, 0.1))
    np.testing.assert_array_equal(result, np.zeros(7))
    assert info["collision"]
