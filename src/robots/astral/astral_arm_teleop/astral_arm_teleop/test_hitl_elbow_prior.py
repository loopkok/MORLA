"""HITL ownership must not inject an absolute human elbow prior into IK."""

import numpy as np

from astral_arm_teleop.astral_arm_teleop_node import AstralTeleopArmNode
from morla_core.ownership import PolicyLease


def test_human_takeover_disables_prior_without_changing_standalone_teleop():
    node = AstralTeleopArmNode.__new__(AstralTeleopArmNode)
    node._use_human_elbow = True
    node._policy_lease = PolicyLease()
    calls = []
    node._human_psi_ref = lambda target: calls.append(target) or 0.42
    target = np.eye(4)

    assert node._control_psi_ref(target) == 0.42
    assert calls == [target]

    assert node._policy_lease.update('{"activity":"human"}')
    assert node._control_psi_ref(target) is None
    assert calls == [target]

    assert node._policy_lease.update('{"activity":null}')
    assert node._control_psi_ref(target) == 0.42
    assert calls == [target, target]

    node._use_human_elbow = False
    assert node._control_psi_ref(target) is None
    assert calls == [target, target]
