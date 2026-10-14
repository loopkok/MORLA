"""Pure decision: left Touch gripClick → teleop start vs HITL takeover.

When the policy node is *active* (``/policy_inference/state`` activity ∈
{policy, playback}), the left grip must NOT arm teleop — ``/teleop/start``
would re-arm the arm teleop node and race the policy writing
``joint_commands`` (last-writer-wins, 150Hz teleop vs 30Hz policy). Instead
grip becomes the HITL takeover trigger (``/policy_inference/cmd`` =
"takeover"). Otherwise it keeps starting teleop (re-capture vr_init + arm).

Right-controller A likewise routes: in HUMAN it is "release" (hand control
back to the policy); otherwise the collection stack owns it (``vr_collect_control``).

No ROS here — pure decision, offline-testable.
"""

from __future__ import annotations

from typing import Optional, Sequence

# Policy/playback running: the arm is owned by the policy — grip = takeover.
ACTIVE_ACTIVITIES = frozenset(("policy", "playback"))

# Right Touch primary button (A) index, matching vr_collect_logic.BUTTON_A.
RIGHT_A = 0


def decide_start_action(activity: Optional[str]) -> str:
    """Return ``"takeover"`` when policy/playback is active, else
    ``"teleop_start"``.

    ``activity`` comes from ``/policy_inference/state`` (None when the policy
    node is not running / no state yet).
    """
    if activity in ACTIVE_ACTIVITIES:
        return "takeover"
    return "teleop_start"


def decide_release_action(
    activity: Optional[str],
    prev_buttons: Sequence[int],
    cur_buttons: Sequence[int],
) -> bool:
    """Right A rising edge during HUMAN → release (hand control back to policy).

    Returns True to publish ``/policy_inference/cmd`` = "release". This gate
    lives in the always-running teleop stack so HITL release works during an
    inference session (``vr_collect_control`` only runs with the collection
    stack). ``activity`` = the inference ``activity`` field.
    """
    if activity != "human":
        return False
    prev = list(prev_buttons) + [0] * 6
    cur = list(cur_buttons) + [0] * 6
    return bool(not prev[RIGHT_A] and cur[RIGHT_A])
