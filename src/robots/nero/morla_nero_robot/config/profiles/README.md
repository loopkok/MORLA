# Nero profile registry

The registered layouts are single-side XHand (19), dual-arm only (14), and
dual-arm plus dual XHand (38). `nero_dual_xhand` is frozen in this order:
left arm, right arm, left hand, right hand. Full dual-XHand inference requires a
model declaring the matching 38-dimensional layout; the standard 32-dimensional
pi05 layout cannot be used.

Nero arm limits, initialization poses, CAN names, and the Quest-to-arm matrix
are copied from the source IK/configuration contract. Hardware confirmation is
tracked separately and is not implied by profile validation.
