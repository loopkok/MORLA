# XHand retargeting

This package consumes MORLA's unified raw 21-point Quest landmarks at
`/hand_landmarks/{left,right}`. Each node selects one side, applies the XHand
legacy MANO/pinky preprocessing exactly once, resolves its URDF through the
`robot_description` asset manifest, and runs DexPilot.

The node publishes only a named 12-position `sensor_msgs/JointState` command
to `/{side}_hand/joint_commands`. Its visualization output is isolated at
`/{side}_hand/retarget_targets`; it never publishes `joint_states`. The
`xhand_adapter` is the sole owner of standard measured hand state and native
XHand command conversion. Human retargeting is gated by the shared start/armed
signal and `PolicyLease`; it cannot write while policy or playback owns the
hand writer.

The right `index_bend_joint` sign conversion is configured on the retarget
boundary and is not repeated by the adapter. The old direct HOME services,
global arming publisher, and source-local robot models were removed. Launch the
whole robot from `morla_bringup` with one profile and either `mode:=sim` or
`mode:=real`.

The active profile provides all twelve lower/upper limits. DexPilot targets are
bounded to those configured limits before publication; the adapter rejects
external command values outside them.

DexPilot's Python dependency and its Torch backend are deployment dependencies;
the repository test suite separately validates the preprocessing and named
joint mapping even when those optional model packages are unavailable.
