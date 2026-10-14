# XHand retargeting dependencies

MORLA keeps the retargeting implementation in the `xhand_retargeting` ROS
package and resolves robot meshes/URDF only through `robot_description`.
Python dependencies for this workstation were validated with:

- `dex_retargeting==0.5.0`
- `torch==2.5.1+cpu` (Python 3.10 x86-64 smoke-test environment)
- `pin==3.8.0`

The CPU Torch wheel was installed into the invoking user's Python 3.10 site
directory to execute DexPilot tests. It is not a universal deployment lock:
select the appropriate CPU, CUDA, or target-architecture Torch wheel on each
deployment host. `xhand_control_ros2` currently contains an x86-64
`libxhand_control.so`; the XHand real driver is not deployable on ARM64 until a
matching licensed vendor SDK library is supplied and its ABI/dependencies are
verified. The copied vendor SDK contains a user agreement and the ROS package
license field is unresolved; preserve those terms and obtain distribution
approval before redistributing the vendor files.

The retargeting node is model-backed and fails startup if DexPilot cannot load
its configured model. Pure preprocessing, name/sign mapping, adapter contracts,
and MuJoCo tests remain runnable independently of Torch.
