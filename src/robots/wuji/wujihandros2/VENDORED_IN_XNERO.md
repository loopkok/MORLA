# Vendored into astral_ws

This tree is the upstream [wujihandros2](https://github.com/wuji-technology/wujihandros2) driver stack,
placed under `astral_ws/src/robots/wuji/wujihandros2` so `colcon` builds
`wujihand_msgs`, `wujihand_driver`, and `wujihand_bringup` with the rest of the workspace.

Local adaptations:
- `wujihand_driver/CMakeLists.txt` also searches `$WUJIHANDCPP_PREFIX` / `~/.local/opt/wujihandcpp`
- `external/wuji-description` may be a local copy of wuji-description (`hand/body` = `wuji_description`); currently unused if empty
- `wujihandcpp` and `wuji_description` have no rosdep keys, so they are not declared as
  package.xml dependencies (rosdep cannot resolve them). `wujihandcpp` is found via
  `WUJIHANDCPP_PREFIX`, `~/.local/opt/wujihandcpp`, or the in-tree source layout;
  `wuji_description` must be installed from the vendor and is required by the launch
  files. Install them per the vendor instructions before launching the bringup.
