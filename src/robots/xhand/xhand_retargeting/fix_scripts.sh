#!/bin/bash
# Workaround for colcon-ros bug: scripts install to bin/ instead of lib/<pkg>/
# Run this after every colcon build that includes xhand_retargeting.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="$(dirname "$(dirname "$SCRIPT_DIR")")"
INSTALL_DIR="$WORKSPACE/install/xhand_retargeting"

if [ ! -d "$INSTALL_DIR/bin" ]; then
    echo "xhand_retargeting not installed. Run colcon build first."
    exit 1
fi

mkdir -p "$INSTALL_DIR/lib/xhand_retargeting"
for exe in "$INSTALL_DIR/bin/"*; do
    name=$(basename "$exe")
    cp "$exe" "$INSTALL_DIR/lib/xhand_retargeting/$name"
done

echo "Fixed: scripts copied to lib/xhand_retargeting/"
echo "Now run: source $WORKSPACE/install/setup.bash"
