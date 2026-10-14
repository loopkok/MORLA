#!/usr/bin/env bash
# Install wujihandcpp into ~/.local/opt/wujihandcpp (no sudo).
# Prefer: sudo apt install ./wujihandcpp-*-amd64.deb when possible.
set -euo pipefail

VERSION="${1:-1.8.0}"
PREFIX="${WUJIHANDCPP_PREFIX:-$HOME/.local/opt/wujihandcpp}"
DEB="/tmp/wujihandcpp-${VERSION}-amd64.deb"
URL="https://github.com/wuji-technology/wujihandpy/releases/download/v${VERSION}/wujihandcpp-${VERSION}-amd64.deb"
MIRROR="https://ghfast.top/${URL}"

mkdir -p "$(dirname "$DEB")"
if [[ ! -f "$DEB" ]]; then
  echo "Downloading wujihandcpp ${VERSION}..."
  if ! curl -fL --connect-timeout 20 --max-time 180 -o "$DEB" "$URL"; then
    echo "Direct GitHub failed, trying mirror..."
    curl -fL --connect-timeout 20 --max-time 180 -o "$DEB" "$MIRROR"
  fi
fi

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
dpkg-deb -x "$DEB" "$TMP"
rm -rf "$PREFIX"
mkdir -p "$PREFIX"
cp -a "$TMP/usr/." "$PREFIX/"

echo "Installed to $PREFIX"
echo "Add to your shell before running the driver:"
echo "  export WUJIHANDCPP_PREFIX=\"$PREFIX\""
echo "  export LD_LIBRARY_PATH=\"\$WUJIHANDCPP_PREFIX/lib:\${LD_LIBRARY_PATH:-}\""
