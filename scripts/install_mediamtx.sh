#!/bin/bash
# Download and install MediaMTX into siyi_dashboard/thirdparty/mediamtx/.
# Mirrors the original siyi_monitoring install pattern, but resolves paths
# relative to this script's location (works regardless of cwd).
set -euo pipefail

MEDIAMTX_VERSION="${MEDIAMTX_VERSION:-v1.18.1}"
TARBALL="mediamtx_${MEDIAMTX_VERSION}_linux_amd64.tar.gz"
URL="https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/${TARBALL}"

# Resolve install dir from script location, not pwd.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
INSTALL_DIR="$PKG_DIR/thirdparty/mediamtx"

# Check wget
if ! command -v wget >/dev/null 2>&1; then
  echo "Installing wget..."
  sudo apt update
  sudo apt install -y wget
  echo "wget installed."
fi
printf "wget version: %s\n" "$(wget --version | head -n 1)"

mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR/.."

echo "Downloading MediaMTX ${MEDIAMTX_VERSION}..."
wget -O "$TARBALL" "$URL"

echo "Extracting MediaMTX..."
tar -xzf "$TARBALL" -C "$INSTALL_DIR"

echo "Cleaning up tarball..."
rm -f "$TARBALL"

echo -n "MediaMTX version: "
"$INSTALL_DIR/mediamtx" --version 2>/dev/null \
  || "$INSTALL_DIR/mediamtx" -version 2>/dev/null \
  || echo "(unknown)"

# Generate mediamtx.yml
"$SCRIPT_DIR/write_mediamtx_config.sh" "$INSTALL_DIR"

echo "Done. Binary: $INSTALL_DIR/mediamtx"
echo "Run with:    $SCRIPT_DIR/run_mediamtx.sh"
