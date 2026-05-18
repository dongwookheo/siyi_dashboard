#!/bin/bash
# Convenience launcher for the bundled MediaMTX install.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
INSTALL_DIR="$PKG_DIR/thirdparty/mediamtx"

if [[ ! -x "$INSTALL_DIR/mediamtx" ]]; then
  echo "MediaMTX binary not found at $INSTALL_DIR/mediamtx"
  echo "Run install first:  $SCRIPT_DIR/install_mediamtx.sh"
  exit 1
fi

cd "$INSTALL_DIR"
exec ./mediamtx mediamtx.yml
