#!/bin/bash
# Install GStreamer modules required by run_camera.sh.
# Idempotent: skips packages already installed, then verifies critical elements.
set -euo pipefail

# Package -> what we use it for
#   gstreamer1.0-tools         gst-launch-1.0, gst-inspect-1.0
#   gstreamer1.0-plugins-base  videoconvert, raw video caps
#   gstreamer1.0-plugins-good  v4l2src (camera capture)
#   gstreamer1.0-plugins-bad   various advanced elements (H.264 helpers, etc.)
#   gstreamer1.0-plugins-ugly  x264enc (H.264 encoder)
#   gstreamer1.0-libav         ffmpeg-based codecs (fallback decode/encode)
#   gstreamer1.0-rtsp          rtspclientsink (RTSP push to MediaMTX)
PACKAGES=(
  gstreamer1.0-tools
  gstreamer1.0-plugins-base
  gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad
  gstreamer1.0-plugins-ugly
  gstreamer1.0-libav
  gstreamer1.0-rtsp
)

# Find missing packages
MISSING=()
for pkg in "${PACKAGES[@]}"; do
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then
    MISSING+=("$pkg")
  fi
done

if [[ ${#MISSING[@]} -eq 0 ]]; then
  echo "All camera dependencies already installed."
else
  echo "Installing: ${MISSING[*]}"
  sudo apt update
  sudo apt install -y "${MISSING[@]}"
fi

# Verify the elements run_camera.sh actually uses
echo
echo "Verifying GStreamer elements:"
ELEMENTS=(v4l2src videoconvert x264enc rtspclientsink)
ALL_OK=true
for el in "${ELEMENTS[@]}"; do
  if gst-inspect-1.0 "$el" >/dev/null 2>&1; then
    printf "  %-20s OK\n" "$el"
  else
    printf "  %-20s MISSING\n" "$el"
    ALL_OK=false
  fi
done

if $ALL_OK; then
  echo
  echo "Done. run_camera.sh is ready to use."
else
  echo
  echo "Some elements still missing — check the package mapping above." >&2
  exit 1
fi
