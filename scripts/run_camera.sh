#!/bin/bash
# Publish camera to local MediaMTX via GStreamer (RTSP push).
# All knobs configurable via env vars; defaults match the original siyi_monitoring setup.
set -euo pipefail

DEVICE="${DEVICE:-/dev/video4}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-30}"
BITRATE_KBPS="${BITRATE_KBPS:-2000}"
STREAM="${STREAM:-mystream}"
RTSP_HOST="${RTSP_HOST:-localhost}"
RTSP_PORT="${RTSP_PORT:-8554}"

if [[ ! -e "$DEVICE" ]]; then
  echo "Camera device not found: $DEVICE"
  echo "Available devices:"
  ls /dev/video* 2>/dev/null || echo "  (none)"
  exit 1
fi

if ! command -v gst-launch-1.0 >/dev/null 2>&1 || ! gst-inspect-1.0 rtspclientsink >/dev/null 2>&1; then
  echo "Required GStreamer elements missing. Run:"
  echo "  $(dirname "${BASH_SOURCE[0]}")/install_camera_deps.sh"
  exit 1
fi

URL="rtsp://${RTSP_HOST}:${RTSP_PORT}/${STREAM}"
echo "Publishing $DEVICE @ ${WIDTH}x${HEIGHT}@${FPS}fps ${BITRATE_KBPS}kbps -> $URL"

exec gst-launch-1.0 v4l2src device="$DEVICE" \
  ! video/x-raw,width="$WIDTH",height="$HEIGHT",framerate="${FPS}/1" \
  ! videoconvert \
  ! x264enc tune=zerolatency bitrate="$BITRATE_KBPS" speed-preset=ultrafast key-int-max="$FPS" \
  ! rtspclientsink location="$URL"
