"""
Bridge ROS sensor_msgs/Image streams to MediaMTX via RTSP push.

Single node, two pipelines:

  /camera/camera/color/image_raw            ──► appsrc ──► x264enc ──► rtspclientsink rtsp://.../rgb
  /camera/camera/depth/image_rect_raw       ──► appsrc ──► x264enc ──► rtspclientsink rtsp://.../depth
  (depth is colorized RGB8 when realsense2_camera is launched with colorizer.enable:=true)

MediaMTX must be running with `rgb` / `depth` paths set to `source: publisher`.
Browser pulls these via WHEP at http://<host>:8889/{rgb,depth}/whep.

Notes
-----
On ROS Jazzy / Ubuntu 24.04, importing `gi` at module load corrupts libffi/
signal-handler state and `rcl_node_init` segfaults. We therefore defer the
`gi` import until *after* `rclpy.init()`. GStreamer bus messages are polled
from a ROS timer rather than via `bus.add_signal_watch()` + a GLib MainLoop
(same root cause).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

# realsense2_camera publishes RELIABLE by default; BEST_EFFORT subscriber is
# compatible and drops backlog under pressure instead of stalling the pipeline.
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ROS sensor_msgs encoding -> GStreamer video/x-raw format.
ENCODING_MAP = {
    "rgb8": "RGB",
    "bgr8": "BGR",
    "mono8": "GRAY8",
    "rgba8": "RGBA",
    "bgra8": "BGRA",
}

# Populated by main() after rclpy.init(). All Gst.* usage below assumes this
# is already imported and Gst.init(None) has been called.
Gst = None  # type: ignore


class _StreamPipeline:
    """One ROS subscription + GStreamer pipeline pushing to a MediaMTX path.

    The pipeline is built lazily on the first Image message so that caps can
    be inlined in `parse_launch` — mirroring the v4l2src pipeline shape that
    is known to work with this MediaMTX setup. Setting caps dynamically on
    appsrc via `set_property` after PLAYING doesn't propagate cleanly to
    rtspclientsink's ANNOUNCE and the stream never registers.
    """

    def __init__(
        self,
        node: Node,
        name: str,
        topic: str,
        rtsp_url: str,
        bitrate_kbps: int,
        fps_hint: int,
    ):
        self._node = node
        self._name = name
        self._topic = topic
        self._rtsp_url = rtsp_url
        self._bitrate_kbps = bitrate_kbps
        self._fps_hint = fps_hint

        self._pipeline = None
        self._appsrc = None
        self._bus = None
        self._last_attempt = 0.0
        self._retry_cooldown_sec = 2.0  # don't reconnect more than every 2s

        node.get_logger().info(
            f"[{name}] subscribed to {topic}; pipeline starts on first frame"
        )
        node.create_subscription(Image, topic, self._on_image, IMAGE_QOS)

    def _build_and_start(self, msg: Image) -> bool:
        fmt = ENCODING_MAP.get(msg.encoding)
        if fmt is None:
            self._node.get_logger().warn(
                f"[{self._name}] unsupported encoding '{msg.encoding}' "
                f"(expected rgb8/bgr8); dropping"
            )
            return False

        # Mirror the working v4l2src pipeline: caps inlined immediately after
        # source, no h264parse, simple knobs.
        desc = (
            f"appsrc name=src is-live=true do-timestamp=true format=time "
            f"caps=video/x-raw,format={fmt},"
            f"width={msg.width},height={msg.height},framerate={self._fps_hint}/1 "
            f"! videoconvert "
            f"! x264enc tune=zerolatency speed-preset=ultrafast "
            f"bitrate={self._bitrate_kbps} "
            f"! rtspclientsink location={self._rtsp_url}"
        )
        self._pipeline = Gst.parse_launch(desc)
        self._appsrc = self._pipeline.get_by_name("src")
        self._bus = self._pipeline.get_bus()

        ret = self._pipeline.set_state(Gst.State.PLAYING)
        self._node.get_logger().info(
            f"[{self._name}] PLAYING -> {self._rtsp_url} "
            f"(caps: {fmt} {msg.width}x{msg.height}@{self._fps_hint}fps, "
            f"state: {ret.value_nick})"
        )
        return True

    def _on_image(self, msg: Image) -> None:
        if self._pipeline is None:
            now = time.monotonic()
            if now - self._last_attempt < self._retry_cooldown_sec:
                return
            self._last_attempt = now
            if not self._build_and_start(msg):
                return

        buf = Gst.Buffer.new_wrapped(bytes(msg.data))
        ret = self._appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            self._node.get_logger().warn(
                f"[{self._name}] push-buffer returned {ret}",
                throttle_duration_sec=2.0,
            )

    def _reset_pipeline(self) -> None:
        """Tear down the pipeline so the next frame triggers a rebuild."""
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
        self._pipeline = None
        self._appsrc = None
        self._bus = None

    def poll_bus(self) -> None:
        """Drain non-blocking bus messages; log errors/warnings."""
        if self._bus is None:
            return
        msg_types = (
            Gst.MessageType.ERROR | Gst.MessageType.WARNING | Gst.MessageType.EOS
        )
        while True:
            msg = self._bus.pop_filtered(msg_types)
            if msg is None:
                return
            if msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                self._node.get_logger().error(
                    f"[{self._name}] gst error: {err.message} ({dbg}); "
                    f"will rebuild pipeline on next frame"
                )
                self._reset_pipeline()
                return
            elif msg.type == Gst.MessageType.WARNING:
                warn, dbg = msg.parse_warning()
                self._node.get_logger().warn(
                    f"[{self._name}] gst warn: {warn.message} ({dbg})"
                )
            elif msg.type == Gst.MessageType.EOS:
                self._node.get_logger().warn(
                    f"[{self._name}] gst EOS; will rebuild pipeline on next frame"
                )
                self._reset_pipeline()
                return

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)


class ImageBridge(Node):
    def __init__(self):
        super().__init__("image_bridge")

        # Load gi/Gst AFTER super().__init__() finishes. If gi is imported
        # before `rcl_node_init` runs, libffi/signal state gets corrupted and
        # rcl segfaults on ROS Jazzy / Ubuntu 24.04. Importing here keeps
        # rcl_node_init's environment clean.
        global Gst
        if Gst is None:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst as _Gst

            _Gst.init(None)
            Gst = _Gst

        rtsp_host = self.declare_parameter("rtsp_host", "localhost").value
        rtsp_port = self.declare_parameter("rtsp_port", 8554).value
        fps_hint = self.declare_parameter("fps_hint", 30).value
        rgb_topic = self.declare_parameter(
            "rgb_topic", "/camera/camera/color/image_raw"
        ).value
        depth_topic = self.declare_parameter(
            "depth_topic", "/camera/camera/depth/image_rect_raw"
        ).value
        rgb_bitrate = self.declare_parameter("rgb_bitrate_kbps", 500).value
        depth_bitrate = self.declare_parameter("depth_bitrate_kbps", 500).value

        base = f"rtsp://{rtsp_host}:{rtsp_port}"
        self._streams = [
            _StreamPipeline(
                self, "rgb", rgb_topic, f"{base}/rgb", rgb_bitrate, fps_hint
            ),
            # _StreamPipeline(
            #     self, "depth", depth_topic, f"{base}/depth", depth_bitrate, fps_hint
            # ),
        ]

        # Drain GStreamer bus messages from rclpy's executor — no GLib MainLoop.
        self.create_timer(0.5, self._poll_all_buses)

    def _poll_all_buses(self) -> None:
        for s in self._streams:
            s.poll_bus()

    def destroy_node(self):
        for s in self._streams:
            s.stop()
        super().destroy_node()


def main() -> None:
    # gi import is intentionally deferred until inside ImageBridge.__init__,
    # AFTER super().__init__() completes. See note there.
    rclpy.init()
    node = ImageBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
