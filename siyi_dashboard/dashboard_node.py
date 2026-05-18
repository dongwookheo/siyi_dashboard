"""
ROS 2 node + Tornado WebSocket server + static file serving.

Telemetry:
  /joy (sensor_msgs/Joy)                         -> JSON text frame
  PointCloud2 configured by pointcloud_topic     -> compressed JSON text frame

RealSense color/depth flow over WebRTC via MediaMTX (see image_bridge.py).
The browser pulls those via WHEP at :8889/{rgb,depth}/whep; this node
serves the page and the telemetry WebSocket on :8888.
"""

import base64
import json
import math
import struct
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, PointCloud2, PointField

import tornado.ioloop
import tornado.web
import tornado.websocket

from ament_index_python.packages import get_package_share_directory


CLIENTS: "set[tornado.websocket.WebSocketHandler]" = set()

POINT_CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        CLIENTS.add(self)

    def on_close(self):
        CLIENTS.discard(self)

    def check_origin(self, origin):
        # Controller browser hits us from a different host inside the SIYI link.
        return True


def _broadcast(text: str) -> None:
    dead = []
    for c in CLIENTS:
        try:
            c.write_message(text)
        except Exception:
            dead.append(c)
    for c in dead:
        CLIENTS.discard(c)


class DashboardNode(Node):
    def __init__(self, ioloop: tornado.ioloop.IOLoop):
        super().__init__("siyi_dashboard")
        self._ioloop = ioloop
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.get_logger().info("siyi_dashboard subscribed: /joy")

        self._pointcloud_topic = self.declare_parameter(
            "pointcloud_topic", "/camera/camera/depth/color/points"
        ).value
        self._pointcloud_max_points = int(
            self.declare_parameter("pointcloud_max_points", 24576).value
        )
        self._pointcloud_rate_hz = float(
            self.declare_parameter("pointcloud_rate_hz", 6.0).value
        )
        self._pointcloud_last_publish = 0.0

        if self._pointcloud_topic:
            self.create_subscription(
                PointCloud2,
                self._pointcloud_topic,
                self._on_pointcloud,
                POINT_CLOUD_QOS,
            )
            self.get_logger().info(
                "siyi_dashboard subscribed: "
                f"{self._pointcloud_topic} "
                f"(max {self._pointcloud_max_points} pts, "
                f"{self._pointcloud_rate_hz:.1f} Hz)"
            )

    def _on_joy(self, msg: Joy) -> None:
        payload = {
            "topic": "/joy",
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "axes": list(msg.axes),
            "buttons": list(msg.buttons),
        }
        text = json.dumps(payload)
        self._ioloop.add_callback(_broadcast, text)

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        if not CLIENTS:
            return

        now = time.monotonic()
        if self._pointcloud_rate_hz > 0.0:
            interval = 1.0 / self._pointcloud_rate_hz
            if now - self._pointcloud_last_publish < interval:
                return
        self._pointcloud_last_publish = now

        payload = self._compress_pointcloud(msg)
        text = json.dumps(payload, separators=(",", ":"))
        self._ioloop.add_callback(_broadcast, text)

    def _compress_pointcloud(self, msg: PointCloud2) -> dict:
        fields = {field.name: field for field in msg.fields}
        required = ("x", "y", "z")
        missing = [name for name in required if name not in fields]
        bad_types = [
            name
            for name in required
            if name in fields and fields[name].datatype != PointField.FLOAT32
        ]

        base_payload = {
            "topic": "/pointcloud",
            "source_topic": self._pointcloud_topic,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id,
            "width": msg.width,
            "height": msg.height,
        }
        if missing or bad_types:
            base_payload["_error"] = (
                f"PointCloud2 requires float32 x/y/z fields "
                f"(missing={missing}, bad_types={bad_types})"
            )
            return base_payload

        max_points = max(1, self._pointcloud_max_points)
        total_points = int(msg.width) * int(msg.height)
        if total_points <= 0 or msg.point_step <= 0 or msg.row_step <= 0:
            base_payload["_error"] = "PointCloud2 has no readable points"
            return base_payload

        endian = ">" if msg.is_bigendian else "<"
        unpack_float = struct.Struct(endian + "f").unpack_from
        x_offset = fields["x"].offset
        y_offset = fields["y"].offset
        z_offset = fields["z"].offset
        max_offset = max(x_offset, y_offset, z_offset) + 4
        data = memoryview(msg.data)
        data_len = len(data)

        # Deterministic stride sampling avoids random CPU cost and keeps the
        # Android browser's WebGL buffer bounded.
        stride = max(1, math.ceil(total_points / max_points))
        sampled = []
        min_x = min_y = min_z = math.inf
        max_x = max_y = max_z = -math.inf

        for idx in range(0, total_points, stride):
            row = idx // msg.width
            col = idx - row * msg.width
            base = row * msg.row_step + col * msg.point_step
            if base + max_offset > data_len:
                continue

            x = unpack_float(data, base + x_offset)[0]
            y = unpack_float(data, base + y_offset)[0]
            z = unpack_float(data, base + z_offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            sampled.append((x, y, z))
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            min_z = min(min_z, z)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
            max_z = max(max_z, z)
            if len(sampled) >= max_points:
                break

        if not sampled:
            base_payload["_error"] = "PointCloud2 contained no finite x/y/z points"
            return base_payload

        bounds = [min_x, max_x, min_y, max_y, min_z, max_z]

        def encode_axis(value: float, low: float, high: float) -> int:
            if high <= low:
                return 0
            normalized = (value - low) / (high - low)
            return max(0, min(65535, round(normalized * 65535.0)))

        packed = bytearray(len(sampled) * 6)
        for i, (x, y, z) in enumerate(sampled):
            struct.pack_into(
                "<HHH",
                packed,
                i * 6,
                encode_axis(x, min_x, max_x),
                encode_axis(y, min_y, max_y),
                encode_axis(z, min_z, max_z),
            )

        base_payload.update(
            {
                "encoding": "xyz_uint16_base64",
                "bounds": bounds,
                "points": base64.b64encode(packed).decode("ascii"),
                "point_count": len(sampled),
                "original_point_count": total_points,
                "stride": stride,
            }
        )
        return base_payload


def _resolve_web_dir() -> Path:
    share = Path(get_package_share_directory("siyi_dashboard")) / "web"
    if share.exists():
        return share
    # Fallback for `python -m` style invocation before install.
    return Path(__file__).resolve().parent.parent / "web"


def _make_app(web_dir: Path) -> tornado.web.Application:
    return tornado.web.Application(
        [
            (r"/ws", WSHandler),
            (
                r"/(.*)",
                tornado.web.StaticFileHandler,
                {"path": str(web_dir), "default_filename": "index.html"},
            ),
        ]
    )


def main() -> None:
    rclpy.init()

    ioloop = tornado.ioloop.IOLoop.current()
    web_dir = _resolve_web_dir()
    app = _make_app(web_dir)
    app.listen(8888)

    node = DashboardNode(ioloop)
    node.get_logger().info(f"http://0.0.0.0:8888 (web dir: {web_dir})")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        ioloop.start()
    except KeyboardInterrupt:
        pass
    finally:
        ioloop.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
