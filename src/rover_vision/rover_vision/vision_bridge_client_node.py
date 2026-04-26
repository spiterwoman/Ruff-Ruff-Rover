from __future__ import annotations

import base64
import json
from urllib import error as urllib_error
from urllib import request as urllib_request

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from rover_interfaces.msg import TargetTrack, WhistleEvent


class VisionBridgeClientNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_bridge_client_node")

        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("frame_rate_hz", 5.0)
        self.declare_parameter("image_timeout_s", 1.0)
        self.declare_parameter("request_timeout_s", 2.0)
        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 8765)
        self.declare_parameter("server_path", "/infer")
        self.declare_parameter("status_log_period_s", 2.0)

        latch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.track_pub = self.create_publisher(TargetTrack, "/target_track", 10)
        self.ready_pub = self.create_publisher(Bool, "/vision/ready", latch_qos)
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("image_topic").value),
            self._image_callback,
            image_qos,
        )
        self.create_subscription(WhistleEvent, "/whistle/event", self._whistle_callback, latch_qos)

        self.latest_image: bytes | None = None
        self.latest_image_stamp = None
        self.whistle_doa_deg: float | None = None
        self.whistle_epoch = 0
        self.ready = False
        self.last_status_log_at = self.get_clock().now()
        self.last_error = ""

        period = 1.0 / max(0.1, float(self.get_parameter("frame_rate_hz").value))
        self.create_timer(period, self._tick)

    def _publish_ready(self, ready: bool) -> None:
        if self.ready == ready:
            return
        self.ready = ready
        self.ready_pub.publish(Bool(data=ready))
        self.get_logger().info(f"Vision bridge ready: {ready}")

    def _publish_track(self, payload: dict) -> None:
        msg = TargetTrack()
        msg.visible = bool(payload.get("visible", False))
        msg.face_locked = bool(payload.get("face_locked", False))
        msg.bbox_cx_px = float(payload.get("bbox_cx_px", 0.0))
        msg.bbox_cy_px = float(payload.get("bbox_cy_px", 0.0))
        msg.bbox_w_px = float(payload.get("bbox_w_px", 0.0))
        msg.bbox_h_px = float(payload.get("bbox_h_px", 0.0))
        msg.image_error_x_norm = float(payload.get("image_error_x_norm", 0.0))
        msg.bearing_deg = float(payload.get("bearing_deg", 0.0))
        msg.range_estimate_m = float(payload.get("range_estimate_m", 0.0))
        msg.detection_confidence = float(payload.get("detection_confidence", 0.0))
        msg.face_similarity = float(payload.get("face_similarity", 0.0))
        self.track_pub.publish(msg)

    def _image_callback(self, msg: CompressedImage) -> None:
        self.latest_image = bytes(msg.data)
        self.latest_image_stamp = self.get_clock().now()

    def _whistle_callback(self, msg: WhistleEvent) -> None:
        self.whistle_epoch += 1
        self.whistle_doa_deg = float(msg.doa_deg)

    def _latest_image_is_fresh(self) -> bool:
        if self.latest_image is None or self.latest_image_stamp is None:
            return False
        age = (self.get_clock().now() - self.latest_image_stamp).nanoseconds / 1e9
        return age <= float(self.get_parameter("image_timeout_s").value)

    def _empty_track(self) -> dict:
        return {
            "visible": False,
            "face_locked": False,
            "bbox_cx_px": 0.0,
            "bbox_cy_px": 0.0,
            "bbox_w_px": 0.0,
            "bbox_h_px": 0.0,
            "image_error_x_norm": 0.0,
            "bearing_deg": 0.0,
            "range_estimate_m": 0.0,
            "detection_confidence": 0.0,
            "face_similarity": 0.0,
        }

    def _server_url(self) -> str:
        host = str(self.get_parameter("server_host").value).strip()
        port = int(self.get_parameter("server_port").value)
        path = str(self.get_parameter("server_path").value).strip() or "/infer"
        if not path.startswith("/"):
            path = f"/{path}"
        return f"http://{host}:{port}{path}"

    def _log_status(self, message: str) -> None:
        period = float(self.get_parameter("status_log_period_s").value)
        if period <= 0.0:
            return
        now = self.get_clock().now()
        since_last = (now - self.last_status_log_at).nanoseconds / 1e9
        if since_last < period and message == self.last_error:
            return
        self.last_status_log_at = now
        self.last_error = message
        self.get_logger().warning(message)

    def _tick(self) -> None:
        if not self._latest_image_is_fresh():
            self._publish_ready(False)
            self._publish_track(self._empty_track())
            return

        payload = {
            "image_jpeg_b64": base64.b64encode(self.latest_image).decode("ascii"),
            "whistle_epoch": self.whistle_epoch,
            "whistle_doa_deg": self.whistle_doa_deg,
        }
        encoded = json.dumps(payload).encode("utf-8")
        request = urllib_request.Request(
            self._server_url(),
            data=encoded,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib_request.urlopen(
                request,
                timeout=float(self.get_parameter("request_timeout_s").value),
            ) as response:
                body = response.read().decode("utf-8")
        except (urllib_error.URLError, TimeoutError, ConnectionError) as exc:
            self._publish_ready(False)
            self._publish_track(self._empty_track())
            self._log_status(f"Vision bridge request failed: {exc}")
            return

        try:
            result = json.loads(body)
        except json.JSONDecodeError as exc:
            self._publish_ready(False)
            self._publish_track(self._empty_track())
            self._log_status(f"Vision bridge returned invalid JSON: {exc}")
            return

        ready = bool(result.get("ready", False))
        self._publish_ready(ready)
        self._publish_track(result.get("track", self._empty_track()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionBridgeClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
