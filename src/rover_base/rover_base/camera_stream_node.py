from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

try:
    import cv2
except Exception:
    cv2 = None


class CameraStreamNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_stream_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("frame_rate_hz", 15.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("image_topic", "/camera/image/compressed")

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

        image_topic = str(self.get_parameter("image_topic").value)
        self.image_pub = self.create_publisher(CompressedImage, image_topic, image_qos)
        self.ready_pub = self.create_publisher(Bool, "/camera/ready", latch_qos)
        self.cap: Optional[object] = None
        self.ready = False

        period = 1.0 / float(self.get_parameter("frame_rate_hz").value)
        self.create_timer(period, self._tick)

    def _publish_ready(self, ready: bool) -> None:
        if self.ready == ready:
            return
        self.ready = ready
        self.ready_pub.publish(Bool(data=ready))
        self.get_logger().info(f"Camera ready: {ready}")

    def _ensure_camera(self) -> bool:
        if cv2 is None:
            return False
        if self.cap is not None and self.cap.isOpened():
            return True
        self.cap = cv2.VideoCapture(int(self.get_parameter("camera_index").value))
        if not self.cap or not self.cap.isOpened():
            self.cap = None
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("frame_width").value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("frame_height").value))
        return True

    def _tick(self) -> None:
        if not self._ensure_camera():
            self._publish_ready(False)
            return

        ok, frame = self.cap.read()
        if not ok:
            self._publish_ready(False)
            return

        encode_ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.get_parameter("jpeg_quality").value)],
        )
        if not encode_ok:
            self._publish_ready(False)
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.image_pub.publish(msg)
        self._publish_ready(True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
