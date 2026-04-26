from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from rover_interfaces.msg import TargetTrack, WhistleEvent
from rover_vision.vision_core import VisionConfig, VisionProcessor, cv2, np


class VisionTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_target_node")

        self.declare_parameter("use_local_camera", False)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("frame_rate_hz", 15.0)
        self.declare_parameter("camera_hfov_deg", 120.0)
        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("image_timeout_s", 1.0)
        self.declare_parameter("detect_every_n_frames", 3)
        self.declare_parameter("yolo_model", "yolo11n.pt")
        self.declare_parameter("yolo_imgsz", 416)
        self.declare_parameter("yolo_confidence", 0.35)
        self.declare_parameter("person_width_m", 0.45)
        self.declare_parameter("tracker_max_age_s", 1.0)
        self.declare_parameter("whistle_bearing_sigma_deg", 25.0)
        self.declare_parameter("face_match_threshold", 0.35)
        self.declare_parameter("enable_face_reid", True)
        self.declare_parameter("yunet_model_path", "models/face_detection_yunet_2023mar.onnx")
        self.declare_parameter("sface_model_path", "models/face_recognition_sface_2021dec.onnx")

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
        self.create_subscription(WhistleEvent, "/whistle/event", self._whistle_callback, latch_qos)
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("image_topic").value),
            self._image_callback,
            image_qos,
        )

        self.cap = None
        self.ready = False
        self.latest_frame = None
        self.latest_frame_stamp = None
        self.whistle_epoch = 0
        self._last_runtime_error = ""
        self.processor = VisionProcessor(
            VisionConfig(
                frame_width=int(self.get_parameter("frame_width").value),
                frame_height=int(self.get_parameter("frame_height").value),
                camera_hfov_deg=float(self.get_parameter("camera_hfov_deg").value),
                detect_every_n_frames=int(self.get_parameter("detect_every_n_frames").value),
                yolo_model=str(self.get_parameter("yolo_model").value),
                yolo_imgsz=int(self.get_parameter("yolo_imgsz").value),
                yolo_confidence=float(self.get_parameter("yolo_confidence").value),
                person_width_m=float(self.get_parameter("person_width_m").value),
                tracker_max_age_s=float(self.get_parameter("tracker_max_age_s").value),
                whistle_bearing_sigma_deg=float(self.get_parameter("whistle_bearing_sigma_deg").value),
                face_match_threshold=float(self.get_parameter("face_match_threshold").value),
                enable_face_reid=bool(self.get_parameter("enable_face_reid").value),
                yunet_model_path=str(self.get_parameter("yunet_model_path").value),
                sface_model_path=str(self.get_parameter("sface_model_path").value),
            )
        )

        period = 1.0 / float(self.get_parameter("frame_rate_hz").value)
        self.create_timer(period, self._tick)

    def _publish_ready(self, ready: bool) -> None:
        if self.ready == ready:
            return
        self.ready = ready
        self.ready_pub.publish(Bool(data=ready))
        self.get_logger().info(f"Vision ready: {ready}")

    def _whistle_callback(self, msg: WhistleEvent) -> None:
        self.whistle_epoch += 1
        self.processor.set_active_whistle(float(msg.doa_deg), self.whistle_epoch)

    def _image_callback(self, msg: CompressedImage) -> None:
        if bool(self.get_parameter("use_local_camera").value):
            return
        if cv2 is None or np is None:
            return
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.latest_frame = frame
        self.latest_frame_stamp = self.get_clock().now()

    def _publish_track(self, track_data: dict) -> None:
        msg = TargetTrack()
        msg.visible = bool(track_data.get("visible", False))
        msg.face_locked = bool(track_data.get("face_locked", False))
        msg.bbox_cx_px = float(track_data.get("bbox_cx_px", 0.0))
        msg.bbox_cy_px = float(track_data.get("bbox_cy_px", 0.0))
        msg.bbox_w_px = float(track_data.get("bbox_w_px", 0.0))
        msg.bbox_h_px = float(track_data.get("bbox_h_px", 0.0))
        msg.image_error_x_norm = float(track_data.get("image_error_x_norm", 0.0))
        msg.bearing_deg = float(track_data.get("bearing_deg", 0.0))
        msg.range_estimate_m = float(track_data.get("range_estimate_m", 0.0))
        msg.detection_confidence = float(track_data.get("detection_confidence", 0.0))
        msg.face_similarity = float(track_data.get("face_similarity", 0.0))
        self.track_pub.publish(msg)

    def _ensure_camera(self) -> bool:
        if not bool(self.get_parameter("use_local_camera").value):
            return False
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

    def _have_usable_frame(self) -> bool:
        if bool(self.get_parameter("use_local_camera").value):
            return self._ensure_camera()
        if self.latest_frame is None or self.latest_frame_stamp is None:
            return False
        age = (self.get_clock().now() - self.latest_frame_stamp).nanoseconds / 1e9
        return age <= float(self.get_parameter("image_timeout_s").value)

    def _read_frame(self):
        if bool(self.get_parameter("use_local_camera").value):
            if self.cap is None:
                return False, None
            return self.cap.read()
        if not self._have_usable_frame():
            return False, None
        return True, self.latest_frame.copy()

    def _runtime_ready(self) -> bool:
        camera_ok = self._have_usable_frame()
        detector_ok = self.processor.ensure_runtime()
        if not detector_ok and self.processor.last_error != self._last_runtime_error:
            self._last_runtime_error = self.processor.last_error
            self.get_logger().error(self.processor.last_error)
        ready = camera_ok and detector_ok
        self._publish_ready(ready)
        return ready

    def _tick(self) -> None:
        if not self._runtime_ready():
            self._publish_track(self.processor.empty_track_data())
            return

        ok, frame = self._read_frame()
        if not ok:
            self._publish_ready(False)
            self._publish_track(self.processor.empty_track_data())
            return

        try:
            selected = self.processor.process_frame(frame)
        except Exception as exc:
            self.get_logger().error(f"Vision processing failed: {exc}")
            self._publish_track(self.processor.empty_track_data())
            return

        self._publish_track(self.processor.candidate_to_track_data(selected))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
