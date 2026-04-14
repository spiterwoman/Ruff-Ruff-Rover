from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from rover_interfaces.msg import TargetTrack, WhistleEvent

try:
    import numpy as np
except Exception:
    np = None

try:
    import cv2
except Exception:
    cv2 = None

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


@dataclass
class Candidate:
    bbox: tuple[float, float, float, float]
    confidence: float
    image_error_x_norm: float
    bearing_deg: float
    range_estimate_m: float
    face_similarity: float
    face_locked: bool
    feature: Optional[object]


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
        self.detector = None
        self.face_detector = None
        self.face_recognizer = None
        self.tracker = None
        self.tracker_bbox: Optional[tuple[float, float, float, float]] = None
        self.tracker_stamp = None
        self.last_candidate: Optional[Candidate] = None
        self.active_whistle: Optional[WhistleEvent] = None
        self.face_gallery: deque = deque(maxlen=8)
        self.ready = False
        self.frame_index = 0
        self.face_cosine_metric = 0
        self.latest_frame = None
        self.latest_frame_stamp = None

        period = 1.0 / float(self.get_parameter("frame_rate_hz").value)
        self.create_timer(period, self._tick)

    def _publish_ready(self, ready: bool) -> None:
        if self.ready == ready:
            return
        self.ready = ready
        self.ready_pub.publish(Bool(data=ready))
        self.get_logger().info(f"Vision ready: {ready}")

    def _whistle_callback(self, msg: WhistleEvent) -> None:
        self.active_whistle = msg
        self.tracker = None
        self.tracker_bbox = None
        self.tracker_stamp = None
        self.last_candidate = None
        self.face_gallery.clear()

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

    def _invisible_track(self) -> TargetTrack:
        msg = TargetTrack()
        msg.visible = False
        msg.face_locked = False
        msg.bbox_cx_px = 0.0
        msg.bbox_cy_px = 0.0
        msg.bbox_w_px = 0.0
        msg.bbox_h_px = 0.0
        msg.image_error_x_norm = 0.0
        msg.bearing_deg = 0.0
        msg.range_estimate_m = 0.0
        msg.detection_confidence = 0.0
        msg.face_similarity = 0.0
        return msg

    def _publish_track(self, candidate: Optional[Candidate]) -> None:
        if candidate is None:
            self.track_pub.publish(self._invisible_track())
            return
        x_value, y_value, width_value, height_value = candidate.bbox
        msg = TargetTrack()
        msg.visible = True
        msg.face_locked = bool(candidate.face_locked)
        msg.bbox_cx_px = x_value + width_value / 2.0
        msg.bbox_cy_px = y_value + height_value / 2.0
        msg.bbox_w_px = width_value
        msg.bbox_h_px = height_value
        msg.image_error_x_norm = float(candidate.image_error_x_norm)
        msg.bearing_deg = float(candidate.bearing_deg)
        msg.range_estimate_m = float(candidate.range_estimate_m)
        msg.detection_confidence = float(candidate.confidence)
        msg.face_similarity = float(candidate.face_similarity)
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

    def _ensure_detector(self) -> bool:
        if YOLO is None:
            return False
        if self.detector is not None:
            return True
        model_path = str(self.get_parameter("yolo_model").value)
        try:
            self.detector = YOLO(model_path)
            return True
        except Exception as exc:
            self.get_logger().error(f"YOLO load failed: {exc}")
            self.detector = None
            return False

    def _ensure_face_models(self) -> None:
        if not bool(self.get_parameter("enable_face_reid").value):
            return
        if cv2 is None:
            return
        if self.face_detector is not None and self.face_recognizer is not None:
            return
        detector_path = str(self.get_parameter("yunet_model_path").value)
        recognizer_path = str(self.get_parameter("sface_model_path").value)
        try:
            self.face_detector = cv2.FaceDetectorYN.create(detector_path, "", (320, 320))
            self.face_recognizer = cv2.FaceRecognizerSF.create(recognizer_path, "")
            self.face_cosine_metric = getattr(cv2, "FaceRecognizerSF_FR_COSINE", 0)
        except Exception:
            self.face_detector = None
            self.face_recognizer = None

    def _runtime_ready(self) -> bool:
        camera_ok = self._have_usable_frame()
        detector_ok = self._ensure_detector()
        self._ensure_face_models()
        ready = camera_ok and detector_ok
        self._publish_ready(ready)
        return ready

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

    def _make_tracker(self):
        if cv2 is None:
            return None
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
            return cv2.legacy.TrackerCSRT_create()
        if hasattr(cv2, "TrackerCSRT_create"):
            return cv2.TrackerCSRT_create()
        return None

    def _initialize_tracker(self, frame, bbox: tuple[float, float, float, float]) -> None:
        tracker = self._make_tracker()
        if tracker is None:
            self.tracker = None
            self.tracker_bbox = None
            self.tracker_stamp = None
            return
        tracker.init(frame, bbox)
        self.tracker = tracker
        self.tracker_bbox = bbox
        self.tracker_stamp = self.get_clock().now()

    def _update_tracker(self, frame) -> Optional[Candidate]:
        if self.tracker is None or self.tracker_stamp is None or self.tracker_bbox is None:
            return None
        age = (self.get_clock().now() - self.tracker_stamp).nanoseconds / 1e9
        if age > float(self.get_parameter("tracker_max_age_s").value):
            self.tracker = None
            self.tracker_bbox = None
            self.tracker_stamp = None
            return None
        ok, bbox = self.tracker.update(frame)
        if not ok:
            self.tracker = None
            self.tracker_bbox = None
            self.tracker_stamp = None
            return None
        self.tracker_bbox = tuple(float(v) for v in bbox)
        self.tracker_stamp = self.get_clock().now()
        base = self._candidate_from_bbox(self.tracker_bbox, self.last_candidate.confidence if self.last_candidate else 0.5)
        if self.last_candidate is not None:
            base.face_similarity = self.last_candidate.face_similarity
            base.face_locked = self.last_candidate.face_locked
        return base

    def _candidate_from_bbox(self, bbox: tuple[float, float, float, float], confidence: float) -> Candidate:
        frame_width = float(self.get_parameter("frame_width").value)
        frame_height = float(self.get_parameter("frame_height").value)
        hfov = math.radians(float(self.get_parameter("camera_hfov_deg").value))
        focal_px = frame_width / (2.0 * math.tan(hfov / 2.0))
        x_value, y_value, width_value, height_value = bbox
        center_x = x_value + width_value / 2.0
        image_error = (center_x - (frame_width / 2.0)) / (frame_width / 2.0)
        bearing_deg = math.degrees(math.atan2(-(center_x - (frame_width / 2.0)), focal_px))
        range_estimate = 0.0
        if width_value > 1.0:
            range_estimate = float(self.get_parameter("person_width_m").value) * focal_px / width_value
        _ = frame_height, height_value
        return Candidate(
            bbox=bbox,
            confidence=float(confidence),
            image_error_x_norm=float(image_error),
            bearing_deg=float(bearing_deg),
            range_estimate_m=float(range_estimate),
            face_similarity=0.0,
            face_locked=False,
            feature=None,
        )

    def _detect_people(self, frame) -> list[Candidate]:
        results = self.detector.predict(
            source=frame,
            imgsz=int(self.get_parameter("yolo_imgsz").value),
            conf=float(self.get_parameter("yolo_confidence").value),
            classes=[0],
            verbose=False,
            device="cpu",
        )
        detections: list[Candidate] = []
        if not results:
            return detections
        boxes = results[0].boxes
        if boxes is None:
            return detections
        for box in boxes:
            xyxy = box.xyxy[0].tolist()
            x1, y1, x2, y2 = [float(value) for value in xyxy]
            bbox = (x1, y1, max(0.0, x2 - x1), max(0.0, y2 - y1))
            confidence = float(box.conf[0].item())
            detections.append(self._candidate_from_bbox(bbox, confidence))
        return detections

    def _detect_faces(self, frame):
        if self.face_detector is None:
            return []
        self.face_detector.setInputSize((frame.shape[1], frame.shape[0]))
        _, faces = self.face_detector.detect(frame)
        if faces is None:
            return []
        return list(faces)

    def _face_inside_bbox(self, face, bbox: tuple[float, float, float, float]) -> bool:
        fx, fy, fw, fh = [float(value) for value in face[:4]]
        bx, by, bw, bh = bbox
        return fx >= bx and fy >= by and (fx + fw) <= (bx + bw) and (fy + fh) <= (by + bh)

    def _extract_face_feature(self, frame, face):
        if self.face_recognizer is None:
            return None
        try:
            aligned = self.face_recognizer.alignCrop(frame, face)
            return self.face_recognizer.feature(aligned)
        except Exception:
            return None

    def _face_similarity(self, feature) -> float:
        if feature is None or not self.face_gallery or self.face_recognizer is None:
            return 0.0
        similarities = [
            float(self.face_recognizer.match(feature, reference, self.face_cosine_metric))
            for reference in self.face_gallery
        ]
        return max(similarities) if similarities else 0.0

    def _apply_face_information(self, frame, detections: list[Candidate]) -> None:
        faces = self._detect_faces(frame)
        if not faces:
            return
        threshold = float(self.get_parameter("face_match_threshold").value)
        for detection in detections:
            matching_faces = [face for face in faces if self._face_inside_bbox(face, detection.bbox)]
            if not matching_faces:
                continue
            feature = self._extract_face_feature(frame, matching_faces[0])
            similarity = self._face_similarity(feature)
            detection.feature = feature
            detection.face_similarity = similarity
            detection.face_locked = similarity >= threshold

    def _angle_difference_deg(self, a_value: float, b_value: float) -> float:
        delta = a_value - b_value
        while delta > 180.0:
            delta -= 360.0
        while delta < -180.0:
            delta += 360.0
        return abs(delta)

    def _iou(self, box_a: tuple[float, float, float, float], box_b: tuple[float, float, float, float]) -> float:
        ax, ay, aw, ah = box_a
        bx, by, bw, bh = box_b
        x_left = max(ax, bx)
        y_top = max(ay, by)
        x_right = min(ax + aw, bx + bw)
        y_bottom = min(ay + ah, by + bh)
        if x_right <= x_left or y_bottom <= y_top:
            return 0.0
        intersection = (x_right - x_left) * (y_bottom - y_top)
        union = aw * ah + bw * bh - intersection
        return intersection / union if union > 0.0 else 0.0

    def _score_candidate(self, candidate: Candidate) -> float:
        sigma = float(self.get_parameter("whistle_bearing_sigma_deg").value)
        angle_score = 0.5
        if self.active_whistle is not None:
            diff = self._angle_difference_deg(candidate.bearing_deg, float(self.active_whistle.doa_deg))
            angle_score = math.exp(-0.5 * (diff / max(sigma, 1.0)) ** 2)
        score = candidate.confidence * 0.6 + angle_score * 0.4
        if candidate.face_similarity > 0.0:
            score += candidate.face_similarity
        if self.tracker_bbox is not None:
            score += 0.25 * self._iou(candidate.bbox, self.tracker_bbox)
        return score

    def _select_candidate(self, frame, detections: list[Candidate]) -> Optional[Candidate]:
        if self.face_detector is not None and self.face_recognizer is not None:
            self._apply_face_information(frame, detections)
        if not detections:
            return None
        return max(detections, key=self._score_candidate)

    def _tick(self) -> None:
        if not self._runtime_ready():
            self._publish_track(None)
            return

        ok, frame = self._read_frame()
        if not ok:
            self._publish_ready(False)
            self._publish_track(None)
            return

        self.frame_index += 1
        tracker_candidate = self._update_tracker(frame)
        selected: Optional[Candidate] = None

        if self.active_whistle is None and tracker_candidate is None and self.last_candidate is None:
            self._publish_track(None)
            return

        if self.frame_index % int(self.get_parameter("detect_every_n_frames").value) == 0 or tracker_candidate is None:
            try:
                detections = self._detect_people(frame)
            except Exception as exc:
                self.get_logger().error(f"YOLO inference failed: {exc}")
                self._publish_track(None)
                return
            selected = self._select_candidate(frame, detections)

        if selected is None:
            selected = tracker_candidate

        if selected is not None:
            self.last_candidate = selected
            self._initialize_tracker(frame, selected.bbox)
            if selected.feature is not None:
                self.face_gallery.append(selected.feature)

        self._publish_track(selected)


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
