from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

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
except Exception as exc:
    YOLO = None
    YOLO_IMPORT_ERROR = exc
else:
    YOLO_IMPORT_ERROR = None


@dataclass
class VisionConfig:
    frame_width: int = 1280
    frame_height: int = 720
    camera_hfov_deg: float = 120.0
    detect_every_n_frames: int = 3
    yolo_model: str = "yolo11n.pt"
    yolo_imgsz: int = 416
    yolo_confidence: float = 0.35
    person_width_m: float = 0.45
    tracker_max_age_s: float = 1.0
    whistle_bearing_sigma_deg: float = 25.0
    face_match_threshold: float = 0.35
    enable_face_reid: bool = True
    yunet_model_path: str = "models/face_detection_yunet_2023mar.onnx"
    sface_model_path: str = "models/face_recognition_sface_2021dec.onnx"


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


class VisionProcessor:
    def __init__(self, config: VisionConfig) -> None:
        self.config = config
        self.detector = None
        self.face_detector = None
        self.face_recognizer = None
        self.face_cosine_metric = 0
        self.tracker = None
        self.tracker_bbox: Optional[tuple[float, float, float, float]] = None
        self.tracker_stamp: Optional[float] = None
        self.last_candidate: Optional[Candidate] = None
        self.active_whistle_doa_deg: Optional[float] = None
        self.active_whistle_epoch: int = -1
        self.face_gallery: deque = deque(maxlen=8)
        self.frame_index = 0
        self.last_error = ""

    @property
    def dependencies_ready(self) -> bool:
        return np is not None and cv2 is not None and YOLO is not None

    def set_active_whistle(self, doa_deg: Optional[float], epoch: Optional[int] = None) -> None:
        if epoch is not None and epoch != self.active_whistle_epoch:
            self.active_whistle_epoch = epoch
            self.reset_tracking()
        self.active_whistle_doa_deg = doa_deg

    def reset_tracking(self) -> None:
        self.tracker = None
        self.tracker_bbox = None
        self.tracker_stamp = None
        self.last_candidate = None
        self.face_gallery.clear()

    def ensure_runtime(self) -> bool:
        if np is None:
            self.last_error = "numpy import failed"
            return False
        if cv2 is None:
            self.last_error = "opencv-python import failed"
            return False
        if YOLO is None:
            detail = f": {YOLO_IMPORT_ERROR}" if YOLO_IMPORT_ERROR else ""
            self.last_error = f"ultralytics import failed{detail}"
            return False

        detector_ok = self._ensure_detector()
        self._ensure_face_models()
        return detector_ok

    def process_frame(self, frame) -> Optional[Candidate]:
        if not self.ensure_runtime():
            return None

        self.frame_index += 1
        tracker_candidate = self._update_tracker(frame)
        selected: Optional[Candidate] = None

        if self.active_whistle_doa_deg is None and tracker_candidate is None and self.last_candidate is None:
            return None

        detect_every = max(1, int(self.config.detect_every_n_frames))
        if self.frame_index % detect_every == 0 or tracker_candidate is None:
            detections = self._detect_people(frame)
            selected = self._select_candidate(frame, detections)

        if selected is None:
            selected = tracker_candidate

        if selected is not None:
            self.last_candidate = selected
            self._initialize_tracker(frame, selected.bbox)
            if selected.feature is not None:
                self.face_gallery.append(selected.feature)

        return selected

    def detect_people_for_debug(self, frame) -> list[Candidate]:
        if not self.ensure_runtime():
            return []
        detections = self._detect_people(frame)
        if self.face_detector is not None and self.face_recognizer is not None:
            self._apply_face_information(frame, detections)
        return detections

    def empty_track_data(self) -> dict:
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

    def candidate_to_track_data(self, candidate: Optional[Candidate]) -> dict:
        if candidate is None:
            return self.empty_track_data()

        x_value, y_value, width_value, height_value = candidate.bbox
        return {
            "visible": True,
            "face_locked": bool(candidate.face_locked),
            "bbox_cx_px": x_value + width_value / 2.0,
            "bbox_cy_px": y_value + height_value / 2.0,
            "bbox_w_px": width_value,
            "bbox_h_px": height_value,
            "image_error_x_norm": float(candidate.image_error_x_norm),
            "bearing_deg": float(candidate.bearing_deg),
            "range_estimate_m": float(candidate.range_estimate_m),
            "detection_confidence": float(candidate.confidence),
            "face_similarity": float(candidate.face_similarity),
        }

    def annotate_frame(
        self,
        frame,
        detections: list[Candidate],
        selected: Optional[Candidate] = None,
        active_whistle_doa_deg: Optional[float] = None,
    ):
        if cv2 is None:
            return frame

        annotated = frame.copy()
        for detection in detections:
            is_selected = selected is not None and self._iou(detection.bbox, selected.bbox) >= 0.7
            color = (0, 255, 0) if is_selected else (0, 180, 255)
            x_value, y_value, width_value, height_value = [int(round(value)) for value in detection.bbox]
            x_value = max(0, x_value)
            y_value = max(0, y_value)
            width_value = max(1, width_value)
            height_value = max(1, height_value)
            cv2.rectangle(
                annotated,
                (x_value, y_value),
                (x_value + width_value, y_value + height_value),
                color,
                2,
            )

            label = (
                f"person {detection.confidence:.2f} "
                f"b={detection.bearing_deg:+.1f} "
                f"r={detection.range_estimate_m:.1f}m"
            )
            if detection.face_similarity > 0.0:
                label += f" face={detection.face_similarity:.2f}"
            cv2.putText(
                annotated,
                label,
                (x_value, max(18, y_value - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )

        header_lines = []
        if active_whistle_doa_deg is None:
            header_lines.append("whistle: none")
        else:
            header_lines.append(f"whistle: {active_whistle_doa_deg:+.1f} deg")
        if selected is None:
            header_lines.append("selected: none")
        else:
            header_lines.append(
                f"selected bearing={selected.bearing_deg:+.1f} deg range={selected.range_estimate_m:.1f} m"
            )

        for index, text in enumerate(header_lines):
            y_value = 24 + (index * 22)
            cv2.putText(
                annotated,
                text,
                (12, y_value),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                text,
                (12, y_value),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )

        return annotated

    def _ensure_detector(self) -> bool:
        if self.detector is not None:
            return True
        try:
            self.detector = YOLO(self.config.yolo_model)
            self.last_error = ""
            return True
        except Exception as exc:
            self.last_error = f"YOLO load failed: {exc}"
            self.detector = None
            return False

    def _ensure_face_models(self) -> None:
        if not self.config.enable_face_reid:
            return
        if self.face_detector is not None and self.face_recognizer is not None:
            return
        try:
            self.face_detector = cv2.FaceDetectorYN.create(
                self.config.yunet_model_path,
                "",
                (320, 320),
            )
            self.face_recognizer = cv2.FaceRecognizerSF.create(
                self.config.sface_model_path,
                "",
            )
            self.face_cosine_metric = getattr(cv2, "FaceRecognizerSF_FR_COSINE", 0)
        except Exception:
            self.face_detector = None
            self.face_recognizer = None

    def _make_tracker(self):
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
        self.tracker_stamp = time.monotonic()

    def _update_tracker(self, frame) -> Optional[Candidate]:
        if self.tracker is None or self.tracker_stamp is None or self.tracker_bbox is None:
            return None
        age = time.monotonic() - self.tracker_stamp
        if age > float(self.config.tracker_max_age_s):
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
        self.tracker_stamp = time.monotonic()
        base = self._candidate_from_bbox(
            self.tracker_bbox,
            self.last_candidate.confidence if self.last_candidate else 0.5,
        )
        if self.last_candidate is not None:
            base.face_similarity = self.last_candidate.face_similarity
            base.face_locked = self.last_candidate.face_locked
        return base

    def _candidate_from_bbox(self, bbox: tuple[float, float, float, float], confidence: float) -> Candidate:
        frame_width = float(self.config.frame_width)
        hfov = math.radians(float(self.config.camera_hfov_deg))
        focal_px = frame_width / (2.0 * math.tan(hfov / 2.0))
        x_value, y_value, width_value, height_value = bbox
        center_x = x_value + width_value / 2.0
        image_error = (center_x - (frame_width / 2.0)) / (frame_width / 2.0)
        bearing_deg = math.degrees(math.atan2(-(center_x - (frame_width / 2.0)), focal_px))
        range_estimate = 0.0
        if width_value > 1.0:
            range_estimate = float(self.config.person_width_m) * focal_px / width_value
        _ = y_value, height_value
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
            imgsz=int(self.config.yolo_imgsz),
            conf=float(self.config.yolo_confidence),
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
        threshold = float(self.config.face_match_threshold)
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
        sigma = float(self.config.whistle_bearing_sigma_deg)
        angle_score = 0.5
        if self.active_whistle_doa_deg is not None:
            diff = self._angle_difference_deg(candidate.bearing_deg, float(self.active_whistle_doa_deg))
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
