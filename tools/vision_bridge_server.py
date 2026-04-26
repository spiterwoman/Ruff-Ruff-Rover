#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import json
import sys
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = REPO_ROOT / "src" / "rover_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))

from rover_vision.vision_core import VisionConfig, VisionProcessor, cv2, np  # noqa: E402


def load_params(params_file: Path) -> dict:
    with params_file.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    return document.get("vision_target_node", {}).get("ros__parameters", {})


def resolve_model_path(path_value: str) -> str:
    candidate = Path(path_value)
    if candidate.is_absolute():
        return str(candidate)
    return str((REPO_ROOT / candidate).resolve())


class VisionBridgeServer:
    def __init__(self, config: VisionConfig) -> None:
        self.processor = VisionProcessor(config)
        self.lock = threading.Lock()

    def infer(self, image_jpeg_b64: str, whistle_epoch: int, whistle_doa_deg) -> dict:
        if np is None or cv2 is None:
            return {
                "ready": False,
                "error": "opencv-python and numpy are required on the Windows host",
                "track": self.processor.empty_track_data(),
            }

        try:
            raw_image = base64.b64decode(image_jpeg_b64.encode("ascii"))
        except Exception as exc:
            return {
                "ready": False,
                "error": f"invalid base64 image payload: {exc}",
                "track": self.processor.empty_track_data(),
            }

        buffer = np.frombuffer(raw_image, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            return {
                "ready": False,
                "error": "failed to decode JPEG image",
                "track": self.processor.empty_track_data(),
            }

        with self.lock:
            self.processor.set_active_whistle(whistle_doa_deg, whistle_epoch)
            ready = self.processor.ensure_runtime()
            if not ready:
                return {
                    "ready": False,
                    "error": self.processor.last_error,
                    "track": self.processor.empty_track_data(),
                }
            try:
                candidate = self.processor.process_frame(frame)
            except Exception as exc:
                return {
                    "ready": False,
                    "error": f"vision processing failed: {exc}",
                    "track": self.processor.empty_track_data(),
                }

        return {
            "ready": True,
            "track": self.processor.candidate_to_track_data(candidate),
        }


class VisionBridgeRequestHandler(BaseHTTPRequestHandler):
    server_version = "RuffRuffVisionBridge/1.0"

    def do_GET(self) -> None:
        if self.path != "/health":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self._send_json(HTTPStatus.OK, {"ok": True})

    def do_POST(self) -> None:
        if self.path != "/infer":
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)
        try:
            payload = json.loads(raw_body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ready": False, "error": f"invalid JSON: {exc}"})
            return

        response = self.server.bridge.infer(  # type: ignore[attr-defined]
            image_jpeg_b64=str(payload.get("image_jpeg_b64", "")),
            whistle_epoch=int(payload.get("whistle_epoch", -1)),
            whistle_doa_deg=payload.get("whistle_doa_deg"),
        )
        self._send_json(HTTPStatus.OK, response)

    def log_message(self, fmt: str, *args) -> None:
        print(fmt % args)

    def _send_json(self, status: HTTPStatus, payload: dict) -> None:
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


def build_config(params: dict) -> VisionConfig:
    return VisionConfig(
        frame_width=int(params.get("frame_width", 1280)),
        frame_height=int(params.get("frame_height", 720)),
        camera_hfov_deg=float(params.get("camera_hfov_deg", 120.0)),
        detect_every_n_frames=int(params.get("detect_every_n_frames", 3)),
        yolo_imgsz=int(params.get("yolo_imgsz", 416)),
        yolo_confidence=float(params.get("yolo_confidence", 0.35)),
        person_width_m=float(params.get("person_width_m", 0.45)),
        tracker_max_age_s=float(params.get("tracker_max_age_s", 1.0)),
        whistle_bearing_sigma_deg=float(params.get("whistle_bearing_sigma_deg", 25.0)),
        face_match_threshold=float(params.get("face_match_threshold", 0.35)),
        enable_face_reid=bool(params.get("enable_face_reid", True)),
        yolo_model=resolve_model_path(str(params.get("yolo_model", "yolo11n.pt"))),
        yunet_model_path=resolve_model_path(
            str(params.get("yunet_model_path", "models/face_detection_yunet_2023mar.onnx"))
        ),
        sface_model_path=resolve_model_path(
            str(params.get("sface_model_path", "models/face_recognition_sface_2021dec.onnx"))
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Windows-side HTTP bridge for Ruff-Ruff-Rover vision.")
    parser.add_argument(
        "--params-file",
        default=str(REPO_ROOT / "src" / "rover_bringup" / "config" / "rover_params.yaml"),
        help="Path to rover_params.yaml",
    )
    parser.add_argument("--host", default="0.0.0.0", help="Bind address for the HTTP server")
    parser.add_argument("--port", default=8765, type=int, help="Port for the HTTP server")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    params = load_params(Path(args.params_file))
    config = build_config(params)

    server = ThreadingHTTPServer((args.host, args.port), VisionBridgeRequestHandler)
    server.bridge = VisionBridgeServer(config)  # type: ignore[attr-defined]
    print(f"Vision bridge server listening on http://{args.host}:{args.port}")
    print("Waiting for Pi requests...")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down vision bridge server.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
