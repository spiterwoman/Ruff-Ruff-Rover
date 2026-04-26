from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
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
        self.declare_parameter("auto_exposure_mode", 3.0)
        self.declare_parameter("exposure", -1.0)
        self.declare_parameter("gain", -1.0)
        self.declare_parameter("brightness", -1.0)
        self.declare_parameter("contrast", -1.0)
        self.declare_parameter("saturation", -1.0)
        self.declare_parameter("sharpness", -1.0)
        self.declare_parameter("gamma", -1.0)
        self.declare_parameter("backlight_compensation", -1.0)
        self.declare_parameter("auto_white_balance", True)
        self.declare_parameter("white_balance_temperature", -1.0)
        self.declare_parameter("auto_focus", True)
        self.declare_parameter("focus", -1.0)
        self.declare_parameter("log_camera_settings_on_apply", True)

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
        self.camera_settings_dirty = True
        self._last_settings_signature = None
        self.add_on_set_parameters_callback(self._on_parameters_changed)

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
            self._apply_camera_settings()
            return True
        self.cap = cv2.VideoCapture(int(self.get_parameter("camera_index").value))
        if not self.cap or not self.cap.isOpened():
            self.cap = None
            return False
        self.camera_settings_dirty = True
        self._apply_camera_settings(force=True)
        return True

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        positive_int_names = {"frame_width", "frame_height"}
        positive_float_names = {"frame_rate_hz"}
        live_control_names = {
            "frame_width",
            "frame_height",
            "auto_exposure_mode",
            "exposure",
            "gain",
            "brightness",
            "contrast",
            "saturation",
            "sharpness",
            "gamma",
            "backlight_compensation",
            "auto_white_balance",
            "white_balance_temperature",
            "auto_focus",
            "focus",
            "log_camera_settings_on_apply",
        }

        for parameter in parameters:
            if parameter.name in positive_int_names and int(parameter.value) <= 0:
                return SetParametersResult(successful=False, reason=f"{parameter.name} must be > 0")
            if parameter.name == "camera_index" and int(parameter.value) < 0:
                return SetParametersResult(successful=False, reason="camera_index must be >= 0")
            if parameter.name == "jpeg_quality" and not 1 <= int(parameter.value) <= 100:
                return SetParametersResult(successful=False, reason="jpeg_quality must be between 1 and 100")
            if parameter.name in positive_float_names and float(parameter.value) <= 0.0:
                return SetParametersResult(successful=False, reason=f"{parameter.name} must be > 0")

        if any(parameter.name in live_control_names for parameter in parameters):
            self.camera_settings_dirty = True

        return SetParametersResult(successful=True)

    def _camera_settings_signature(self):
        return (
            int(self.get_parameter("frame_width").value),
            int(self.get_parameter("frame_height").value),
            float(self.get_parameter("auto_exposure_mode").value),
            float(self.get_parameter("exposure").value),
            float(self.get_parameter("gain").value),
            float(self.get_parameter("brightness").value),
            float(self.get_parameter("contrast").value),
            float(self.get_parameter("saturation").value),
            float(self.get_parameter("sharpness").value),
            float(self.get_parameter("gamma").value),
            float(self.get_parameter("backlight_compensation").value),
            bool(self.get_parameter("auto_white_balance").value),
            float(self.get_parameter("white_balance_temperature").value),
            bool(self.get_parameter("auto_focus").value),
            float(self.get_parameter("focus").value),
        )

    def _set_cap_prop(self, name: str, prop_name: str, value: float) -> None:
        if self.cap is None or cv2 is None or not hasattr(cv2, prop_name):
            return
        prop_id = getattr(cv2, prop_name)
        ok = self.cap.set(prop_id, float(value))
        if not ok:
            self.get_logger().warning(f"Camera did not accept {name}={value}")

    def _get_cap_prop(self, prop_name: str) -> Optional[float]:
        if self.cap is None or cv2 is None or not hasattr(cv2, prop_name):
            return None
        prop_id = getattr(cv2, prop_name)
        try:
            value = float(self.cap.get(prop_id))
        except Exception:
            return None
        if math.isnan(value):
            return None
        return value

    def _apply_optional_prop(self, name: str, prop_name: str, value: float) -> None:
        if value < 0.0:
            return
        self._set_cap_prop(name, prop_name, value)

    def _apply_camera_settings(self, force: bool = False) -> None:
        if self.cap is None or cv2 is None or not self.cap.isOpened():
            return

        signature = self._camera_settings_signature()
        if not force and not self.camera_settings_dirty and signature == self._last_settings_signature:
            return

        self._set_cap_prop("frame_width", "CAP_PROP_FRAME_WIDTH", int(self.get_parameter("frame_width").value))
        self._set_cap_prop("frame_height", "CAP_PROP_FRAME_HEIGHT", int(self.get_parameter("frame_height").value))
        self._set_cap_prop(
            "auto_exposure_mode",
            "CAP_PROP_AUTO_EXPOSURE",
            float(self.get_parameter("auto_exposure_mode").value),
        )
        self._apply_optional_prop("exposure", "CAP_PROP_EXPOSURE", float(self.get_parameter("exposure").value))
        self._apply_optional_prop("gain", "CAP_PROP_GAIN", float(self.get_parameter("gain").value))
        self._apply_optional_prop("brightness", "CAP_PROP_BRIGHTNESS", float(self.get_parameter("brightness").value))
        self._apply_optional_prop("contrast", "CAP_PROP_CONTRAST", float(self.get_parameter("contrast").value))
        self._apply_optional_prop("saturation", "CAP_PROP_SATURATION", float(self.get_parameter("saturation").value))
        self._apply_optional_prop("sharpness", "CAP_PROP_SHARPNESS", float(self.get_parameter("sharpness").value))
        self._apply_optional_prop("gamma", "CAP_PROP_GAMMA", float(self.get_parameter("gamma").value))
        self._apply_optional_prop(
            "backlight_compensation",
            "CAP_PROP_BACKLIGHT",
            float(self.get_parameter("backlight_compensation").value),
        )
        self._set_cap_prop(
            "auto_white_balance",
            "CAP_PROP_AUTO_WB",
            1.0 if bool(self.get_parameter("auto_white_balance").value) else 0.0,
        )
        self._apply_optional_prop(
            "white_balance_temperature",
            "CAP_PROP_WB_TEMPERATURE",
            float(self.get_parameter("white_balance_temperature").value),
        )
        self._set_cap_prop(
            "auto_focus",
            "CAP_PROP_AUTOFOCUS",
            1.0 if bool(self.get_parameter("auto_focus").value) else 0.0,
        )
        self._apply_optional_prop("focus", "CAP_PROP_FOCUS", float(self.get_parameter("focus").value))

        self._last_settings_signature = signature
        self.camera_settings_dirty = False

        if bool(self.get_parameter("log_camera_settings_on_apply").value):
            self.get_logger().info(
                "Camera settings applied: size=%sx%s auto_exposure=%.2f exposure=%s gain=%s brightness=%s contrast=%s saturation=%s sharpness=%s gamma=%s backlight=%s auto_wb=%s wb_temp=%s auto_focus=%s focus=%s"
                % (
                    int(self.get_parameter("frame_width").value),
                    int(self.get_parameter("frame_height").value),
                    float(self.get_parameter("auto_exposure_mode").value),
                    self._get_cap_prop("CAP_PROP_EXPOSURE"),
                    self._get_cap_prop("CAP_PROP_GAIN"),
                    self._get_cap_prop("CAP_PROP_BRIGHTNESS"),
                    self._get_cap_prop("CAP_PROP_CONTRAST"),
                    self._get_cap_prop("CAP_PROP_SATURATION"),
                    self._get_cap_prop("CAP_PROP_SHARPNESS"),
                    self._get_cap_prop("CAP_PROP_GAMMA"),
                    self._get_cap_prop("CAP_PROP_BACKLIGHT"),
                    self._get_cap_prop("CAP_PROP_AUTO_WB"),
                    self._get_cap_prop("CAP_PROP_WB_TEMPERATURE"),
                    self._get_cap_prop("CAP_PROP_AUTOFOCUS"),
                    self._get_cap_prop("CAP_PROP_FOCUS"),
                )
            )

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
