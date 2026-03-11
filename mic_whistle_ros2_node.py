from __future__ import annotations

import json
import threading
from dataclasses import asdict

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, String
except Exception as exc:
    raise SystemExit("Run this file inside a ROS 2 environment with rclpy + std_msgs installed.") from exc

try:
    from .whistle_detector import AudioConfig, ReSpeakerWhistleTracker, WhistleConfig
except ImportError:
    from whistle_detector import AudioConfig, ReSpeakerWhistleTracker, WhistleConfig


class MicWhistleNode(Node):
    def __init__(self) -> None:
        super().__init__("mic_whistle_node")

        # Audio / firmware parameters
        self.declare_parameter("device_index", -1)
        self.declare_parameter("rate", 16000)
        self.declare_parameter("chunk", 1024)
        self.declare_parameter("firmware_channels", 6)
        self.declare_parameter("detect_channel", 0)

        # Whistle detector parameters
        self.declare_parameter("band_min_hz", 1800.0)
        self.declare_parameter("band_max_hz", 4500.0)
        self.declare_parameter("min_rms", 700.0)
        self.declare_parameter("min_band_ratio", 0.45)
        self.declare_parameter("min_peak_ratio", 0.18)
        self.declare_parameter("min_prominence_db", 12.0)
        self.declare_parameter("consecutive_hits_required", 3)
        self.declare_parameter("cooldown_s", 1.0)

        # ReSpeaker-specific features
        self.declare_parameter("use_usb_control", True)
        self.declare_parameter("use_led_ring", False)
        self.declare_parameter("doa_samples", 5)
        self.declare_parameter("doa_sample_delay_s", 0.03)

        self.detected_pub = self.create_publisher(Bool, "/whistle/detected", 10)
        self.angle_pub = self.create_publisher(Float32, "/whistle/angle_deg", 10)
        self.freq_pub = self.create_publisher(Float32, "/whistle/peak_freq_hz", 10)
        self.event_pub = self.create_publisher(String, "/whistle/event_json", 10)

        self.get_logger().info("Mic whistle node starting...")
        self._thread = threading.Thread(target=self._run_tracker, daemon=True)
        self._thread.start()

    def _publish_event(self, event_dict: dict) -> None:
        self.detected_pub.publish(Bool(data=True))

        doa = event_dict.get("doa_angle_deg")
        if doa is not None:
            self.angle_pub.publish(Float32(data=float(doa)))

        peak_freq = event_dict.get("peak_freq_hz")
        if peak_freq is not None:
            self.freq_pub.publish(Float32(data=float(peak_freq)))

        self.event_pub.publish(String(data=json.dumps(event_dict)))
        self.get_logger().info(f"Whistle detected: {event_dict}")

    def _run_tracker(self) -> None:
        device_index = int(self.get_parameter("device_index").value)
        if device_index < 0:
            device_index = None

        audio_cfg = AudioConfig(
            rate=int(self.get_parameter("rate").value),
            chunk=int(self.get_parameter("chunk").value),
            firmware_channels=int(self.get_parameter("firmware_channels").value),
            detect_channel=int(self.get_parameter("detect_channel").value),
            input_device_index=device_index,
        )

        whistle_cfg = WhistleConfig(
            band_min_hz=float(self.get_parameter("band_min_hz").value),
            band_max_hz=float(self.get_parameter("band_max_hz").value),
            min_rms=float(self.get_parameter("min_rms").value),
            min_band_ratio=float(self.get_parameter("min_band_ratio").value),
            min_peak_ratio=float(self.get_parameter("min_peak_ratio").value),
            min_prominence_db=float(self.get_parameter("min_prominence_db").value),
            consecutive_hits_required=int(self.get_parameter("consecutive_hits_required").value),
            cooldown_s=float(self.get_parameter("cooldown_s").value),
            doa_samples=int(self.get_parameter("doa_samples").value),
            doa_sample_delay_s=float(self.get_parameter("doa_sample_delay_s").value),
            use_respeaker_usb=bool(self.get_parameter("use_usb_control").value),
            use_led_ring=bool(self.get_parameter("use_led_ring").value),
        )

        tracker = ReSpeakerWhistleTracker(audio_cfg, whistle_cfg)
        tracker.run(lambda event: self._publish_event(asdict(event)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MicWhistleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()