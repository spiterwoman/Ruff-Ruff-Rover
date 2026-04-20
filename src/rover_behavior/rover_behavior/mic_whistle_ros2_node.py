from __future__ import annotations

import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
except Exception as exc:
    raise SystemExit("Run this file inside a ROS 2 environment with rclpy installed.") from exc

try:
    from rover_interfaces.msg import WhistleEvent
except Exception as exc:
    raise SystemExit("Run this file inside a ROS 2 environment with rover_interfaces built.") from exc

try:
    from .drivers.whistle_detector import AudioConfig, WhistleConfig, XVF3800WhistleTracker
except ImportError:
    from .drivers.whistle_detector import AudioConfig, WhistleConfig, XVF3800WhistleTracker


class MicWhistleNode(Node):
    def __init__(self) -> None:
        super().__init__("mic_whistle_node")

        self.declare_parameter("device_index", -1)
        self.declare_parameter("rate", 16000)
        self.declare_parameter("chunk", 1024)
        self.declare_parameter("channels", 2)
        self.declare_parameter("detect_channel", 0)
        self.declare_parameter("use_frequency_gate", False)
        self.declare_parameter("band_min_hz", 1800.0)
        self.declare_parameter("band_max_hz", 4500.0)
        self.declare_parameter("min_rms", 700.0)
        self.declare_parameter("min_band_ratio", 0.45)
        self.declare_parameter("min_peak_ratio", 0.18)
        self.declare_parameter("min_prominence_db", 12.0)
        self.declare_parameter("consecutive_hits_required", 3)
        self.declare_parameter("cooldown_s", 1.0)
        self.declare_parameter("use_usb_control", True)
        self.declare_parameter("doa_samples", 5)
        self.declare_parameter("doa_sample_delay_s", 0.03)
        self.declare_parameter("doa_offset_deg", 0.0)
        self.declare_parameter("doa_clockwise_positive", True)

        latch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.event_pub = self.create_publisher(WhistleEvent, "/whistle/event", latch_qos)
        self.ready_pub = self.create_publisher(Bool, "/whistle/ready", latch_qos)
        self._last_ready = False
        self._thread = threading.Thread(target=self._run_tracker, daemon=True)
        self._thread.start()

    def _wrap_degrees(self, value: float) -> float:
        while value > 180.0:
            value -= 360.0
        while value < -180.0:
            value += 360.0
        return value

    def _to_time_msg(self, seconds_value: float):
        whole = int(seconds_value)
        frac = seconds_value - whole
        stamp = self.get_clock().now().to_msg()
        stamp.sec = whole
        stamp.nanosec = int(frac * 1_000_000_000)
        return stamp

    def _publish_ready(self, ready: bool) -> None:
        if ready == self._last_ready:
            return
        self._last_ready = ready
        self.ready_pub.publish(Bool(data=ready))
        self.get_logger().info(f"Whistle ready: {ready}")

    def _publish_event(self, event) -> None:
        doa_offset = float(self.get_parameter("doa_offset_deg").value)
        doa_clockwise_positive = bool(self.get_parameter("doa_clockwise_positive").value)
        robot_frame_doa = -event.doa_deg if doa_clockwise_positive else event.doa_deg
        robot_frame_doa = self._wrap_degrees(robot_frame_doa + doa_offset)
        msg = WhistleEvent()
        msg.stamp = self._to_time_msg(event.timestamp)
        msg.doa_deg = float(robot_frame_doa)
        msg.peak_freq_hz = float(event.peak_freq_hz)
        msg.confidence = float(event.confidence)
        self.event_pub.publish(msg)
        self.get_logger().info(
            f"Whistle event doa={msg.doa_deg:.1f} peak={msg.peak_freq_hz:.1f}Hz conf={msg.confidence:.2f}"
        )

    def _run_tracker(self) -> None:
        while rclpy.ok():
            device_index = int(self.get_parameter("device_index").value)
            if device_index < 0:
                device_index = None
            audio_cfg = AudioConfig(
                rate=int(self.get_parameter("rate").value),
                chunk=int(self.get_parameter("chunk").value),
                channels=int(self.get_parameter("channels").value),
                detect_channel=int(self.get_parameter("detect_channel").value),
                input_device_index=device_index,
            )
            whistle_cfg = WhistleConfig(
                use_frequency_gate=bool(self.get_parameter("use_frequency_gate").value),
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
                use_usb_control=bool(self.get_parameter("use_usb_control").value),
            )
            try:
                tracker = XVF3800WhistleTracker(audio_cfg, whistle_cfg)
                tracker.run(self._publish_event, self._publish_ready)
            except Exception as exc:
                self._publish_ready(False)
                self.get_logger().error(f"Whistle tracker error: {exc}")
                time.sleep(1.0)


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
