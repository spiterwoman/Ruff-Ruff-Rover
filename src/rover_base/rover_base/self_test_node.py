from __future__ import annotations

from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String


class SelfTestPhase(str, Enum):
    WAITING = "WAITING"
    LEFT_PULSE = "LEFT_PULSE"
    LEFT_SETTLE = "LEFT_SETTLE"
    RIGHT_PULSE = "RIGHT_PULSE"
    RIGHT_SETTLE = "RIGHT_SETTLE"
    READY = "READY"
    FAIL = "FAIL"


class SelfTestNode(Node):
    def __init__(self) -> None:
        super().__init__("self_test_node")

        self.declare_parameter("startup_timeout_s", 45.0)
        self.declare_parameter("heartbeat_timeout_s", 1.0)
        self.declare_parameter("sensor_timeout_s", 2.0)
        self.declare_parameter("pulse_speed", 0.12)
        self.declare_parameter("pulse_duration_s", 0.35)
        self.declare_parameter("settle_duration_s", 0.35)
        self.declare_parameter("min_wheel_position_delta", 0.05)
        self.declare_parameter("left_forward_sign", 1.0)
        self.declare_parameter("right_forward_sign", 1.0)
        self.declare_parameter("range_min_valid_m", 0.02)
        self.declare_parameter("range_max_valid_m", 2.0)

        latch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ready_pub = self.create_publisher(Bool, "/self_test/ready", latch_qos)
        self.status_pub = self.create_publisher(String, "/self_test/status", latch_qos)

        self.create_subscription(Bool, "/pico/heartbeat", self._heartbeat_callback, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(Vector3, "/wheel_state", self._wheel_state_callback, 10)
        self.create_subscription(Range, "/range/front_left", self._left_range_callback, 10)
        self.create_subscription(Range, "/range/front_right", self._right_range_callback, 10)
        self.create_subscription(Bool, "/camera/ready", self._camera_ready_callback, latch_qos)
        self.create_subscription(Bool, "/vision/ready", self._vision_ready_callback, latch_qos)
        self.create_subscription(Bool, "/whistle/ready", self._whistle_ready_callback, latch_qos)

        self.phase = SelfTestPhase.WAITING
        self.started_at = self.get_clock().now()
        self.phase_started_at = self.started_at
        self.heartbeat_stamp = None
        self.odom_stamp = None
        self.joint_stamp = None
        self.left_range_stamp = None
        self.right_range_stamp = None
        self.left_range = 0.0
        self.right_range = 0.0
        self.camera_ready = False
        self.vision_ready = False
        self.whistle_ready = False
        self.left_position: Optional[float] = None
        self.right_position: Optional[float] = None
        self.baseline_left: Optional[float] = None
        self.baseline_right: Optional[float] = None
        self._publish_ready(False)
        self._publish_status(self.phase)
        self.create_timer(0.05, self._tick)

    def _publish_ready(self, value: bool) -> None:
        self.ready_pub.publish(Bool(data=value))

    def _publish_status(self, phase: SelfTestPhase, detail: str = "") -> None:
        suffix = f": {detail}" if detail else ""
        message = f"{phase.value}{suffix}"
        self.status_pub.publish(String(data=message))
        self.get_logger().info(message)

    def _set_phase(self, phase: SelfTestPhase, detail: str = "") -> None:
        if self.phase == phase and not detail:
            return
        self.phase = phase
        self.phase_started_at = self.get_clock().now()
        self._publish_status(phase, detail)

    def _heartbeat_callback(self, _: Bool) -> None:
        self.heartbeat_stamp = self.get_clock().now()

    def _odom_callback(self, _: Odometry) -> None:
        self.odom_stamp = self.get_clock().now()

    def _wheel_state_callback(self, msg: Vector3) -> None:
        self.joint_stamp = self.get_clock().now()
        self.left_position = float(msg.x)
        self.right_position = float(msg.y)

    def _left_range_callback(self, msg: Range) -> None:
        self.left_range = float(msg.range)
        self.left_range_stamp = self.get_clock().now()

    def _right_range_callback(self, msg: Range) -> None:
        self.right_range = float(msg.range)
        self.right_range_stamp = self.get_clock().now()

    def _vision_ready_callback(self, msg: Bool) -> None:
        self.vision_ready = bool(msg.data)

    def _camera_ready_callback(self, msg: Bool) -> None:
        self.camera_ready = bool(msg.data)

    def _whistle_ready_callback(self, msg: Bool) -> None:
        self.whistle_ready = bool(msg.data)

    def _stamp_fresh(self, stamp, timeout_value: float) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= timeout_value

    def _ranges_valid(self) -> bool:
        timeout_value = float(self.get_parameter("sensor_timeout_s").value)
        if not self._stamp_fresh(self.left_range_stamp, timeout_value):
            return False
        if not self._stamp_fresh(self.right_range_stamp, timeout_value):
            return False
        min_value = float(self.get_parameter("range_min_valid_m").value)
        max_value = float(self.get_parameter("range_max_valid_m").value)
        return min_value <= self.left_range <= max_value and min_value <= self.right_range <= max_value

    def _dependencies_ready(self) -> bool:
        sensor_timeout = float(self.get_parameter("sensor_timeout_s").value)
        heartbeat_timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        return (
            self._stamp_fresh(self.heartbeat_stamp, heartbeat_timeout)
            and self._stamp_fresh(self.odom_stamp, sensor_timeout)
            and self._stamp_fresh(self.joint_stamp, sensor_timeout)
            and self._ranges_valid()
            and self.camera_ready
            and self.vision_ready
            and self.whistle_ready
            and self.left_position is not None
            and self.right_position is not None
        )

    def _zero_cmd(self) -> None:
        self.cmd_pub.publish(Twist())

    def _left_only_cmd(self) -> Twist:
        speed = float(self.get_parameter("pulse_speed").value)
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = -speed
        return cmd

    def _right_only_cmd(self) -> Twist:
        speed = float(self.get_parameter("pulse_speed").value)
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = speed
        return cmd

    def _capture_baseline(self) -> None:
        self.baseline_left = self.left_position
        self.baseline_right = self.right_position

    def _phase_elapsed(self) -> float:
        return (self.get_clock().now() - self.phase_started_at).nanoseconds / 1e9

    def _check_wheel_delta(self, side: str) -> bool:
        if self.baseline_left is None or self.baseline_right is None:
            return False
        if self.left_position is None or self.right_position is None:
            return False
        min_delta = float(self.get_parameter("min_wheel_position_delta").value)
        left_delta = self.left_position - self.baseline_left
        right_delta = self.right_position - self.baseline_right
        if side == "left":
            expected = float(self.get_parameter("left_forward_sign").value)
            return abs(left_delta) >= min_delta and left_delta * expected > 0.0
        expected = float(self.get_parameter("right_forward_sign").value)
        return abs(right_delta) >= min_delta and right_delta * expected > 0.0

    def _mark_fail(self, detail: str) -> None:
        self._zero_cmd()
        self._publish_ready(False)
        self._set_phase(SelfTestPhase.FAIL, detail)

    def _tick(self) -> None:
        if self.phase == SelfTestPhase.FAIL:
            self._zero_cmd()
            return

        if self.phase == SelfTestPhase.READY:
            if not self._dependencies_ready():
                self._mark_fail("dependency lost")
                return
            self._publish_ready(True)
            return

        if self.phase != SelfTestPhase.WAITING and not self._dependencies_ready():
            self._mark_fail("dependency lost during self-test")
            return

        started_elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if started_elapsed > float(self.get_parameter("startup_timeout_s").value):
            self._mark_fail("startup timeout")
            return

        if self.phase == SelfTestPhase.WAITING:
            self._publish_ready(False)
            if not self._dependencies_ready():
                self._zero_cmd()
                return
            self._capture_baseline()
            self._set_phase(SelfTestPhase.LEFT_PULSE)
            return

        pulse_duration = float(self.get_parameter("pulse_duration_s").value)
        settle_duration = float(self.get_parameter("settle_duration_s").value)

        if self.phase == SelfTestPhase.LEFT_PULSE:
            if self._phase_elapsed() < pulse_duration:
                self.cmd_pub.publish(self._left_only_cmd())
                return
            self._zero_cmd()
            self._set_phase(SelfTestPhase.LEFT_SETTLE)
            return

        if self.phase == SelfTestPhase.LEFT_SETTLE:
            if self._phase_elapsed() < settle_duration:
                return
            if not self._check_wheel_delta("left"):
                self._mark_fail("left wheel encoder check failed")
                return
            self._capture_baseline()
            self._set_phase(SelfTestPhase.RIGHT_PULSE)
            return

        if self.phase == SelfTestPhase.RIGHT_PULSE:
            if self._phase_elapsed() < pulse_duration:
                self.cmd_pub.publish(self._right_only_cmd())
                return
            self._zero_cmd()
            self._set_phase(SelfTestPhase.RIGHT_SETTLE)
            return

        if self.phase == SelfTestPhase.RIGHT_SETTLE:
            if self._phase_elapsed() < settle_duration:
                return
            if not self._check_wheel_delta("right"):
                self._mark_fail("right wheel encoder check failed")
                return
            self._zero_cmd()
            self._publish_ready(True)
            self._set_phase(SelfTestPhase.READY)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SelfTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._zero_cmd()
        node.destroy_node()
        rclpy.shutdown()
