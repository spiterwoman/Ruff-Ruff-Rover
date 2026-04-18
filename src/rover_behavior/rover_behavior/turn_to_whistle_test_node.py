from __future__ import annotations

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from rover_interfaces.msg import WhistleEvent


class TurnState(str, Enum):
    WAITING_FOR_ODOM = "WAITING_FOR_ODOM"
    IDLE = "IDLE"
    TURNING = "TURNING"
    ALIGNED = "ALIGNED"


class TurnToWhistleTestNode(Node):
    def __init__(self) -> None:
        super().__init__("turn_to_whistle_test_node")

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("turn_tolerance_deg", 10.0)
        self.declare_parameter("turn_speed_rad_s", 0.7)
        self.declare_parameter("turn_timeout_s", 8.0)
        self.declare_parameter("odom_timeout_s", 1.0)

        latch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "/behavior/state", latch_qos)
        self.create_subscription(WhistleEvent, "/whistle/event", self._whistle_callback, latch_qos)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)

        self.state = TurnState.WAITING_FOR_ODOM
        self.have_odom = False
        self.odom_stamp = None
        self.current_yaw = 0.0
        self.turn_target_heading = 0.0
        self.turn_started_at = self.get_clock().now()
        self._publish_state()

        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.create_timer(period, self._control_loop)

    def _publish_state(self) -> None:
        self.state_pub.publish(String(data=self.state.value))
        self.get_logger().info(f"Turn test state: {self.state.value}")

    def _set_state(self, state: TurnState) -> None:
        if self.state == state:
            return
        self.state = state
        if state == TurnState.TURNING:
            self.turn_started_at = self.get_clock().now()
        self._publish_state()

    def _normalize_angle(self, value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def _publish_cmd(self, angular: float) -> None:
        msg = Twist()
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_cmd(0.0)

    def _odom_is_fresh(self) -> bool:
        if self.odom_stamp is None:
            return False
        age = (self.get_clock().now() - self.odom_stamp).nanoseconds / 1e9
        return age <= float(self.get_parameter("odom_timeout_s").value)

    def _whistle_callback(self, msg: WhistleEvent) -> None:
        if not self.have_odom or not self._odom_is_fresh():
            self.get_logger().warning("Ignoring whistle because odometry is not ready.")
            return
        self.turn_target_heading = self._normalize_angle(self.current_yaw + math.radians(float(msg.doa_deg)))
        self.get_logger().info(
            f"New whistle target: doa={float(msg.doa_deg):.1f} deg target_yaw={math.degrees(self.turn_target_heading):.1f} deg"
        )
        self._set_state(TurnState.TURNING)

    def _odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)
        self.odom_stamp = self.get_clock().now()
        if not self.have_odom:
            self.have_odom = True
            if self.state == TurnState.WAITING_FOR_ODOM:
                self._set_state(TurnState.IDLE)

    def _control_loop(self) -> None:
        if not self.have_odom or not self._odom_is_fresh():
            self._stop()
            if self.state != TurnState.WAITING_FOR_ODOM:
                self._set_state(TurnState.WAITING_FOR_ODOM)
            return

        if self.state == TurnState.WAITING_FOR_ODOM:
            self._set_state(TurnState.IDLE)

        if self.state == TurnState.ALIGNED:
            self._stop()
            return

        if self.state != TurnState.TURNING:
            self._stop()
            return

        elapsed = (self.get_clock().now() - self.turn_started_at).nanoseconds / 1e9
        if elapsed > float(self.get_parameter("turn_timeout_s").value):
            self.get_logger().warning("Turn timeout reached; stopping.")
            self._stop()
            self._set_state(TurnState.IDLE)
            return

        error = self._normalize_angle(self.turn_target_heading - self.current_yaw)
        tolerance = math.radians(float(self.get_parameter("turn_tolerance_deg").value))
        if abs(error) <= tolerance:
            self._stop()
            self._set_state(TurnState.ALIGNED)
            return

        turn_speed = float(self.get_parameter("turn_speed_rad_s").value)
        self._publish_cmd(turn_speed if error > 0.0 else -turn_speed)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurnToWhistleTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
