from __future__ import annotations

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String

from rover_interfaces.msg import WhistleEvent


class TurnState(str, Enum):
    WAITING_FOR_ODOM = "WAITING_FOR_ODOM"
    IDLE = "IDLE"
    TURNING = "TURNING"
    APPROACHING = "APPROACHING"
    ALIGNED = "ALIGNED"
    ARRIVED = "ARRIVED"


class TurnToWhistleTestNode(Node):
    def __init__(self) -> None:
        super().__init__("turn_to_whistle_test_node")

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("turn_tolerance_deg", 10.0)
        self.declare_parameter("turn_speed_rad_s", 0.7)
        default_turn_speed = float(self.get_parameter("turn_speed_rad_s").value)
        self.declare_parameter("turn_angular_kp", 0.8)
        self.declare_parameter("max_angular_speed_rad_s", default_turn_speed)
        self.declare_parameter("min_turn_speed_rad_s", min(0.5, default_turn_speed))
        self.declare_parameter("use_open_loop_turn", False)
        self.declare_parameter("turn_timeout_s", 8.0)
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("turn_timeout_scale", 1.5)
        self.declare_parameter("turn_timeout_padding_s", 0.5)
        self.declare_parameter("turn_divergence_grace_s", 0.35)
        self.declare_parameter("turn_divergence_margin_deg", 10.0)
        self.declare_parameter("turn_allow_direction_recovery", True)
        self.declare_parameter("turn_debug_log_period_s", 0.5)
        self.declare_parameter("drive_forward_after_turn", True)
        self.declare_parameter("forward_speed_mps", 0.18)
        self.declare_parameter("forward_stop_distance_m", 0.45)
        self.declare_parameter("forward_timeout_s", 8.0)
        self.declare_parameter("range_timeout_s", 1.0)
        self.declare_parameter("forward_range_source", "front_center")
        self.declare_parameter("front_center_sensor", "front_left")
        self.declare_parameter("forward_debug_log_period_s", 0.5)

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
        self.create_subscription(Range, "/range/front_left", self._left_range_callback, 20)
        self.create_subscription(Range, "/range/front_right", self._right_range_callback, 20)

        self.state = TurnState.WAITING_FOR_ODOM
        self.have_odom = False
        self.odom_stamp = None
        self.current_yaw = 0.0
        self.turn_target_heading = 0.0
        self.turn_started_at = self.get_clock().now()
        self.turn_initial_error_abs = 0.0
        self.turn_best_error_abs = math.inf
        self.turn_output_sign = 1.0
        self.turn_reversed_once = False
        self.turn_last_log_at = self.turn_started_at
        self.turn_requested_angle = 0.0
        self.turn_duration_s = 0.0
        self.turn_command_angular = 0.0
        self.left_range = math.inf
        self.right_range = math.inf
        self.left_range_stamp = None
        self.right_range_stamp = None
        self.forward_started_at = self.turn_started_at
        self.forward_last_log_at = self.turn_started_at
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
            self.turn_last_log_at = self.turn_started_at
        elif state == TurnState.APPROACHING:
            self.forward_started_at = self.get_clock().now()
            self.forward_last_log_at = self.forward_started_at
        self._publish_state()

    def _normalize_angle(self, value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def _use_open_loop_turn(self) -> bool:
        return bool(self.get_parameter("use_open_loop_turn").value)

    def _publish_cmd(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _turn_command(self, error: float) -> float:
        angular = float(self.get_parameter("turn_angular_kp").value) * error
        max_speed = float(self.get_parameter("max_angular_speed_rad_s").value)
        if max_speed <= 0.0:
            return 0.0
        angular = max(-max_speed, min(max_speed, angular))

        min_speed = min(float(self.get_parameter("min_turn_speed_rad_s").value), max_speed)
        if min_speed > 0.0 and abs(angular) < min_speed:
            angular = math.copysign(min_speed, angular if angular != 0.0 else error)
        return self.turn_output_sign * angular

    def _open_loop_turn_speed(self) -> float:
        speed = abs(float(self.get_parameter("turn_speed_rad_s").value))
        if speed <= 0.0:
            speed = abs(float(self.get_parameter("max_angular_speed_rad_s").value))
        return speed

    def _effective_turn_timeout(self) -> float:
        min_turn_speed = min(
            float(self.get_parameter("min_turn_speed_rad_s").value),
            float(self.get_parameter("max_angular_speed_rad_s").value),
        )
        if min_turn_speed <= 0.0:
            return float(self.get_parameter("turn_timeout_s").value)

        expected = self.turn_initial_error_abs / min_turn_speed
        scaled = (
            expected * float(self.get_parameter("turn_timeout_scale").value)
            + float(self.get_parameter("turn_timeout_padding_s").value)
        )
        return min(float(self.get_parameter("turn_timeout_s").value), max(0.5, scaled))

    def _maybe_log_turn_progress(self, elapsed: float, error: float, command: float) -> None:
        period = float(self.get_parameter("turn_debug_log_period_s").value)
        if period <= 0.0:
            return
        now = self.get_clock().now()
        since_last = (now - self.turn_last_log_at).nanoseconds / 1e9
        if since_last < period:
            return
        self.turn_last_log_at = now
        self.get_logger().info(
            "Turning: yaw=%.1f target=%.1f error=%.1f cmd=%.2f elapsed=%.2fs recovered=%s"
            % (
                math.degrees(self.current_yaw),
                math.degrees(self.turn_target_heading),
                math.degrees(error),
                command,
                elapsed,
                self.turn_reversed_once,
            )
        )

    def _maybe_log_forward_progress(self, elapsed: float, front_range: float, command: float) -> None:
        period = float(self.get_parameter("forward_debug_log_period_s").value)
        if period <= 0.0:
            return
        now = self.get_clock().now()
        since_last = (now - self.forward_last_log_at).nanoseconds / 1e9
        if since_last < period:
            return
        self.forward_last_log_at = now
        self.get_logger().info(
            "Driving forward: range=%.2fm speed=%.2f elapsed=%.2fs source=%s"
            % (
                front_range,
                command,
                elapsed,
                str(self.get_parameter("forward_range_source").value),
            )
        )

    def _stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _odom_is_fresh(self) -> bool:
        if self.odom_stamp is None:
            return False
        age = (self.get_clock().now() - self.odom_stamp).nanoseconds / 1e9
        return age <= float(self.get_parameter("odom_timeout_s").value)

    def _have_fresh_odom(self) -> bool:
        return self.have_odom and self._odom_is_fresh()

    def _fresh_range(self, value: float, stamp) -> float:
        if stamp is None:
            return math.inf
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > float(self.get_parameter("range_timeout_s").value):
            return math.inf
        return value

    def _selected_forward_range(self) -> float:
        source = str(self.get_parameter("forward_range_source").value)
        center_sensor = str(self.get_parameter("front_center_sensor").value)
        left_value = self._fresh_range(self.left_range, self.left_range_stamp)
        right_value = self._fresh_range(self.right_range, self.right_range_stamp)
        if source == "front_center":
            preferred = right_value if center_sensor == "front_right" else left_value
            fallback = left_value if center_sensor == "front_right" else right_value
            if math.isfinite(preferred):
                return preferred
            return fallback
        if source == "front_right":
            return right_value
        if source == "closest":
            return min(left_value, right_value)
        return left_value

    def _should_drive_forward(self) -> bool:
        return bool(self.get_parameter("drive_forward_after_turn").value)

    def _finish_turn(self) -> None:
        self._stop()
        if self._should_drive_forward():
            self._set_state(TurnState.APPROACHING)
            return
        self._set_state(TurnState.ALIGNED)

    def _whistle_callback(self, msg: WhistleEvent) -> None:
        if not self._use_open_loop_turn() and not self._have_fresh_odom():
            self.get_logger().warning("Ignoring whistle because odometry is not ready.")
            return
        if self.state in (TurnState.TURNING, TurnState.APPROACHING):
            self.get_logger().info("Ignoring whistle while current action is in progress.")
            return
        self.turn_requested_angle = math.radians(float(msg.doa_deg))
        self.turn_target_heading = self._normalize_angle(self.current_yaw + self.turn_requested_angle)
        self.turn_initial_error_abs = abs(self.turn_requested_angle)
        self.turn_best_error_abs = self.turn_initial_error_abs
        self.turn_output_sign = 1.0
        self.turn_reversed_once = False
        if self._use_open_loop_turn():
            turn_speed = self._open_loop_turn_speed()
            if turn_speed <= 0.0:
                self.get_logger().warning("Ignoring whistle because turn_speed_rad_s is not positive.")
                return
            self.turn_duration_s = self.turn_initial_error_abs / turn_speed
            self.turn_command_angular = math.copysign(turn_speed, self.turn_requested_angle) if self.turn_initial_error_abs > 0.0 else 0.0
        else:
            self.turn_duration_s = 0.0
            self.turn_command_angular = 0.0
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

    def _left_range_callback(self, msg: Range) -> None:
        self.left_range = float(msg.range)
        self.left_range_stamp = self.get_clock().now()

    def _right_range_callback(self, msg: Range) -> None:
        self.right_range = float(msg.range)
        self.right_range_stamp = self.get_clock().now()

    def _control_loop(self) -> None:
        if self.state == TurnState.TURNING and not self._use_open_loop_turn() and not self._have_fresh_odom():
            self._stop()
            if self.state != TurnState.WAITING_FOR_ODOM:
                self._set_state(TurnState.WAITING_FOR_ODOM)
            return

        if self.state == TurnState.WAITING_FOR_ODOM:
            if self._use_open_loop_turn() or self._have_fresh_odom():
                self._set_state(TurnState.IDLE)
            else:
                self._stop()
            return

        if self.state in (TurnState.ALIGNED, TurnState.ARRIVED):
            self._stop()
            return

        if self.state == TurnState.APPROACHING:
            elapsed = (self.get_clock().now() - self.forward_started_at).nanoseconds / 1e9
            front_range = self._selected_forward_range()
            if not math.isfinite(front_range):
                self.get_logger().warning("Forward drive waiting for a fresh front range reading.")
                self._stop()
                return

            if front_range <= float(self.get_parameter("forward_stop_distance_m").value):
                self.get_logger().info(
                    "Forward drive stopping at range %.2fm (threshold %.2fm)."
                    % (
                        front_range,
                        float(self.get_parameter("forward_stop_distance_m").value),
                    )
                )
                self._stop()
                self._set_state(TurnState.ARRIVED)
                return

            timeout_s = float(self.get_parameter("forward_timeout_s").value)
            if timeout_s > 0.0 and elapsed > timeout_s:
                self.get_logger().warning(
                    "Forward drive timeout reached after %.2fs with range %.2fm; stopping."
                    % (elapsed, front_range)
                )
                self._stop()
                self._set_state(TurnState.IDLE)
                return

            command = float(self.get_parameter("forward_speed_mps").value)
            self._maybe_log_forward_progress(elapsed, front_range, command)
            self._publish_cmd(command, 0.0)
            return

        if self.state != TurnState.TURNING:
            self._stop()
            return

        elapsed = (self.get_clock().now() - self.turn_started_at).nanoseconds / 1e9
        tolerance = math.radians(float(self.get_parameter("turn_tolerance_deg").value))

        if self._use_open_loop_turn():
            if self.turn_initial_error_abs <= tolerance or elapsed >= self.turn_duration_s:
                self._finish_turn()
                return

            if self._have_fresh_odom():
                error = self._normalize_angle(self.turn_target_heading - self.current_yaw)
                if abs(error) <= tolerance:
                    self._finish_turn()
                    return

                timeout_s = self._effective_turn_timeout()
                if elapsed > timeout_s:
                    self.get_logger().warning(
                        "Open-loop turn timeout reached after %.2fs for requested %.1f deg; stopping."
                        % (elapsed, math.degrees(self.turn_initial_error_abs))
                    )
                    self._stop()
                    self._set_state(TurnState.IDLE)
                    return

                command = math.copysign(self._open_loop_turn_speed(), error)
                self._maybe_log_turn_progress(elapsed, error, command)
                self._publish_cmd(0.0, command)
                return

            command = self.turn_command_angular
            self._maybe_log_turn_progress(elapsed, self.turn_requested_angle, command)
            self._publish_cmd(0.0, command)
            return

        timeout_s = self._effective_turn_timeout()
        if elapsed > timeout_s:
            self.get_logger().warning(
                "Turn timeout reached after %.2fs for requested %.1f deg; stopping."
                % (elapsed, math.degrees(self.turn_initial_error_abs))
            )
            self._stop()
            self._set_state(TurnState.IDLE)
            return

        error = self._normalize_angle(self.turn_target_heading - self.current_yaw)
        error_abs = abs(error)
        if error_abs < self.turn_best_error_abs:
            self.turn_best_error_abs = error_abs

        grace_s = float(self.get_parameter("turn_divergence_grace_s").value)
        divergence_margin = math.radians(float(self.get_parameter("turn_divergence_margin_deg").value))
        if (
            bool(self.get_parameter("turn_allow_direction_recovery").value)
            and not self.turn_reversed_once
            and elapsed >= grace_s
            and error_abs > self.turn_best_error_abs + divergence_margin
        ):
            self.turn_output_sign *= -1.0
            self.turn_reversed_once = True
            self.turn_best_error_abs = error_abs
            self.get_logger().warning(
                "Turn error diverged to %.1f deg while targeting %.1f deg; reversing turn command sign."
                % (math.degrees(error_abs), math.degrees(self.turn_initial_error_abs))
            )

        if error_abs <= tolerance:
            self._finish_turn()
            return

        command = self._turn_command(error)
        self._maybe_log_turn_progress(elapsed, error, command)
        self._publish_cmd(0.0, command)


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
