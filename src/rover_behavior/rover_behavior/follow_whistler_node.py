from __future__ import annotations

import math
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

from rover_interfaces.msg import TargetTrack, WhistleEvent


class BehaviorState(str, Enum):
    IDLE = "IDLE"
    TURN_TO_DOA = "TURN_TO_DOA"
    ACQUIRE = "ACQUIRE"
    APPROACH = "APPROACH"
    REACQUIRE = "REACQUIRE"
    ARRIVED = "ARRIVED"


class FollowWhistlerNode(Node):
    def __init__(self) -> None:
        super().__init__("follow_whistler_node")

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("turn_tolerance_deg", 10.0)
        self.declare_parameter("turn_speed_rad_s", 0.7)
        self.declare_parameter("search_turn_speed_rad_s", 0.55)
        self.declare_parameter("search_sweep_extent_deg", 35.0)
        self.declare_parameter("acquire_timeout_s", 5.0)
        self.declare_parameter("reacquire_timeout_s", 8.0)
        self.declare_parameter("target_timeout_s", 0.75)
        self.declare_parameter("range_timeout_s", 1.0)
        self.declare_parameter("arrival_range_m", 1.0)
        self.declare_parameter("arrival_bbox_width_px", 560.0)
        self.declare_parameter("center_tolerance_norm", 0.08)
        self.declare_parameter("linear_kp", 0.5)
        self.declare_parameter("angular_kp", 1.2)
        self.declare_parameter("max_linear_speed_mps", 0.35)
        self.declare_parameter("min_linear_speed_mps", 0.08)
        self.declare_parameter("max_angular_speed_rad_s", 0.9)
        self.declare_parameter("soft_avoid_distance_m", 0.60)
        self.declare_parameter("hard_stop_distance_m", 0.35)
        self.declare_parameter("avoid_turn_speed_rad_s", 0.7)

        latch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "/behavior/state", latch_qos)
        self.create_subscription(WhistleEvent, "/whistle/event", self._whistle_callback, latch_qos)
        self.create_subscription(TargetTrack, "/target_track", self._track_callback, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(Range, "/range/front_left", self._left_range_callback, 20)
        self.create_subscription(Range, "/range/front_right", self._right_range_callback, 20)
        self.create_subscription(Bool, "/self_test/ready", self._ready_callback, latch_qos)

        self.state = BehaviorState.IDLE
        self.latest_track: Optional[TargetTrack] = None
        self.track_stamp = None
        self.last_whistle: Optional[WhistleEvent] = None
        self.ready = False
        self.has_seen_ready = False
        self.have_odom = False
        self.current_yaw = 0.0
        self.left_range = math.inf
        self.right_range = math.inf
        self.left_range_stamp = None
        self.right_range_stamp = None
        self.turn_target_heading = 0.0
        self.search_center_heading = 0.0
        self.search_leg = 0
        self.search_target_heading = 0.0
        self.state_entered_at = self.get_clock().now()
        self.last_seen_target_heading = 0.0
        self._publish_state()

        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.create_timer(period, self._control_loop)

    def _ready_callback(self, msg: Bool) -> None:
        previous = self.ready
        self.ready = bool(msg.data)
        if self.ready:
            self.has_seen_ready = True
        if previous and not self.ready:
            self._publish_cmd(0.0, 0.0)
            self._set_state(BehaviorState.IDLE)

    def _whistle_callback(self, msg: WhistleEvent) -> None:
        self.last_whistle = msg
        if not self.have_odom:
            return
        if self.state not in (BehaviorState.IDLE, BehaviorState.ARRIVED):
            return
        self.turn_target_heading = self._normalize_angle(self.current_yaw + math.radians(msg.doa_deg))
        self.search_center_heading = self.turn_target_heading
        self.search_leg = 0
        self._set_state(BehaviorState.TURN_TO_DOA)

    def _track_callback(self, msg: TargetTrack) -> None:
        self.latest_track = msg
        self.track_stamp = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        self.have_odom = True
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def _left_range_callback(self, msg: Range) -> None:
        self.left_range = float(msg.range)
        self.left_range_stamp = self.get_clock().now()

    def _right_range_callback(self, msg: Range) -> None:
        self.right_range = float(msg.range)
        self.right_range_stamp = self.get_clock().now()

    def _set_state(self, state: BehaviorState) -> None:
        if self.state == state:
            return
        self.state = state
        self.state_entered_at = self.get_clock().now()
        if state in (BehaviorState.ACQUIRE, BehaviorState.REACQUIRE):
            self.search_leg = 0
        self._publish_state()

    def _publish_state(self) -> None:
        self.state_pub.publish(String(data=self.state.value))
        self.get_logger().info(f"Behavior state: {self.state.value}")

    def _normalize_angle(self, value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def _publish_cmd(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

    def _track_is_fresh(self) -> bool:
        if self.latest_track is None or self.track_stamp is None:
            return False
        age = (self.get_clock().now() - self.track_stamp).nanoseconds / 1e9
        return age <= float(self.get_parameter("target_timeout_s").value)

    def _track_visible(self) -> bool:
        return self._track_is_fresh() and bool(self.latest_track.visible)

    def _fresh_range(self, value: float, stamp) -> float:
        if stamp is None:
            return math.inf
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > float(self.get_parameter("range_timeout_s").value):
            return math.inf
        return value

    def _arrived(self, track: TargetTrack) -> bool:
        centered = abs(track.image_error_x_norm) <= float(self.get_parameter("center_tolerance_norm").value)
        close_by_bbox = track.bbox_w_px >= float(self.get_parameter("arrival_bbox_width_px").value)
        close_by_range = 0.0 < track.range_estimate_m <= float(self.get_parameter("arrival_range_m").value)
        return centered and (close_by_bbox or close_by_range)

    def _approach_command(self, track: TargetTrack) -> tuple[float, float]:
        angular = -float(self.get_parameter("angular_kp").value) * float(track.image_error_x_norm)
        angular_limit = float(self.get_parameter("max_angular_speed_rad_s").value)
        angular = max(-angular_limit, min(angular_limit, angular))

        linear = 0.0
        if track.range_estimate_m > 0.0:
            error = track.range_estimate_m - float(self.get_parameter("arrival_range_m").value)
            if error > 0.0:
                linear = float(self.get_parameter("linear_kp").value) * error
                linear = max(
                    float(self.get_parameter("min_linear_speed_mps").value),
                    min(float(self.get_parameter("max_linear_speed_mps").value), linear),
                )
        elif track.bbox_w_px > 0.0:
            ratio = max(
                0.0,
                1.0 - (track.bbox_w_px / float(self.get_parameter("arrival_bbox_width_px").value)),
            )
            linear = ratio * float(self.get_parameter("max_linear_speed_mps").value)

        if abs(track.image_error_x_norm) > 0.4:
            linear = min(linear, float(self.get_parameter("min_linear_speed_mps").value))

        return linear, angular

    def _apply_obstacle_safety(self, desired_linear: float, desired_angular: float) -> tuple[float, float]:
        left_value = self._fresh_range(self.left_range, self.left_range_stamp)
        right_value = self._fresh_range(self.right_range, self.right_range_stamp)
        soft_limit = float(self.get_parameter("soft_avoid_distance_m").value)
        hard_limit = float(self.get_parameter("hard_stop_distance_m").value)
        avoid_turn = float(self.get_parameter("avoid_turn_speed_rad_s").value)

        if left_value < hard_limit and right_value < hard_limit:
            return 0.0, avoid_turn if right_value > left_value else -avoid_turn
        if left_value < hard_limit:
            return 0.0, -avoid_turn
        if right_value < hard_limit:
            return 0.0, avoid_turn
        if left_value < soft_limit and right_value < soft_limit:
            return 0.0, avoid_turn if right_value > left_value else -avoid_turn
        if left_value < soft_limit:
            return 0.0, min(-avoid_turn, desired_angular)
        if right_value < soft_limit:
            return 0.0, max(avoid_turn, desired_angular)
        return desired_linear, desired_angular

    def _search_step(self, center_heading: float) -> tuple[bool, float]:
        sweep_extent = math.radians(float(self.get_parameter("search_sweep_extent_deg").value))
        if self.search_leg == 0:
            self.search_leg = 1
            self.search_target_heading = self._normalize_angle(center_heading + sweep_extent)
        error = self._normalize_angle(self.search_target_heading - self.current_yaw)
        tolerance = math.radians(float(self.get_parameter("turn_tolerance_deg").value))
        if abs(error) <= tolerance:
            if self.search_leg == 1:
                self.search_leg = 2
                self.search_target_heading = self._normalize_angle(center_heading - sweep_extent)
                return False, 0.0
            return True, 0.0
        turn_speed = float(self.get_parameter("search_turn_speed_rad_s").value)
        return False, turn_speed if error > 0.0 else -turn_speed

    def _handle_acquire(self, reacquire: bool) -> None:
        if self._track_visible():
            self._set_state(BehaviorState.APPROACH)
            return

        timeout = (
            float(self.get_parameter("reacquire_timeout_s").value)
            if reacquire
            else float(self.get_parameter("acquire_timeout_s").value)
        )
        elapsed = (self.get_clock().now() - self.state_entered_at).nanoseconds / 1e9
        if elapsed < timeout and self.search_leg == 0:
            self._publish_cmd(0.0, 0.0)
            return

        done, angular = self._search_step(self.search_center_heading)
        if done or elapsed >= timeout:
            self._set_state(BehaviorState.IDLE)
            self._publish_cmd(0.0, 0.0)
            return
        self._publish_cmd(0.0, angular)

    def _control_loop(self) -> None:
        if not self.ready:
            if self.has_seen_ready:
                self._publish_cmd(0.0, 0.0)
            return

        if not self.have_odom:
            self._publish_cmd(0.0, 0.0)
            return

        if self.state == BehaviorState.IDLE:
            self._publish_cmd(0.0, 0.0)
            return

        if self.state == BehaviorState.TURN_TO_DOA:
            error = self._normalize_angle(self.turn_target_heading - self.current_yaw)
            tolerance = math.radians(float(self.get_parameter("turn_tolerance_deg").value))
            if abs(error) <= tolerance:
                self._set_state(BehaviorState.ACQUIRE)
                self._publish_cmd(0.0, 0.0)
                return
            turn_speed = float(self.get_parameter("turn_speed_rad_s").value)
            self._publish_cmd(0.0, turn_speed if error > 0.0 else -turn_speed)
            return

        if self.state == BehaviorState.ACQUIRE:
            self._handle_acquire(reacquire=False)
            return

        if self.state == BehaviorState.REACQUIRE:
            self._handle_acquire(reacquire=True)
            return

        if self.state == BehaviorState.APPROACH:
            if not self._track_visible():
                self.search_center_heading = self.last_seen_target_heading if self.last_seen_target_heading else self.current_yaw
                self._set_state(BehaviorState.REACQUIRE)
                self._publish_cmd(0.0, 0.0)
                return
            track = self.latest_track
            self.last_seen_target_heading = self._normalize_angle(self.current_yaw + math.radians(track.bearing_deg))
            if self._arrived(track):
                self._set_state(BehaviorState.ARRIVED)
                self._publish_cmd(0.0, 0.0)
                return
            linear, angular = self._approach_command(track)
            linear, angular = self._apply_obstacle_safety(linear, angular)
            self._publish_cmd(linear, angular)
            return

        if self.state == BehaviorState.ARRIVED:
            self._publish_cmd(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowWhistlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
