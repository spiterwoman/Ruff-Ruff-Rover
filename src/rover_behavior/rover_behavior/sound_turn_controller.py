from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class SoundTurnController(Node):
    def __init__(self):
        super().__init__("sound_turn_controller")

        self.declare_parameter("front_angle_deg", 0.0)
        self.declare_parameter("angle_tolerance_deg", 15.0)
        self.declare_parameter("turn_speed", 0.6)

        self.front_angle = float(self.get_parameter("front_angle_deg").value)
        self.tolerance = float(self.get_parameter("angle_tolerance_deg").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.angle_sub = self.create_subscription(
            Float32,
            "/whistle/angle_deg",
            self.angle_callback,
            10,
        )

        self.get_logger().info("Sound turn controller started.")

    def normalize_error(self, angle_deg: float) -> float:
        error = angle_deg - self.front_angle
        while error > 180.0:
            error -= 360.0
        while error < -180.0:
            error += 360.0
        return error

    def angle_callback(self, msg: Float32) -> None:
        angle = float(msg.data)
        error = self.normalize_error(angle)

        twist = Twist()

        if abs(error) <= self.tolerance:
            twist.angular.z = 0.0
            self.get_logger().info(f"Sound centered: {angle:.1f} deg -> stop")
        elif error > 0:
            twist.angular.z = self.turn_speed
            self.get_logger().info(f"Sound at {angle:.1f} deg -> turning left")
        else:
            twist.angular.z = -self.turn_speed
            self.get_logger().info(f"Sound at {angle:.1f} deg -> turning right")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SoundTurnController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()