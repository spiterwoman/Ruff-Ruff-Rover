#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Subscribers ---
        # Whistle node — only need detected flag
        # sound_turn_controller handles turning toward the whistle angle
        self.create_subscription(Bool, '/whistle/detected', self.whistle_detected_callback, 10)

        # Vision node (person bounding box: [x, y, w, h, confidence])
        self.create_subscription(Float32MultiArray, '/person_detected', self.person_callback, 10)

        # Ultrasonics only (no LiDAR on this rover)
        self.create_subscription(Float32MultiArray, '/ultrasonic_sensors', self.ultrasonic_callback, 10)

        # --- Camera PID parameters (ELP-USB100W07M-MHV120: 1280x720, 120deg HFOV) ---
        self.Kp = 0.001
        self.image_center_x = 640       # Half of 1280px width
        self.target_bbox_width = 800    # Stop when person fills ~2/3 of 1280px frame

        # --- Obstacle thresholds (meters) ---
        self.OBSTACLE_THRESHOLD = 0.3
        self.STOP_DISTANCE = 0.5

        # --- State ---
        self.whistle_detected = False
        self.person_detected = False
        self.person_bbox = None         # dict: x, y, w, h

        # --- Ultrasonic readings ---
        self.ultrasonic_front = float('inf')
        self.ultrasonic_left  = float('inf')
        self.ultrasonic_right = float('inf')

        # --- Control loop: 20 Hz ---
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Path Planning Node initialized — waiting for whistle...')

    #  Callbacks                                                           

    def whistle_detected_callback(self, msg: Bool):
        self.whistle_detected = msg.data
        if msg.data:
            self.get_logger().info('Whistle detected! Ready to track person.')

    def person_callback(self, msg: Float32MultiArray):
        """Vision node publishes [x, y, w, h, confidence]."""
        if len(msg.data) >= 5 and msg.data[4] > 0.7:
            self.person_detected = True
            self.person_bbox = {
                'x': msg.data[0],
                'y': msg.data[1],
                'w': msg.data[2],
                'h': msg.data[3],
            }
        else:
            self.person_detected = False
            self.person_bbox = None

    def ultrasonic_callback(self, msg: Float32MultiArray):
        """Expects [front, left, right] in meters."""
        if len(msg.data) >= 3:
            self.ultrasonic_front = msg.data[0]
            self.ultrasonic_left  = msg.data[1]
            self.ultrasonic_right = msg.data[2]

    #  Control logic                                                       

    def camera_pid_control(self):
        """
        Returns (linear_vel, angular_vel) based on person position in frame.
        Centers person horizontally and approaches until close enough.
        """
        if not self.person_detected or self.person_bbox is None:
            return 0.0, 0.0

        center_x = self.person_bbox['x'] + self.person_bbox['w'] / 2
        error_x  = center_x - self.image_center_x

        angular = self.Kp * error_x
        angular = max(-0.5, min(0.5, angular))

        linear = 0.2 if self.person_bbox['w'] < self.target_bbox_width else 0.0

        return linear, angular

    def obstacle_avoidance(self, desired_linear, desired_angular):
        """
        Overrides desired velocities if an obstacle is detected.
        Ultrasonics only (no LiDAR on this rover).
        """
        if self.ultrasonic_front > self.OBSTACLE_THRESHOLD:
            return desired_linear, desired_angular
        elif self.ultrasonic_right > self.OBSTACLE_THRESHOLD:
            self.get_logger().warn('Obstacle ahead — turning right.')
            return 0.0, -0.5
        elif self.ultrasonic_left > self.OBSTACLE_THRESHOLD:
            self.get_logger().warn('Obstacle ahead and right — turning left.')
            return 0.0, 0.5
        else:
            self.get_logger().warn('All directions blocked — reversing.')
            return -0.1, 0.0
        
    #  Main control loop (20 Hz timer)                                    
  
    def control_loop(self):
        # Priority 1: No whistle heard yet — stay still
        # sound_turn_controller handles turning toward the whistle angle
        if not self.whistle_detected:
            self.publish_cmd(0.0, 0.0)
            return

        # Priority 2: Person visible — camera PID takes over from sound_turn_controller
        if self.person_detected:
            linear, angular = self.camera_pid_control()
            linear, angular = self.obstacle_avoidance(linear, angular)
            self.publish_cmd(linear, angular)

        # If whistle heard but no person yet, let sound_turn_controller handle turning

    def publish_cmd(self, linear, angular):
        cmd = Twist()
        cmd.linear.x  = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()