import rclpy
import numpy as np
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarOnlyNavigator(Node):
    def __init__(self):
        super().__init__('lidar_v_avoider')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.threshold = 0.26
        self.turn_speed = 0.4
        self.diag_speed = 0.06
        self.base_speed = 0.07

        self.turn_duration = 0.6
        self.diag_duration = 1.2
        self.centering_duration = 1.2  # fixed time for recentering

        self.state = 'FORWARD'
        self.turn_dir = 0
        self.start_time = None

        # Handle only one obstacle per side
        self.left_obstacle_handled = False
        self.right_obstacle_handled = False

        self.get_logger().info("Node initialized: V-shaped avoidance with fixed directional recentering (1 obstacle per side)")

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        clean = lambda x: x[(np.isfinite(x)) & (x > 0.05) & (x < 3.0)]

        left = np.min(clean(ranges[10:50])) if clean(ranges[10:50]).size > 0 else float('inf')
        right = np.min(clean(ranges[300:350])) if clean(ranges[300:350]).size > 0 else float('inf')
        front_zone = clean(np.concatenate((ranges[355:], ranges[:5])))
        front = np.min(front_zone) if front_zone.size > 0 else float('inf')

        self.get_logger().info(f"[DISTANCES] Left: {left:.2f} m | Right: {right:.2f} m | Front: {front:.2f} m")

        cmd = Twist()
        now = time.time()

        if self.state == 'FORWARD':
            # Only detect unhandled obstacles
            obstacle_left = left < self.threshold and not self.left_obstacle_handled
            obstacle_right = right < self.threshold and not self.right_obstacle_handled
            obstacle_front = front < self.threshold

            if obstacle_front or obstacle_left or obstacle_right:
                # Priority to the free side
                if obstacle_left and not obstacle_right:
                    self.turn_dir = -1  # turn right
                    self.left_obstacle_handled = True
                elif obstacle_right and not obstacle_left:
                    self.turn_dir = 1  # turn left
                    self.right_obstacle_handled = True
                else:
                    # both sides blocked or front obstacle
                    self.turn_dir = 1 if left > right else -1
                    if self.turn_dir == 1:
                        self.right_obstacle_handled = True
                    else:
                        self.left_obstacle_handled = True

                self.state = 'TURN_AWAY'
                self.start_time = now
                self.get_logger().warn("Obstacle detected → starting avoidance (rotation)")
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed * self.turn_dir
            else:
                cmd.linear.x = self.base_speed
                cmd.angular.z = 0.0
                self.get_logger().info("Normal forward motion")

        elif self.state == 'TURN_AWAY':
            if now - self.start_time < self.turn_duration:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed * self.turn_dir
                self.get_logger().info("Rotating in place (V phase 1)")
            else:
                self.state = 'DIAGONAL'
                self.start_time = now
                cmd.linear.x = self.diag_speed
                cmd.angular.z = self.turn_speed * 0.5 * self.turn_dir
                self.get_logger().info("Starting diagonal motion (V phase 2)")

        elif self.state == 'DIAGONAL':
            if now - self.start_time < self.diag_duration:
                cmd.linear.x = self.diag_speed * 1.5
                cmd.angular.z = self.turn_speed * 0.5 * self.turn_dir
                self.get_logger().info("Diagonal motion to avoid obstacle")
            else:
                self.state = 'CENTERING'
                self.start_time = now
                self.get_logger().info("Diagonal complete → starting fixed recentering")

        elif self.state == 'CENTERING':
            if now - self.start_time < self.centering_duration:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.turn_speed * 1.1 * self.turn_dir
                self.get_logger().info("Automatic recentering (reverse rotation)")
            else:
                self.state = 'FORWARD'
                cmd.linear.x = self.base_speed
                cmd.angular.z = 0.0
                self.get_logger().info("Recenter complete → resuming forward motion")

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LidarOnlyNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Manual shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
