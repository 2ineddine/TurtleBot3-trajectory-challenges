

import rclpy, cv2, time, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class AutonomousObstacleAvoider(Node):
    def __init__(self):
        super().__init__('autonomous_obstacle_avoider')

        # Publishers / Subscribers
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel',    10)
        self.challenge3_pub = self.create_publisher(Bool, '/Challenge3', 10)
        self.create_subscription(Image,      '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan,  '/scan',             self.scan_callback,  10)

        # Behaviour parameters
        self.OBSTACLE_DISTANCE = 0.3     # [m] trigger distance
        self.AVOID_DURATION    = 3.0     # [s] time spent in avoidance
        self.TURN_SPEED        = 0.8     # [rad/s] initial turn
        self.LINE_SPEED        = 0.05    # [m/s] nominal forward speed
        self.KP                = 0.002   # gain for centring error
        self.RECENTER_DELAY    = 2.0     # [s] pause after 2nd obstacle

        # State variables
        self.last_red   = None
        self.last_green = None
        self.image_width = 640
        self.avoid_start_time = None
        self.current_line     = None      # 'red' or 'green' during avoidance
        self.obstacles_passed = 0
        self.last_left_dist   = float('inf')
        self.last_right_dist  = float('inf')

        # HSV thresholds
        self.RED1_LO, self.RED1_HI = np.array([0,   40, 40]),  np.array([12,  255, 255])
        self.RED2_LO, self.RED2_HI = np.array([165, 40, 40]),  np.array([180, 255, 255])
        self.GREEN_LO, self.GREEN_HI = np.array([45,  30, 80]), np.array([98,  255, 255])

        self.get_logger().info("AutonomousObstacleAvoider node initialised")

    # ───────────────────────── IMAGE CALLBACK ─────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, self.image_width = frame.shape[:2]
            roi = frame[int(h * 0.34):int(h * 0.8), :]

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            red_mask1  = cv2.inRange(hsv, self.RED1_LO, self.RED1_HI)
            red_mask2  = cv2.inRange(hsv, self.RED2_LO, self.RED2_HI)
            red_mask   = cv2.bitwise_or(red_mask1, red_mask2)
            green_mask = cv2.inRange(hsv, self.GREEN_LO, self.GREEN_HI)

            self.last_red   = self.get_line_position(red_mask)
            self.last_green = self.get_line_position(green_mask)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    # ───────────────────────── LIDAR CALLBACK ─────────────────────────
    def scan_callback(self, msg):
        try:
            ranges = np.array(msg.ranges)
            left  = self.filter_scan_data(ranges[0:40])
            right = self.filter_scan_data(ranges[320:359])

            self.last_left_dist  = left
            self.last_right_dist = right

            twist = Twist()
            now = time.time()

            # Decide whether to start avoidance
            if self.avoid_start_time is None:
                if left < self.OBSTACLE_DISTANCE:
                    self.initiate_avoidance(direction='right', timestamp=now)
                elif right < self.OBSTACLE_DISTANCE:
                    self.initiate_avoidance(direction='left', timestamp=now)

            # If in avoidance: continue or complete
            if self.avoid_start_time:
                if now - self.avoid_start_time < self.AVOID_DURATION:
                    self.execute_avoidance(twist)
                else:
                    self.complete_avoidance()
                    return  # node may shut down inside complete_avoidance
            else:
                self.follow_line(twist)

            self.cmd_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"LiDAR processing error: {e}")

    # ───────────────────────── LINE LOGIC ─────────────────────────
    def follow_line(self, twist):
        center_img = self.image_width // 2
        if self.last_red is not None and self.last_green is not None:
            mid   = (self.last_red + self.last_green) / 2
            error = mid - center_img
            twist.linear.x = self.LINE_SPEED
            twist.angular.z = -self.KP * error
        elif self.last_red is not None:
            error = self.last_red - center_img
            twist.linear.x = self.LINE_SPEED * 0.8
            twist.angular.z = -self.KP * error * 0.5
        elif self.last_green is not None:
            error = self.last_green - center_img
            twist.linear.x = self.LINE_SPEED * 0.8
            twist.angular.z = -self.KP * error * 0.5
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # spin to search

    # ───────────────────────── AVOIDANCE LOGIC ─────────────────────────
    def initiate_avoidance(self, direction, timestamp):
        self.get_logger().warn(f"Obstacle detected → turning {direction}")
        self.avoid_start_time = timestamp
        self.current_line = 'green' if direction == 'left' else 'red'
        twist = Twist()
        twist.angular.z = self.TURN_SPEED if direction == 'left' else -self.TURN_SPEED
        self.cmd_pub.publish(twist)
        time.sleep(0.5)  # brief spin before reacquiring a line

    def execute_avoidance(self, twist):
        center = self.image_width // 2
        line_pos = self.last_red if self.current_line == 'red' else self.last_green

        if line_pos is not None:
            error = line_pos - center
            twist.linear.x = self.LINE_SPEED
            twist.angular.z = -self.KP * error
        else:
            twist.linear.x = self.LINE_SPEED * 0.5
            twist.angular.z = 0.0

    def complete_avoidance(self):
        self.avoid_start_time = None
        self.current_line = None
        self.obstacles_passed += 1
        self.get_logger().info(f"Avoidance finished ({self.obstacles_passed}/2)")

        if self.obstacles_passed >= 2:
            self.get_logger().info("Two obstacles cleared → recentring before hand-over")
            time.sleep(self.RECENTER_DELAY)                   # allow recentering
            self.cmd_pub.publish(Twist())                     # stop the robot
            self.challenge3_pub.publish(Bool(data=True))      # trigger Challenge 3
            self.get_logger().info("Hand-over done → shutting down node")
            rclpy.shutdown()

    # ───────────────────────── HELPERS ─────────────────────────
    def filter_scan_data(self, sector):
        valid = sector[(sector > 0.05) & (sector < 2.0) & np.isfinite(sector)]
        return np.mean(valid) if valid.size > 2 else float('inf')

    def get_line_position(self, mask):
        M = cv2.moments(mask)
        return int(M['m10'] / M['m00']) if M['m00'] > 200 else None


# ───────────────────────── MAIN ENTRY POINT ─────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
