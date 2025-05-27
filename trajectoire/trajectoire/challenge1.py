import rclpy, cv2, time, numpy as np, queue, threading
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

class Challenge1Node(Node):
    def __init__(self):
        super().__init__('challenge1_node')

        self.bridge = CvBridge()
        self.linear_speed = 0.08
        self.angular_gain = 0.014

        self.last_valid_red = None
        self.last_valid_green = None
        self.image_width = 352

        self.avoide_obstacle = False
        self.roundabout_active = False
        self.roundabout_direction = "left"
        self.roundabout_start_time = None
        self.roundabout_duration = 4.2
        self.sent_stop_twist = False

        # Blue line detection
        self.blue_detected = False
        self.blue_area_thresh = 200

        # HSV color ranges
        self.BLUE_LO, self.BLUE_HI   = (98, 180, 120), (112, 255, 180)
        self.GREEN_LO, self.GREEN_HI = (45, 30, 80),   (98, 255, 255)
        self.RED1_LO,  self.RED1_HI  = (0, 40, 40),    (12, 255, 255)
        self.RED2_LO,  self.RED2_HI  = (165, 40, 40),  (180, 255, 255)

        self.front_angle = 80
        self.stop_limit = 0.25
        self.obstacle_cleared_time = None
        self.false_echo_duration = 2.0
        self.lost_start_time = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.challenge2_pub = self.create_publisher(Bool, '/Challenge2', 10)

        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_timer(0.05, self.timer_callback)

        self.image_queue = queue.Queue(maxsize=1)
        threading.Thread(target=self.image_processing_loop, daemon=True).start()

        self.get_logger().info("Challenge1 ready: line + obstacle + roundabout + auto-kill on blue line")

    def image_callback(self, msg):
        try:
            self.image_queue.put_nowait(msg)
        except queue.Full:
            pass

    def image_processing_loop(self):
        while rclpy.ok():
            try:
                msg = self.image_queue.get(timeout=0.1)
                self.process_image(msg)
            except queue.Empty:
                continue

    def process_image(self, msg):
        if self.avoide_obstacle:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        h, self.image_width, _ = img.shape
        roi = img[int(h * 0.34):int(h * 0.8), :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_blue_ = cv2.inRange(hsv, self.BLUE_LO, self.BLUE_HI)
        mask_green = cv2.bitwise_and(cv2.inRange(hsv, self.GREEN_LO, self.GREEN_HI), cv2.bitwise_not(mask_blue_))
        mask_red = cv2.bitwise_and(
            cv2.bitwise_or(cv2.inRange(hsv, self.RED1_LO, self.RED1_HI), cv2.inRange(hsv, self.RED2_LO, self.RED2_HI)),
            cv2.bitwise_not(mask_blue_))

        # Check for blue line
        blue_area = cv2.countNonZero(mask_blue_)
        if blue_area > self.blue_area_thresh and not self.blue_detected:
            self.get_logger().info("Blue line detected - stopping and exiting Challenge1")
            self.blue_detected = True
            self.cmd_pub.publish(Twist())
            self.challenge2_pub.publish(Bool(data=True))
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        detection_mask = cv2.bitwise_or(mask_red, mask_green)
        cv2.imshow("Detection ligne", detection_mask)
        cv2.waitKey(1)

        Mr, Mg = cv2.moments(mask_red), cv2.moments(mask_green)
        self.last_valid_red = int(Mr['m10'] / Mr['m00']) if Mr['m00'] > 200 else None
        self.last_valid_green = int(Mg['m10'] / Mg['m00']) if Mg['m00'] > 200 else None

        self.compute_and_publish_cmd()

    def compute_and_publish_cmd(self):
        if self.avoide_obstacle:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()
        center_point = self.image_width // 2

        if self.last_valid_red is not None and self.last_valid_green is not None:
            dist = abs(self.last_valid_red - self.last_valid_green)
            canal = (self.last_valid_red + self.last_valid_green) // 2
            err = canal - center_point
            gain = self.angular_gain * (1.4 if dist < 60 else 1.0)
            cmd.linear.x = self.linear_speed
            cmd.angular.z = -gain * err

            if dist < 40.0 and not self.roundabout_active:
                self.roundabout_active = True
                self.roundabout_start_time = time.time()
                self.get_logger().info("Roundabout triggered")

        elif self.last_valid_red is not None or self.last_valid_green is not None:
            base_speed = 0.7
            cmd.linear.x = self.linear_speed * base_speed
            distance = abs((self.last_valid_red if self.last_valid_red is not None else self.last_valid_green) - center_point)
            boost = max(0.0, 0.47 * (1 - (distance / 175))) if abs(distance <= 175) else 0.0
            cmd.angular.z = boost if self.last_valid_red is not None else -boost
            if distance <= 30:
                cmd.linear.x = self.linear_speed * 0.2

        else:
            if self.lost_start_time is None:
                self.lost_start_time = time.time()
            lost_duration = time.time() - self.lost_start_time
            if lost_duration < 70.0:
                cmd.linear.x = self.linear_speed * 0.5
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        def clean(values):
            return values[(np.isfinite(values)) & (values > 0.05) & (values < 3.0)]

        left_values = clean(ranges[2:30])
        right_values = clean(ranges[330:358])
        front_values = clean(np.concatenate((ranges[80:100], ranges[260:280])))

        left = np.mean(left_values) if left_values.size > 0 else float('nan')
        right = np.mean(right_values) if right_values.size > 0 else float('nan')
        front = np.mean(front_values) if front_values.size > 0 else float('nan')

        obstacle_left = not np.isnan(left) and left < self.stop_limit
        obstacle_right = not np.isnan(right) and right < self.stop_limit
        obstacle_front = not np.isnan(front) and front < self.stop_limit

        if obstacle_left or obstacle_right or obstacle_front:
            if not self.avoide_obstacle:
                self.cmd_pub.publish(Twist())
                self.avoide_obstacle = True
                self.obstacle_cleared_time = None
        else:
            if self.avoide_obstacle:
                if self.obstacle_cleared_time is None:
                    self.obstacle_cleared_time = time.time()
                    return
                if time.time() - self.obstacle_cleared_time > self.false_echo_duration:
                    self.avoide_obstacle = False

    def timer_callback(self):
        if self.avoide_obstacle:
            return

        if self.roundabout_active:
            if self.roundabout_start_time is None:
                self.roundabout_start_time = time.time()
                return

            elapsed = time.time() - self.roundabout_start_time
            if elapsed < self.roundabout_duration:
                twist = Twist()
                twist.linear.x = self.linear_speed * 0.3
                twist.angular.z = 0.5 if self.roundabout_direction == "left" else -0.8
                self.cmd_pub.publish(twist)
            else:
                self.roundabout_active = False
                self.roundabout_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = Challenge1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
