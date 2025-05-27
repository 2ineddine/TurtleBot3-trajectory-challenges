import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class TennisBallFollower(Node):
    def __init__(self):
        super().__init__('tennis_ball_follower')
        
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # HSV thresholds for tennis ball (green)
        self.lower_ball = np.array([25, 80, 80])
        self.upper_ball = np.array([35, 255, 255])

        # HSV thresholds for goal (red)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

        self.goal_detected = False
        self.ball_lost_counter = 0
        self.searching = False

    def image_callback(self, msg):
        # Convert the compressed image data to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if frame is None:
            self.get_logger().error("Image decoding failed.")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Ball detection (green)
        mask_ball = cv2.inRange(hsv, self.lower_ball, self.upper_ball)
        contours_ball, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Goal detection (red)
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_goal = mask_red1 | mask_red2
        contours_goal, _ = cv2.findContours(mask_goal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_ball:
            largest_contour = max(contours_ball, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 200:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cx = x + w // 2
                cy = y + h // 2

                # Draw the bounding box and center point of the ball
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)

                self.move_towards_ball(cx, w, frame.shape[1])

                # If the ball is close and the goal is detected
                if area > 3000 and contours_goal:
                    self.get_logger().info("Goal detected! Pushing toward the goal.")
                    self.push_towards_goal()
            else:
                self.start_searching()

        else:
            self.ball_lost_counter += 1
            self.get_logger().warn(f"Ball lost... {self.ball_lost_counter} frames without detection.")
            if self.ball_lost_counter > 20:
                self.start_searching()

        # Display the frames and masks
        cv2.imshow("Tennis Ball Tracking", frame)
        cv2.imshow("Mask Ball", mask_ball)
        cv2.imshow("Mask Goal", mask_goal)
        cv2.waitKey(1)

    def move_towards_ball(self, cx, ball_width, frame_width):
        # Calculate error and apply proportional control
        error = cx - (frame_width // 2)
        twist = Twist()
        twist.linear.x = max(0.05, 0.05 + (ball_width / 1000.0))
        twist.angular.z = -error / 300.0
        self.get_logger().info(f"Moving toward the ball - Error: {error}, Speed: {twist.linear.x}")
        self.cmd_pub.publish(twist)

    def push_towards_goal(self):
        # Move forward slowly toward the goal
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def start_searching(self):
        # Start rotating in place to find the ball
        self.searching = True
        twist = Twist()
        twist.angular.z = 0.3
        self.cmd_pub.publish(twist)
        self.get_logger().info("Searching for the ball... Rotating.")

    def stop_robot(self):
        # Stop the robot
        twist = Twist()
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TennisBallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
