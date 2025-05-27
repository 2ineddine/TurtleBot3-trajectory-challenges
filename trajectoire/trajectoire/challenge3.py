import rclpy, numpy as np, time
from rclpy.node        import Node
from sensor_msgs.msg   import LaserScan
from std_msgs.msg      import Bool
from geometry_msgs.msg import Twist

class CorridorNavigationNode(Node):
    def __init__(self):
        super().__init__('corridor_navigation_node')

        # Publishers and subscribers
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel',             10)
        self.corr_pub = self.create_publisher(Bool,  '/Corridor_Navigation', 10)
        self.create_subscription(LaserScan, '/scan',       self.scan_cb, 10)
        self.create_subscription(Bool,      '/Challenge3', self.ch3_cb,  10)

        # Internal state
        self.mode             = 'IDLE'    # IDLE → ACTIVE
        self.mode_since       = time.time()
        self.block_exit_delay = 10.0      # Seconds before allowing shutdown
        self.can_exit         = False
        self.K_ang            = -2.0
        self.FALLBACK_DIST    = 0.15

        self.corr_pub.publish(Bool(data=False))
        self.get_logger().info("CorridorNavigationNode initialized. Waiting for Challenge3 signal.")

        self.create_timer(0.05, self.timer_cb)

    # ─────────────────────────── CALLBACKS ───────────────────────────

    def ch3_cb(self, msg: Bool):
        if msg.data and self.mode == 'IDLE':
            self.mode = 'ACTIVE'
            self.mode_since = time.time()
            self.can_exit = False
            self.corr_pub.publish(Bool(data=True))
            self.get_logger().info("Challenge3 flag received. Corridor following ACTIVE (exit locked for 10s).")

    def scan_cb(self, msg: LaserScan):
        if self.mode != 'ACTIVE':
            return

        rng = np.array(msg.ranges)
        nl = rng[0:25]
        nr = rng[335:360]
        fl = rng[70:90]
        fr = rng[270:290]

        def mean_or_nan(arr):
            vals = arr[(arr > 0.05) & (arr < 1.2)]
            return np.nan if vals.size == 0 else np.mean(vals)

        nl_m = mean_or_nan(nl)
        nr_m = mean_or_nan(nr)
        fl_m = mean_or_nan(fl)
        fr_m = mean_or_nan(fr)

        # Replace NaNs with fallback values
        nl_m = nl_m if not np.isnan(nl_m) else self.FALLBACK_DIST
        nr_m = nr_m if not np.isnan(nr_m) else self.FALLBACK_DIST
        fl_m = fl_m if not np.isnan(fl_m) else self.FALLBACK_DIST
        fr_m = fr_m if not np.isnan(fr_m) else self.FALLBACK_DIST

        # Allow exit only after timeout
        if self.can_exit:
            if np.isnan(nr_m):
                self.get_logger().info("Opening on right detected. Exiting corridor.")
                self.finish_corridor()
                return
            if fl_m > 0.9 and fr_m > 0.9:
                self.get_logger().info("Front clear. Corridor traversal complete.")
                self.finish_corridor()
                return

        # Trajectory correction
        err = 0.3 * (nr_m - nl_m) + 1.1 * (fr_m - fl_m)
        twist = Twist()
        twist.linear.x = 0.02
        twist.angular.z = self.K_ang * err
        self.cmd_pub.publish(twist)

    def timer_cb(self):
        if self.mode == 'ACTIVE' and not self.can_exit:
            if time.time() - self.mode_since > self.block_exit_delay:
                self.can_exit = True
                self.get_logger().info("10 seconds elapsed. Exit from corridor is now allowed.")

    def finish_corridor(self):
        self.cmd_pub.publish(Twist())              # stop robot
        self.corr_pub.publish(Bool(data=False))    # corridor ended
        self.get_logger().info("CorridorNavigationNode: Task complete. Shutting down.")
        rclpy.shutdown()

# ─────────────────────────── MAIN ───────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CorridorNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
