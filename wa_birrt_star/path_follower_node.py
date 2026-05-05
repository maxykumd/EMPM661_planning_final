import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg      import Path, Odometry
from geometry_msgs.msg import Twist


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower')
        self.get_logger().info('Path Follower Node started')

        # State
        self.current_pos  = None
        self.current_yaw  = 0.0
        self.path         = []
        self.waypoint_idx = 0
        self.active       = False

        # Tuning
        self.waypoint_threshold = 0.30 #how close robot need to get to wp before considered reached and moving to next
        self.linear_speed       = 0.12
        self.angular_gain       = 1.5
        self.max_angular        = 1.0

        # Subscribers
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Waiting for path...')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pos = (x, y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            return
        self.path = []
        for pose in msg.poses:
            self.path.append((
                pose.pose.position.x,
                pose.pose.position.y
            ))
        self.waypoint_idx = 0
        self.active       = True
        self.get_logger().info(
            f'New path received — {len(self.path)} waypoints'
        )

    def control_loop(self):
        if not self.active:
            return
        if self.current_pos is None:
            return

        cx, cy = self.current_pos

        # Skip reached waypoints
        while self.waypoint_idx < len(self.path) - 1:
            target = self.path[self.waypoint_idx]
            dx     = target[0] - cx
            dy     = target[1] - cy
            if math.sqrt(dx**2 + dy**2) < self.waypoint_threshold:
                self.waypoint_idx += 1
                self.get_logger().info(
                    f'Waypoint {self.waypoint_idx}/{len(self.path)} reached'
                )
            else:
                break

        # Check if we're close enough to the final goal
        final_goal = self.path[-1]
        dx = final_goal[0] - self.current_pos[0]
        dy = final_goal[1] - self.current_pos[1]
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        if dist_to_goal < self.waypoint_threshold or \
        self.waypoint_idx >= len(self.path):
            self.stop_robot()
            self.active = False
            self.get_logger().info('Goal reached!')
            return

        # Lookahead 1 waypoints ahead for smoother motion
        lookahead = min(self.waypoint_idx + 1, len(self.path) - 1)
        target    = self.path[lookahead]
        dx        = target[0] - cx
        dy        = target[1] - cy

        angle_to_target = math.atan2(dy, dx)
        angle_error     = angle_to_target - self.current_yaw

        # Normalize to [-pi, pi]
        while angle_error >  math.pi: angle_error -= 2 * math.pi
        while angle_error < -math.pi: angle_error += 2 * math.pi

        cmd = Twist()
        if abs(angle_error) > 0.4:
            # Rotate in place
            cmd.linear.x  = 0.0
            cmd.angular.z = float(max(-self.max_angular,
                                min(self.max_angular,
                                    self.angular_gain * angle_error)))
        else:
            # Move forward with correction
            cmd.linear.x  = self.linear_speed
            cmd.angular.z = float(max(-self.max_angular,
                                min(self.max_angular,
                                    self.angular_gain * angle_error)))
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()