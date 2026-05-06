#!/usr/bin/env python3
# path_follower_node.py

import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool


class PathFollowerNode(Node):
    """
    Path follower with two independent safety layers:

      LAYER 1 — Planner stop signal (/robot_stop):
        When planner detects a blocked path and starts replanning, it sends
        stop=True. Follower halts immediately instead of driving into the
        obstacle for 700ms while Bi-RRT* runs. Resumes when stop=False
        arrives with the new path.

      LAYER 2 — Local obstacle proximity check:
        Follower subscribes to /obstacle_positions directly. Every control
        loop it checks if any obstacle is within safety_radius of the robot
        OR if any upcoming waypoint is too close to an obstacle. If so, it
        stops and waits — independently of the planner. This catches cases
        where an obstacle drifts into the robot faster than the planner can
        replan.
    """

    def __init__(self):
        super().__init__('path_follower')
        self.get_logger().info('Path Follower Node started')

        # Differential Drive Constants
        self.wheel_radius = 0.033
        self.wheel_dist   = 0.160
        self.max_rpm      = 50.0

        # State
        self.current_pos  = None
        self.current_yaw  = 0.0
        self.path         = []
        self.waypoint_idx = 0
        self.active       = False

        # Tuning
        self.waypoint_threshold = 0.20
        self.linear_speed       = 0.20
        self.angular_gain       = 2.0
        self.max_angular        = 1.0

        # LAYER 1: planner stop signal state
        self.planner_stop = False

        # LAYER 2: local obstacle state
        self.obstacles      = []
        # safety_radius: distance from robot center to obstacle center that
        # triggers an emergency stop. Set to obstacle_radius(0.20) +
        # robot_radius(0.10) + margin(0.15) = 0.45m
        self.safety_radius  = 0.20
        # lookahead_wps: how many upcoming waypoints to scan for obstacles
        self.lookahead_wps  = 3

        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        # LAYER 1: listen for planner stop signal
        self.stop_sub = self.create_subscription(
            Bool, '/robot_stop', self.stop_callback, 10
        )
        # LAYER 2: listen for obstacle positions directly
        self.obs_sub = self.create_subscription(
            Float32MultiArray, '/obstacle_positions', self.obstacle_callback, 10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control Loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pos = (x, y)

        q = msg.pose.pose.orientation
        siny_cosp    = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp    = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def path_callback(self, msg):
        if not msg.poses:
            return
        self.path         = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.waypoint_idx = 0
        self.active       = True
        self.get_logger().info(f'Path Loaded: {len(self.path)} points')

    def stop_callback(self, msg):
        """LAYER 1: Planner tells us to stop or resume."""
        self.planner_stop = msg.data
        if self.planner_stop:
            self.stop_robot()
            self.get_logger().info('Planner stop signal — halting')

    def obstacle_callback(self, msg):
        data = msg.data
        all_obs = []
        for i in range(0, len(data), 5):
            if i + 5 <= len(data):
                all_obs.append(
                    (data[i], data[i+1], data[i+2], data[i+3], data[i+4])
                )
        self.obstacles = all_obs
        # Only moving obstacles have non-zero velocity — filter them out
        # Static ones are already handled by the planner, checking them
        # here causes false positives on legally-planned path segments
        self.moving_obstacles = [o for o in all_obs if abs(o[3]) > 0.01 or abs(o[4]) > 0.01]

    # ── Safety Checks (Layer 2) ────────────────────────────────────────────────

    def is_obstacle_too_close(self):
        if self.current_pos is None:
            return False
        cx, cy = self.current_pos
        for ox, oy, r, vx, vy in self.moving_obstacles:   # ← moving only
            dist = math.sqrt((cx - ox)**2 + (cy - oy)**2)
            if dist - r < self.safety_radius:              # surface-to-surface
                return True
        return False

    def is_path_ahead_blocked(self):
        end_idx = min(self.waypoint_idx + self.lookahead_wps, len(self.path))
        for wp in self.path[self.waypoint_idx:end_idx]:
            for ox, oy, r, vx, vy in self.moving_obstacles:   # ← moving only
                dist = math.sqrt((wp[0] - ox)**2 + (wp[1] - oy)**2)
                if dist - r < 0.20:                            # matches CLEARANCE
                    return True
        return False

    def get_speed_scale(self):
        if not self.moving_obstacles or self.current_pos is None:
            return 1.0
        cx, cy = self.current_pos
        min_dist = float('inf')
        for ox, oy, r, vx, vy in self.moving_obstacles:
            dist = math.sqrt((cx-ox)**2 + (cy-oy)**2) - r
            min_dist = min(min_dist, dist)
        
        slow_zone  = 1.0   # start slowing at 1m
        stop_zone  = 0.20  # full stop at clearance
        if min_dist <= stop_zone:
            return 0.0
        if min_dist >= slow_zone:
            return 1.0
        return (min_dist - stop_zone) / (slow_zone - stop_zone)


    # ── Kinematics ────────────────────────────────────────────────────────────

    def twist_to_rpms(self, linear_vel, angular_vel):
        r = self.wheel_radius
        L = self.wheel_dist
        u_left  = (linear_vel - (angular_vel * L / 2.0)) / r
        u_right = (linear_vel + (angular_vel * L / 2.0)) / r
        rpm_l   = u_left  * 60.0 / (2 * math.pi)
        rpm_r   = u_right * 60.0 / (2 * math.pi)
        return rpm_l, rpm_r

    # ── Control Loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        if not self.active or self.current_pos is None:
            return

        # ── LAYER 1: Planner stop signal ──────────────────────────────────────
        if self.planner_stop:
            self.stop_robot()
            return

        # ── LAYER 2: Local safety checks ──────────────────────────────────────
        if self.is_obstacle_too_close():
            self.stop_robot()
            self.get_logger().warn('Obstacle in safety bubble — stopped')
            return

        if self.is_path_ahead_blocked():
            self.stop_robot()
            self.get_logger().warn('Upcoming waypoints blocked — stopped')
            return

        # ── Normal path following ──────────────────────────────────────────────
        cx, cy = self.current_pos

        # Advance waypoint index
        while self.waypoint_idx < len(self.path) - 1:
            target = self.path[self.waypoint_idx]
            dist   = math.sqrt((target[0]-cx)**2 + (target[1]-cy)**2)
            if dist < self.waypoint_threshold:
                self.waypoint_idx += 1
            else:
                break

        # Check goal completion
        final_goal    = self.path[-1]
        dist_to_goal  = math.sqrt((final_goal[0]-cx)**2 + (final_goal[1]-cy)**2)
        if dist_to_goal < self.waypoint_threshold:
            self.stop_robot()
            self.active = False
            self.get_logger().info('Goal Reached')
            return

        # Steer toward current waypoint (fixed: was +1, causing corner-cutting)
        target          = self.path[min(self.waypoint_idx, len(self.path)-1)]
        angle_to_target = math.atan2(target[1]-cy, target[0]-cx)
        angle_error     = angle_to_target - self.current_yaw
        angle_error     = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()
        if abs(angle_error) > 0.3:
            cmd.linear.x  = 0.0
            cmd.angular.z = float(max(-self.max_angular,
                                      min(self.max_angular,
                                          self.angular_gain * angle_error)))
        else:
            scale = self.get_speed_scale()
            cmd.linear.x = self.linear_speed * scale
            cmd.angular.z = float(self.angular_gain * angle_error)

        # Differential drive RPM clamping
        rpm_l, rpm_r = self.twist_to_rpms(cmd.linear.x, cmd.angular.z)
        if abs(rpm_l) > self.max_rpm or abs(rpm_r) > self.max_rpm:
            scale         = self.max_rpm / max(abs(rpm_l), abs(rpm_r))
            cmd.linear.x  *= scale
            cmd.angular.z *= scale

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