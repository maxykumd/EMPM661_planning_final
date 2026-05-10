#!/usr/bin/env python3
# path_follower_node.py

import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg       import Path, Odometry
from geometry_msgs.msg  import Twist
from std_msgs.msg       import Float32MultiArray, Bool


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower')
        self.get_logger().info('Path Follower Node started')

        # Differential drive constants
        self.wheel_radius = 0.033
        self.wheel_dist   = 0.160
        self.max_rpm      = 50.0

        # State
        self.current_pos      = None
        self.current_yaw      = 0.0
        self.path             = []
        self.waypoint_idx     = 0
        self.active           = False
        self.planner_stop     = False
        self.moving_obstacles = []

        # Tuning
        self.waypoint_threshold = 0.15 # how close to "reach" a waypoint
        self.linear_speed       = 0.20
        self.angular_gain       = 2.5
        self.max_angular        = 1.0
        self.turn_threshold     = 0.20
        self.safety_radius      = 0.20   # stop if obstacle this close
        self.slow_radius        = 0.80   # start decelerating at this dist

        # Subscribe to recieve : route x,y / robot_pose / obstacle_pose
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/robot_stop', self.stop_callback, 10)
        self.create_subscription(Float32MultiArray, '/obstacle_positions', self.obs_callback,  10)

        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop) # run control loop every 0.1s

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg): # store pos and yaw
        p = msg.pose.pose.position
        self.current_pos = (p.x, p.y)
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )

    def path_callback(self, msg):
        """ Update internal path and replanning"""
        if not msg.poses:
            return
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        # Start from closest point on new path — don't drive backward on replan
        if self.current_pos and self.active:
            self.waypoint_idx = min(range(len(new_path)),key=lambda i: math.dist(self.current_pos, new_path[i]))
        else:
            self.waypoint_idx = 0 # start at beginning of path

        self.path   = new_path
        self.active = True
        self.get_logger().info(f'Path loaded: {len(self.path)} waypoints, idx={self.waypoint_idx}')

    def stop_callback(self, msg):
        self.planner_stop = msg.data
        if self.planner_stop:
            self.stop_robot()

    def obs_callback(self, msg):
        data = msg.data
        self.moving_obstacles = []
        for i in range(0, len(data), 5):
            if i + 5 > len(data): break    
            # extract data : x,y pos radius velocity x,y   
            ox, oy, r, vx, vy = data[i], data[i+1], data[i+2],data[i+3], data[i+4]
            # Filter for 'Moving' where obstacles v>0.05m/s
            is_moving_in_x = abs(vx) > 0.05
            is_moving_in_y = abs(vy) > 0.05
            
            if is_moving_in_x or is_moving_in_y:
                obstacle_tuple = (ox, oy, r, vx, vy) # Store obstacle as tuple
                self.moving_obstacles.append(obstacle_tuple)

    # Velocity-aware obstacle scoring ────────────────────────────────
    def _closest_obstacle_info(self):
        """Calculate score for each obstacle by how far + fast obj move to robot"""
        if not self.moving_obstacles or self.current_pos is None:
            return None

        cx, cy  = self.current_pos # robot center
        best    = None
        best_score = float('inf')

        for ox, oy, r, vx, vy in self.moving_obstacles:
            dx, dy   = ox - cx, oy - cy
            dist     = math.hypot(dx, dy) # straight-line dist
            surf_dist = dist - r

            # Project obstacle velocity onto direction toward robot
            # Positive = moving away, Negative = approaching
            relative_speed = (dx*vx + dy*vy) / (dist + 1e-6)

            # Score: penalise approaching obstacles
            score = surf_dist - 0.5 * relative_speed
            if score < best_score:
                best_score = score
                best = (surf_dist, relative_speed) # dist + incoming obs speed

        return best

    def _speed_scale(self):
        """ Brakes harder when obstacle is approaching vs moving away. """
        info = self._closest_obstacle_info()
        if info is None:
            return 1.0

        # either stop or go full speed
        dist, relative_speed = info
        if dist <= self.safety_radius:
            return 0.0
        if dist >= self.slow_radius:
            return 1.0

        factor = (dist - self.safety_radius) / (self.slow_radius - self.safety_radius)

        # If obstacle approaching (relative_speed < 0), cut speed in half
        # so robot has more time to react before obstacle arrives
        if relative_speed < 0:
            factor *= 0.5

        return max(0.0, min(1.0, factor))

    def _path_ahead_blocked(self):
        """Check next 3 waypoints against moving obstacles. Return false if everything clear"""
        end = min(self.waypoint_idx + 3, len(self.path))
        return any(
            math.hypot(wp[0]-ox, wp[1]-oy) - r < self.safety_radius # dist to obs surface
            for wp in self.path[self.waypoint_idx:end]
            for ox, oy, r, *_ in self.moving_obstacles
        )

    # ── Kinematics ────────────────────────────────────────────────────────────

    def _clamp_rpms(self, cmd):
        """ Convert twist cmd to wheel rpm with differential drive kinematic"""
        r, L  = self.wheel_radius, self.wheel_dist
        rpm_l = (cmd.linear.x - cmd.angular.z*L/2) / r * 60/(2*math.pi)
        rpm_r = (cmd.linear.x + cmd.angular.z*L/2) / r * 60/(2*math.pi)
        peak  = max(abs(rpm_l), abs(rpm_r))
        if peak > self.max_rpm:
            s = self.max_rpm / peak
            cmd.linear.x  *= s
            cmd.angular.z *= s
        return cmd

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        if not self.active or self.current_pos is None:
            return

        #1 : planner force stop if deadend or replan
        if self.planner_stop:
            self.stop_robot()
            return

        # 2 : stop if obs inside safety_radius or in next 3 waypoint
        scale = self._speed_scale()
        if scale == 0.0 or self._path_ahead_blocked():
            self.stop_robot()
            return

        #3 : Advance waypoint index
        cx, cy = self.current_pos
        while self.waypoint_idx < len(self.path) - 1:
            if math.dist((cx,cy), self.path[self.waypoint_idx]) < self.waypoint_threshold:
                self.waypoint_idx += 1
            else:
                break

        # Goal check
        if math.dist((cx,cy), self.path[-1]) < self.waypoint_threshold:
            self.stop_robot()
            self.active = False
            self.get_logger().info('Goal reached')
            return

        # Steer toward current waypoint
        tx, ty    = self.path[min(self.waypoint_idx, len(self.path)-1)]
        angle_err = math.atan2(ty-cy, tx-cx) - self.current_yaw
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err)) #angle wrapping

        # Either turn-inplace or drive forward
        cmd = Twist()
        if abs(angle_err) > self.turn_threshold: # angle error to big -> turn in place
            cmd.linear.x  = 0.0
            cmd.angular.z = float(max(-self.max_angular,
                                      min(self.max_angular,
                                          self.angular_gain * angle_err)))
        else: # drive while correcting angle
            cmd.linear.x  = float(self.linear_speed * scale)
            cmd.angular.z = float(self.angular_gain * angle_err)

        self.cmd_pub.publish(self._clamp_rpms(cmd))

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