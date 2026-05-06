# planner_node.py

import rclpy
from rclpy.node import Node
import sys
import os
import math

from geometry_msgs.msg      import PoseStamped, Point
from nav_msgs.msg           import Path, Odometry
from std_msgs.msg           import Float32MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos              import QoSProfile, DurabilityPolicy, ReliabilityPolicy

_this_dir = os.path.dirname(os.path.abspath(__file__))
_algo_dir = os.path.join(_this_dir, 'algorithms')
sys.path.insert(0, _algo_dir)

import importlib.util

def _load_algo(name, filename):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(_algo_dir, filename))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

_wa  = _load_algo('wa_birrt_star_algo', 'wa_birrt_star.py')
_rrt = _load_algo('rrt_algo',           'rrt.py')
_md  = _load_algo('mock_data_algo',     'mock_data.py')

wa_birrt_star     = _wa.wa_birrt_star
is_collision_free = _rrt.is_collision_free_path
MAP_SIZE, STEP_SIZE, GOAL_BIAS = _md.MAP_SIZE, _md.STEP_SIZE, _md.GOAL_BIAS
REWIRE_RADIUS, SIGMA, CLEARANCE = _md.REWIRE_RADIUS, _md.SIGMA, _md.CLEARANCE


class PlannerNode(Node):

    def __init__(self):
        super().__init__('birrt_star_planner')
        self.get_logger().info('WA* + Bi-RRT* Planner Node starting...')

        self.current_pos       = None
        self.current_goal      = None
        self.current_path      = None
        self.obstacles         = []
        self.fw_tree           = None
        self.rv_tree           = None
        self.active_planning   = False
        self.last_replan_time  = 0.0
        self.blocked_counter   = 0
        self.blocked_threshold = 3
        self._replan_count     = 0

        self.create_subscription(PoseStamped,       '/goal_pose',          self.goal_callback,     10)
        self.create_subscription(Odometry,          '/odom',               self.odom_callback,     10)
        self.create_subscription(Float32MultiArray, '/obstacle_positions',  self.obstacle_callback, 10)

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub     = self.create_publisher(Path,        '/planned_path',       qos)
        self.path_viz_pub = self.create_publisher(Marker,      '/path_visualization', qos)
        self.marker_pub   = self.create_publisher(MarkerArray, '/planning_markers',   10)
        self.stop_pub     = self.create_publisher(Bool,        '/robot_stop',         10)

        self.create_timer(0.15, self.check_path_validity)
        self.get_logger().info('Ready — waiting for goal on /goal_pose...')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self, stop: bool):
        msg = Bool(); msg.data = stop
        self.stop_pub.publish(msg)

    def _planning_obstacles(self):
        """
        Inflate obstacles for planning — ALL margin baked in here.
        Bi-RRT* uses clearance=0.0 internally.
          static:  r + CLEARANCE
          moving:  r + CLEARANCE + speed*1.5 + center shifted 0.8s forward
        """
        return [
            (x + vx*0.8, y + vy*0.8,
             r + CLEARANCE + math.hypot(vx, vy)*1.5,
             vx, vy)
            for x, y, r, vx, vy in self.obstacles
        ]

    def _is_time_collision_free(self, p1, p2, t0, robot_speed=0.20):
        """
        Time-aware segment collision check against RAW obstacles.
        Adds CLEARANCE here since raw obstacle radii are unmodified.
        Uses uncertainty growth for moving obstacles to account for
        prediction error increasing over time.

        Checks where each obstacle WILL BE when the robot arrives at
        each point along the segment — not where it is right now.
        """
        dist = math.dist(p1, p2)
        if dist < 1e-6:
            return True
        steps = max(10, int(dist / 0.05))   # 5cm resolution

        for i in range(steps + 1):
            alpha = i / steps
            x = p1[0] + alpha*(p2[0]-p1[0])
            y = p1[1] + alpha*(p2[1]-p1[1])
            t = t0 + alpha*dist/robot_speed

            for ox, oy, r, vx, vy in self.obstacles:
                px = ox + vx*t
                py = oy + vy*t
                # uncertainty grows with time for moving obstacles only
                speed = math.hypot(vx, vy)
                uncertainty = 0.07*t if speed > 0.01 else 0.0
                effective_r = r + CLEARANCE + uncertainty
                if math.hypot(x-px, y-py) < effective_r:
                    return False
        return True

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.current_pos = (p.x, p.y)

    def obstacle_callback(self, msg):
        """Read positions and velocities directly from publisher."""
        data = msg.data
        obs  = []
        for i in range(0, len(data), 5):
            if i+5 > len(data): break
            obs.append((data[i], data[i+1], data[i+2], data[i+3], data[i+4]))
        self.obstacles = obs

    def goal_callback(self, msg):
        self.current_goal    = (msg.pose.position.x, msg.pose.position.y)
        self.blocked_counter = 0
        self._replan_count   = 0
        self.get_logger().info(f'Goal: {self.current_goal}')
        if self.current_pos is None:
            self.current_pos = (0.3, 5.5)

        # Wait up to 2s for obstacle data before planning
        if not self.obstacles:
            self.get_logger().warn('Waiting for obstacle data...')
            import time
            deadline = time.time() + 2.0
            while not self.obstacles and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)

        if not self.obstacles:
            self.get_logger().warn('No obstacle data — using mock_data fallback')
            self.obstacles = list(_md.OBSTACLES)

        self.run_planner()

    # ── Planning ──────────────────────────────────────────────────────────────

    def run_planner(self):
        if not self.current_pos or not self.current_goal:
            return

        self._stop(True)
        self._replan_count += 1
        self.get_logger().info(
            f'Planning {self.current_pos} → {self.current_goal} '
            f'({len(self.obstacles)} obstacles) [plan #{self._replan_count}]'
        )

        path, fw_tree, rv_tree = wa_birrt_star(
            start         = self.current_pos,
            goal          = self.current_goal,
            obstacles     = self._planning_obstacles(),
            map_size      = MAP_SIZE,
            max_iter      = 8000,
            step_size     = STEP_SIZE,
            sigma         = SIGMA,
            goal_bias     = GOAL_BIAS,
            rewire_radius = REWIRE_RADIUS,
            wa_epsilon    = 1.8
        )

        if path is None:
            self.get_logger().error('No path found!')
            self._stop(False)
            return

        self.current_path    = path
        self.active_planning = True
        self.fw_tree         = fw_tree
        self.rv_tree         = rv_tree
        self.blocked_counter = 0

        length = sum(math.dist(path[i], path[i-1]) for i in range(1, len(path)))
        self.get_logger().info(f'Path: {length:.2f}m  {len(path)} waypoints')

        self.publish_path(path)
        self.publish_markers(fw_tree, rv_tree, path)
        self.publish_path_viz(path)
        self._stop(False)

    def check_path_validity(self):
        if not self.current_path or not self.active_planning or not self.current_pos:
            return
        if not self.obstacles:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < 0.5:
            return

        closest = min(range(len(self.current_path)),
                      key=lambda i: math.dist(self.current_pos, self.current_path[i]))

        path_blocked = False
        TIME_HORIZON = 4.0
        t_elapsed    = 0.0

        for i in range(closest, len(self.current_path)-1):
            seg_len = math.dist(self.current_path[i], self.current_path[i+1])
            if t_elapsed > TIME_HORIZON:
                break
            # Uses raw obstacles + adds CLEARANCE inside the function
            if not self._is_time_collision_free(
                self.current_path[i], self.current_path[i+1], t_elapsed
            ):
                self.get_logger().warn(
                    f'Segment {i} blocked at t={t_elapsed:.1f}s — replan needed'
                )
                path_blocked = True
                break
            t_elapsed += seg_len / 0.20

        if path_blocked:
            self.blocked_counter += 1
            self.get_logger().info(
                f'Blocked count: {self.blocked_counter}/{self.blocked_threshold}'
            )
            if self.blocked_counter >= self.blocked_threshold:
                self.blocked_counter  = 0
                self.last_replan_time = now
                self.run_planner()
        else:
            self.blocked_counter = 0

    # ── Publishers ────────────────────────────────────────────────────────────

    def publish_path(self, path):
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for x, y in path:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        self.get_logger().info(f'Published {len(path)} waypoints')

    def publish_markers(self, fw_tree, rv_tree, path):
        arr = MarkerArray()
        mid = 0

        def tree_marker(node, ns, r, g, b):
            nonlocal mid
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = ns; m.id = mid; mid += 1
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.02
            m.color.r = float(r); m.color.g = float(g)
            m.color.b = float(b); m.color.a = 0.5
            p1 = Point(); p1.x,p1.y = float(node["pos"][0]), float(node["pos"][1])
            p2 = Point(); p2.x,p2.y = float(node["parent"]["pos"][0]), float(node["parent"]["pos"][1])
            m.points = [p1, p2]
            arr.markers.append(m)

        for node in fw_tree:
            if node["parent"]: tree_marker(node, 'fw', 0.0, 0.0, 1.0)
        for node in rv_tree:
            if node["parent"]: tree_marker(node, 'rv', 0.0, 1.0, 0.0)

        if path and len(path) > 1:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = 'path'; m.id = mid
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.05
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0
            for x, y in path:
                p = Point(); p.x = float(x); p.y = float(y)
                m.points.append(p)
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def publish_path_viz(self, path):
        if not path or len(path) < 2: return
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = 'planned_path'; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.05
        m.color.g = 1.0; m.color.a = 1.0
        for x, y in path:
            p = Point(); p.x = float(x); p.y = float(y); p.z = 0.05
            m.points.append(p)
        self.path_viz_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()