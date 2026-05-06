# planner_node.py

import rclpy
from rclpy.node import Node
import numpy as np
import sys
import os
import math

from geometry_msgs.msg      import PoseStamped, Point
from nav_msgs.msg           import Path, Odometry
from std_msgs.msg           import Float32MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

_this_dir = os.path.dirname(os.path.abspath(__file__))
_algo_dir = os.path.join(_this_dir, 'algorithms')
sys.path.insert(0, _algo_dir)
sys.path.insert(0, _this_dir)

print(f"[DEBUG] this_dir:  {_this_dir}")
print(f"[DEBUG] algo_dir:  {_algo_dir}")
print(f"[DEBUG] exists:    {os.path.exists(_algo_dir)}")

from birrt_star import birrt_star
from birrt      import try_connect_tree, merge_path
from rrt        import is_collision_free_path, is_collision
# CLEARANCE imported so get_planning_obstacles can bake it into all obstacles
from mock_data  import START, GOAL, MAP_SIZE, STEP_SIZE, GOAL_BIAS, REWIRE_RADIUS, SIGMA, CLEARANCE


class PlannerNode(Node):

    def __init__(self):
        super().__init__('birrt_star_planner')
        self.get_logger().info('Bi-RRT* Planner Node starting...')

        self.current_pos        = None
        self.current_goal       = None
        self.current_path       = None
        self.obstacles          = []
        self.obs_history        = {}
        self.obstacle_timestamp = 0.0
        self.fw_tree            = None
        self.rv_tree            = None
        self.active_planning    = False
        self.last_replan_time   = 0.0

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,    '/odom',      self.odom_callback, 10)
        self.obs_sub  = self.create_subscription(Float32MultiArray, '/obstacle_positions', self.obstacle_callback, 10)

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub     = self.create_publisher(Path,        '/planned_path',       qos)
        self.path_viz_pub = self.create_publisher(Marker,      '/path_visualization', qos)
        self.marker_pub   = self.create_publisher(MarkerArray, '/planning_markers',   10)
        self.stop_pub     = self.create_publisher(Bool,        '/robot_stop',         10)

        self.check_timer = self.create_timer(0.15, self.check_path_validity)
        self.get_logger().info('Ready — waiting for goal on /goal_pose...')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_stop(self, stop: bool):
        msg = Bool(); msg.data = stop
        self.stop_pub.publish(msg)

    def get_planning_obstacles(self):
        """
        Build the obstacle list for birrt_star (which uses clearance=0.0).
        ALL safety margin must be baked into the radius here.

        KEY FIX: Static obstacles (speed=0) previously got:
            inflated_r = r + speed*1.5 = r + 0 = r   ← NO margin at all!
        Now ALL obstacles get CLEARANCE baked in:
            static:  inflated_r = r + CLEARANCE
            moving:  inflated_r = r + CLEARANCE + speed*1.5
        """
        planning = []
        for x, y, r, vx, vy in self.obstacles:
            speed = math.sqrt(vx**2 + vy**2)
            # Shift center by planning latency (0 effect for static)
            px = x + vx * 0.8
            py = y + vy * 0.8
            # Bake in clearance for ALL + velocity uncertainty for moving
            inflated_r = r + CLEARANCE + speed * 1.5
            planning.append((px, py, inflated_r, vx, vy))
        return planning

    def get_predicted_obstacles(self, dt=0.5):
        """Shift centers forward dt seconds. Uses RAW radii for validity checks."""
        return [(x + vx*dt, y + vy*dt, r, vx, vy)
                for x, y, r, vx, vy in self.obstacles]

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def obstacle_callback(self, msg):
        """Estimate velocity from position history — no cheating on vx/vy."""
        data         = msg.data
        current_time = self.get_clock().now().nanoseconds / 1e9
        new_obstacles = []

        for i in range(0, len(data), 5):
            if i + 5 <= len(data):
                x, y, r = data[i], data[i+1], data[i+2]
                obs_id  = i // 5
                if obs_id in self.obs_history:
                    prev_x, prev_y, prev_t = self.obs_history[obs_id]
                    dt = current_time - prev_t
                    vx = (x - prev_x) / dt if dt > 0.01 else 0.0
                    vy = (y - prev_y) / dt if dt > 0.01 else 0.0
                else:
                    vx, vy = 0.0, 0.0
                self.obs_history[obs_id] = (x, y, current_time)
                new_obstacles.append((x, y, r, vx, vy))

        self.obstacles = new_obstacles

    def goal_callback(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Goal: {self.current_goal}')
        if self.current_pos is None:
            self.current_pos = (0.3, 5.5)
        self.run_planner()

    # ── Planning ──────────────────────────────────────────────────────────────

    def run_planner(self):
        if self.current_pos is None or self.current_goal is None:
            return

        self._publish_stop(True)
        self.get_logger().info(f'Planning {self.current_pos} → {self.current_goal}')

        path, fw_tree, rv_tree = birrt_star(
            start         = self.current_pos,
            goal          = self.current_goal,
            obstacles     = self.get_planning_obstacles(),
            map_size      = MAP_SIZE,
            max_iter      = 5000,
            step_size     = STEP_SIZE,
            sigma         = SIGMA,
            goal_bias     = GOAL_BIAS,
            rewire_radius = REWIRE_RADIUS
        )

        if path is None:
            self.get_logger().error('No path found!')
            self._publish_stop(False)
            return

        self.current_path    = path
        self.active_planning = True
        self.fw_tree         = fw_tree
        self.rv_tree         = rv_tree

        total = sum(math.sqrt((path[i][0]-path[i-1][0])**2 +
                              (path[i][1]-path[i-1][1])**2)
                    for i in range(1, len(path)))
        self.get_logger().info(f'Path: {total:.2f}m, {len(path)} waypoints')

        self.publish_path(path)
        self.publish_markers(fw_tree, rv_tree, path)
        self.publish_path_visualization(path)
        self._publish_stop(False)

    def check_path_validity(self):
        if self.current_path is None or not self.active_planning:
            return
        if self.current_pos is None or len(self.obstacles) == 0:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_replan_time < 0.5:
            return

        # Find closest waypoint
        min_dist, closest_idx = float('inf'), 0
        for i, wp in enumerate(self.current_path):
            d = math.sqrt((wp[0]-self.current_pos[0])**2 +
                          (wp[1]-self.current_pos[1])**2)
            if d < min_dist:
                min_dist, closest_idx = d, i

        # Check real-time positions with default clearance (raw radii here)
        for i in range(closest_idx, len(self.current_path) - 1):
            if not is_collision_free_path(self.current_path[i],
                                          self.current_path[i+1],
                                          self.obstacles):  # default CLEARANCE
                self.get_logger().warn(f'Segment {i} blocked — replanning')
                self.last_replan_time = current_time
                self.run_planner()
                return

        # Check predicted positions
        predicted = self.get_predicted_obstacles(dt=0.5)
        for i in range(closest_idx, len(self.current_path) - 1):
            if not is_collision_free_path(self.current_path[i],
                                          self.current_path[i+1],
                                          predicted):  # default CLEARANCE
                self.get_logger().warn(f'Segment {i} predicted blocked — replanning')
                self.last_replan_time = current_time
                self.run_planner()
                return

    # ── Publishers ────────────────────────────────────────────────────────────

    def publish_path(self, path):
        ros_path = Path()
        ros_path.header.stamp    = self.get_clock().now().to_msg()
        ros_path.header.frame_id = 'map'
        for pt in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)
        self.get_logger().info(f'Published {len(path)} waypoints')

    def publish_markers(self, fw_tree, rv_tree, path):
        marker_array = MarkerArray()
        mid = 0
        for node in fw_tree:
            if node["parent"] is not None:
                m = Marker()
                m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'forward_tree'; m.id = mid; mid += 1
                m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.02
                m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 0.5
                p1 = Point(); p1.x = float(node["pos"][0]);           p1.y = float(node["pos"][1]);           p1.z = 0.0
                p2 = Point(); p2.x = float(node["parent"]["pos"][0]); p2.y = float(node["parent"]["pos"][1]); p2.z = 0.0
                m.points = [p1, p2]; marker_array.markers.append(m)
        for node in rv_tree:
            if node["parent"] is not None:
                m = Marker()
                m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'reverse_tree'; m.id = mid; mid += 1
                m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.02
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.5
                p1 = Point(); p1.x = float(node["pos"][0]);           p1.y = float(node["pos"][1]);           p1.z = 0.0
                p2 = Point(); p2.x = float(node["parent"]["pos"][0]); p2.y = float(node["parent"]["pos"][1]); p2.z = 0.0
                m.points = [p1, p2]; marker_array.markers.append(m)
        if path and len(path) > 1:
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'path'; m.id = mid
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.05
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0
            for pt in path:
                p = Point(); p.x = float(pt[0]); p.y = float(pt[1]); p.z = 0.0
                m.points.append(p)
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    def publish_path_visualization(self, path):
        if not path or len(path) < 2: return
        m = Marker()
        m.header.frame_id = 'odom'; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'planned_path'; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD; m.scale.x = 0.05
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
        for pt in path:
            p = Point(); p.x = float(pt[0]); p.y = float(pt[1]); p.z = 0.05
            m.points.append(p)
        self.path_viz_pub.publish(m)
        self.get_logger().info(f'Path viz: {len(path)} points')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()