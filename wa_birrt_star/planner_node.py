import rclpy
from rclpy.node import Node
import numpy as np
import sys
import os

# ROS2 message types
from geometry_msgs.msg      import PoseStamped, Point
from nav_msgs.msg           import Path, Odometry
from std_msgs.msg           import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# imports for ROS2 
_this_dir = os.path.dirname(os.path.abspath(__file__))
_algo_dir = os.path.join(_this_dir, 'algorithms')
sys.path.insert(0, _algo_dir)
sys.path.insert(0, _this_dir)


# Correct
from birrt_star import birrt_star
from birrt      import try_connect_tree, merge_path
from rrt        import is_collision_free_path, is_collision
from mock_data  import START, GOAL, MAP_SIZE, STEP_SIZE, GOAL_BIAS, REWIRE_RADIUS, SIGMA

class PlannerNode(Node):
    """
    ROS2 node that wraps Bi-RRT* planner.
    Listens for goals, runs planner, publishes paths.
    """

    def __init__(self):
        super().__init__('birrt_star_planner')
        self.get_logger().info('Bi-RRT* Planner Node starting...')

        # State 
        self.current_pos  = None   # robot's current position (x,y)
        self.current_goal = None   # current goal position (x,y)
        self.current_path = None   # current planned path
        self.obstacles    = []     # current obstacle list
        self.fw_tree      = None   # forward tree (for visualization)
        self.rv_tree      = None   # reverse tree (for visualization)
        self.active_planning  = False    
        self.last_replan_time = 0.0      

        # Subscribers --------------------------------------
        # Goal position from RViz or topic
        self.goal_sub = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)

        # Robot position from odometry
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        # Moving obstacle positions
        # Format: flat array [x1,y1,r1,vx1,vy1, x2,y2,r2,vx2,vy2, ...]
        self.obs_sub = self.create_subscription(
            Float32MultiArray,'/obstacle_positions',self.obstacle_callback,10)

        # Publishers --------------------------------------
        # Planned path for robot to follow
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.path_pub = self.create_publisher(Path, '/planned_path', qos)
        self.path_viz_pub = self.create_publisher(Marker, '/path_visualization', qos)

        # Tree visualization for RViz
        self.marker_pub = self.create_publisher(MarkerArray,'/planning_markers',10)

        # Timer — check path validity every 0.5 seconds 
        self.check_timer = self.create_timer(0.5,self.check_path_validity)

        self.get_logger().info('Subscribers and publishers ready')
        self.get_logger().info('Waiting for goal on /goal_pose...')


    # Callbacks --------------------------------------
    def odom_callback(self, msg):
        """Receive robot's current pose from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pos = (x, y)

    def obstacle_callback(self, msg):
        """
        Receive moving obstacle positions from Person 2's node.
        Format: flat array [x1,y1,r1,vx1,vy1, x2,y2,r2,vx2,vy2, ...]
        Convert to list of tuples.
        """
        data = msg.data
        self.obstacles = []

        # Each obstacle is 5 values: x, y, radius, vx, vy
        for i in range(0, len(data), 5):
            if i + 5 <= len(data): # make sure all data is there
                obs = (data[i], data[i+1],data[i+2],data[i+3], data[i+4])
                self.obstacles.append(obs)

    def goal_callback(self, msg):
        """Receive new goal position and run the planner. Call when new goal published to /goal_pose"""
        # Extract goal position
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.current_goal = (gx, gy)

        self.get_logger().info(f'New goal received: ({gx:.2f}, {gy:.2f})')

        # Need current position to plan from
        # Find this line in goal_callback
        if self.current_pos is None:
            self.get_logger().warn('No odometry received yet — using default start')
            self.current_pos = (0.3, 5.5)   # spawn position of robot

        # Run the planner
        self.run_planner()

    # Planning --------------------------------------

    def run_planner(self):
        """
        Run Bi-RRT* from current position to current goal. Publish path
        """
        if self.current_pos is None or self.current_goal is None:
            return

        start = self.current_pos
        goal = self.current_goal

        self.get_logger().info(f'Planning from {start} to {goal}...')
        self.get_logger().info(f'Obstacles: {len(self.obstacles)}')

        # Run Bi-RRT*
        path, fw_tree, rv_tree = birrt_star(
            start         = start,
            goal          = goal,
            obstacles     = self.obstacles,
            map_size      = MAP_SIZE,
            max_iter      = 5000,
            step_size     = STEP_SIZE,
            sigma         = SIGMA,
            goal_bias     = GOAL_BIAS,
            rewire_radius = REWIRE_RADIUS
        )

        if path is None:
            self.get_logger().error('No path found!')
            return

        # Store results
        self.current_path = path
        self.active_planning = True
        self.fw_tree = fw_tree
        self.rv_tree = rv_tree

        # Calculate path length for logging
        total_length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_length += np.sqrt(dx**2 + dy**2)

        self.get_logger().info(
            f'Path found! Length: {total_length:.2f}m '
            f'Waypoints: {len(path)}'
        )

        # Publish path and visualization
        self.publish_path(path)
        self.publish_markers(fw_tree, rv_tree, path)
        self.publish_path_visualization(path)  

    def check_path_validity(self):
        if self.current_path is None:
            return
        if not self.active_planning:
            return
        if len(self.obstacles) == 0:
            return
        if self.current_pos is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_replan_time < 1.0:
            return

        # Find closest waypoint to current position
        min_dist = float('inf')
        closest_idx = 0
        for i, wp in enumerate(self.current_path):
            dx   = wp[0] - self.current_pos[0]
            dy   = wp[1] - self.current_pos[1]
            dist = np.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist    = dist
                closest_idx = i

        # Only check segments ahead of robot
        for i in range(closest_idx, len(self.current_path) - 1):
            seg_start = self.current_path[i]
            seg_end   = self.current_path[i + 1]

            if not is_collision_free_path(seg_start, seg_end, self.obstacles):
                self.get_logger().warn(
                    f'Path blocked at segment {i}! Replanning...'
                )
                self.last_replan_time = current_time
                self.run_planner()
                return


    # Publishers  ------------------------------------------------------
    def publish_path(self, path):
        """
        Convert path list to ROS2 nav_msgs/Path and publish.
        """
        ros_path = Path()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = 'map'

        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)
        self.get_logger().info(f'Published path with {len(path)} waypoints')

    def publish_markers(self, fw_tree, rv_tree, path):
        """
        Publish tree and path as RViz markers.
        Person 3's visualization node subscribes to these.

        Forward tree → blue lines |  Reverse tree → green lines | Path → red line
        """
        marker_array = MarkerArray()
        marker_id    = 0

        # Forward Tree — Blue Lines --------------------------------------
        for node in fw_tree:
            if node["parent"] is not None:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'forward_tree'
                m.id = marker_id
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.02 # line width
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 1.0  # blue
                m.color.a = 0.5  # semi transparent

                p1 = Point()
                p1.x  = float(node["pos"][0])
                p1.y  = float(node["pos"][1])
                p1.z  = 0.0

                p2 = Point()
                p2.x  = float(node["parent"]["pos"][0])
                p2.y  = float(node["parent"]["pos"][1])
                p2.z  = 0.0

                m.points = [p1, p2]
                marker_array.markers.append(m)
                marker_id += 1

        # Reverse Tree — Green Lines ------------------------
        for node in rv_tree:
            if node["parent"] is not None:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'reverse_tree'
                m.id = marker_id
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.02
                m.color.r = 0.0
                m.color.g = 1.0 # green
                m.color.b = 0.0
                m.color.a = 0.5

                p1 = Point()
                p1.x = float(node["pos"][0])
                p1.y = float(node["pos"][1])
                p1.z = 0.0

                p2 = Point()
                p2.x = float(node["parent"]["pos"][0])
                p2.y = float(node["parent"]["pos"][1])
                p2.z = 0.0

                m.points = [p1, p2]
                marker_array.markers.append(m)
                marker_id += 1

        # Path — Red Line -------------------------------------------
        if path is not None and len(path) > 1:
            m              = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'path'
            m.id = marker_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05 # thicker line for path
            m.color.r = 1.0 # red
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0 # fully opaque

            for point in path:
                p    = Point()
                p.x  = float(point[0])
                p.y  = float(point[1])
                p.z  = 0.0
                m.points.append(p)

            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)
    

    def publish_path_visualization(self, path):
        """
        Publish path as a green line in Gazebo/RViz so we can
        see exactly where the robot is trying to go.
        """
        if path is None or len(path) < 2:
            return

        m              = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns              = 'planned_path'
        m.id              = 0
        m.type            = Marker.LINE_STRIP
        m.action          = Marker.ADD
        m.scale.x         = 0.05   # line width in meters
        m.color.r         = 0.0
        m.color.g         = 1.0    # green
        m.color.b         = 0.0
        m.color.a         = 1.0

        for point in path:
            p   = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.05   # slightly above ground
            m.points.append(p)

        self.path_viz_pub.publish(m)
        self.get_logger().info(
            f'Path visualization published — {len(path)} points'
        )


# Entry Point ----------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down planner node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()