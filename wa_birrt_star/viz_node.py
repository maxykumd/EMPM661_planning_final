import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
import math


class VizNode(Node):
    def __init__(self):
        super().__init__('viz_node')
        self.replan_count = 0
        self.obstacles    = []

        # Subscribers
        self.create_subscription(Float32MultiArray, '/obstacle_positions', self.obs_callback, 10)
        self.create_subscription(Bool,              '/robot_stop',         self.stop_callback, 10)

        # Publishers
        self.obs_pub  = self.create_publisher(MarkerArray,   '/obstacle_markers', 10)
        self.text_pub = self.create_publisher(Marker,        '/replan_counter',   10)
        self.map_pub  = self.create_publisher(OccupancyGrid, '/planning_map',     10)

        # Publish map once per second — obstacles may move
        self.create_timer(1.0, self.publish_map)

        self.get_logger().info('Viz node started')

    # ── Map constants — must match mock_data.py ───────────────────────────────
    MAP_W      = 6.0
    MAP_H      = 6.0
    RESOLUTION = 0.05   # 5cm per cell — fine enough to show obstacle edges
    CLEARANCE  = 0.20

    def _make_map(self):
        """
        Build an OccupancyGrid from current obstacle list.
        Each cell is 100 (occupied) if inside an obstacle+clearance zone,
        0 (free) otherwise. Static map boundary walls are also marked.
        """
        cols = int(self.MAP_W / self.RESOLUTION)
        rows = int(self.MAP_H / self.RESOLUTION)
        data = [0] * (cols * rows)

        # Mark boundary walls
        for c in range(cols):
            for r in range(rows):
                x = c * self.RESOLUTION + self.RESOLUTION/2
                y = r * self.RESOLUTION + self.RESOLUTION/2
                # Boundary clearance zone
                if (x < self.CLEARANCE or x > self.MAP_W - self.CLEARANCE or
                        y < self.CLEARANCE or y > self.MAP_H - self.CLEARANCE):
                    data[r * cols + c] = 60   # grey — boundary zone

        # Mark obstacles
        for ox, oy, r, vx, vy in self.obstacles:
            # Inflate radius for visualization
            vis_r = r + self.CLEARANCE

            # Bounding box in cells
            c_min = max(0, int((ox - vis_r) / self.RESOLUTION))
            c_max = min(cols-1, int((ox + vis_r) / self.RESOLUTION) + 1)
            r_min = max(0, int((oy - vis_r) / self.RESOLUTION))
            r_max = min(rows-1, int((oy + vis_r) / self.RESOLUTION) + 1)

            for c in range(c_min, c_max+1):
                for r_ in range(r_min, r_max+1):
                    cx = c * self.RESOLUTION + self.RESOLUTION/2
                    cy = r_ * self.RESOLUTION + self.RESOLUTION/2
                    dist = math.hypot(cx-ox, cy-oy)
                    if dist < r:
                        data[r_ * cols + c] = 100  # solid obstacle
                    elif dist < vis_r:
                        data[r_ * cols + c] = 50   # clearance zone

        return data, rows, cols

    def publish_map(self):
        if not self.obstacles:
            return

        data, rows, cols = self._make_map()

        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'

        grid.info.resolution = self.RESOLUTION
        grid.info.width      = cols
        grid.info.height     = rows
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        grid.data = data
        self.map_pub.publish(grid)

    # ── Obstacle markers ──────────────────────────────────────────────────────

    def obs_callback(self, msg):
        data = msg.data
        obs  = []
        arr  = MarkerArray()

        for i in range(0, len(data), 5):
            if i+5 > len(data): break
            x, y, r, vx, vy = data[i], data[i+1], data[i+2], data[i+3], data[i+4]
            obs.append((x, y, r, vx, vy))
            is_moving = abs(vx) > 0.01 or abs(vy) > 0.01

            # Cylinder body
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = 'obstacles'; m.id = i // 5
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x    = float(x)
            m.pose.position.y    = float(y)
            m.pose.position.z    = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = float(r * 2)
            m.scale.y = float(r * 2)
            m.scale.z = 0.3
            m.color.r = 1.0 if is_moving else 0.8
            m.color.g = 0.5 if is_moving else 0.2
            m.color.b = 0.0 if is_moving else 0.2
            m.color.a = 0.9
            arr.markers.append(m)

            # Velocity arrow for moving obstacles
            if is_moving:
                speed = math.hypot(vx, vy)
                if speed > 0.01:
                    a = Marker()
                    a.header.frame_id = 'odom'
                    a.header.stamp    = self.get_clock().now().to_msg()
                    a.ns = 'velocity'; a.id = i // 5
                    a.type = Marker.ARROW; a.action = Marker.ADD
                    a.pose.position.x    = float(x)
                    a.pose.position.y    = float(y)
                    a.pose.position.z    = 0.35
                    yaw = math.atan2(vy, vx)
                    a.pose.orientation.z = math.sin(yaw/2)
                    a.pose.orientation.w = math.cos(yaw/2)
                    a.scale.x = float(speed * 2.0)
                    a.scale.y = 0.06; a.scale.z = 0.06
                    a.color.r = 0.0; a.color.g = 1.0
                    a.color.b = 1.0; a.color.a = 1.0
                    arr.markers.append(a)

        self.obstacles = obs
        self.obs_pub.publish(arr)

    def stop_callback(self, msg):
        if msg.data:
            self.replan_count += 1
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = 'stats'; m.id = 0
        m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
        m.pose.position.x = 0.3
        m.pose.position.y = 5.8
        m.pose.position.z = 0.5
        m.scale.z = 0.35
        m.color.r = 1.0; m.color.g = 1.0
        m.color.b = 1.0; m.color.a = 1.0
        m.text = f'Replans: {self.replan_count}'
        self.text_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()