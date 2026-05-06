import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class StaticOnlyPublisher(Node):
    """
    Publishes static obstacle positions to /obstacle_positions.
    Must match mock_data.py OBSTACLES and planning_world_static.world exactly.
    """

    def __init__(self):
        super().__init__('static_only_publisher')
        self.pub = self.create_publisher(Float32MultiArray, '/obstacle_positions', 10)

        # Format: (x, y, radius, vx, vy) — must match mock_data.py OBSTACLES exactly
        self.obstacles = [
            (2.8, 5.0, 0.65, 0.0, 0.0),   # wall top (large)
            (2.8, 4.0, 0.35, 0.0, 0.0),   # wall upper-mid
            (2.8, 3.0, 0.35, 0.0, 0.0),   # wall lower-mid
            (2.8, 1.0, 0.35, 0.0, 0.0),   # wall bottom — gap above at y≈2.0
            (2.0, 1.0, 0.40, 0.0, 0.0),   # bottom blocker
            (3.8, 1.25, 0.45, 0.0, 0.0),  # guardian
        ]

        self.create_timer(0.1, self.publish)
        self.get_logger().info(f'Publishing {len(self.obstacles)} static obstacles')

    def publish(self):
        msg  = Float32MultiArray()
        data = []
        for obs in self.obstacles:
            data.extend(obs)
        msg.data = data
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticOnlyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()