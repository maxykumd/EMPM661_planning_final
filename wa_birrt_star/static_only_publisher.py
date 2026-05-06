import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class StaticOnlyPublisher(Node):
    """
    Publishes 6 static obstacle positions matching
    planning_world_static.world exactly.
    """

    def __init__(self):
        super().__init__('static_only_publisher')

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/obstacle_positions',
            10
        )

        # Must match planning_world_static.world exactly
        # Format: [x, y, radius, vx, vy]
        self.obstacles = [
            [1.5, 1.5, 0.25, 0.0, 0.0],
            [3.0, 1.5, 0.25, 0.0, 0.0],
            [1.5, 3.0, 0.25, 0.0, 0.0],
            [3.0, 3.0, 0.25, 0.0, 0.0],
            [4.5, 1.5, 0.25, 0.0, 0.0],
            [2.5, 4.5, 0.25, 0.0, 0.0],
        ]

        self.timer = self.create_timer(0.1, self.publish)
        self.get_logger().info(
            f'Publishing {len(self.obstacles)} static obstacles'
        )

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