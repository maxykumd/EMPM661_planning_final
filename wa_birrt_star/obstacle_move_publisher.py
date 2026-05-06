import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class MovingObstaclePublisher(Node):
    def __init__(self):
        super().__init__('moving_obstacle_publisher')

        self.pub = self.create_publisher(Float32MultiArray, '/obstacle_positions', 10)
        self.vel_pubs = {
            'moving_obs_1': self.create_publisher(Twist, '/moving_obs_1/cmd_vel', 10),
            'moving_obs_2': self.create_publisher(Twist, '/moving_obs_2/cmd_vel', 10),
            'moving_obs_3': self.create_publisher(Twist, '/moving_obs_3/cmd_vel', 10),
        }

        # Static obstacles — must match mock_data.py and .world exactly
        self.static_obstacles = [
            [2.8, 5.0, 0.65, 0.0, 0.0],
            [2.8, 4.0, 0.35, 0.0, 0.0],
            [2.8, 3.0, 0.35, 0.0, 0.0],
            [2.8, 1.0, 0.35, 0.0, 0.0],
            [0.55, 0.55, 0.40, 0.0, 0.0],  # bottom wall left
            [1.35, 0.55, 0.40, 0.0, 0.0],  # bottom wall mid
            [2.15, 0.55, 0.40, 0.0, 0.0],  # bottom wall right
            [3.8,  1.5,  0.45, 0.0, 0.0],  # guardian
        ]

        self.moving_obstacles = [
            [1.5, 2.0, 0.20,  0.25,  0.00, 'moving_obs_1'],
            [1.5, 3.5, 0.20,  0.00,  0.20, 'moving_obs_2'],
            [4.5, 2.5, 0.20,  0.15, -0.15, 'moving_obs_3'],
        ]

        self.bounce_limits = [
            (0.4, 5.6, 0.4, 5.6),
            (0.4, 5.6, 0.4, 5.6),
            (3.2, 5.6, 0.4, 5.6),
        ]

        self.create_timer(0.1, self.update)
        self.get_logger().info('Moving obstacle publisher started')

    def update(self):
        for obs, limits in zip(self.moving_obstacles, self.bounce_limits):
            x, y, r, vx, vy, name = obs
            x_min, x_max, y_min, y_max = limits
            x += vx * 0.1; y += vy * 0.1
            if x - r < x_min or x + r > x_max: vx = -vx; x += vx * 0.1
            if y - r < y_min or y + r > y_max: vy = -vy; y += vy * 0.1
            obs[0]=x; obs[1]=y; obs[3]=vx; obs[4]=vy
            cmd = Twist(); cmd.linear.x=float(vx); cmd.linear.y=float(vy)
            self.vel_pubs[name].publish(cmd)

        data = []
        for obs in self.static_obstacles: data.extend(obs[:5])
        for obs in self.moving_obstacles:  data.extend(obs[:5])
        msg = Float32MultiArray(); msg.data = data
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstaclePublisher()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()