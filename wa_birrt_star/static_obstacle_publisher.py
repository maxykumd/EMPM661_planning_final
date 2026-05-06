import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math


class MovingObstaclePublisher(Node):

    def __init__(self):
        super().__init__('moving_obstacle_publisher')

        # Planner publisher
        self.pub = self.create_publisher(
            Float32MultiArray, '/obstacle_positions', 10
        )

        # Velocity publishers for each moving obstacle
        self.vel_pubs = {
            'moving_obs_1': self.create_publisher(Twist, '/moving_obs_1/cmd_vel', 10),
            'moving_obs_2': self.create_publisher(Twist, '/moving_obs_2/cmd_vel', 10),
            'moving_obs_3': self.create_publisher(Twist, '/moving_obs_3/cmd_vel', 10),
        }

        self.map_min = 0.4
        self.map_max = 5.6

        self.static_obstacles = [
            [1.5, 1.5, 0.25, 0.0, 0.0],
            [4.5, 1.5, 0.25, 0.0, 0.0],
            [2.5, 4.5, 0.25, 0.0, 0.0],
        ]

        # [x, y, radius, vx, vy, name]
        self.moving_obstacles = [
            [3.0, 1.5, 0.20,  0.3,  0.0,  'moving_obs_1'],
            [1.5, 3.5, 0.20,  0.0,  0.25, 'moving_obs_2'],
            [3.5, 3.5, 0.20,  0.2,  0.2,  'moving_obs_3'],
        ]

        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info('Moving obstacle publisher started')

    def update(self):
        for obs in self.moving_obstacles:
            x, y, r, vx, vy, name = obs

            # Update position estimate
            x += vx * 0.1
            y += vy * 0.1

            # Bounce off walls — reverse velocity
            if x - r < self.map_min or x + r > self.map_max:
                vx = -vx
                x += vx * 0.1
            if y - r < self.map_min or y + r > self.map_max:
                vy = -vy
                y += vy * 0.1

            obs[0] = x
            obs[1] = y
            obs[3] = vx
            obs[4] = vy

            # Send velocity command to move the obstacle
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.linear.y = float(vy)
            self.vel_pubs[name].publish(cmd)

        # Publish all positions to planner
        msg  = Float32MultiArray()
        data = []
        for obs in self.static_obstacles:
            data.extend(obs[:5])
        for obs in self.moving_obstacles:
            data.extend(obs[:5])
        msg.data = data
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()