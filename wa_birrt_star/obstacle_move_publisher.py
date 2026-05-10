import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MovingObstaclePublisher(Node):
    """
    Publishes obstacle positions to /obstacle_positions.
    Positions read from Gazebo odometry (ground truth).
    Bounce logic uses a deadband to prevent getting stuck at boundaries.
    """

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
            [2.8,  5.0,  0.65, 0.0, 0.0],
            [2.8,  4.0,  0.35, 0.0, 0.0],
            [2.8,  3.0,  0.35, 0.0, 0.0],
            [2.8,  1.0,  0.35, 0.0, 0.0],
            [0.55, 0.55, 0.40, 0.0, 0.0],
            [1.35, 0.55, 0.40, 0.0, 0.0],
            [2.15, 0.55, 0.40, 0.0, 0.0],
            [3.8,  1.5,  0.45, 0.0, 0.0],
        ]

        # Moving obstacle state — x/y updated from odom
        self.moving_obstacles = [
            [1.5, 2.5, 0.15, 0.0, 0.0, 'moving_obs_1'], #mid
            [1.5, 4.0, 0.15, 0.0, 0.0, 'moving_obs_2'],#top
            [4.5, 2.5, 0.15, 0.0, 0.0, 'moving_obs_3'],
        ]

        # Commanded velocities — the ONLY source of truth for direction
        # These are what we actually send; we reverse them on bounce
        self.cmd_vels = [
            [ 0.25,  0.00],   # obs1: horizontal
            [ 0.25,  0.00],   # obs2: horizontal
            [ 0.00, -0.25],   # obs3: vertical
        ]

        # Bounce limits (x_min, x_max, y_min, y_max)
        self.bounce_limits = [
            (1.5, 2.5, 1.5, 2.5),   # obs1: moves horizontally x±0.5 around 1.5
            (1.0, 2.2, 3.0, 4.0),   # obs2: moves vertically   y±0.5 around 3.5
            (4.0, 5.0, 1.0, 3.0),   # obs3: moves vertically   y±0.5 around 2.5
        ]

        # Track whether we already bounced this cycle to prevent double-reverse
        self._bounced = [False, False, False]

        # Odometry tracking for position + velocity estimation
        self._prev      = {'moving_obs_1': None, 'moving_obs_2': None, 'moving_obs_3': None}
        self._prev_time = {'moving_obs_1': None, 'moving_obs_2': None, 'moving_obs_3': None}

        self.create_subscription(Odometry, '/moving_obs_1/odom',
                                 lambda m: self._odom_cb(m, 0, 'moving_obs_1'), 10)
        self.create_subscription(Odometry, '/moving_obs_2/odom',
                                 lambda m: self._odom_cb(m, 1, 'moving_obs_2'), 10)
        self.create_subscription(Odometry, '/moving_obs_3/odom',
                                 lambda m: self._odom_cb(m, 2, 'moving_obs_3'), 10)

        self.create_timer(0.1, self.update)
        self.get_logger().info('Moving obstacle publisher started')

    def _odom_cb(self, msg, idx, name):
        """Update position from Gazebo ground truth. Compute velocity."""
        now = self.get_clock().now().nanoseconds / 1e9
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y

        prev   = self._prev[name]
        prev_t = self._prev_time[name]
        if prev is not None and prev_t is not None:
            dt = now - prev_t
            if dt > 0.01:
                raw_vx = (x - prev[0]) / dt
                raw_vy = (y - prev[1]) / dt
                alpha  = 0.3
                self.moving_obstacles[idx][3] = alpha*raw_vx + (1-alpha)*self.moving_obstacles[idx][3]
                self.moving_obstacles[idx][4] = alpha*raw_vy + (1-alpha)*self.moving_obstacles[idx][4]

        self.moving_obstacles[idx][0] = x
        self.moving_obstacles[idx][1] = y
        self._prev[name]      = (x, y)
        self._prev_time[name] = now

    def update(self):
        for i, (obs, limits) in enumerate(zip(self.moving_obstacles, self.bounce_limits)):
            x, y, r, vx, vy, name = obs
            x_min, x_max, y_min, y_max = limits
            cvx, cvy = self.cmd_vels[i]

            bounced = False

            # Bounce on x — only reverse if moving TOWARD the boundary
            # This prevents double-reversals that cause freezing
            if x - r < x_min and cvx < 0:
                cvx = abs(cvx)   # force positive direction
                bounced = True
            elif x + r > x_max and cvx > 0:
                cvx = -abs(cvx)  # force negative direction
                bounced = True

            # Bounce on y — same logic
            if y - r < y_min and cvy < 0:
                cvy = abs(cvy)
                bounced = True
            elif y + r > y_max and cvy > 0:
                cvy = -abs(cvy)
                bounced = True

            # If somehow velocity is zero (stuck), kick it
            if abs(cvx) < 0.01 and abs(cvy) < 0.01:
                self.get_logger().warn(f'{name} velocity is zero — resetting')
                cvx, cvy = self.cmd_vels[i][0], self.cmd_vels[i][1]
                if abs(cvx) < 0.01 and abs(cvy) < 0.01:
                    cvx = 0.20  # emergency kick

            self.cmd_vels[i] = [cvx, cvy]

            cmd = Twist()
            cmd.linear.x = float(cvx)
            cmd.linear.y = float(cvy)
            self.vel_pubs[name].publish(cmd)

        # Publish all positions
        data = []
        for obs in self.static_obstacles: data.extend(obs[:5])
        for obs in self.moving_obstacles:  data.extend(obs[:5])
        msg = Float32MultiArray()
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