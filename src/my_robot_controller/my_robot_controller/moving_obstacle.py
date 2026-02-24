#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MultiMovingObstacles(Node):
    def __init__(self):
        super().__init__("multi_moving_obstacle_node")

        self.pub1 = self.create_publisher(Twist, "/obs_1/cmd_vel", 10)
        self.pub2 = self.create_publisher(Twist, "/obs_2/cmd_vel", 10)
        self.pub3 = self.create_publisher(Twist, "/obs_3/cmd_vel", 10)

        # 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info("MultiMovingObstacles active: publishing /obs_[1..3]/cmd_vel")

    def timer_callback(self):
        t = (self.get_clock().now().nanoseconds * 1e-9) - self.start_time

        # obs_1: cepat di y, tapi dibatasi amplitude agar tidak 'teleport feel'
        m1 = Twist()
        m1.linear.y = 2.2 * math.sin(t * 12.0)  # fast, but not insane
        self.pub1.publish(m1)

        # obs_2: meluncur ke arah robot
        m2 = Twist()
        m2.linear.x = -1.5
        self.pub2.publish(m2)

        # obs_3: zigzag + mendekat perlahan
        m3 = Twist()
        m3.linear.y = 1.8 * math.cos(t * 1.6)
        m3.linear.x = -0.6
        self.pub3.publish(m3)


def main(args=None):
    rclpy.init(args=args)
    node = MultiMovingObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
