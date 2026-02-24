#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def percentile(vals, p: float) -> float:
    if not vals:
        return float("inf")
    s = sorted(vals)
    i = int(round((len(s) - 1) * clamp(p, 0.0, 1.0)))
    return s[i]

class GoalAvoidNode(Node):
    def __init__(self):
        super().__init__("goal_avoid_node")

        # Topics (Matching your project)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        # Goal position (Setting goal far enough to test avoidance)
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tolerance", 0.35)

        # Motion Profile
        self.declare_parameter("cruise_linear", 0.22)
        self.declare_parameter("max_angular", 1.1)

        # Avoidance Parameters
        self.declare_parameter("enter_avoid_distance", 0.85)
        self.declare_parameter("exit_avoid_distance", 1.10)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("control_rate_hz", 20.0)

        self.sub_scan = self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 10)
        self.sub_odom = self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)

        self.last_scan: Optional[LaserScan] = None
        self.last_odom: Optional[Odometry] = None
        self.in_avoid = False
        self.turn_dir = +1
        self.last_cmd = Twist()

        hz = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.loop)
        self.get_logger().info("Goal Avoidance Node (Reference Optimized) Started.")

    def on_scan(self, msg: LaserScan): self.last_scan = msg
    def on_odom(self, msg: Odometry): self.last_odom = msg

    def sector_vals(self, scan: LaserScan, a_min: float, a_max: float):
        def angle_to_index(angle_rad):
            a = clamp(angle_rad, scan.angle_min, scan.angle_max)
            return int(round((a - scan.angle_min) / scan.angle_increment))

        i0, i1 = angle_to_index(a_min), angle_to_index(a_max)
        lo, hi = (i0, i1) if i0 <= i1 else (i1, i0)
        
        out = []
        for i in range(lo, hi + 1):
            r = scan.ranges[i]
            if not math.isfinite(r) or r <= scan.range_min + 0.03:
                r = scan.range_max
            out.append(clamp(r, scan.range_min, scan.range_max))
        return out

    def loop(self):
        if self.last_scan is None or self.last_odom is None: return

        # Target Params
        goal_x, goal_y = self.get_parameter("goal_x").value, self.get_parameter("goal_y").value
        tol = self.get_parameter("goal_tolerance").value
        cruise = self.get_parameter("cruise_linear").value
        max_ang = self.get_parameter("max_angular").value

        # Pose & Math
        x, y = self.last_odom.pose.pose.position.x, self.last_odom.pose.pose.position.y
        yaw = yaw_from_quat(self.last_odom.pose.pose.orientation)
        dx, dy = goal_x - x, goal_y - y
        dist = math.hypot(dx, dy)

        target = Twist()

        # 1. Check Goal
        if dist < tol:
            self.get_logger().info("Goal Reached!")
            self.pub_cmd.publish(Twist())
            return

        # 2. Avoidance Hysteresis
        front_vals = self.sector_vals(self.last_scan, math.radians(-20), math.radians(20))
        front_d = percentile(front_vals, 0.20)
        
        enter, exit_ = self.get_parameter("enter_avoid_distance").value, self.get_parameter("exit_avoid_distance").value
        
        if not self.in_avoid:
            if front_d < enter: self.in_avoid = True
        else:
            if front_d > exit_: self.in_avoid = False

        # 3. Decision Making
        if self.in_avoid:
            # Pick side based on wider opening
            left = percentile(self.sector_vals(self.last_scan, 0.0, math.radians(90)), 0.15)
            right = percentile(self.sector_vals(self.last_scan, math.radians(-90), 0.0), 0.15)
            self.turn_dir = 1.0 if left > right else -1.0
            
            stop_d = self.get_parameter("stop_distance").value
            if front_d < stop_d:
                target.linear.x = 0.0
                target.angular.z = self.turn_dir * max_ang
            else:
                # Proportional avoidance
                d = clamp((front_d - stop_d) / (enter - stop_d), 0.0, 1.0)
                target.linear.x = cruise * d
                target.angular.z = self.turn_dir * max_ang * (1.0 - d)
        else:
            # Normal Go-to-Goal
            goal_heading = math.atan2(dy, dx)
            error = math.atan2(math.sin(goal_heading - yaw), math.cos(goal_heading - yaw))
            target.linear.x = cruise
            target.angular.z = clamp(1.5 * error, -max_ang, max_ang)

        self.pub_cmd.publish(target)
        self.last_cmd = target

def main(args=None):
    rclpy.init(args=args)
    node = GoalAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()