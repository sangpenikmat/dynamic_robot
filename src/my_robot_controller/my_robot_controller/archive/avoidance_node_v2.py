#!/usr/bin/env python3
import math
from typing import List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Helper Functions
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def finite_or(value: float, fallback: float) -> float:
    return value if math.isfinite(value) else fallback

def percentile(values: List[float], p: float) -> float:
    if not values: return float("inf")
    s = sorted(values)
    idx = int(round((len(s) - 1) * clamp(p, 0.0, 1.0)))
    return s[idx]

class AvoidanceNodeV2(Node):
    def __init__(self) -> None:
        super().__init__("avoidance_node_v2")

        # Parameters (Optimized for Dynamic Obstacles)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("control_rate_hz", 30.0) # Faster control loop
        
        # Geometry Settings
        self.declare_parameter("front_angle_deg", 35.0) 
        self.declare_parameter("safety_threshold", 0.95) # Distance to trigger avoidance
        self.declare_parameter("stop_threshold", 0.55)   # Distance to emergency stop
        
        # Speed Settings
        self.declare_parameter("max_linear", 0.45)
        self.declare_parameter("max_angular", 1.5)
        self.declare_parameter("smoothing_alpha", 0.65)

        # QoS for Jazzy/Harmonic
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Best for fast LiDAR data
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_scan = self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.on_scan, qos)
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)

        self.last_scan: Optional[LaserScan] = None
        self.last_cmd = Twist()
        self.turn_dir = 1.0

        rate = self.get_parameter("control_rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info("Advanced Avoidance Node V2 (English) Started.")

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def get_sector_dist(self, scan: LaserScan, angle_min_deg: float, angle_max_deg: float) -> float:
        # Helper to get distance in specific angle range
        ranges = []
        angle_min = math.radians(angle_min_deg)
        angle_max = math.radians(angle_max_deg)
        
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if angle_min <= angle <= angle_max:
                r = finite_or(r, scan.range_max)
                if r > scan.range_min:
                    ranges.append(r)
        
        return percentile(ranges, 0.15) if ranges else scan.range_max

    def control_loop(self) -> None:
        if self.last_scan is None:
            return

        scan = self.last_scan
        
        # 1. Perception: Check 3 Key Directions
        front_d = self.get_sector_dist(scan, -20.0, 20.0)
        left_d  = self.get_sector_dist(scan, 20.0, 70.0)
        right_d = self.get_sector_dist(scan, -70.0, -20.0)

        # Parameters
        safety = self.get_parameter("safety_threshold").value
        stop   = self.get_parameter("stop_threshold").value
        cruise = self.get_parameter("max_linear").value
        max_ang = self.get_parameter("max_angular").value

        target = Twist()

        # 2. Strategy: Decision Making
        if front_d > safety and left_d > safety * 0.8 and right_d > safety * 0.8:
            # CLEAR PATH
            target.linear.x = cruise
            target.angular.z = 0.0
        elif front_d < stop:
            # EMERGENCY: Obstacle too close! Stop and rotate
            target.linear.x = -0.05 # Back up slightly
            self.turn_dir = 1.0 if left_d > right_d else -1.0
            target.angular.z = self.turn_dir * max_ang
            self.get_logger().warn("EMERGENCY STOP - Rotating to clear path")
        else:
            # PREVENTIVE AVOIDANCE: Scale speed and turn proportional to distance
            # Closer obstacle = Faster turn, Slower linear speed
            avoidance_weight = 1.0 - ((front_d - stop) / (safety - stop))
            avoidance_weight = clamp(avoidance_weight, 0.0, 1.0)
            
            target.linear.x = cruise * (1.0 - avoidance_weight)
            self.turn_dir = 1.0 if left_d > right_d else -1.0
            target.angular.z = self.turn_dir * max_ang * avoidance_weight
            self.get_logger().info(f"Avoiding... Weight: {avoidance_weight:.2f}")

        # 3. Smoothing output to prevent jitter
        alpha = self.get_parameter("smoothing_alpha").value
        final_cmd = Twist()
        final_cmd.linear.x = (1.0 - alpha) * self.last_cmd.linear.x + alpha * target.linear.x
        final_cmd.angular.z = (1.0 - alpha) * self.last_cmd.angular.z + alpha * target.angular.z

        self.pub_cmd.publish(final_cmd)
        self.last_cmd = final_cmd

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()