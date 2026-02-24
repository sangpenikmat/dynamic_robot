#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class IntelligentAvoidance(Node):
    def __init__(self):
        super().__init__('intelligent_avoidance')
        
        # 1. Parameters
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('danger_dist', 0.8) # Jarak mulai waspada
        self.declare_parameter('stop_dist', 0.4)   # Jarak berhenti total
        
        # 2. Pub/Sub (Pastikan topik sesuai dengan sim.launch.py)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # 3. State Variables
        self.pose = None
        self.scan_data = None
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        
        self.get_logger().info("Intelligent Avoidance System Activated")

    def scan_cb(self, msg): self.scan_data = msg
    def odom_cb(self, msg): self.pose = msg.pose.pose

    def get_yaw(self, q):
        # Convert quaternion to euler yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_sector_min(self, start_deg, end_deg):
        if not self.scan_data: return 10.0
        
        # Convert degrees to indices
        msg = self.scan_data
        idx_start = int((math.radians(start_deg) - msg.angle_min) / msg.angle_increment)
        idx_end = int((math.radians(end_deg) - msg.angle_min) / msg.angle_increment)
        
        # Handle wrap around if necessary (untuk 360 LiDAR)
        indices = range(min(idx_start, idx_end), max(idx_start, idx_end))
        vals = [msg.ranges[i] for i in indices if msg.range_min < msg.ranges[i] < msg.range_max]
        
        return min(vals) if vals else 10.0

    def control_loop(self):
        if not self.pose or not self.scan_data: return

        # --- POSITION LOGIC ---
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        curr_x = self.pose.position.x
        curr_y = self.pose.position.y
        curr_yaw = self.get_yaw(self.pose.orientation)
        
        dist_to_goal = math.hypot(gx - curr_x, gy - curr_y)
        angle_to_goal = math.atan2(gy - curr_y, gx - curr_x)
        heading_error = math.atan2(math.sin(angle_to_goal - curr_yaw), math.cos(angle_to_goal - curr_yaw))

        # --- PERCEPTION LOGIC (5 Sectors) ---
        # Kita bagi depan menjadi lebih detail agar tidak stuck
        left_dist  = self.get_sector_min(30, 90)
        front_dist = self.get_sector_min(-30, 30)
        right_dist = self.get_sector_min(-90, -30)
        
        danger = self.get_parameter('danger_dist').value
        stop   = self.get_parameter('stop_dist').value
        max_v  = self.get_parameter('max_speed').value

        cmd = Twist()

        # --- DECISION ENGINE ---
        if dist_to_goal < 0.3:
            self.get_logger().info("GOAL REACHED!")
            self.pub_cmd.publish(Twist()) # Stop
            return

        if front_dist < stop:
            # SANGAT DEKAT: Berhenti dan putar di tempat ke arah yang lebih lega
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 if left_dist > right_dist else -0.8
            self.get_logger().warn("Obstacle too close! Emergency Spinning.")
        
        elif front_dist < danger:
            # WASPADA: Mulai menghindar sambil tetap bergerak maju pelan
            # Gunakan 'Repulsive Force' dari rintangan
            cmd.linear.x = max_v * 0.4 
            # Jika rintangan lebih dekat di kanan, belok kiri, dan sebaliknya
            avoid_turn = 1.0 if left_dist > right_dist else -1.0
            cmd.angular.z = avoid_turn * 1.2
            self.get_logger().info(f"Obstacle detected ({front_dist:.2f}m). Shifting path.")
        
        else:
            # AMAN: Menuju Goal (Go-To-Goal)
            cmd.linear.x = max_v
            # P control sederhana untuk heading
            cmd.angular.z = 1.5 * heading_error
            # Batasi kecepatan sudut agar tidak liar
            cmd.angular.z = max(-1.2, min(1.2, cmd.angular.z))

        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = IntelligentAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist()) # Force stop on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()