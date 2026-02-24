#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class AvoidanceNode(Node):
    """
    Clean reactive avoidance:
      - Cruise straight unless front says "avoid"
      - Turn direction chosen by clearance (left vs right) only when needed
      - Optional TTC for dynamic objects (front-only)
      - Fail-safe stop if /scan stalls
      - RViz debug markers + predicted path
    """

    def __init__(self):
        super().__init__("avoidance_node")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        # Timing
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("scan_timeout_sec", 0.4)

        # Motion
        self.declare_parameter("cruise_linear", 0.22)
        self.declare_parameter("min_linear", 0.03)
        self.declare_parameter("max_angular", 1.20)
        self.declare_parameter("cmd_smoothing_alpha", 0.55)

        # Geometry (forward = 0 rad)
        self.declare_parameter("front_half_angle_deg", 35.0)
        self.declare_parameter("side_half_angle_deg", 95.0)
        self.declare_parameter("zero_gap_deg", 3.0)  # skip +/- gap near 0

        # Distance thresholds
        self.declare_parameter("safety_distance", 1.25)
        self.declare_parameter("stop_distance", 0.55)

        # TTC (dynamic)
        self.declare_parameter("use_ttc", True)
        self.declare_parameter("approach_rate_eps", 0.20)
        self.declare_parameter("ttc_threshold", 1.20)
        self.declare_parameter("ttc_stop", 0.50)
        self.declare_parameter("drdt_smoothing_alpha", 0.35)

        # Turn stability
        self.declare_parameter("turn_hold_time", 0.60)

        # Debug
        self.declare_parameter("debug", True)

        scan_topic = self.get_parameter("scan_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.on_scan, qos)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        self.pub_min_range = self.create_publisher(Float32, "/avoidance/min_range", 10)
        self.pub_min_ttc = self.create_publisher(Float32, "/avoidance/min_ttc", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/avoidance/markers", 10)
        self.pub_path = self.create_publisher(Path, "/avoidance/predicted_path", 10)

        self.last_scan: Optional[LaserScan] = None
        self.last_scan_wall: float = 0.0

        self.prev_ranges: Optional[List[float]] = None
        self.prev_stamp: Optional[float] = None
        self.prev_drdt: Optional[List[float]] = None

        self.last_cmd = Twist()
        self.turn_dir: int = 0
        self.turn_dir_until: float = 0.0

        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(f"[avoidance_node] sub={scan_topic} pub={cmd_topic}")

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_wall = self.get_clock().now().nanoseconds * 1e-9

    # ---------- scan helpers ----------
    def preprocess_ranges(self, scan: LaserScan) -> List[float]:
        out = []
        rmin, rmax = scan.range_min, scan.range_max
        margin = 0.03  # ignore near range_min (self-hit noise)
        for r in scan.ranges:
            if (not math.isfinite(r)) or (r <= rmin + margin):
                out.append(rmax)
            else:
                out.append(clamp(r, rmin, rmax))
        return out

    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        a = clamp(angle_rad, scan.angle_min, scan.angle_max)
        i = int(round((a - scan.angle_min) / scan.angle_increment))
        return int(clamp(i, 0, len(scan.ranges) - 1))

    def sector_indices(self, scan: LaserScan, a_min: float, a_max: float) -> range:
        i0 = self.angle_to_index(scan, a_min)
        i1 = self.angle_to_index(scan, a_max)
        if i0 <= i1:
            return range(i0, i1 + 1)
        return range(i1, i0 + 1)

    def smooth_cmd(self, target: Twist) -> Twist:
        a = float(self.get_parameter("cmd_smoothing_alpha").value)
        out = Twist()
        out.linear.x = (1.0 - a) * self.last_cmd.linear.x + a * target.linear.x
        out.angular.z = (1.0 - a) * self.last_cmd.angular.z + a * target.angular.z
        return out

    # ---------- metrics ----------
    def compute_front(self, scan: LaserScan, ranges: List[float]) -> float:
        front_half = math.radians(float(self.get_parameter("front_half_angle_deg").value))
        front = self.sector_indices(scan, -front_half, +front_half)
        return min(ranges[i] for i in front) if front else scan.range_max

    def compute_min_ttc_front(
        self,
        scan: LaserScan,
        ranges: List[float],
        prev_ranges: Optional[List[float]],
        dt: float,
    ) -> float:
        if not bool(self.get_parameter("use_ttc").value):
            return float("inf")
        if prev_ranges is None or dt <= 1e-3:
            return float("inf")

        front_half = math.radians(float(self.get_parameter("front_half_angle_deg").value))
        front = self.sector_indices(scan, -front_half, +front_half)

        approach_eps = float(self.get_parameter("approach_rate_eps").value)
        alpha = float(self.get_parameter("drdt_smoothing_alpha").value)

        drdt_raw = [(ranges[i] - prev_ranges[i]) / dt for i in range(len(ranges))]
        drdt_raw = [clamp(v, -15.0, 15.0) for v in drdt_raw]

        if self.prev_drdt is None or len(self.prev_drdt) != len(drdt_raw):
            drdt = drdt_raw
        else:
            drdt = [(1 - alpha) * self.prev_drdt[i] + alpha * drdt_raw[i] for i in range(len(drdt_raw))]

        self.prev_drdt = drdt

        min_ttc = float("inf")
        for i in front:
            r = ranges[i]
            v = drdt[i]
            # gate: only meaningful, not far, not near range_min
            if v < -approach_eps and (scan.range_min + 0.10) < r < 6.0:
                ttc = r / max(-v, 1e-3)
                if ttc < min_ttc:
                    min_ttc = ttc
        return min_ttc

    def pick_turn_dir_clearance(self, scan: LaserScan, ranges: List[float]) -> int:
        side_half = math.radians(float(self.get_parameter("side_half_angle_deg").value))
        gap = math.radians(float(self.get_parameter("zero_gap_deg").value))

        left = self.sector_indices(scan, +gap, +side_half)
        right = self.sector_indices(scan, -side_half, -gap)

        left_min = min(ranges[i] for i in left) if left else scan.range_max
        right_min = min(ranges[i] for i in right) if right else scan.range_max

        if left_min > right_min:
            return +1
        if right_min > left_min:
            return -1
        return +1

    # ---------- debug ----------
    def publish_debug(self, scan: LaserScan, min_r: float, min_ttc: float, cmd: Twist):
        if not bool(self.get_parameter("debug").value):
            return

        self.pub_min_range.publish(Float32(data=float(min_r)))
        self.pub_min_ttc.publish(Float32(data=float(min_ttc if math.isfinite(min_ttc) else 999.0)))

        ma = MarkerArray()

        safety = float(self.get_parameter("safety_distance").value)

        m0 = Marker()
        m0.header = scan.header
        m0.ns = "avoidance"
        m0.id = 0
        m0.type = Marker.LINE_STRIP
        m0.action = Marker.ADD
        m0.scale.x = 0.02
        m0.color.a = 0.8
        m0.color.r = 1.0
        m0.color.g = 1.0
        m0.color.b = 0.0

        steps = 72
        m0.points = []
        for k in range(steps + 1):
            a = 2.0 * math.pi * (k / steps)
            m0.points.append(Point(x=safety * math.cos(a), y=safety * math.sin(a), z=0.0))
        ma.markers.append(m0)


        m1 = Marker()
        m1.header = scan.header
        m1.ns = "avoidance"
        m1.id = 1
        m1.type = Marker.ARROW
        m1.action = Marker.ADD
        m1.scale.x = 0.08
        m1.scale.y = 0.16
        m1.scale.z = 0.16
        m1.color.a = 0.9
        m1.color.r = 0.2
        m1.color.g = 1.0
        m1.color.b = 0.2
        L = clamp(abs(cmd.linear.x) * 1.6, 0.2, 1.0)
        m1.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=L, y=0.0, z=0.0)]
        ma.markers.append(m1)

        self.pub_markers.publish(ma)

        path = Path()
        path.header = scan.header
        x, y, yaw = 0.0, 0.0, 0.0
        dt = 0.1
        horizon = 2.0
        v = float(cmd.linear.x)
        w = float(cmd.angular.z)
        for _ in range(int(horizon / dt)):
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += w * dt
            ps = PoseStamped()
            ps.header = scan.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            path.poses.append(ps)
        self.pub_path.publish(path)

    # ---------- main loop ----------
    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Fail-safe: stop if scan stalls
        timeout = float(self.get_parameter("scan_timeout_sec").value)
        if self.last_scan is None or (now - self.last_scan_wall) > timeout:
            z = Twist()
            self.pub_cmd.publish(z)
            self.last_cmd = z
            return

        scan = self.last_scan
        ranges = self.preprocess_ranges(scan)

        # dt from scan stamp with guard
        now_stamp = stamp_to_sec(scan.header.stamp)
        if self.prev_stamp is None:
            dt = 0.1
        else:
            dt = now_stamp - self.prev_stamp
            if dt < 0.02 or dt > 0.5:
                dt = 0.1

        min_r = self.compute_front(scan, ranges)
        min_ttc = self.compute_min_ttc_front(scan, ranges, self.prev_ranges, dt)

        safety = float(self.get_parameter("safety_distance").value)
        stop_d = float(self.get_parameter("stop_distance").value)
        ttc_th = float(self.get_parameter("ttc_threshold").value)
        ttc_stop = float(self.get_parameter("ttc_stop").value)

        cruise = float(self.get_parameter("cruise_linear").value)
        min_lin = float(self.get_parameter("min_linear").value)
        max_ang = float(self.get_parameter("max_angular").value)
        hold = float(self.get_parameter("turn_hold_time").value)

        close = min_r < safety
        very_close = min_r < stop_d
        ttc_risky = math.isfinite(min_ttc) and (min_ttc < ttc_th)
        ttc_urgent = math.isfinite(min_ttc) and (min_ttc < ttc_stop)

        # Cruise: ALWAYS straight
        if not close and not ttc_risky:
            target = Twist()
            target.linear.x = cruise
            target.angular.z = 0.0
            self.turn_dir = 0
        else:
            # Determine dir only when needed
            if now < self.turn_dir_until and self.turn_dir != 0:
                dir_ = self.turn_dir
            else:
                dir_ = self.pick_turn_dir_clearance(scan, ranges)
                self.turn_dir = dir_
                self.turn_dir_until = now + hold

            target = Twist()

            # Emergency: bypass smoothing
            if very_close or ttc_urgent:
                target.linear.x = 0.0
                target.angular.z = float(dir_) * max_ang
                self.pub_cmd.publish(target)
                self.last_cmd = target
                self.publish_debug(scan, min_r, min_ttc, target)
                self.prev_ranges = ranges
                self.prev_stamp = now_stamp
                return

            # Avoid scaling
            if math.isfinite(min_ttc):
                t = clamp((min_ttc - ttc_stop) / max(ttc_th - ttc_stop, 1e-3), 0.0, 1.0)
            else:
                t = 0.5
            d = clamp((min_r - stop_d) / max(safety - stop_d, 1e-3), 0.0, 1.0)
            scale = min(t, d)

            target.linear.x = min_lin + (cruise - min_lin) * scale
            urgency = 1.0 - scale
            target.angular.z = float(dir_) * max_ang * clamp(0.35 + 0.9 * urgency, 0.0, 1.0)

        cmd = self.smooth_cmd(target)
        self.pub_cmd.publish(cmd)
        self.last_cmd = cmd

        self.publish_debug(scan, min_r, min_ttc, cmd)

        self.prev_ranges = ranges
        self.prev_stamp = now_stamp


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()