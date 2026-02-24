#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def finite_or(value: float, fallback: float) -> float:
    return value if math.isfinite(value) else fallback


def percentile(values: List[float], p: float) -> float:
    """p in [0..1]. Robust metric to avoid single-beam outliers."""
    if not values:
        return float("inf")
    s = sorted(values)
    idx = int(round((len(s) - 1) * clamp(p, 0.0, 1.0)))
    return s[idx]


class AvoidanceNodeV2(Node):
    """
    Reactive avoidance (statis dulu) yang lebih stabil:
      - Definisi 'depan' lebih sempit
      - Pakai percentile/median (anti outlier), bukan min()
      - Ada hysteresis: masuk mode avoid & keluar mode avoid beda threshold
      - Belok proporsional (tidak selalu minimal besar)
      - Fail-safe stop jika /scan macet
    """

    def __init__(self) -> None:
        super().__init__("avoidance_node_v2")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        # Timing
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("scan_timeout_sec", 0.5)

        # Geometry
        self.declare_parameter("front_half_angle_deg", 20.0)   # lebih sempit dari 35°
        self.declare_parameter("side_half_angle_deg", 90.0)

        # Distance thresholds (statis dulu)
        # Masuk avoid kalau depan < enter_avoid
        # Balik cruise kalau depan > exit_avoid  (hysteresis)
        self.declare_parameter("enter_avoid_distance", 0.85)
        self.declare_parameter("exit_avoid_distance", 1.05)
        self.declare_parameter("stop_distance", 0.45)

        # Motion
        self.declare_parameter("cruise_linear", 0.20)
        self.declare_parameter("min_linear", 0.03)
        self.declare_parameter("max_angular", 1.2)

        # Smoothing
        self.declare_parameter("cmd_smoothing_alpha", 0.55)

        # Robust metric
        # 0.2 artinya ambil nilai "cukup dekat" tapi tidak ekstrem (anti outlier)
        self.declare_parameter("front_percentile", 0.20)

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

        self.last_scan: Optional[LaserScan] = None
        self.last_scan_wall: float = 0.0

        self.last_cmd = Twist()
        self.in_avoid_mode = False
        self.turn_dir = +1  # default arah belok saat simetris

        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(f"[avoidance_node_v2] sub={scan_topic} pub={cmd_topic}")

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.last_scan_wall = self.get_clock().now().nanoseconds * 1e-9

    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        a = clamp(angle_rad, scan.angle_min, scan.angle_max)
        i = int(round((a - scan.angle_min) / scan.angle_increment))
        return int(clamp(i, 0, len(scan.ranges) - 1))

    def sector_values(self, scan: LaserScan, a_min: float, a_max: float) -> List[float]:
        i0 = self.angle_to_index(scan, a_min)
        i1 = self.angle_to_index(scan, a_max)
        lo, hi = (i0, i1) if i0 <= i1 else (i1, i0)

        rmin = scan.range_min
        rmax = scan.range_max
        margin = 0.03

        vals: List[float] = []
        for i in range(lo, hi + 1):
            r = scan.ranges[i]
            r = finite_or(r, rmax)
            if r <= rmin + margin:
                r = rmax
            vals.append(clamp(r, rmin, rmax))
        return vals

    def smooth_cmd(self, target: Twist) -> Twist:
        a = float(self.get_parameter("cmd_smoothing_alpha").value)
        out = Twist()
        out.linear.x = (1.0 - a) * self.last_cmd.linear.x + a * target.linear.x
        out.angular.z = (1.0 - a) * self.last_cmd.angular.z + a * target.angular.z
        return out

    def front_distance(self, scan: LaserScan) -> float:
        half = math.radians(float(self.get_parameter("front_half_angle_deg").value))
        vals = self.sector_values(scan, -half, +half)
        p = float(self.get_parameter("front_percentile").value)
        return percentile(vals, p) if vals else scan.range_max

    def side_clearance(self, scan: LaserScan) -> int:
        side_half = math.radians(float(self.get_parameter("side_half_angle_deg").value))

        # kiri: [0..+side_half], kanan: [-side_half..0]
        left_vals = self.sector_values(scan, 0.0, +side_half)
        right_vals = self.sector_values(scan, -side_half, 0.0)

        # pakai percentile juga (lebih stabil)
        left = percentile(left_vals, 0.15) if left_vals else scan.range_max
        right = percentile(right_vals, 0.15) if right_vals else scan.range_max

        if left > right:
            return +1
        if right > left:
            return -1
        return self.turn_dir  # tahan arah sebelumnya kalau sama

    def control_loop(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        timeout = float(self.get_parameter("scan_timeout_sec").value)
        if self.last_scan is None or (now - self.last_scan_wall) > timeout:
            z = Twist()
            self.pub_cmd.publish(z)
            self.last_cmd = z
            self.in_avoid_mode = False
            return

        scan = self.last_scan
        front_d = self.front_distance(scan)

        enter = float(self.get_parameter("enter_avoid_distance").value)
        exit_ = float(self.get_parameter("exit_avoid_distance").value)
        stop_d = float(self.get_parameter("stop_distance").value)

        cruise = float(self.get_parameter("cruise_linear").value)
        min_lin = float(self.get_parameter("min_linear").value)
        max_ang = float(self.get_parameter("max_angular").value)

        # hysteresis
        if not self.in_avoid_mode:
            if front_d < enter:
                self.in_avoid_mode = True
        else:
            if front_d > exit_:
                self.in_avoid_mode = False

        target = Twist()

        if not self.in_avoid_mode:
            target.linear.x = cruise
            target.angular.z = 0.0
        else:
            # pilih arah belok dari clearance kiri/kanan
            self.turn_dir = self.side_clearance(scan)
            dir_ = float(self.turn_dir)

            # jika sangat dekat -> stop + putar
            if front_d < stop_d:
                target.linear.x = 0.0
                target.angular.z = dir_ * max_ang
            else:
                # skala berdasarkan jarak: makin dekat -> makin belok, makin pelan
                # scale 0..1
                d = clamp((front_d - stop_d) / max(enter - stop_d, 1e-3), 0.0, 1.0)

                target.linear.x = min_lin + (cruise - min_lin) * d
                target.angular.z = dir_ * max_ang * (1.0 - d)  # tidak ada “minimal belok”

        cmd = self.smooth_cmd(target)
        self.pub_cmd.publish(cmd)
        self.last_cmd = cmd


def main(args=None) -> None:
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
